// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 sensor driver for the Samsung S5K2L7 image sensor
 *
 * 12MP (4032x3024) rear-facing raw Bayer sensor, 4x MIPI CSI-2 D-PHY lanes,
 * RAW10. Found as the primary rear camera on e.g. the Xiaomi Redmi Note 5
 * Pro (whyred).
 *
 *
 * Register list and power-up sequence reverse engineered from a CAMDBG
 * register trace captured off the vendor msm-4.4 downstream kernel
 * (whyred_s5k2l7_ofilm_cn_i). PLL/link-frequency values below are derived
 * from the sensor's own PLL registers (0x0300-0x0312) using the standard
 * SMIA/CCS PLL formula, cross-checked against the captured line/frame
 * timing for self-consistency -- see the comment above
 * s5k2l7_link_freq_menu[] for the derivation. This has NOT been checked
 * against the confidential Samsung datasheet, so please re-verify if you
 * ever get your hands on one.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/*
 * Register map follows the common SMIA/MIPI-CCS layout for the base
 * control/timing block (0x0000-0x04xx), with a Samsung-specific paged
 * "indirect" access window (0x6028/0x602a/0x6f12) used by the vendor
 * init/tuning blob for everything else. The indirect window works like
 * this: write the page/bank id to 0x6028, write the byte offset within
 * that page to 0x602a, then each subsequent write to 0x6f12 stores one
 * 16-bit word at that offset and auto-increments it. We don't rely on
 * the auto-increment -- the table below issues an explicit 0x602a write
 * before every 0x6f12 write, exactly mirroring what the vendor blob does
 * on the wire, which is the safest way to replay a sequence we can't
 * fully decode semantically.
 */
#define S5K2L7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K2L7_CHIP_ID			0x20c7

#define S5K2L7_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K2L7_MODE_STREAMING		BIT(0)
#define S5K2L7_MODE_STANDBY		0

#define S5K2L7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K2L7_HFLIP			BIT(0)
#define S5K2L7_VFLIP			BIT(1)

#define S5K2L7_REG_GROUP_HOLD		CCI_REG8(0x0104)

#define S5K2L7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K2L7_EXPOSURE_MIN		4
#define S5K2L7_EXPOSURE_STEP		1
#define S5K2L7_EXPOSURE_MARGIN		8

/*
 * Companion register the vendor driver always writes alongside a coarse
 * integration time update, wrapped in the same group hold. Its exact
 * function isn't documented anywhere we have access to -- it stays at a
 * constant value (1) throughout the capture regardless of the exposure
 * value being set, so it does not look like a gain code. Kept here and
 * replayed verbatim for parity with the one sequence we know works on
 * real hardware.
 */
#define S5K2L7_REG_EXPOSURE_AUX	CCI_REG16(0x020e)
#define S5K2L7_EXPOSURE_AUX_VAL	0x0001

/*
 * Companion write the vendor blob issues, wrapped in the SAME group hold
 * as every coarse-exposure update, on every single exposure change while
 * actively streaming -- not a one-off pre-stream leftover as previously
 * assumed. Confirmed from a full-session CCI trace (s5k2l7.log): this
 * exact value (0x000c) is written here on all 34 exposure/group-hold
 * cycles captured across two separate capture bursts, always at this
 * fixed value regardless of the actual coarse exposure value being set
 * at 0x0202 in the same cycle (unlike the initial mode-config write to
 * this same register address, which uses the real VTS of 0x0c5a/3162,
 * see s5k2l7_4032x3024_regs[]).
 *
 * This lives at the same register address (0x0340) that the SMIA/CCS
 * FRAME_LENGTH_LINES/VTS field normally occupies, but 12 lines is not a
 * physically sane frame length for a 3024-line-tall mode, so this is
 * almost certainly NOT real VTS in this per-exposure context -- more
 * likely a vendor-specific "commit/latch" token the sensor expects as
 * part of its atomic group-hold exposure-update protocol.
 */
#define S5K2L7_REG_EXPOSURE_LATCH	CCI_REG16(0x0340)
#define S5K2L7_EXPOSURE_LATCH_VAL	0x000c


/*
 * Standard SMIA ANALOGUE_GAIN_CODE_GLOBAL location. Not exercised in the
 * s5k2l7.log trace itself (that clip only sweeps exposure), but the
 * vendor HAL's `dumpsys media.camera` output (cam_info.txt) confirms
 * android.sensor.info.sensitivityRange = [100, 3200] and
 * android.sensor.maxAnalogSensitivity = 1600, i.e. ISO100 is unity gain
 * and the sensor does exactly 16x in the analog domain (1600-3200 is
 * digital gain layered on top by the ISP, not this register).
 *
 * The max below assumes the common SMIA "gain = 256/(256-code)" mapping
 * (code=0 -> 1x, which lines up neatly with ISO100 being the baseline):
 * 16x -> code = 256 - 256/16 = 240. That formula itself is inferred, not
 * confirmed -- if it turns out to be a plain linear code instead, this
 * max is wrong by some constant factor and needs adjusting empirically
 * (shoot a grey card, compare reported vs actual brightness).
 */
#define S5K2L7_REG_AGAIN		CCI_REG16(0x0204)
#define S5K2L7_AGAIN_MIN		0
#define S5K2L7_AGAIN_MAX		240
#define S5K2L7_AGAIN_STEP		1
#define S5K2L7_AGAIN_DEFAULT		0

#define S5K2L7_REG_VTS			CCI_REG16(0x0340)
#define S5K2L7_VTS_MAX			0xffff

#define S5K2L7_REG_HTS			CCI_REG16(0x0342)
#define S5K2L7_REG_X_ADDR_START		CCI_REG16(0x0344)
#define S5K2L7_REG_Y_ADDR_START		CCI_REG16(0x0346)
#define S5K2L7_REG_X_ADDR_END		CCI_REG16(0x0348)
#define S5K2L7_REG_Y_ADDR_END		CCI_REG16(0x034a)
#define S5K2L7_REG_X_OUTPUT_SIZE	CCI_REG16(0x034c)
#define S5K2L7_REG_Y_OUTPUT_SIZE	CCI_REG16(0x034e)

#define S5K2L7_NATIVE_WIDTH		4032
#define S5K2L7_NATIVE_HEIGHT		3024

/*
 * 4 data lanes (DATA0-DATA3) + clock, confirmed against the schematic
 * (EMI1201=CLK, EMI1202=DATA1, EMI1203=DATA0, EMI1204=DATA2,
 * EMI1205=DATA3). This matches what the CSID0/CSIPHY0 register trace in
 * a full CAMDBG capture already suggested: CORE_CTRL_0 = 0x00032103
 * decodes to lane_cnt=4 via mainline's `(lane_cnt - 1) | (lane_assign
 * << 4)` layout (camss-csid-4-1.c / camss-csid-4-7.c), and the same
 * trace configures four identical per-lane CSIPHY blocks rather than
 * three. Earlier notes had this sensor down as 3-lane -- that was wrong.
 */
#define S5K2L7_DATA_LANES		4
#define S5K2L7_MCLK_FREQ		(24 * HZ_PER_MHZ)

/*
 * Link frequency / pixel rate derivation
 * =======================================
 * From the mode-config portion of the trace:
 *   0x0136 EXTCLK_FREQUENCY_MHZ    = 0x1800  -> 24.0 MHz (8.8 fixed point)
 *   0x0304 pre_pll_clk_div         = 6
 *   0x0306 pll_multiplier          = 480 (0x01e0)
 *   0x0308 op_pix_clk_div          = 8
 *   0x030a op_sys_clk_div          = 1
 *   0x0300 vt_pix_clk_div          = 4
 *   0x0302 vt_sys_clk_div          = 1
 *
 * PLL_VCO     = EXTCLK / pre_pll_clk_div * pll_multiplier
 *             = 24MHz / 6 * 480 = 1920 MHz
 * vt_pix_clk  = PLL_VCO / vt_sys_clk_div / vt_pix_clk_div = 480 MHz
 * op_pix_clk  = PLL_VCO / op_sys_clk_div / op_pix_clk_div = 240 MHz
 *
 * Cross-check via frame timing (hts=10160, vts=3162, both confirmed
 * straight from the trace, see s5k2l7_supported_modes[] below):
 *   frame rate = vt_pix_clk / (hts * vts) = 480MHz / 32,125,920 ~= 14.9 fps
 * ...which is a believable rate for full 12MP readout in still-capture
 * mode, and matches the trace being three back-to-back short capture
 * bursts rather than continuous 30fps preview.
 *
 * op_pix_clk (240 MHz) is treated as the V4L2 pixel_rate -- this part
 * doesn't depend on lane count. Inverting the usual
 * "pixel_rate = link_freq * 2 * lanes / bpp" formula for 4 lanes gives:
 *   link_freq = op_pix_clk * bpp / (2 * lanes) = 240e6*10/8 = 300 MHz
 * (previously computed as 400 MHz under the old 3-lane assumption --
 * NOTE: the macro below used to still say 400 MHz even after this
 * comment was updated for 4 lanes; that mismatch is fixed now, see
 * S5K2L7_LINK_FREQ_300MHZ. The aggregate bandwidth pixel_rate*bpp =
 * 2.4Gbps is identical either way, it's just split across a different
 * number of lanes, which is a good internal consistency check but not
 * proof either lane count is right).
 * This round-trips back to exactly 240,000,000 through
 * s5k2l7_freq_to_pixel_rate() below. The sensor's trace also contains a
 * second PLL-shaped register block (0x030c-0x0312) that isn't accounted
 * for above; if this is a genuine second PLL feeding the MIPI TX
 * independently of the vt/op split used here, the real link_freq could
 * differ. Treat this value as "self-consistent and plausible", not
 * "confirmed against the datasheet".
 */
#define S5K2L7_LINK_FREQ_300MHZ		(300ULL * HZ_PER_MHZ)
//#define S5K2L7_LINK_FREQ_544_5MHZ	(544.5 * HZ_PER_MHZ)

#define to_s5k2l7(_sd)			container_of(_sd, struct s5k2l7, sd)

static const s64 s5k2l7_link_freq_menu[] = {
        S5K2L7_LINK_FREQ_300MHZ,
        //S5K2L7_LINK_FREQ_544_5MHZ,
};

/*
 * Base (no flip) CFA order confirmed as GRBG via
 * android.sensor.info.colorFilterArrangement in the vendor HAL's
 * `dumpsys media.camera` output (cam_info.txt, camera IDs 0 and 3, both
 * of which are this sensor) -- this is no longer a guess.
 *
 * Index order must match (vflip << 1 | hflip), see
 * s5k2l7_get_format_code().
 */
static const u32 s5k2l7_mbus_formats[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5k2l7_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k2l7_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 exposure_default;

	struct s5k2l7_reg_list reg_list;
};

struct s5k2l7 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator *vdda;	/* 2.8V analog */
	struct regulator *vddd;	/* 1.2V digital core */
	struct regulator *vddio;	/* 1.8V digital I/O */

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	const struct s5k2l7_mode *mode;
};

/*
 * Full power-up / mode-config register sequence for the only mode this
 * driver supports (4032x3024). Extracted verbatim, in order, from a
 * known-working capture on real whyred hardware (s5k2l7.log, lines 1-337,
 * i.e. everything the vendor driver wrote between the CHIP_ID read and
 * the first MODE_SELECT=streaming write). The three back-to-back capture
 * bursts in that trace replay this exact same sequence three times with
 * byte-for-byte identical values, so a single clean pass is all that's
 * needed here.
 *
 * The trailing default-exposure group-hold write the vendor blob issues
 * right before streaming (coarse_integration_time=10, plus a VTS write
 * down to 12 that does not look physically sane for a 3024-line-tall
 * mode) has been deliberately left out -- default exposure/VTS are set
 * through the V4L2 control framework instead, see s5k2l7_init_controls().
 */
static const struct cci_reg_sequence s5k2l7_4032x3024_regs[] = {
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x0101), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0xbbf4 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6010), 0x0001 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0xf466), 0x000c },
	{ CCI_REG16(0xf468), 0x000d },
	{ CCI_REG16(0xf488), 0x0008 },
	{ CCI_REG16(0xf414), 0x0007 },
	{ CCI_REG16(0xf416), 0x0004 },
	{ CCI_REG16(0x30c6), 0x0100 },
	{ CCI_REG16(0x30ca), 0x0300 },
	{ CCI_REG16(0x30c8), 0x05dc },
	{ CCI_REG16(0x6b36), 0x5200 },
	{ CCI_REG16(0x6b38), 0x0000 },
	{ CCI_REG16(0x0b04), 0x0101 },
	{ CCI_REG16(0x3094), 0x2800 },
	{ CCI_REG16(0x3096), 0x5400 },
	{ CCI_REG16(0x303e), 0x0000 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x16c0 },
	{ CCI_REG16(0x6f12), 0x0046 },
	{ CCI_REG16(0x6f12), 0x0046 },
	{ CCI_REG16(0x602a), 0x09e4 },
	{ CCI_REG16(0x6f12), 0xf484 },
	{ CCI_REG16(0x602a), 0x16ba },
	{ CCI_REG16(0x6f12), 0x2608 },
	{ CCI_REG16(0x602a), 0x16be },
	{ CCI_REG16(0x6f12), 0x2608 },
	{ CCI_REG16(0x602a), 0x15f4 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x1282 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x602a), 0x127a },
	{ CCI_REG16(0x6f12), 0x0009 },
	{ CCI_REG16(0x602a), 0x14c8 },
	{ CCI_REG16(0x6f12), 0x00be },
	{ CCI_REG16(0x602a), 0x15ee },
	{ CCI_REG16(0x6f12), 0x0107 },
	{ CCI_REG16(0x602a), 0x4c64 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x4c6a },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x602a), 0x09a2 },
	{ CCI_REG16(0x6f12), 0x00f8 },
	{ CCI_REG16(0x6f12), 0x007f },
	{ CCI_REG16(0x6f12), 0x00ff },
	{ CCI_REG16(0x6f12), 0x0084 },
	{ CCI_REG16(0x602a), 0x09b0 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x6f12), 0xd0d0 },
	{ CCI_REG16(0x6f12), 0x0011 },
	{ CCI_REG16(0x6f12), 0x0013 },
	{ CCI_REG16(0x6f12), 0xd8d0 },
	{ CCI_REG16(0x602a), 0x09da },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e0 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x6f12), 0x000d },
	{ CCI_REG16(0x602a), 0x09e6 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xd004 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x6f12), 0xd804 },
	{ CCI_REG16(0x602a), 0x0af4 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x4b2e },
	{ CCI_REG16(0x6f12), 0x00cc },
	{ CCI_REG16(0x602a), 0x4b60 },
	{ CCI_REG16(0x6f12), 0x0133 },
	{ CCI_REG16(0x602a), 0x4b92 },
	{ CCI_REG16(0x6f12), 0x00cc },
	{ CCI_REG16(0x602a), 0x4b56 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b88 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b4c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b7e },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b42 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b74 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x15ec },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x0aec },
	{ CCI_REG16(0x6f12), 0x0207 },
	{ CCI_REG16(0x602a), 0x0b02 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x0bd8 },
	{ CCI_REG16(0x6f12), 0x06e0 },
	{ CCI_REG16(0x602a), 0x357c },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4040 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x4b04 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x7970 },
	{ CCI_REG16(0x6218), 0x7150 },
	{ CCI_REG16(0x0344), 0x0000 },
	{ CCI_REG16(0x0346), 0x0000 },
	{ CCI_REG16(0x0348), 0x1fbf },
	{ CCI_REG16(0x034a), 0x0bdf },
	{ CCI_REG16(0x034c), 0x0fc0 },
	{ CCI_REG16(0x034e), 0x0bd0 },
	{ CCI_REG16(0x0408), 0x0010 },
	{ CCI_REG16(0x040a), 0x0008 },
	{ CCI_REG16(0x0900), 0x0011 },
	{ CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 },
	{ CCI_REG16(0x0384), 0x0001 },
	{ CCI_REG16(0x0386), 0x0001 },
	{ CCI_REG16(0x0400), 0x0000 },
	{ CCI_REG16(0x0404), 0x0010 },
	{ CCI_REG16(0x3060), 0x0000 },
	{ CCI_REG16(0x0114), 0x0300 },
	{ CCI_REG16(0x0110), 0x1002 },
	{ CCI_REG16(0x0136), 0x1800 },
	{ CCI_REG16(0x0304), 0x0006 },
	{ CCI_REG16(0x0306), 0x01e0 },
	{ CCI_REG16(0x0302), 0x0001 },
	{ CCI_REG16(0x0300), 0x0004 },
	{ CCI_REG16(0x030c), 0x0001 },
	{ CCI_REG16(0x030e), 0x0004 },
	{ CCI_REG16(0x0310), 0x00de },
	{ CCI_REG16(0x0312), 0x0000 },
	{ CCI_REG16(0x030a), 0x0001 },
	{ CCI_REG16(0x0308), 0x0008 },
	{ CCI_REG16(0x0342), 0x27b0 },
	{ CCI_REG16(0x0340), 0x0c5a },
	{ CCI_REG16(0x021e), 0x0000 },
	{ CCI_REG16(0x3098), 0x0300 },
	{ CCI_REG16(0x309a), 0x0102 },
	{ CCI_REG16(0x30bc), 0x0131 },
	{ CCI_REG16(0x30a6), 0x0010 },
	{ CCI_REG16(0x30a8), 0x0002 },
	{ CCI_REG16(0x30aa), 0x0fc0 },
	{ CCI_REG16(0x30ac), 0x02f4 },
	{ CCI_REG16(0x30a0), 0x0000 },
	{ CCI_REG16(0x30a4), 0x0000 },
	{ CCI_REG16(0x6a0c), 0xffff },
	{ CCI_REG16(0xf41e), 0x2180 },
	{ CCI_REG16(0x6028), 0x2000 },
	{ CCI_REG16(0x602a), 0x0990 },
	{ CCI_REG16(0x6f12), 0x0040 },
	{ CCI_REG16(0x602a), 0x0af8 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x27b8 },
	{ CCI_REG16(0x6f12), 0x0100 },
	{ CCI_REG16(0x602a), 0x2aa8 },
	{ CCI_REG16(0x6f12), 0x0001 },
	{ CCI_REG16(0x602a), 0x1698 },
	{ CCI_REG16(0x6f12), 0x00a2 },
	{ CCI_REG16(0x602a), 0x169c },
	{ CCI_REG16(0x6f12), 0x0028 },
	{ CCI_REG16(0x6f12), 0x0030 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c18 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c28 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x6f12), 0x0c30 },
	{ CCI_REG16(0x602a), 0x16c6 },
	{ CCI_REG16(0x6f12), 0x122f },
	{ CCI_REG16(0x6f12), 0x4328 },
	{ CCI_REG16(0x602a), 0x1604 },
	{ CCI_REG16(0x6f12), 0x0004 },
	{ CCI_REG16(0x602a), 0x14ca },
	{ CCI_REG16(0x6f12), 0x0096 },
	{ CCI_REG16(0x602a), 0x16ce },
	{ CCI_REG16(0x6f12), 0x06c0 },
	{ CCI_REG16(0x6f12), 0x0101 },
	{ CCI_REG16(0x602a), 0x13aa },
	{ CCI_REG16(0x6f12), 0x02a0 },
	{ CCI_REG16(0x602a), 0x13a2 },
	{ CCI_REG16(0x6f12), 0x028c },
	{ CCI_REG16(0x602a), 0x139a },
	{ CCI_REG16(0x6f12), 0x0278 },
	{ CCI_REG16(0x602a), 0x1392 },
	{ CCI_REG16(0x6f12), 0x0264 },
	{ CCI_REG16(0x602a), 0x138a },
	{ CCI_REG16(0x6f12), 0x0250 },
	{ CCI_REG16(0x602a), 0x1382 },
	{ CCI_REG16(0x6f12), 0x023c },
	{ CCI_REG16(0x602a), 0x137a },
	{ CCI_REG16(0x6f12), 0x0228 },
	{ CCI_REG16(0x602a), 0x1372 },
	{ CCI_REG16(0x6f12), 0x0214 },
	{ CCI_REG16(0x602a), 0x136a },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x1362 },
	{ CCI_REG16(0x6f12), 0x01ec },
	{ CCI_REG16(0x602a), 0x135a },
	{ CCI_REG16(0x6f12), 0x01d8 },
	{ CCI_REG16(0x602a), 0x1352 },
	{ CCI_REG16(0x6f12), 0x01c4 },
	{ CCI_REG16(0x602a), 0x134a },
	{ CCI_REG16(0x6f12), 0x01b0 },
	{ CCI_REG16(0x602a), 0x1342 },
	{ CCI_REG16(0x6f12), 0x029b },
	{ CCI_REG16(0x602a), 0x133a },
	{ CCI_REG16(0x6f12), 0x0291 },
	{ CCI_REG16(0x602a), 0x1332 },
	{ CCI_REG16(0x6f12), 0x0287 },
	{ CCI_REG16(0x602a), 0x132a },
	{ CCI_REG16(0x6f12), 0x027d },
	{ CCI_REG16(0x602a), 0x1322 },
	{ CCI_REG16(0x6f12), 0x0273 },
	{ CCI_REG16(0x602a), 0x131a },
	{ CCI_REG16(0x6f12), 0x0269 },
	{ CCI_REG16(0x602a), 0x1312 },
	{ CCI_REG16(0x6f12), 0x025f },
	{ CCI_REG16(0x602a), 0x130a },
	{ CCI_REG16(0x6f12), 0x0255 },
	{ CCI_REG16(0x602a), 0x1302 },
	{ CCI_REG16(0x6f12), 0x024b },
	{ CCI_REG16(0x602a), 0x12fa },
	{ CCI_REG16(0x6f12), 0x0241 },
	{ CCI_REG16(0x602a), 0x12f2 },
	{ CCI_REG16(0x6f12), 0x0237 },
	{ CCI_REG16(0x602a), 0x12ea },
	{ CCI_REG16(0x6f12), 0x022d },
	{ CCI_REG16(0x602a), 0x12e2 },
	{ CCI_REG16(0x6f12), 0x0223 },
	{ CCI_REG16(0x602a), 0x12da },
	{ CCI_REG16(0x6f12), 0x0219 },
	{ CCI_REG16(0x602a), 0x12d2 },
	{ CCI_REG16(0x6f12), 0x020f },
	{ CCI_REG16(0x602a), 0x12ca },
	{ CCI_REG16(0x6f12), 0x0205 },
	{ CCI_REG16(0x602a), 0x12c2 },
	{ CCI_REG16(0x6f12), 0x01fb },
	{ CCI_REG16(0x602a), 0x12ba },
	{ CCI_REG16(0x6f12), 0x01f1 },
	{ CCI_REG16(0x602a), 0x12b2 },
	{ CCI_REG16(0x6f12), 0x01e7 },
	{ CCI_REG16(0x602a), 0x12aa },
	{ CCI_REG16(0x6f12), 0x01dd },
	{ CCI_REG16(0x602a), 0x12a2 },
	{ CCI_REG16(0x6f12), 0x01d3 },
	{ CCI_REG16(0x602a), 0x129a },
	{ CCI_REG16(0x6f12), 0x01c9 },
	{ CCI_REG16(0x602a), 0x1292 },
	{ CCI_REG16(0x6f12), 0x01bf },
	{ CCI_REG16(0x602a), 0x128a },
	{ CCI_REG16(0x6f12), 0x01b5 },
	{ CCI_REG16(0x602a), 0x11e2 },
	{ CCI_REG16(0x6f12), 0x0258 },
	{ CCI_REG16(0x602a), 0x0c32 },
	{ CCI_REG16(0x6f12), 0x000a },
	{ CCI_REG16(0x602a), 0x0b16 },
	{ CCI_REG16(0x6f12), 0x0200 },
	{ CCI_REG16(0x602a), 0x0c20 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d78 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0d98 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0da8 },
	{ CCI_REG16(0x6f12), 0x0361 },
	{ CCI_REG16(0x602a), 0x0ae4 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29e6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29f0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2966 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29a6 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x2970 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x29b0 },
	{ CCI_REG16(0x6f12), 0x0fe0 },
	{ CCI_REG16(0x602a), 0x0ae6 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29e8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29f2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2968 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29a8 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x2972 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x29b2 },
	{ CCI_REG16(0x6f12), 0x0be0 },
	{ CCI_REG16(0x602a), 0x0bb0 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x0b62 },
	{ CCI_REG16(0x6f12), 0x0142 },
	{ CCI_REG16(0x602a), 0x5160 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x09f2 },
	{ CCI_REG16(0x6f12), 0x0180 },
	{ CCI_REG16(0x6f12), 0x2180 },
	{ CCI_REG16(0x6f12), 0xf41e },
	{ CCI_REG16(0x602a), 0x0b06 },
	{ CCI_REG16(0x6f12), 0x0800 },
	{ CCI_REG16(0x602a), 0x5694 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x602a), 0x569c },
	{ CCI_REG16(0x6f12), 0x010a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0240 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x602a), 0x56d8 },
	{ CCI_REG16(0x6f12), 0xffc0 },
	{ CCI_REG16(0x602a), 0x56e0 },
	{ CCI_REG16(0x6f12), 0x010a },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6f12), 0x0240 },
	{ CCI_REG16(0x6f12), 0x0000 },
	{ CCI_REG16(0x6028), 0x4000 },
	{ CCI_REG16(0x6214), 0x79f0 },
	{ CCI_REG16(0x6218), 0x79f0 },
};

static const struct s5k2l7_mode s5k2l7_supported_modes[] = {
	{
		.width = S5K2L7_NATIVE_WIDTH,
		.height = S5K2L7_NATIVE_HEIGHT,
		.hts = 10160,		/* 0x27b0, confirmed from trace */
		.vts = 3162,		/* 0x0c5a, confirmed from trace */
		.exposure_default = 1580,
		.reg_list = {
			.regs = s5k2l7_4032x3024_regs,
			.num_regs = ARRAY_SIZE(s5k2l7_4032x3024_regs),
		},
	},
};

static const char * const s5k2l7_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
	"Fade to grey color bars",
	"PN9",
};

static inline u64 s5k2l7_freq_to_pixel_rate(u64 freq)
{
	return div_u64(freq * 2 * S5K2L7_DATA_LANES, 10);
}

static int s5k2l7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k2l7 *s5k2l7 = container_of(ctrl->handler, struct s5k2l7,
					      ctrl_handler);
	const struct s5k2l7_mode *mode = s5k2l7->mode;
	s64 exposure_max;
	int ret = 0;

	/* Propagate change of current control to related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = mode->height + ctrl->val - S5K2L7_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k2l7->exposure,
					 s5k2l7->exposure->minimum,
					  exposure_max,
					  s5k2l7->exposure->step,
					  min_t(s64, s5k2l7->exposure->val, exposure_max));
	}

	/* Only apply to hardware when the sensor is powered and streaming */
	if (pm_runtime_get_if_in_use(s5k2l7->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(s5k2l7->regmap, S5K2L7_REG_GROUP_HOLD, 1, &ret);
		cci_write(s5k2l7->regmap, S5K2L7_REG_EXPOSURE_LATCH,
			  S5K2L7_EXPOSURE_LATCH_VAL, &ret);
		cci_write(s5k2l7->regmap, S5K2L7_REG_EXPOSURE, ctrl->val, &ret);
		cci_write(s5k2l7->regmap, S5K2L7_REG_EXPOSURE_AUX,
			  S5K2L7_EXPOSURE_AUX_VAL, &ret);
		cci_write(s5k2l7->regmap, S5K2L7_REG_GROUP_HOLD, 0, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_AGAIN, ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = cci_write(s5k2l7->regmap, S5K2L7_REG_ORIENTATION,
				(s5k2l7->vflip->val ? S5K2L7_VFLIP : 0) |
				 (s5k2l7->hflip->val ? S5K2L7_HFLIP : 0), NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k2l7->regmap, CCI_REG16(0x0600), ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k2l7->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k2l7_ctrl_ops = {
	.s_ctrl = s5k2l7_set_ctrl,
};

static int s5k2l7_init_controls(struct s5k2l7 *s5k2l7)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k2l7->ctrl_handler;
	const struct s5k2l7_mode *mode = s5k2l7->mode;
	struct v4l2_fwnode_device_properties props;
	s64 pixel_rate, hblank, vblank, exposure_max;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 9);
	if (ret)
		return ret;

	s5k2l7->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k2l7_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(s5k2l7_link_freq_menu) - 1,
					0, s5k2l7_link_freq_menu);
	if (s5k2l7->link_freq)
		s5k2l7->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = s5k2l7_freq_to_pixel_rate(s5k2l7_link_freq_menu[0]);
	s5k2l7->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       pixel_rate, pixel_rate, 1,
					       pixel_rate);
	if (s5k2l7->pixel_rate)
		s5k2l7->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->hts - mode->width;
	s5k2l7->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (s5k2l7->hblank)
		s5k2l7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Created before vblank, deliberately: EXPOSURE_LATCH and VTS are
	 * the SAME register (0x0340, see the comment above
	 * S5K2L7_REG_EXPOSURE_LATCH). __v4l2_ctrl_handler_setup() applies
	 * every control's s_ctrl in the order controls were created
	 * (list_for_each_entry over hdl->ctrls, which is creation-order,
	 * not id-order or any other order). If vblank were created first,
	 * its correct s_ctrl write of the real VTS (mode->vts) would run,
	 * then exposure's s_ctrl would immediately overwrite that same
	 * register with the exposure-cycle latch token (0x000c) as part of
	 * applying its own default - leaving VTS at 12 for a
	 * mode->height=3024 mode right before MODE_SELECT=streaming is
	 * written. Creating exposure first means vblank's write is the one
	 * that's still standing when streaming starts; the exposure latch
	 * write only matters again on a live exposure change, and that
	 * path calls this s_ctrl in isolation - creation order doesn't
	 * affect it.
	 */
	exposure_max = mode->vts - S5K2L7_EXPOSURE_MARGIN;
	s5k2l7->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K2L7_EXPOSURE_MIN, exposure_max,
					     S5K2L7_EXPOSURE_STEP,
					     mode->exposure_default);

	vblank = mode->vts - mode->height;
	s5k2l7->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   S5K2L7_VTS_MAX - mode->height, 1,
					   vblank);

	s5k2l7->again = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN,
					  S5K2L7_AGAIN_MIN, S5K2L7_AGAIN_MAX,
					  S5K2L7_AGAIN_STEP, S5K2L7_AGAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k2l7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k2l7_test_pattern_menu) - 1,
				     0, 0, s5k2l7_test_pattern_menu);

	s5k2l7->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (s5k2l7->hflip)
		s5k2l7->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	s5k2l7->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k2l7_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k2l7->vflip)
		s5k2l7->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ret = v4l2_fwnode_device_parse(s5k2l7->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k2l7_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5k2l7->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k2l7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	const struct s5k2l7_reg_list *reg_list = &s5k2l7->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k2l7->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(s5k2l7->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret) {
		dev_err(s5k2l7->dev, "failed to write mode regs: %d\n", ret);
		goto error;
	}

	ret = __v4l2_ctrl_handler_setup(s5k2l7->sd.ctrl_handler);
	if (ret) {
		dev_err(s5k2l7->dev, "failed to apply controls: %d\n", ret);
		goto error;
	}

	ret = cci_write(s5k2l7->regmap, S5K2L7_REG_MODE_SELECT,
			S5K2L7_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(s5k2l7->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	return 0;

error:
	pm_runtime_put_autosuspend(s5k2l7->dev);

	return ret;
}

static int s5k2l7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	int ret;

	ret = cci_write(s5k2l7->regmap, S5K2L7_REG_MODE_SELECT,
			S5K2L7_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5k2l7->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k2l7->dev);

	return ret;
}

static u32 s5k2l7_get_format_code(struct s5k2l7 *s5k2l7)
{
	unsigned int i;

	i = (s5k2l7->vflip->val ? 2 : 0) | (s5k2l7->hflip->val ? 1 : 0);

	return s5k2l7_mbus_formats[i];
}

static void s5k2l7_update_pad_format(struct s5k2l7 *s5k2l7,
				     const struct s5k2l7_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k2l7_get_format_code(s5k2l7);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k2l7_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	const struct s5k2l7_mode *mode;
	s64 hblank, vblank, exposure_max;

	mode = v4l2_find_nearest_size(s5k2l7_supported_modes,
				      ARRAY_SIZE(s5k2l7_supported_modes),
				       width, height,
				       fmt->format.width, fmt->format.height);

	s5k2l7_update_pad_format(s5k2l7, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || s5k2l7->mode == mode)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k2l7->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k2l7->vblank, vblank,
				 S5K2L7_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k2l7->vblank, vblank);

	exposure_max = mode->vts - S5K2L7_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k2l7->exposure, S5K2L7_EXPOSURE_MIN,
				 exposure_max, S5K2L7_EXPOSURE_STEP,
				  mode->exposure_default);
	__v4l2_ctrl_s_ctrl(s5k2l7->exposure, mode->exposure_default);

	if (s5k2l7->sd.ctrl_handler->error)
		return s5k2l7->sd.ctrl_handler->error;

	s5k2l7->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k2l7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5k2l7_get_format_code(s5k2l7);

	return 0;
}

static int s5k2l7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	if (fse->index >= ARRAY_SIZE(s5k2l7_supported_modes))
		return -EINVAL;

	if (fse->code != s5k2l7_get_format_code(s5k2l7))
		return -EINVAL;

	fse->min_width = s5k2l7_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k2l7_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k2l7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5k2l7->mode->width;
		sel->r.height = s5k2l7->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k2l7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5k2l7->mode->width,
			.height = s5k2l7->mode->height,
		},
	};

	s5k2l7_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k2l7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k2l7_pad_ops = {
	.set_fmt = s5k2l7_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k2l7_get_selection,
	.enum_mbus_code = s5k2l7_enum_mbus_code,
	.enum_frame_size = s5k2l7_enum_frame_size,
	.enable_streams = s5k2l7_enable_streams,
	.disable_streams = s5k2l7_disable_streams,
};

static const struct v4l2_subdev_ops s5k2l7_subdev_ops = {
	.video = &s5k2l7_video_ops,
	.pad = &s5k2l7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k2l7_internal_ops = {
	.init_state = s5k2l7_init_state,
};

static const struct media_entity_operations s5k2l7_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k2l7_identify_sensor(struct s5k2l7 *s5k2l7)
{
	u64 val;
	int ret;

	ret = cci_read(s5k2l7->regmap, S5K2L7_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(s5k2l7->dev, ret,
				      "failed to read chip id\n");

	if (val != S5K2L7_CHIP_ID)
		return dev_err_probe(s5k2l7->dev, -ENODEV,
				      "chip id mismatch: expected 0x%04x, got 0x%04llx\n",
				      S5K2L7_CHIP_ID, val);

	return 0;
}

static int s5k2l7_check_hwcfg(struct s5k2l7 *s5k2l7)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k2l7->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5K2L7_DATA_LANES,
			},
		},
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K2L7_DATA_LANES) {
		dev_err(s5k2l7->dev, "invalid number of data lanes: %u (expected %u)\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes, S5K2L7_DATA_LANES);
		ret = -EINVAL;
		goto out_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k2l7->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
					s5k2l7_link_freq_menu,
					ARRAY_SIZE(s5k2l7_link_freq_menu),
					&freq_bitmap);

out_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k2l7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);
	int ret;

	/*
	 * Order and delays below match the whyred downstream board's own
	 * named cam-power-seq for this exact sensor (eeprom1 in
	 * sdm660-camera-sensor-mtp_whyred.dtsi), not a guess:
	 *   reset=assert, vdig, vio, (vaf - not ours), vana, mclk, reset=release
	 *   delays (ms):    -     5    0        5        5    5       5
	 * Notably vana (analog) is enabled LAST among the regulators, right
	 * before MCLK -- not second, as an earlier version of this driver
	 * had it. Analog often feeds the sensor's internal PLL/MIPI
	 * transmitter specifically, so getting this order wrong could still
	 * leave CCI/I2C working fine (that mostly just needs vio) while the
	 * high-speed MIPI link never comes up cleanly.
	 */
	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 1);

	if (s5k2l7->vddd) {
		ret = regulator_enable(s5k2l7->vddd);
		if (ret)
			return ret;

		usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);
	}

	if (s5k2l7->vddio) {
		ret = regulator_enable(s5k2l7->vddio);
		if (ret)
			goto disable_vddd;
	}

	if (s5k2l7->vdda) {
		ret = regulator_enable(s5k2l7->vdda);
		if (ret)
			goto disable_vddio;

		usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);
	}

	ret = clk_prepare_enable(s5k2l7->mclk);
	if (ret)
		goto disable_vdda;

	usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);

	/*
	 * Release reset only after MCLK is stable, then give the sensor
	 * time to boot before the first CCI transaction. Downstream only
	 * waits 5ms here; this stays at the more generous 11-15ms observed
	 * in CAMDBG-PWRSEQ.log for extra margin, since a longer wait here is
	 * never harmful.
	 */
	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 0);
	usleep_range(11 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	return 0;

disable_vdda:
	if (s5k2l7->vdda)
		regulator_disable(s5k2l7->vdda);

disable_vddio:
	if (s5k2l7->vddio)
		regulator_disable(s5k2l7->vddio);

disable_vddd:
	if (s5k2l7->vddd) {
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
		regulator_disable(s5k2l7->vddd);
	}

	return ret;
}

static int s5k2l7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	gpiod_set_value_cansleep(s5k2l7->reset_gpio, 1);

	clk_disable_unprepare(s5k2l7->mclk);

	if (s5k2l7->vdda)
		regulator_disable(s5k2l7->vdda);

	if (s5k2l7->vddio)
		regulator_disable(s5k2l7->vddio);

	if (s5k2l7->vddd) {
		usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);
		regulator_disable(s5k2l7->vddd);
	}

	return 0;
}

static int s5k2l7_probe(struct i2c_client *client)
{
	struct s5k2l7 *s5k2l7;
	unsigned long freq;
	int ret;

	s5k2l7 = devm_kzalloc(&client->dev, sizeof(*s5k2l7), GFP_KERNEL);
	if (!s5k2l7)
		return -ENOMEM;

	s5k2l7->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k2l7->sd, client, &s5k2l7_subdev_ops);

	s5k2l7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k2l7->regmap))
		return dev_err_probe(s5k2l7->dev, PTR_ERR(s5k2l7->regmap),
				     "failed to init CCI\n");

	s5k2l7->mclk = devm_v4l2_sensor_clk_get(s5k2l7->dev, NULL);
	if (IS_ERR(s5k2l7->mclk))
		return dev_err_probe(s5k2l7->dev, PTR_ERR(s5k2l7->mclk),
				     "failed to get MCLK\n");

	freq = clk_get_rate(s5k2l7->mclk);
	if (freq != S5K2L7_MCLK_FREQ)
		return dev_err_probe(s5k2l7->dev, -EINVAL,
				     "MCLK frequency %lu Hz is not supported (expected %lu)\n",
				     freq, S5K2L7_MCLK_FREQ);

	ret = s5k2l7_check_hwcfg(s5k2l7);
	if (ret)
		return dev_err_probe(s5k2l7->dev, ret,
				     "failed to check HW configuration\n");

	s5k2l7->reset_gpio = devm_gpiod_get_optional(s5k2l7->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k2l7->reset_gpio))
		return dev_err_probe(s5k2l7->dev, PTR_ERR(s5k2l7->reset_gpio),
				     "cannot get reset GPIO\n");

	s5k2l7->vdda = devm_regulator_get_optional(s5k2l7->dev, "vdda");
	if (IS_ERR(s5k2l7->vdda)) {
		ret = PTR_ERR(s5k2l7->vdda);
		if (ret != -ENODEV)
			return dev_err_probe(s5k2l7->dev, ret,
					     "failed to get vdda regulator\n");
		s5k2l7->vdda = NULL;
	}

	s5k2l7->vddd = devm_regulator_get_optional(s5k2l7->dev, "vddd");
	if (IS_ERR(s5k2l7->vddd)) {
		ret = PTR_ERR(s5k2l7->vddd);
		if (ret != -ENODEV)
			return dev_err_probe(s5k2l7->dev, ret,
					     "failed to get vddd regulator\n");
		s5k2l7->vddd = NULL;
	}

	s5k2l7->vddio = devm_regulator_get_optional(s5k2l7->dev, "vddio");
	if (IS_ERR(s5k2l7->vddio)) {
		ret = PTR_ERR(s5k2l7->vddio);
		if (ret != -ENODEV)
			return dev_err_probe(s5k2l7->dev, ret,
					     "failed to get vddio regulator\n");
		s5k2l7->vddio = NULL;
	}

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = s5k2l7_power_on(s5k2l7->dev);
	if (ret)
		return ret;

	ret = s5k2l7_identify_sensor(s5k2l7);
	if (ret)
		goto err_power_off;

	s5k2l7->mode = &s5k2l7_supported_modes[0];
	ret = s5k2l7_init_controls(s5k2l7);
	if (ret) {
		dev_err_probe(s5k2l7->dev, ret, "failed to init controls\n");
		goto err_power_off;
	}

	s5k2l7->sd.state_lock = s5k2l7->ctrl_handler.lock;
	s5k2l7->sd.internal_ops = &s5k2l7_internal_ops;
	s5k2l7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k2l7->sd.entity.ops = &s5k2l7_subdev_entity_ops;
	s5k2l7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k2l7->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k2l7->sd.entity, 1, &s5k2l7->pad);
	if (ret) {
		dev_err_probe(s5k2l7->dev, ret,
			      "failed to init media entity pads\n");
		goto err_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&s5k2l7->sd);
	if (ret < 0) {
		dev_err_probe(s5k2l7->dev, ret,
			      "failed to init subdev active state\n");
		goto err_media_entity_cleanup;
	}

	pm_runtime_set_active(s5k2l7->dev);
	pm_runtime_enable(s5k2l7->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k2l7->sd);
	if (ret < 0) {
		dev_err_probe(s5k2l7->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto err_subdev_cleanup;
	}

	pm_runtime_set_autosuspend_delay(s5k2l7->dev, 1000);
	pm_runtime_use_autosuspend(s5k2l7->dev);
	pm_runtime_idle(s5k2l7->dev);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&s5k2l7->sd);
	pm_runtime_disable(s5k2l7->dev);
	pm_runtime_set_suspended(s5k2l7->dev);

err_media_entity_cleanup:
	media_entity_cleanup(&s5k2l7->sd.entity);

err_ctrl_handler_free:
	v4l2_ctrl_handler_free(s5k2l7->sd.ctrl_handler);

err_power_off:
	s5k2l7_power_off(s5k2l7->dev);

	return ret;
}

static void s5k2l7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k2l7 *s5k2l7 = to_s5k2l7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k2l7->dev);

	if (!pm_runtime_status_suspended(s5k2l7->dev)) {
		s5k2l7_power_off(s5k2l7->dev);
		pm_runtime_set_suspended(s5k2l7->dev);
	}
}

static const struct dev_pm_ops s5k2l7_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k2l7_power_off, s5k2l7_power_on, NULL)
};

static const struct of_device_id s5k2l7_of_match[] = {
	{ .compatible = "samsung,s5k2l7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k2l7_of_match);

static struct i2c_driver s5k2l7_i2c_driver = {
	.driver = {
		.name = "s5k2l7",
		.pm = &s5k2l7_pm_ops,
		.of_match_table = s5k2l7_of_match,
	},
	.probe = s5k2l7_probe,
	.remove = s5k2l7_remove,
};

module_i2c_driver(s5k2l7_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K2L7 image sensor driver");
MODULE_LICENSE("GPL");
