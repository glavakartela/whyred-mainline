/// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K5E8 CMOS Image Sensor driver
 *
 * Portrait/depth rear camera on Xiaomi Redmi Note 5 Pro (whyred),
 * SDM636/660. Physical port: csiphy1 / csid1 / cci-master 0.
 *
 * WORK IN PROGRESS - register table built from a live CCI trace off a
 * running downstream (msm-4.4 camera_v2) kernel via custom debug
 * instrumentation, not from a datasheet. Power/GPIO/regulator names
 * below are taken directly from sdm660-camera-sensor-mtp_whyred.dtsi
 * (qcom,camera@0) - confirmed real values.
 *
 * Note the DT node's own pinctrl/gpio-req-tbl-label text calls this
 * port "CAMIF_MCLK0"/"CAM_RESET0" even though it's wired to
 * MCLK3_CLK_SRC and GPIO 35/52/45 - that is a label reused across
 * camera@ nodes in this DTS, not an indication this shares physical
 * pins with S5K2L7 (which is the *actual* MCLK0/RESET0 = GPIO 32/48).
 *
 * CONFIRMED: this sensor has NO autofocus actuator - fixed focus
 * lens (only S5K2L7, the rear/main camera, has AF on this device).
 * See the CAM_VAF struct field comment below for what that means for
 * the GPIO 45 label found in the downstream DT.
 *
 * Known gaps, do not ship without addressing these:
 *  - Only ONE mode populated (whatever the vendor app used during
 *    capture) - matches the confirmed native resolution (2592x1944),
 *    so it's likely the actual full mode, not a reduced one.
 *  - Control ranges (exposure/gain) unknown.
 *
 * [RESOLVED] Power-up sequencing/timing: never caught in a live
 * trace (every session only caught this camera's power-DOWN), but
 * the full qcom,cam-power-seq-type/-val/-delay triplet was found
 * directly in the stock whyred DTSI for this sensor instead - no
 * capture needed. See s5k5e8_power_on() for the decoded real order.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define S5K5E8_REG_CHIP_ID		0x0000	/* confirmed via live CCI
						 * trace: sid=0x2d, reg 0x0000
						 * reads 0x5e80 - matches the
						 * sensor's own model number,
						 * high confidence. */
#define S5K5E8_CHIP_ID			0x5e80

#define S5K5E8_REG_MODE_SELECT		0x0100	/* 0 = standby, 1 = stream */
#define S5K5E8_REG_SW_RESET		0x0104	/* NOTE: this sensor's trace
						 * shows 0x0104 used the way
						 * S5K2L7/OV13855 use 0x0103 -
						 * grouped-parameter-hold /
						 * reset-adjacent register,
						 * kept separate from
						 * init_table like the other
						 * two sensors' reset regs. */
#define S5K5E8_REG_CSI_LANE_MODE	0x0114
#define S5K5E8_REG_FRAME_LENGTH_LINES	0x0340
#define S5K5E8_REG_COARSE_INTEG_TIME	0x0202
/* TODO: unverified guess by analogy with S5K2L7, not
 * trace-confirmed for this sensor. */
#define S5K5E8_REG_ANALOG_GAIN		0x0204

/* Confirmed via android.sensor.info.activeArraySize = [8 8 2592 1944]
 * from a real dumpsys media.camera capture (device 2, "Back",
 * physicalSize 2.921x2.195mm) - this is essentially the sensor's
 * full resolution, NOT a reduced/binned depth-sensor mode as
 * originally speculated. */
#define S5K5E8_DEFAULT_WIDTH		2592
#define S5K5E8_DEFAULT_HEIGHT		1944

struct s5k5e8_reg {
	u16 address;
	u16 val;
};

/*
 * Config table: unique registers, first-seen order, final converged
 * value. 0x0100, 0x0104, 0x0a00/0x0a02 (unrelated EEPROM read window)
 * excluded - see file banner.
 */
static const struct s5k5e8_reg s5k5e8_init_table[] = {
	{ 0x3906, 0x007e },
	{ 0x3c01, 0x000f },
	{ 0x3c14, 0x0004 },
	{ 0x3235, 0x0008 },
	{ 0x3063, 0x002e },
	{ 0x307a, 0x0010 },
	{ 0x3079, 0x0020 },
	{ 0x3070, 0x0005 },
	{ 0x3067, 0x0006 },
	{ 0x3071, 0x0062 },
	{ 0x3203, 0x0043 },
	{ 0x3205, 0x0043 },
	{ 0x320b, 0x0042 },
	{ 0x3007, 0x0000 },
	{ 0x3020, 0x0058 },
	{ 0x300d, 0x0034 },
	{ 0x3021, 0x0002 },
	{ 0x3010, 0x0059 },
	{ 0x3002, 0x0001 },
	{ 0x3005, 0x0001 },
	{ 0x3008, 0x0004 },
	{ 0x300f, 0x0070 },
	{ 0x3017, 0x0010 },
	{ 0x3019, 0x0019 },
	{ 0x300c, 0x0062 },
	{ 0x3064, 0x0010 },
	{ 0x3c08, 0x000e },
	{ 0x3c31, 0x000d },
	{ 0x3929, 0x0007 },
	{ 0x0136, 0x0018 },
	{ 0x0305, 0x0006 },
	{ 0x3c1f, 0x0000 },
	{ 0x3c17, 0x0000 },
	{ 0x3c0b, 0x0004 },
	{ 0x3c1c, 0x0047 },
	{ 0x3c16, 0x0000 },
	{ 0x0820, 0x0003 },
	{ 0x0114, 0x0001 },
	{ 0x0344, 0x0000 },
	{ 0x034d, 0x0020 },
	{ 0x0900, 0x0000 },
	{ 0x0381, 0x0001 },
	{ 0x0383, 0x0001 },
	{ 0x0385, 0x0001 },
	{ 0x0387, 0x0001 },
	{ 0x0340, 0x0007 },
	{ 0x0200, 0x0000 },
	{ 0x3303, 0x0002 },
	{ 0x3400, 0x0000 },
	{ 0x323b, 0x0000 },
	{ 0x3301, 0x0000 },
	{ 0x3321, 0x0004 },
	{ 0x3306, 0x0000 },
	{ 0x330e, 0x0000 },
	{ 0x0104, 0x0000 },
	{ 0x0202, 0x0003 },
	{ 0x0101, 0x0000 },
	{ 0x3c0d, 0x0000 },
};

struct s5k5e8 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *mclk;		/* MCLK3_CLK_SRC, 24MHz, GPIO 35 */
	struct gpio_desc *reset_gpio;	/* GPIO 52 */
	/*
	 * GPIO 45 (downstream label "CAM_VAF", seq name
	 * sensor_gpio_vaf). CONFIRMED S5K5E8 has no autofocus actuator
	 * (fixed focus lens - only S5K2L7 has AF on this device, and
	 * actuator-cci0, the CCI master this sensor sits on, shows
	 * zero activity across every live trace taken). Despite that,
	 * the real qcom,cam-power-seq-type/-val arrays from the stock
	 * whyred DTSI DO include this GPIO as step 4 of 7 in the
	 * sensor's own power-up sequence - so whatever it actually
	 * drives (probably nothing functional, likely just a board
	 * template artifact shared with camera positions that DO have
	 * an AK73xx-family actuator), it still needs to be toggled for
	 * this sensor to power up correctly, matching the real vendor
	 * behaviour. Unlike S5K2L7 (where this same kind of pin
	 * genuinely belongs to a separate, functional AK7374 subdev),
	 * here there is no such device to hand ownership to, so it's
	 * modelled directly in this driver instead - not as lens
	 * control, just a required power-sequencing step.
	 */
	struct gpio_desc *vaf_gpio;
	struct regulator_bulk_data supplies[3];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;

	bool streaming;
};

/* Confirmed from sdm660-camera-sensor-mtp_whyred.dtsi, qcom,camera@0:
 *   cam_vio-supply  = vreg_l11a_1p8 (mainline label; downstream calls it pm660_l11)                     1.78V - 1.95V
 *   cam_vana-supply = cam_avdd_gpio_regulator        2.8V (comment in DTS)
 *   cam_vdig-supply = cam_rear_dvdd_gpio_regulator   1.2V (comment in DTS)
 */
static const char * const s5k5e8_supply_names[] = {
	"vddio", /* cam_vio, PM660 L11, 1.78V-1.95V */
	"vdda",  /* cam_vana, cam_avdd_gpio_regulator */
	"vddd",  /* cam_vdig, cam_rear_dvdd_gpio_regulator */
};

static inline struct s5k5e8 *to_s5k5e8(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e8, sd);
}

/* 16-bit address, 16-bit data, big-endian - same Samsung/Sony
 * convention as S5K2L7, confirmed by the trace (e.g. 0x0136=0x0018
 * is a PLL predivider setting, only sensible as one word). */
static const struct regmap_config s5k5e8_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int s5k5e8_write_table(struct s5k5e8 *s5k5e8,
			       const struct s5k5e8_reg *table, size_t count)
{
	int ret, i;

	for (i = 0; i < count; i++) {
		ret = regmap_write(s5k5e8->regmap, table[i].address,
				    table[i].val);
		if (ret) {
			dev_err(s5k5e8->dev,
				"failed to write reg 0x%04x = 0x%04x: %d\n",
				table[i].address, table[i].val, ret);
			return ret;
		}
	}
	return 0;
}

/* TODO: same power-sequencing caveat as s5k2l7.c/ov13855.c. */
static int s5k5e8_power_on(struct s5k5e8 *s5k5e8)
{
	int ret;

	/*
	 * REAL sequence - this one didn't even need a live trace, the
	 * full qcom,cam-power-seq-type/-val/-delay triplet was found
	 * directly in the stock whyred DTSI for this sensor:
	 *
	 *   type:  gpio,  vreg, vreg, gpio, vreg, clk,  gpio
	 *   val:   reset, vana, vdig, vaf,  vio,  mclk, reset
	 *   delay: 1,     5,    5,    5,    5,    1,    10  (ms)
	 *
	 * Same shape as OV13855's real sequence in this series: reset
	 * asserted first, MCLK enabled near the end, reset released
	 * LAST (not before MCLK, unlike S5K2L7's own real sequence -
	 * every sensor in this project turned out to have its own
	 * order, no shared template held for any pair of them).
	 */
	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 1); /* assert reset */
	usleep_range(1000, 1100);

	ret = regulator_enable(s5k5e8->supplies[1].consumer); /* vana */
	if (ret)
		return ret;
	usleep_range(5000, 5100);

	ret = regulator_enable(s5k5e8->supplies[2].consumer); /* vdig */
	if (ret)
		goto err_vana;
	usleep_range(5000, 5100);

	/* See the vaf_gpio struct field comment: toggled here because
	 * the real vendor sequence does so, not because it drives any
	 * confirmed-functional actuator on this sensor. */
	gpiod_set_value_cansleep(s5k5e8->vaf_gpio, 1);
	usleep_range(5000, 5100);

	ret = regulator_enable(s5k5e8->supplies[0].consumer); /* vio */
	if (ret)
		goto err_vdig;
	usleep_range(5000, 5100);

	ret = clk_prepare_enable(s5k5e8->mclk);
	if (ret)
		goto err_vio;
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 0); /* release reset */
	usleep_range(10000, 10100);

	return 0;

err_vio:
	regulator_disable(s5k5e8->supplies[0].consumer);
err_vdig:
	gpiod_set_value_cansleep(s5k5e8->vaf_gpio, 0);
	regulator_disable(s5k5e8->supplies[2].consumer);
err_vana:
	regulator_disable(s5k5e8->supplies[1].consumer);
	return ret;
}

static void s5k5e8_power_off(struct s5k5e8 *s5k5e8)
{
	/* Reverse of the real up-order. */
	clk_disable_unprepare(s5k5e8->mclk);
	gpiod_set_value_cansleep(s5k5e8->reset_gpio, 1);
	regulator_disable(s5k5e8->supplies[0].consumer); /* vio */
	gpiod_set_value_cansleep(s5k5e8->vaf_gpio, 0);
	regulator_disable(s5k5e8->supplies[2].consumer); /* vdig */
	regulator_disable(s5k5e8->supplies[1].consumer); /* vana */
}

static int s5k5e8_start_streaming(struct s5k5e8 *s5k5e8)
{
	int ret;

	ret = s5k5e8_write_table(s5k5e8, s5k5e8_init_table,
				  ARRAY_SIZE(s5k5e8_init_table));
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&s5k5e8->ctrl_handler);
	if (ret)
		return ret;

	return regmap_write(s5k5e8->regmap, S5K5E8_REG_MODE_SELECT, 0x0001);
}

static int s5k5e8_stop_streaming(struct s5k5e8 *s5k5e8)
{
	return regmap_write(s5k5e8->regmap, S5K5E8_REG_MODE_SELECT, 0x0000);
}

static int s5k5e8_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);
	int ret = 0;

	if (s5k5e8->streaming == !!enable)
		return 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(s5k5e8->dev);
		if (ret < 0)
			return ret;

		ret = s5k5e8_start_streaming(s5k5e8);
		if (ret) {
			pm_runtime_put(s5k5e8->dev);
			return ret;
		}
	} else {
		s5k5e8_stop_streaming(s5k5e8);
		pm_runtime_put(s5k5e8->dev);
	}

	s5k5e8->streaming = enable;
	return ret;
}

static const struct v4l2_subdev_video_ops s5k5e8_video_ops = {
	.s_stream = s5k5e8_s_stream,
};

static int s5k5e8_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10; /* TODO: guessed, not
						   * confirmed. */
	return 0;
}

static int s5k5e8_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = S5K5E8_DEFAULT_WIDTH;
	fmt->format.height = S5K5E8_DEFAULT_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

/* Missing enum_frame_size made VIDIOC_SUBDEV_ENUM_FRAME_SIZE return
 * -EINVAL, so libcamera's format enumeration came back empty
 * ("No image format found", -22) even though probe succeeded. */
static int s5k5e8_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = fse->max_width = S5K5E8_DEFAULT_WIDTH;
	fse->min_height = fse->max_height = S5K5E8_DEFAULT_HEIGHT;
	return 0;
}

static int s5k5e8_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K5E8_DEFAULT_WIDTH;
		sel->r.height = S5K5E8_DEFAULT_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_pad_ops s5k5e8_pad_ops = {
	.enum_mbus_code = s5k5e8_enum_mbus_code,
	.enum_frame_size = s5k5e8_enum_frame_size,
	.get_fmt = s5k5e8_get_fmt,
	.set_fmt = s5k5e8_get_fmt,
	.get_selection = s5k5e8_get_selection,
};

static const struct v4l2_subdev_ops s5k5e8_subdev_ops = {
	.video = &s5k5e8_video_ops,
	.pad = &s5k5e8_pad_ops,
};

static int s5k5e8_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e8 *s5k5e8 = container_of(ctrl->handler, struct s5k5e8,
					      ctrl_handler);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(s5k5e8->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = regmap_write(s5k5e8->regmap,
				    S5K5E8_REG_COARSE_INTEG_TIME, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		/* TODO: 0x0204 is an unverified guess by analogy with
		 * S5K2L7 (same vendor register-map family) - not
		 * trace-confirmed for this specific sensor. */
		ret = regmap_write(s5k5e8->regmap,
				    S5K5E8_REG_ANALOG_GAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = regmap_write(s5k5e8->regmap,
				    S5K5E8_REG_FRAME_LENGTH_LINES,
				    ctrl->val + S5K5E8_DEFAULT_HEIGHT);
		break;
	case V4L2_CID_HBLANK:
	case V4L2_CID_PIXEL_RATE:
		/* Fixed/read-only, see s5k5e8_init_controls(). */
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k5e8->dev);
	return ret;
}

static const struct v4l2_ctrl_ops s5k5e8_ctrl_ops = {
	.s_ctrl = s5k5e8_s_ctrl,
};

static int s5k5e8_init_controls(struct s5k5e8 *s5k5e8)
{
	struct v4l2_ctrl_handler *hdl = &s5k5e8->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 5);

	/* TODO: range placeholder, trace only shows 0x0202=0x0003. */
	s5k5e8->exposure = v4l2_ctrl_new_std(hdl, &s5k5e8_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      0, 4095, 1, 0x0003);

	/* TODO: placeholder range - S5K5E8_REG_ANALOG_GAIN address itself
	 * is an unverified guess, see s5k5e8_s_ctrl(). */
	v4l2_ctrl_new_std(hdl, &s5k5e8_ctrl_ops,
			   V4L2_CID_ANALOGUE_GAIN, 0, 511, 1, 0);

	/* TODO: NOT trace-confirmed (s5k5e8.log only shows a repeated
	 * high-byte-only 0x0007 write to 0x0340, no clean initial full
	 * value like we got for s5k2l7). Default was 0, which
	 * __v4l2_ctrl_handler_setup() pushes to hardware as zero
	 * vertical blanking right before streaming starts - almost
	 * certainly invalid on real sensors (see the s5k2l7.c fix for
	 * the confirmed case of this same bug class). Bumped off zero
	 * as a safety stopgap, not a verified real value. */
	v4l2_ctrl_new_std(hdl, &s5k5e8_ctrl_ops,
			   V4L2_CID_VBLANK, 0, 4095, 1, 40);

	/* Read only - TODO: still an approximation, not trace-confirmed.
	 * Computed as native_width * native_height * ~30fps + ~15%
	 * blanking margin (2592*1944*30*1.15) - was a flat 500000000
	 * shared across all three sensors, which the real hardware
	 * rejected ("Pixel clock is too high for CSIPHY") - this sensor
	 * especially, being much smaller in frame area than the other
	 * two yet sharing the same flat value. fps assumed, not
	 * confirmed from a trace. */
	s5k5e8->pixel_rate = v4l2_ctrl_new_std(hdl, &s5k5e8_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, 174000000, 1, 174000000);
	if (s5k5e8->pixel_rate)
		s5k5e8->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* TODO: placeholder - real HBLANK value not yet pulled from the
	 * trace/regmap. */
	s5k5e8->hblank = v4l2_ctrl_new_std(hdl, &s5k5e8_ctrl_ops,
					    V4L2_CID_HBLANK, 0, 4095, 1, 0);
	if (s5k5e8->hblank)
		s5k5e8->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error)
		return hdl->error;

	s5k5e8->sd.ctrl_handler = hdl;
	return 0;
}

static int s5k5e8_get_regulators(struct s5k5e8 *s5k5e8)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5k5e8_supply_names); i++)
		s5k5e8->supplies[i].supply = s5k5e8_supply_names[i];

	return devm_regulator_bulk_get(s5k5e8->dev,
					ARRAY_SIZE(s5k5e8_supply_names),
					s5k5e8->supplies);
}

static int s5k5e8_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e8 *s5k5e8;
	int ret;

	s5k5e8 = devm_kzalloc(dev, sizeof(*s5k5e8), GFP_KERNEL);
	if (!s5k5e8)
		return -ENOMEM;

	s5k5e8->dev = dev;
	v4l2_i2c_subdev_init(&s5k5e8->sd, client, &s5k5e8_subdev_ops);

	s5k5e8->regmap = devm_regmap_init_i2c(client, &s5k5e8_regmap_config);
	if (IS_ERR(s5k5e8->regmap))
		return PTR_ERR(s5k5e8->regmap);

	s5k5e8->mclk = devm_clk_get(dev, "cam_src_clk");
	if (IS_ERR(s5k5e8->mclk))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->mclk),
				      "failed to get MCLK\n");

	s5k5e8->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(s5k5e8->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->reset_gpio),
				      "failed to get reset GPIO\n");

	s5k5e8->vaf_gpio = devm_gpiod_get_optional(dev, "vaf",
						    GPIOD_OUT_LOW);
	if (IS_ERR(s5k5e8->vaf_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k5e8->vaf_gpio),
				      "failed to get VAF GPIO\n");

	ret = s5k5e8_get_regulators(s5k5e8);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = s5k5e8_power_on(s5k5e8);
	if (ret)
		return ret;

	{
		unsigned int val;
		int ret2 = regmap_read(s5k5e8->regmap, S5K5E8_REG_CHIP_ID,
					&val);
		if (ret2 || val != S5K5E8_CHIP_ID) {
			dev_err(dev, "chip id mismatch: 0x%04x (ret %d)\n",
				val, ret2);
			ret = ret2 ? ret2 : -ENODEV;
			goto err_power_off;
		}
	}

	ret = s5k5e8_init_controls(s5k5e8);
	if (ret)
		goto err_power_off;

	s5k5e8->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k5e8->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k5e8->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s5k5e8->sd.entity, 1, &s5k5e8->pad);
	if (ret)
		goto err_free_ctrls;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k5e8->sd);
	if (ret)
		goto err_pm_disable;

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	media_entity_cleanup(&s5k5e8->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&s5k5e8->ctrl_handler);
err_power_off:
	s5k5e8_power_off(s5k5e8);
	return ret;
}

static void s5k5e8_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k5e8->ctrl_handler);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		s5k5e8_power_off(s5k5e8);
	pm_runtime_set_suspended(&client->dev);
}

static int s5k5e8_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	s5k5e8_power_off(s5k5e8);
	return 0;
}

static int s5k5e8_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e8 *s5k5e8 = to_s5k5e8(sd);

	return s5k5e8_power_on(s5k5e8);
}

static const struct dev_pm_ops s5k5e8_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k5e8_runtime_suspend, s5k5e8_runtime_resume, NULL)
};

static const struct of_device_id s5k5e8_of_match[] = {
	{ .compatible = "samsung,s5k5e8" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k5e8_of_match);

static struct i2c_driver s5k5e8_i2c_driver = {
	.driver = {
		.name = "s5k5e8",
		.pm = &s5k5e8_pm_ops,
		.of_match_table = s5k5e8_of_match,
	},
	.probe = s5k5e8_probe,
	.remove = s5k5e8_remove,
};

module_i2c_driver(s5k5e8_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K5E8 camera sensor driver");
MODULE_LICENSE("GPL");

