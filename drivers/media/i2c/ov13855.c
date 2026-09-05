// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV13855 CMOS Image Sensor driver
 *
 * Front camera on Xiaomi Redmi Note 5 Pro (whyred), SDM636/660.
 * Physical port: csiphy2 / csid2 / cci-master 1.
 *
 * WORK IN PROGRESS - register table built from a live CCI trace off a
 * running downstream (msm-4.4 camera_v2) kernel via custom debug
 * instrumentation, not from a datasheet. Power/GPIO/regulator names
 * below are taken directly from sdm660-camera-sensor-mtp_whyred.dtsi
 * (qcom,camera@2) - confirmed real values. Sequencing/timing between
 * them is still a best-effort guess (see s_power_on() below) - the
 * trace only covers CCI/MMIO traffic, not the analog power-up path.
 *
 * Known gaps, do not ship without addressing these:
 *  - Only ONE mode populated (whatever the vendor app used during
 *    capture) - no explicit resolution decode attempted yet for this
 *    sensor (OV13855's crop/output-size registers were not manually
 *    cross-checked the way S5K2L7's were, unlike that driver).
 *  - Register table is a *final converged snapshot* across three
 *    observed re-init cycles in the same capture (the vendor app
 *    reconfigured the sensor multiple times), not necessarily a
 *    single clean vendor sequence - i.e. this needs a sanity check
 *    against another capture before being trusted as-is.
 *  - Control ranges (exposure/gain) unknown, same caveat as elsewhere
 *    in this series.
 *  - Chip-ID unverified (same caveat as elsewhere).
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

/* RESOLVED: found the real mainline driver for the closely related
 * OV13858 part - chip ID is a 24-bit value starting at 0x300a
 * (0x300a/0x300b/0x300c), expected 0x00d855. Our live CCI trace
 * caught a 16-bit read starting one register later (0x300b = 0xd855)
 * - byte-for-byte consistent: byte[0x300b]=0xd8, byte[0x300c]=0x55
 * match the low two bytes of the real 24-bit ID exactly. OV13855 and
 * OV13858 apparently share the identical chip-id value - plausible
 * for two parts in the same product line. */
#define OV13855_REG_CHIP_ID		0x300a
#define OV13855_CHIP_ID			0x00d855

#define OV13855_REG_MODE_SELECT	0x0100	/* 0 = standby, 1 = stream */
#define OV13855_REG_SW_RESET		0x0103
#define OV13855_REG_GROUP_HOLD		0x3208	/* seen heavily used for
						 * atomic AE updates in the
						 * raw trace - not part of
						 * init_table, handled at
						 * runtime if/when AE control
						 * is implemented. */
#define OV13855_REG_LONG_EXPO_HI	0x3500
#define OV13855_REG_LONG_EXPO_MID	0x3501
#define OV13855_REG_LONG_EXPO_LO	0x3502
#define OV13855_REG_LONG_GAIN_HI	0x3508
#define OV13855_REG_LONG_GAIN_LO	0x3509

/* Confirmed via android.sensor.info.activeArraySize = [8 8 4224 3136]
 * from a real dumpsys media.camera capture (device 1, "Front").
 * Matches the largest mode (4224x3136) in Intel's real mainline
 * ov13858.c driver for the closely related OV13858 part - strong
 * cross-confirmation this sensor family shares that exact silicon
 * resolution. */
#define OV13855_NATIVE_WIDTH		4224
#define OV13855_NATIVE_HEIGHT		3136

struct ov13855_reg {
	u16 address;
	u8 val;
};

/*
 * Final converged config table (see banner - built from the LAST of
 * three observed reconfig cycles in one capture session). 0x0100,
 * 0x0103 excluded (handled separately).
 */
static const struct ov13855_reg ov13855_init_table[] = {
	{ 0x5000, 0x00ed },
	{ 0x3d84, 0x0040 },
	{ 0x3d88, 0x0070 },
	{ 0x3d89, 0x0000 },
	{ 0x3d8a, 0x0070 },
	{ 0x3d8b, 0x000f },
	{ 0x3d81, 0x0001 },
	{ 0x0300, 0x0002 },
	{ 0x030b, 0x0006 },
	{ 0x0312, 0x0011 },
	{ 0x3022, 0x0001 },
	{ 0x3013, 0x0032 },
	{ 0x3016, 0x0072 },
	{ 0x301b, 0x00f0 },
	{ 0x301f, 0x00d0 },
	{ 0x3106, 0x0015 },
	{ 0x3500, 0x0001 },
	{ 0x3508, 0x0006 },
	{ 0x350e, 0x0000 },
	{ 0x3510, 0x0000 },
	{ 0x3600, 0x002b },
	{ 0x3612, 0x0005 },
	{ 0x3620, 0x0080 },
	{ 0x3624, 0x001c },
	{ 0x3640, 0x0010 },
	{ 0x3661, 0x0080 },
	{ 0x3664, 0x0073 },
	{ 0x366e, 0x00ff },
	{ 0x3674, 0x0000 },
	{ 0x3679, 0x000c },
	{ 0x367f, 0x0001 },
	{ 0x3709, 0x0068 },
	{ 0x3714, 0x0024 },
	{ 0x371a, 0x003e },
	{ 0x3737, 0x0004 },
	{ 0x373d, 0x0026 },
	{ 0x3764, 0x0020 },
	{ 0x37a1, 0x0036 },
	{ 0x37a8, 0x003b },
	{ 0x37ab, 0x0031 },
	{ 0x37c2, 0x0004 },
	{ 0x37c5, 0x0000 },
	{ 0x37d8, 0x0003 },
	{ 0x37dc, 0x0002 },
	{ 0x37e0, 0x0000 },
	{ 0x3800, 0x0000 },
	{ 0x3809, 0x0080 },
	{ 0x3811, 0x0010 },
	{ 0x3813, 0x0008 },
	{ 0x3820, 0x00b0 },
	{ 0x3826, 0x0011 },
	{ 0x3829, 0x0003 },
	{ 0x3832, 0x0000 },
	{ 0x3c80, 0x0000 },
	{ 0x3c87, 0x0001 },
	{ 0x3c8c, 0x0019 },
	{ 0x3c90, 0x0000 },
	{ 0x3d8c, 0x0073 },
	{ 0x3f00, 0x000b },
	{ 0x3f03, 0x0000 },
	{ 0x4001, 0x00e0 },
	{ 0x4008, 0x0000 },
	{ 0x4011, 0x00f0 },
	{ 0x4017, 0x0008 },
	{ 0x4050, 0x0004 },
	{ 0x4059, 0x0080 },
	{ 0x405e, 0x0020 },
	{ 0x4500, 0x0007 },
	{ 0x4503, 0x0000 },
	{ 0x450a, 0x0004 },
	{ 0x4809, 0x0004 },
	{ 0x480c, 0x0012 },
	{ 0x481f, 0x0030 },
	{ 0x4833, 0x0010 },
	{ 0x4837, 0x000e },
	{ 0x4902, 0x0001 },
	{ 0x4d00, 0x0003 },
	{ 0x5040, 0x0039 },
	{ 0x5180, 0x0000 },
	{ 0x5200, 0x001b },
	{ 0x520b, 0x0007 },
	{ 0x5300, 0x0004 },
	{ 0x5309, 0x00a5 },
	{ 0x5312, 0x0001 },
	{ 0x531b, 0x00a9 },
	{ 0x5405, 0x0002 },
	{ 0x3622, 0x0030 },
	{ 0x3662, 0x0012 },
	{ 0x3739, 0x0012 },
	{ 0x37d9, 0x000c },
	{ 0x37e1, 0x000a },
	{ 0x4009, 0x000f },
	{ 0x5041, 0x0010 },
	{ 0x5305, 0x0070 },
	{ 0x5307, 0x0080 },
	{ 0x530b, 0x00d3 },
	{ 0x5319, 0x0088 },
	{ 0x3208, 0x00a0 },
	{ 0x380e, 0x0019 },
};

struct ov13855 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *mclk;		/* MCLK1, 24MHz, GPIO 33 */
	struct gpio_desc *reset_gpio;	/* CAM_RESET1, GPIO 47 */
	struct gpio_desc *vdig_gpio;	/* CAM_VDIG - PMIC GPIO
					 * (pm660l_gpios 3), not a named
					 * regulator on this port, unlike
					 * S5K2L7/S5K5E8 which use a real
					 * cam_vdig-supply regulator. */
	struct regulator_bulk_data supplies[2];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;

	bool streaming;
};

/* Confirmed from sdm660-camera-sensor-mtp_whyred.dtsi, qcom,camera@2:
 *   cam_vio-supply  = vreg_l11a_1p8 (mainline label; downstream calls it pm660_l11)  1.8V fixed (min=max=1800000)
 *   cam_vana-supply = cam_rear_avdd_gpio_regulator (fixed, GPIO-backed)
 * No cam_vdig-supply here - VDIG is switched via a plain PMIC GPIO
 * instead (qcom,gpio-vdig = <2>, pm660l_gpios 3), see vdig_gpio above.
 */
static const char * const ov13855_supply_names[] = {
	"dovdd", /* cam_vio, PM660 L11, 1.8V fixed */
	"avdd",  /* cam_vana, cam_rear_avdd_gpio_regulator */
};

static inline struct ov13855 *to_ov13855(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov13855, sd);
}

/* 16-bit address, 8-bit data - standard OmniVision convention,
 * confirmed by the trace (every value fits 0x00-0xff). */
static const struct regmap_config ov13855_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
};

static int ov13855_write_table(struct ov13855 *ov13855,
				const struct ov13855_reg *table, size_t count)
{
	int ret, i;

	for (i = 0; i < count; i++) {
		ret = regmap_write(ov13855->regmap, table[i].address,
				    table[i].val);
		if (ret) {
			dev_err(ov13855->dev,
				"failed to write reg 0x%04x = 0x%02x: %d\n",
				table[i].address, table[i].val, ret);
			return ret;
		}
	}
	return 0;
}

static int ov13855_power_on(struct ov13855 *ov13855)
{
	int ret;

	/*
	 * REAL sequence, captured via a dedicated debug patch in
	 * msm_camera_dt_util.c's msm_camera_power_up() on real whyred
	 * hardware. Notably different from S5K2L7's own sequence in
	 * this series: MCLK is enabled BEFORE reset release here, not
	 * after - each sensor genuinely has its own order downstream,
	 * not a shared template.
	 */
	gpiod_set_value_cansleep(ov13855->reset_gpio, 1); /* assert reset */
	usleep_range(1000, 1100);

	ret = regulator_enable(ov13855->supplies[1].consumer); /* avdd */
	if (ret)
		return ret;
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(ov13855->vdig_gpio, 1);
	usleep_range(1000, 1100);

	ret = regulator_enable(ov13855->supplies[0].consumer); /* dovdd */
	if (ret)
		goto err_vdig;
	usleep_range(1000, 1100);

	ret = clk_prepare_enable(ov13855->mclk);
	if (ret)
		goto err_dovdd;
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(ov13855->reset_gpio, 0); /* release reset */
	usleep_range(2000, 2100);

	return 0;

err_dovdd:
	regulator_disable(ov13855->supplies[0].consumer);
err_vdig:
	gpiod_set_value_cansleep(ov13855->vdig_gpio, 0);
	regulator_disable(ov13855->supplies[1].consumer);
	return ret;
}

static void ov13855_power_off(struct ov13855 *ov13855)
{
	/* Reverse of the real up-order. */
	clk_disable_unprepare(ov13855->mclk);
	gpiod_set_value_cansleep(ov13855->reset_gpio, 1);
	regulator_disable(ov13855->supplies[0].consumer); /* dovdd */
	gpiod_set_value_cansleep(ov13855->vdig_gpio, 0);
	regulator_disable(ov13855->supplies[1].consumer); /* avdd */
}


static int ov13855_start_streaming(struct ov13855 *ov13855)
{
	int ret;

	ret = regmap_write(ov13855->regmap, OV13855_REG_SW_RESET, 0x01);
	if (ret)
		return ret;
	usleep_range(2000, 2500); /* TODO: unverified post-reset delay */

	ret = ov13855_write_table(ov13855, ov13855_init_table,
				   ARRAY_SIZE(ov13855_init_table));
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&ov13855->ctrl_handler);
	if (ret)
		return ret;

	return regmap_write(ov13855->regmap, OV13855_REG_MODE_SELECT, 0x01);
}

static int ov13855_stop_streaming(struct ov13855 *ov13855)
{
	return regmap_write(ov13855->regmap, OV13855_REG_MODE_SELECT, 0x00);
}

static int ov13855_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov13855 *ov13855 = to_ov13855(sd);
	int ret = 0;

	if (ov13855->streaming == !!enable)
		return 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(ov13855->dev);
		if (ret < 0)
			return ret;

		ret = ov13855_start_streaming(ov13855);
		if (ret) {
			pm_runtime_put(ov13855->dev);
			return ret;
		}
	} else {
		ov13855_stop_streaming(ov13855);
		pm_runtime_put(ov13855->dev);
	}

	ov13855->streaming = enable;
	return ret;
}

static const struct v4l2_subdev_video_ops ov13855_video_ops = {
	.s_stream = ov13855_s_stream,
};

static int ov13855_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10; /* Confirmed via
						   * android.sensor.info.
						   * colorFilterArrangement
						   * = BGGR in a real
						   * dumpsys media.camera
						   * capture (device 1,
						   * "Front"). */
	return 0;
}

static int ov13855_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	fmt->format.width = OV13855_NATIVE_WIDTH;
	fmt->format.height = OV13855_NATIVE_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

/* Missing enum_frame_size made VIDIOC_SUBDEV_ENUM_FRAME_SIZE return
 * -EINVAL, so libcamera's format enumeration came back empty
 * ("No image format found", -22) even though probe succeeded. */
static int ov13855_enum_frame_size(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = fse->max_width = OV13855_NATIVE_WIDTH;
	fse->min_height = fse->max_height = OV13855_NATIVE_HEIGHT;
	return 0;
}

static int ov13855_get_selection(struct v4l2_subdev *sd,
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
		sel->r.width = OV13855_NATIVE_WIDTH;
		sel->r.height = OV13855_NATIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_pad_ops ov13855_pad_ops = {
	.enum_mbus_code = ov13855_enum_mbus_code,
	.enum_frame_size = ov13855_enum_frame_size,
	.get_fmt = ov13855_get_fmt,
	.set_fmt = ov13855_get_fmt,
	.get_selection = ov13855_get_selection,
};

static const struct v4l2_subdev_ops ov13855_subdev_ops = {
	.video = &ov13855_video_ops,
	.pad = &ov13855_pad_ops,
};

static int ov13855_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov13855 *ov13855 = container_of(ctrl->handler, struct ov13855,
						ctrl_handler);
	int ret = 0;

	if (!pm_runtime_get_if_in_use(ov13855->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE: {
		/* Real ov13858.c writes this as one 24-bit value at
		 * 0x3500, exposure lines shifted left 4 (<<4) - fixed
		 * from an earlier two-register split write that didn't
		 * match this convention. */
		u32 exp = (u32)ctrl->val << 4;
		u8 buf[3] = { (exp >> 16) & 0xff, (exp >> 8) & 0xff,
			      exp & 0xff };
		ret = regmap_bulk_write(ov13855->regmap,
					 OV13855_REG_LONG_EXPO_HI, buf, 3);
		break;
	}
	case V4L2_CID_ANALOGUE_GAIN: {
		u16 gain = (u16)ctrl->val;
		u8 buf[2] = { (gain >> 8) & 0xff, gain & 0xff };

		ret = regmap_bulk_write(ov13855->regmap,
					 OV13855_REG_LONG_GAIN_HI, buf, 2);
		break;
	}
	case V4L2_CID_VBLANK:
	case V4L2_CID_HBLANK:
	case V4L2_CID_PIXEL_RATE:
		/* Fixed/read-only or no confirmed frame-length register
		 * to write yet - stored value only, see
		 * ov13855_init_controls(). */
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ov13855->dev);
	return ret;
}

static const struct v4l2_ctrl_ops ov13855_ctrl_ops = {
	.s_ctrl = ov13855_s_ctrl,
};

static int ov13855_init_controls(struct ov13855 *ov13855)
{
	struct v4l2_ctrl_handler *hdl = &ov13855->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 5);

	/* Confirmed via the real ov13858.c reference driver: MIN=4,
	 * STEP=1, DEFAULT=0x640. MAX is normally VTS-8 (frame-length
	 * dependent, needs a real mode table to compute properly) -
	 * 0xffff kept as a permissive upper bound until that exists. */
	ov13855->exposure = v4l2_ctrl_new_std(hdl, &ov13855_ctrl_ops,
					       V4L2_CID_EXPOSURE,
					       4, 0xffff, 1, 0x640);

	/* TODO: placeholder range - OV13855_REG_LONG_GAIN_HI/LO write
	 * path exists but the actual gain curve isn't trace-confirmed. */
	v4l2_ctrl_new_std(hdl, &ov13855_ctrl_ops,
			   V4L2_CID_ANALOGUE_GAIN, 0, 0xffff, 1, 0);

	/* TODO: NOT trace-confirmed (ov13855.log only shows fragmentary
	 * high-byte AE-loop writes to 0x380e, no clean initial full
	 * value like we got for s5k2l7). Default was 0, which
	 * __v4l2_ctrl_handler_setup() pushes to hardware as zero
	 * vertical blanking right before streaming starts - almost
	 * certainly invalid on real sensors (see the s5k2l7.c fix for
	 * the confirmed case of this same bug class). Bumped off zero
	 * as a safety stopgap, not a verified real value. */
	v4l2_ctrl_new_std(hdl, &ov13855_ctrl_ops,
			   V4L2_CID_VBLANK, 0, 4095, 1, 40);

	/* Read only - TODO: still an approximation, not trace-confirmed.
	 * Computed as native_width * native_height * ~30fps + ~15%
	 * blanking margin (4224*3136*30*1.15) - was a flat 500000000
	 * shared across all three sensors, which the real hardware
	 * rejected ("Pixel clock is too high for CSIPHY"). fps assumed,
	 * not confirmed from a trace. */
	ov13855->pixel_rate = v4l2_ctrl_new_std(hdl, &ov13855_ctrl_ops,
						 V4L2_CID_PIXEL_RATE,
						 1, 457000000, 1, 457000000);
	if (ov13855->pixel_rate)
		ov13855->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* TODO: placeholder - real HBLANK value not yet pulled from the
	 * trace/regmap. */
	ov13855->hblank = v4l2_ctrl_new_std(hdl, &ov13855_ctrl_ops,
					     V4L2_CID_HBLANK, 0, 4095, 1, 0);
	if (ov13855->hblank)
		ov13855->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (hdl->error)
		return hdl->error;

	ov13855->sd.ctrl_handler = hdl;
	return 0;
}

static int ov13855_get_regulators(struct ov13855 *ov13855)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ov13855_supply_names); i++)
		ov13855->supplies[i].supply = ov13855_supply_names[i];

	return devm_regulator_bulk_get(ov13855->dev,
					ARRAY_SIZE(ov13855_supply_names),
					ov13855->supplies);
}

static int ov13855_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov13855 *ov13855;
	int ret;

	ov13855 = devm_kzalloc(dev, sizeof(*ov13855), GFP_KERNEL);
	if (!ov13855)
		return -ENOMEM;

	ov13855->dev = dev;
	v4l2_i2c_subdev_init(&ov13855->sd, client, &ov13855_subdev_ops);

	ov13855->regmap = devm_regmap_init_i2c(client, &ov13855_regmap_config);
	if (IS_ERR(ov13855->regmap))
		return PTR_ERR(ov13855->regmap);

	ov13855->mclk = devm_clk_get(dev, "cam_src_clk");
	if (IS_ERR(ov13855->mclk))
		return dev_err_probe(dev, PTR_ERR(ov13855->mclk),
				      "failed to get MCLK\n");

	ov13855->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						       GPIOD_OUT_HIGH);
	if (IS_ERR(ov13855->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ov13855->reset_gpio),
				      "failed to get reset GPIO\n");

	ov13855->vdig_gpio = devm_gpiod_get_optional(dev, "vdig",
						      GPIOD_OUT_LOW);
	if (IS_ERR(ov13855->vdig_gpio))
		return dev_err_probe(dev, PTR_ERR(ov13855->vdig_gpio),
				      "failed to get VDIG GPIO\n");

	ret = ov13855_get_regulators(ov13855);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = ov13855_power_on(ov13855);
	if (ret)
		return ret;

	{
		/* val_bits=8 on this regmap, so a plain regmap_read()
		 * can only ever return one byte - not enough for the
		 * 24-bit chip-id at 0x300a/0x300b/0x300c. Read all
		 * three bytes explicitly and combine big-endian, per
		 * the real ov13858.c convention this was cross-checked
		 * against. */
		u8 id_buf[3];
		unsigned int chip_id;
		int ret2 = regmap_bulk_read(ov13855->regmap,
					     OV13855_REG_CHIP_ID, id_buf, 3);
		chip_id = (id_buf[0] << 16) | (id_buf[1] << 8) | id_buf[2];
		if (ret2 || chip_id != OV13855_CHIP_ID) {
			dev_err(dev, "chip id mismatch: 0x%06x (ret %d)\n",
				chip_id, ret2);
			ret = ret2 ? ret2 : -ENODEV;
			goto err_power_off;
		}
	}

	ret = ov13855_init_controls(ov13855);
	if (ret)
		goto err_power_off;

	ov13855->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov13855->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov13855->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov13855->sd.entity, 1, &ov13855->pad);
	if (ret)
		goto err_free_ctrls;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = v4l2_async_register_subdev_sensor(&ov13855->sd);
	if (ret)
		goto err_pm_disable;

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	media_entity_cleanup(&ov13855->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&ov13855->ctrl_handler);
err_power_off:
	ov13855_power_off(ov13855);
	return ret;
}

static void ov13855_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov13855 *ov13855 = to_ov13855(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov13855->ctrl_handler);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ov13855_power_off(ov13855);
	pm_runtime_set_suspended(&client->dev);
}

static int ov13855_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13855 *ov13855 = to_ov13855(sd);

	ov13855_power_off(ov13855);
	return 0;
}

static int ov13855_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov13855 *ov13855 = to_ov13855(sd);

	return ov13855_power_on(ov13855);
}

static const struct dev_pm_ops ov13855_pm_ops = {
	SET_RUNTIME_PM_OPS(ov13855_runtime_suspend, ov13855_runtime_resume, NULL)
};

static const struct of_device_id ov13855_of_match[] = {
	{ .compatible = "ovti,ov13855" },
	{ }
};
MODULE_DEVICE_TABLE(of, ov13855_of_match);

static struct i2c_driver ov13855_i2c_driver = {
	.driver = {
		.name = "ov13855",
		.pm = &ov13855_pm_ops,
		.of_match_table = ov13855_of_match,
	},
	.probe = ov13855_probe,
	.remove = ov13855_remove,
};

module_i2c_driver(ov13855_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV13855 camera sensor driver");
MODULE_LICENSE("GPL");
