// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation
// Copyright Kulesha Evgeniy voodop@gmail.com

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

struct ak73xx_chipdef {
	u8 reg_position;
	u8 reg_cont;
	u8 shift_pos;
	u8 mode_active;
	u8 mode_standby;
	bool has_standby;
	u16 focus_pos_max;
	u16 focus_steps;
	u16 ctrl_steps;
	u16 ctrl_delay_us;
	u16 power_delay_us;
};

static const struct ak73xx_chipdef ak7345_cdef = {
	.reg_position = 0x0,
	.reg_cont = 0x2,
	.shift_pos = 7, /* 9 bits position values, need to << 7 */
	.mode_active = 0x0,
	.has_standby = false,
	.focus_pos_max = 511,
	.focus_steps = 1,
	.ctrl_steps = 16,
	.ctrl_delay_us = 1000,
	.power_delay_us = 20000,
};

static const struct ak73xx_chipdef ak7375_cdef = {
	.reg_position = 0x0,
	.reg_cont = 0x2,
	.shift_pos = 4, /* 12 bits position values, need to << 4 */
	.mode_active = 0x0,
	.mode_standby = 0x40,
	.has_standby = true,
	.focus_pos_max = 4095,
	.focus_steps = 1,
	.ctrl_steps = 64,
	.ctrl_delay_us = 1000,
	.power_delay_us = 10000,
};

/*
 * whyred (Redmi Note 5 Pro, SDM636/660) uses AK7374 on the S5K2L7
 * main rear camera per the downstream stock kernel's actuator
 * bring-up - now fully confirmed via a live CCI trace, see the
 * banner directly above ak7374_cdef below. (Earlier revisions of
 * this file had every field here marked UNCONFIRMED/guessed from
 * AK7375's known register layout - that guess turned out correct.)
 */
/*
 * CONFIRMED via this project's own live CCI trace on real whyred
 * hardware (tagged via the actuator's cci_client dbg_tag added to
 * msm_actuator.c): sid=0x0c matches the Galaxy S9+ reference exactly
 * (freeza-inc/bm-galaxy-sm9ne-plus-exynos, exynos9810-star2lte_
 * common.dtsi actuator@0C). Every reg 0x0002 write seen was 0x0000
 * (mode_active). Every single reg 0x0000 (position) write across 34
 * captured writes had its low nibble equal to zero with no
 * exception - airtight confirmation of a 12-bit value left-shifted
 * by 4. Decoded real positions spanned 680-1256, comfortably inside
 * a 0-4095 (12-bit) range.
 *
 * Still not directly observed: the standby/sleep write (mode_standby)
 * - "vendor_use_sleep_mode" from the Galaxy S9+ DT confirms the
 * capability exists, but this trace only captured live AF-scan
 * activity, never a power-down/standby transition, so mode_standby's
 * exact value (0x40, copied from ak7375) is still an assumption.
 * ctrl_steps/ctrl_delay_us (this driver's own smooth-ramp timing)
 * are also unconfirmed - the real trace showed single direct writes
 * roughly 100ms apart (paced by the HAL's own AF algorithm), not the
 * driver's internal multi-step ramp, so those two fields may not
 * even be meaningfully exercised by how whyred's downstream actually
 * drives this chip.
 */
static const struct ak73xx_chipdef ak7374_cdef = {
	.reg_position = 0x0,		/* CONFIRMED */
	.reg_cont = 0x2,		/* CONFIRMED */
	.shift_pos = 4,			/* CONFIRMED - 12-bit, see banner */
	.mode_active = 0x0,		/* CONFIRMED */
	.mode_standby = 0x40,		/* still unconfirmed, see banner */
	.has_standby = true,		/* CONFIRMED via Galaxy S9+ DT
					 * (vendor_use_sleep_mode) */
	.focus_pos_max = 4095,		/* CONFIRMED - 12-bit, see banner */
	.focus_steps = 1,
	.ctrl_steps = 64,		/* still unconfirmed, see banner */
	.ctrl_delay_us = 1000,		/* still unconfirmed, see banner */
	.power_delay_us = 10000,	/* still unconfirmed */
};

static const char * const ak7375_supply_names[] = {
	"vdd",
	"vio",
};

/* ak7375 device structure */
struct ak7375_device {
	const struct ak73xx_chipdef *cdef;
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ak7375_supply_names)];
	bool active;
};

static inline struct ak7375_device *to_ak7375_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7375_device, ctrls_vcm);
}

static inline struct ak7375_device *sd_to_ak7375_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7375_device, sd);
}

static int ak7375_i2c_write(struct ak7375_device *ak7375,
			     u8 addr, u16 data, u8 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7375->sd);
	u8 buf[3];
	int ret;

	if (size != 1 && size != 2)
		return -EINVAL;

	buf[0] = addr;
	buf[size] = data & 0xff;
	if (size == 2)
		buf[1] = (data >> 8) & 0xff;

	ret = i2c_master_send(client, (const char *)buf, size + 1);
	if (ret < 0)
		return ret;
	if (ret != size + 1)
		return -EIO;

	return 0;
}

static int ak7375_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ak7375_device *dev_vcm = to_ak7375_vcm(ctrl);
	const struct ak73xx_chipdef *cdef = dev_vcm->cdef;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return ak7375_i2c_write(dev_vcm, cdef->reg_position,
					 ctrl->val << cdef->shift_pos, 2);

	return -EINVAL;
}

static const struct v4l2_ctrl_ops ak7375_vcm_ctrl_ops = {
	.s_ctrl = ak7375_set_ctrl,
};

static int ak7375_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int ak7375_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops ak7375_int_ops = {
	.open = ak7375_open,
	.close = ak7375_close,
};

static const struct v4l2_subdev_ops ak7375_ops = { };

static void ak7375_subdev_cleanup(struct ak7375_device *ak7375_dev)
{
	v4l2_async_unregister_subdev(&ak7375_dev->sd);
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
}

static int ak7375_init_controls(struct ak7375_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &ak7375_vcm_ctrl_ops;
	const struct ak73xx_chipdef *cdef = dev_vcm->cdef;

	v4l2_ctrl_handler_init(hdl, 1);

	dev_vcm->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
					    0, cdef->focus_pos_max,
					    cdef->focus_steps, 0);
	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);

	dev_vcm->sd.ctrl_handler = hdl;
	return hdl->error;
}

static int ak7375_probe(struct i2c_client *client)
{
	struct ak7375_device *ak7375_dev;
	int ret;
	unsigned int i;

	ak7375_dev = devm_kzalloc(&client->dev, sizeof(*ak7375_dev),
				  GFP_KERNEL);
	if (!ak7375_dev)
		return -ENOMEM;

	ak7375_dev->cdef = device_get_match_data(&client->dev);

	for (i = 0; i < ARRAY_SIZE(ak7375_supply_names); i++)
		ak7375_dev->supplies[i].supply = ak7375_supply_names[i];

	ret = devm_regulator_bulk_get(&client->dev,
				       ARRAY_SIZE(ak7375_supply_names),
				       ak7375_dev->supplies);
	if (ret) {
		dev_err_probe(&client->dev, ret, "Failed to get regulators\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&ak7375_dev->sd, client, &ak7375_ops);
	ak7375_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7375_dev->sd.internal_ops = &ak7375_int_ops;
	ak7375_dev->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = ak7375_init_controls(ak7375_dev);
	if (ret)
		goto err_cleanup;

	ret = media_entity_pads_init(&ak7375_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	ret = v4l2_async_register_subdev(&ak7375_dev->sd);
	if (ret < 0)
		goto err_cleanup;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_cleanup:
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
	return ret;
}

static void ak7375_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);

	ak7375_subdev_cleanup(ak7375_dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static int __maybe_unused ak7375_vcm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	const struct ak73xx_chipdef *cdef = ak7375_dev->cdef;
	int ret, val;

	if (!ak7375_dev->active)
		return 0;

	for (val = ak7375_dev->focus->val & ~(cdef->ctrl_steps - 1);
	     val >= 0; val -= cdef->ctrl_steps) {
		ret = ak7375_i2c_write(ak7375_dev, cdef->reg_position,
					val << cdef->shift_pos, 2);
		if (ret)
			dev_err_once(dev, "%s I2C failure: %d\n",
				     __func__, ret);
		usleep_range(cdef->ctrl_delay_us, cdef->ctrl_delay_us + 10);
	}

	if (cdef->has_standby) {
		ret = ak7375_i2c_write(ak7375_dev, cdef->reg_cont,
					cdef->mode_standby, 1);
		if (ret)
			dev_err(dev, "%s I2C failure: %d\n", __func__, ret);
	}

	ret = regulator_bulk_disable(ARRAY_SIZE(ak7375_supply_names),
				      ak7375_dev->supplies);
	if (ret)
		return ret;

	ak7375_dev->active = false;
	return 0;
}

static int __maybe_unused ak7375_vcm_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	const struct ak73xx_chipdef *cdef = ak7375_dev->cdef;
	int ret, val;

	if (ak7375_dev->active)
		return 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(ak7375_supply_names),
				     ak7375_dev->supplies);
	if (ret)
		return ret;

	usleep_range(cdef->power_delay_us, cdef->power_delay_us + 500);

	ret = ak7375_i2c_write(ak7375_dev, cdef->reg_cont,
				cdef->mode_active, 1);
	if (ret) {
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);
		return ret;
	}

	for (val = ak7375_dev->focus->val % cdef->ctrl_steps;
	     val <= ak7375_dev->focus->val;
	     val += cdef->ctrl_steps) {
		ret = ak7375_i2c_write(ak7375_dev, cdef->reg_position,
					val << cdef->shift_pos, 2);
		if (ret)
			dev_err_ratelimited(dev, "%s I2C failure: %d\n",
					     __func__, ret);
		usleep_range(cdef->ctrl_delay_us, cdef->ctrl_delay_us + 10);
	}

	ak7375_dev->active = true;
	return 0;
}

static const struct of_device_id ak7375_of_table[] = {
	{ .compatible = "asahi-kasei,ak7345", .data = &ak7345_cdef, },
	{ .compatible = "asahi-kasei,ak7374", .data = &ak7374_cdef, },
	{ .compatible = "asahi-kasei,ak7375", .data = &ak7375_cdef, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ak7375_of_table);

static const struct dev_pm_ops ak7375_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume)
	SET_RUNTIME_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume, NULL)
};

static struct i2c_driver ak7375_i2c_driver = {
	.driver = {
		.name = "ak7375",
		.pm = &ak7375_pm_ops,
		.of_match_table = ak7375_of_table,
	},
	.probe = ak7375_probe,
	.remove = ak7375_remove,
};
module_i2c_driver(ak7375_i2c_driver);

MODULE_AUTHOR("Kulesha Evgeniy <voodop@gmail.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_DESCRIPTION("AK7375 VCM driver");
MODULE_LICENSE("GPL v2");
