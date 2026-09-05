// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2016 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/rmi.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "rmi_driver.h"

#define BUFFER_SIZE_INCREMENT 32

/**
 * struct rmi_i2c_xport - stores information for i2c communication
 *
 * @xport: The transport interface structure
 * @client: The I2C client device structure
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 *
 * @tx_buf: Buffer used for transmitting data to the sensor over i2c.
 * @tx_buf_size: Size of the buffer
 *
 * @supplies: Array of voltage regulators
 * @reset_gpio: Reference to the reset GPIO
 * @startup_delay: Milliseconds to pause after powering up the regulators
 * @reset_delay: Milliseconds to pause after resetting the device
 */
struct rmi_i2c_xport {
	struct rmi_transport_dev xport;
	struct i2c_client *client;

	struct mutex page_mutex;
	int page;

	u8 *tx_buf;
	size_t tx_buf_size;

	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	u32 startup_delay;
	u32 reset_delay;
};

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_dev struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the transport
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_i2c_xport *rmi_i2c, u8 page)
{
	struct i2c_client *client = rmi_i2c->client;
	u8 txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}

	rmi_i2c->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_transport_dev *xport, u16 addr,
			       const void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	size_t tx_size = len + 1;
	int retval;

	mutex_lock(&rmi_i2c->page_mutex);

	if (!rmi_i2c->tx_buf || rmi_i2c->tx_buf_size < tx_size) {
		if (rmi_i2c->tx_buf)
			devm_kfree(&client->dev, rmi_i2c->tx_buf);
		rmi_i2c->tx_buf_size = tx_size + BUFFER_SIZE_INCREMENT;
		rmi_i2c->tx_buf = devm_kzalloc(&client->dev,
					       rmi_i2c->tx_buf_size,
					       GFP_KERNEL);
		if (!rmi_i2c->tx_buf) {
			rmi_i2c->tx_buf_size = 0;
			retval = -ENOMEM;
			goto exit;
		}
	}

	rmi_i2c->tx_buf[0] = addr & 0xff;
	memcpy(rmi_i2c->tx_buf + 1, buf, len);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_master_send(client, rmi_i2c->tx_buf, tx_size);
	if (retval == tx_size)
		retval = 0;
	else if (retval >= 0)
		retval = -EIO;

exit:
	rmi_dbg(RMI_DEBUG_XPORT, &client->dev,
		"write %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static int rmi_i2c_read_block(struct rmi_transport_dev *xport, u16 addr,
			      void *buf, size_t len)
{
	struct rmi_i2c_xport *rmi_i2c =
		container_of(xport, struct rmi_i2c_xport, xport);
	struct i2c_client *client = rmi_i2c->client;
	u8 addr_offset = addr & 0xff;
	int retval;
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.len	= sizeof(addr_offset),
			.buf	= &addr_offset,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		},
	};

	mutex_lock(&rmi_i2c->page_mutex);

	if (RMI_I2C_PAGE(addr) != rmi_i2c->page) {
		retval = rmi_set_page(rmi_i2c, RMI_I2C_PAGE(addr));
		if (retval)
			goto exit;
	}

	retval = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (retval == ARRAY_SIZE(msgs))
		retval = 0; /* success */
	else if (retval >= 0)
		retval = -EIO;

exit:
	rmi_dbg(RMI_DEBUG_XPORT, &client->dev,
		"read %zd bytes at %#06x: %d (%*ph)\n",
		len, addr, retval, (int)len, buf);

	mutex_unlock(&rmi_i2c->page_mutex);
	return retval;
}

static const struct rmi_transport_ops rmi_i2c_ops = {
	.write_block	= rmi_i2c_write_block,
	.read_block	= rmi_i2c_read_block,
};

#ifdef CONFIG_OF
static const struct of_device_id rmi_i2c_of_match[] = {
	{ .compatible = "syna,rmi4-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, rmi_i2c_of_match);
#endif

static void rmi_i2c_regulator_bulk_disable(void *data)
{
	struct rmi_i2c_xport *rmi_i2c = data;

	regulator_bulk_disable(ARRAY_SIZE(rmi_i2c->supplies),
			       rmi_i2c->supplies);
}

static void rmi_i2c_unregister_transport(void *data)
{
	struct rmi_i2c_xport *rmi_i2c = data;

	rmi_unregister_transport_device(&rmi_i2c->xport);
}

#define RMI_I2C_PROBE_RETRIES		5
#define RMI_I2C_PROBE_RETRY_DELAY_MS	100

/*
 * rmi_i2c_reset_toggle - pulse the reset GPIO once, honouring the DT-provided
 * (or default) reset delay.
 */
static void rmi_i2c_reset_toggle(struct rmi_i2c_xport *rmi_i2c)
{
	if (!rmi_i2c->reset_gpio)
		return;

	gpiod_set_value_cansleep(rmi_i2c->reset_gpio, 1);
	usleep_range(10000, 20000);
	gpiod_set_value_cansleep(rmi_i2c->reset_gpio, 0);
	msleep(rmi_i2c->reset_delay ?: DEFAULT_RESET_DELAY_MS);
}

/*
 * rmi_i2c_probe_set_page - like rmi_set_page(), but retries with a fresh
 * reset pulse on failure instead of giving up on the first ENXIO/EIO.
 *
 * Some whyred units come up flaky and don't answer i2c on the first
 * page-select after power-on; a plain fixed startup delay isn't always
 * enough. Toggling reset and trying again a few times fixes probe on
 * those units without having to blindly stretch the boot-time delay for
 * everyone.
 */
static int rmi_i2c_probe_set_page(struct rmi_i2c_xport *rmi_i2c)
{
	struct i2c_client *client = rmi_i2c->client;
	int error;
	int i;

	for (i = 0; i < RMI_I2C_PROBE_RETRIES; i++) {
		error = rmi_set_page(rmi_i2c, 0);
		if (!error)
			return 0;

		dev_warn(&client->dev,
			 "probe page-select attempt %d/%d failed (%d)%s\n",
			 i + 1, RMI_I2C_PROBE_RETRIES, error,
			 rmi_i2c->reset_gpio ? ", resetting" : "");

		if (i + 1 < RMI_I2C_PROBE_RETRIES) {
			rmi_i2c_reset_toggle(rmi_i2c);
			msleep(RMI_I2C_PROBE_RETRY_DELAY_MS);
		}
	}

	return error;
}

static int rmi_i2c_probe(struct i2c_client *client)
{
	struct rmi_device_platform_data *pdata;
	struct rmi_device_platform_data *client_pdata =
					dev_get_platdata(&client->dev);
	struct rmi_i2c_xport *rmi_i2c;
	int error;

	rmi_i2c = devm_kzalloc(&client->dev, sizeof(struct rmi_i2c_xport),
				GFP_KERNEL);
	if (!rmi_i2c)
		return -ENOMEM;

	pdata = &rmi_i2c->xport.pdata;

	if (!client->dev.of_node && client_pdata)
		*pdata = *client_pdata;

	pdata->irq = client->irq;

	rmi_dbg(RMI_DEBUG_XPORT, &client->dev, "Probing %s.\n",
			dev_name(&client->dev));

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"adapter does not support required functionality\n");
		return -ENODEV;
	}

	rmi_i2c->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(rmi_i2c->reset_gpio)) {
		error = PTR_ERR(rmi_i2c->reset_gpio);
		dev_err(&client->dev, "failed to get reset GPIO: %d\n", error);
		return error;
	}
	gpiod_set_consumer_name(rmi_i2c->reset_gpio, "rmi4 reset");

	rmi_i2c->supplies[0].supply = "vdd";
	rmi_i2c->supplies[1].supply = "vio";
	error = devm_regulator_bulk_get(&client->dev,
					 ARRAY_SIZE(rmi_i2c->supplies),
					 rmi_i2c->supplies);
	if (error < 0)
		return error;

	error = regulator_bulk_enable(ARRAY_SIZE(rmi_i2c->supplies),
				       rmi_i2c->supplies);
	if (error < 0)
		return error;

	error = devm_add_action_or_reset(&client->dev,
					  rmi_i2c_regulator_bulk_disable,
					  rmi_i2c);
	if (error)
		return error;

	of_property_read_u32(client->dev.of_node, "syna,startup-delay-ms",
			     &rmi_i2c->startup_delay);

	msleep(rmi_i2c->startup_delay);

	of_property_read_u32(client->dev.of_node, "syna,reset-delay-ms",
			     &rmi_i2c->reset_delay);

	rmi_i2c->client = client;
	mutex_init(&rmi_i2c->page_mutex);

	rmi_i2c_reset_toggle(rmi_i2c);

	rmi_i2c->xport.dev = &client->dev;
	rmi_i2c->xport.proto_name = "i2c";
	rmi_i2c->xport.ops = &rmi_i2c_ops;

	i2c_set_clientdata(client, rmi_i2c);

	/*
	 * Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 * Retry with a reset pulse in between attempts, since some units
	 * don't come up reliably on the first try after power-on.
	 */
	error = rmi_i2c_probe_set_page(rmi_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to set page select to 0 after %d attempts\n",
			RMI_I2C_PROBE_RETRIES);
		return error;
	}

	dev_info(&client->dev, "registering I2C-connected sensor\n");

	error = rmi_register_transport_device(&rmi_i2c->xport);
	if (error) {
		dev_err(&client->dev, "failed to register sensor: %d\n", error);
		return error;
	}

	error = devm_add_action_or_reset(&client->dev,
					  rmi_i2c_unregister_transport,
					  rmi_i2c);
	if (error)
		return error;

	return 0;
}

static int rmi_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	ret = rmi_driver_suspend(rmi_i2c->xport.rmi_dev, true);
	if (ret)
		dev_warn(dev, "Failed to write sleep mode, suspending anyway: %d\n",
			 ret);

	regulator_bulk_disable(ARRAY_SIZE(rmi_i2c->supplies),
			       rmi_i2c->supplies);

	/*
	 * We're cutting power right after this regardless, so a failed
	 * best-effort sleep-mode write shouldn't abort system suspend for
	 * the whole device. Returning an error here made PM core treat
	 * this device's suspend as failed and unwind the entire suspend
	 * cycle, which just spun in a retry loop without ever sleeping.
	 */
	return 0;
}

#define RMI_I2C_RESUME_RETRIES		5
#define RMI_I2C_RESUME_RETRY_DELAY_MS	100

static int rmi_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;
	int i;

	ret = regulator_bulk_enable(ARRAY_SIZE(rmi_i2c->supplies),
				    rmi_i2c->supplies);
	if (ret)
		return ret;

	msleep(rmi_i2c->startup_delay);

	for (i = 0; i < RMI_I2C_RESUME_RETRIES; i++) {
		ret = rmi_driver_resume(rmi_i2c->xport.rmi_dev, true);
		if (!ret)
			return 0;

		dev_warn(dev, "resume attempt %d/%d failed: %d%s\n",
			 i + 1, RMI_I2C_RESUME_RETRIES, ret,
			 i + 1 < RMI_I2C_RESUME_RETRIES ? ", retrying" : "");

		if (i + 1 < RMI_I2C_RESUME_RETRIES)
			msleep(RMI_I2C_RESUME_RETRY_DELAY_MS);
	}

	/*
	 * Best-effort, same reasoning as rmi_i2c_suspend(): don't fail
	 * system resume over a still-unresponsive touch controller after
	 * RMI_I2C_RESUME_RETRIES tries. There's no useful recovery the PM
	 * core can do about it, and returning an error here just gets the
	 * failure logged a second time.
	 */
	return 0;
}

static int rmi_i2c_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	ret = rmi_driver_suspend(rmi_i2c->xport.rmi_dev, false);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	regulator_bulk_disable(ARRAY_SIZE(rmi_i2c->supplies),
			       rmi_i2c->supplies);

	return 0;
}

static int rmi_i2c_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rmi_i2c_xport *rmi_i2c = i2c_get_clientdata(client);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(rmi_i2c->supplies),
				    rmi_i2c->supplies);
	if (ret)
		return ret;

	msleep(rmi_i2c->startup_delay);

	ret = rmi_driver_resume(rmi_i2c->xport.rmi_dev, false);
	if (ret)
		dev_warn(dev, "Failed to resume device: %d\n", ret);

	return 0;
}

static const struct dev_pm_ops rmi_i2c_pm = {
	SYSTEM_SLEEP_PM_OPS(rmi_i2c_suspend, rmi_i2c_resume)
	RUNTIME_PM_OPS(rmi_i2c_runtime_suspend, rmi_i2c_runtime_resume, NULL)
};

static const struct i2c_device_id rmi_id[] = {
	{ "rmi4_i2c" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.name	= "rmi4_i2c",
		.pm	= pm_ptr(&rmi_i2c_pm),
		.of_match_table = of_match_ptr(rmi_i2c_of_match),
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
};

module_i2c_driver(rmi_i2c_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_AUTHOR("Andrew Duggan <aduggan@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
