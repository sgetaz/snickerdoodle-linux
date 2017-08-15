/*
 * Driver for SMSC USB2514 USB 2.0 hub controller driver
 *
 * Copyright (c) 2012-2013 Dongjin Kim (tobetter@xxxxxxxxx)
 * Copyright (c) 2016 Nathael Pajani (nathael.pajani@xxxxxxx)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/usb2514.h>
#include <linux/regmap.h>


#define USB2514_IDS      0x00

#define USB2514_CFG      0x06
#define USB2514_CFG1     0x06
#define USB2514_CFG2     0x07
#define USB2514_CFG3     0x08

#define USB2514_NRD_PDS  0x09
#define USB2514_NRD      0x09
#define USB2514_PDS      0x0a
#define USB2514_PDB      0x0b

#define USB2514_POWER    0x0c

#define USB2514_DESCRIPTORS  0x10

#define USB2514_BC_EN    0xd0

#define USB2514_PM12     0xfb
#define USB2514_PM34     0xfc

#define USB2514_RESET    0xff

struct usb2514 {
	enum usb2514_mode mode;
	struct regmap* regmap;
	struct device* dev;
	struct device* i2c;
	struct device* platform;
	uint8_t port_off_mask;
	int gpio_reset;
};

struct usb2514* usb_hub = NULL;

static int usb2514_reset(struct usb2514 *hub, int state)
{
	if (gpio_is_valid(hub->gpio_reset))
		gpio_set_value_cansleep(hub->gpio_reset, state);

	/* Wait T_HUBINIT == 4ms for hub logic to stabilize */
	if (state)
		usleep_range(4000, 10000);

	return 0;
}

static int usb2514_i2c_reg_block_read(struct usb2514 *hub, int offset, uint8_t* buf, int size)
{
	uint8_t tmp_buf[33]; /* Max read is 32 registers at once */
	int cur_size = 0, tmp_size = 0;
	int err = 0;

	while (cur_size < size) {
		tmp_size = ((size - cur_size) % 32);
		err = regmap_bulk_read(hub->regmap, (offset + cur_size), tmp_buf, (tmp_size + 1));
		if (err < 0) {
			return err;
		}
		memcpy((buf + cur_size), (tmp_buf + 1), tmp_size);
		cur_size += tmp_size;
	}
	return cur_size;
}
static int usb2514_i2c_reg_block_write(struct usb2514 *hub, int offset, uint8_t* buf, int size)
{
	uint8_t tmp_buf[33]; /* Max write is 32 registers at once */
	int cur_size = 0, tmp_size = 0;
	int err = 0;

	while (cur_size < size) {
		tmp_size = ((size - cur_size) % 32);
		memcpy((tmp_buf + 1), (buf + cur_size), tmp_size);
		tmp_buf[0] = tmp_size;
		err = regmap_raw_write(hub->regmap, (offset + cur_size), tmp_buf, (tmp_size + 1));
		if (err < 0) {
			return err;
		}
		cur_size += tmp_size;
	}
	return cur_size;
}

struct usb2514_config_part {
	uint8_t offset;
	uint8_t len;
	uint8_t data[64];
};

struct usb2514_config_part usb2514_config[] = {
	{ USB2514_IDS, 6, { 0x24, 0x04, 0x14, 0x25, 0xB3, 0x0B, }, }, /* Dev ID and such */
	{ USB2514_CFG, 3, { 0x9B, 0x20, 0x02, }, }, /* Config (CFG[1..3]) */
/*
	{ USB2514_NRD_PDS, 3, { 0x00, 0x00, 0x00, }, },
 */
	{ USB2514_POWER, 5, { 0x01, 0x32, 0x01, 0x32, 0x32, }, }, /* Config Power */
/*
	{ USB2514_DESCRIPTORS, 5, { 0, }, },
	{ USB2514_MANUF_DESC, 62, { 0, }, },
	{ USB2514_PROD_DESC, 62, { 0, }, },
	{ USB2514_SERIAL_DESC, 62, { 0, }, },
 */
/*
	{ USB2514_BC_EN, 1, { 0x00, }, },
 */
	{ USB2514_PM12, 1, { 0x21, }, },
	{ USB2514_PM34, 1, { 0x43, }, },
	{ 0, 0, {0}, },
};

static int usb2514_connect(struct usb2514 *hub)
{
	struct device *dev = hub->dev;
	int err = 0, i = 0;
	uint8_t tmp = 0;

	usb2514_reset(hub, 1);

	if (hub->regmap) {
		dev_info(dev, "Updating HUB configuration\n");

		for (i = 0; usb2514_config[i].len != 0; i++) {
			err = usb2514_i2c_reg_block_write(hub, usb2514_config[i].offset,
										usb2514_config[i].data, usb2514_config[i].len);
			if (err < 0) {
				dev_err(dev, "Initial config failed on register group starting at 0x%02x (%d)\n",
								usb2514_config[i].offset, err);
				return err;
			}
		}

		/* PDS : Set the ports which are disabled in self-powered mode. */
		if (hub->port_off_mask) {
			err = usb2514_i2c_reg_block_write(hub, USB2514_PDS, &hub->port_off_mask, 1);
			if (err < 0) {
				dev_err(dev, "PDS failed (%d)\n", err);
				return err;
			}
		}

		/* And set config as done */
		tmp = 0x01;
		err = usb2514_i2c_reg_block_write(hub, USB2514_RESET, &tmp, 1);
		if (err < 0) {
			dev_err(dev, "Config validation failed (%d)\n", err);
			return err;
		}
	}

	hub->mode = USB2514_MODE_HUB;
	dev_info(dev, "switched to HUB mode\n");

	return 0;
}

static int usb2514_switch_mode(struct usb2514 *hub, enum usb2514_mode mode)
{
	struct device *dev = hub->dev;
	int err = 0;

	switch (mode) {
	case USB2514_MODE_HUB:
		err = usb2514_connect(hub);
		break;

	case USB2514_MODE_STANDBY:
		usb2514_reset(hub, 0);
		dev_info(dev, "switched to STANDBY mode\n");
		break;

	default:
		dev_err(dev, "unknown mode is requested\n");
		err = -EINVAL;
		break;
	}

	return err;
}

static const struct regmap_config usb2514_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = USB2514_RESET,
};

static int usb2514_probe(struct usb2514 *hub)
{
	struct device *dev = hub->platform;
	struct usb2514_platform_data *pdata = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	int err;
	u32 mode = USB2514_MODE_HUB;
	const u32 *property;
	int len;

	if (pdata) {
		hub->port_off_mask	= pdata->port_off_mask;
		hub->gpio_reset		= pdata->gpio_reset;
		hub->mode		= pdata->initial_mode;
	} else if (np) {
		property = of_get_property(np, "disabled-ports", &len);
		if (property && (len / sizeof(u32)) > 0) {
			int i;
			for (i = 0; i < len / sizeof(u32); i++) {
				u32 port = be32_to_cpu(property[i]);
				if ((1 <= port) && (port <= 3))
					hub->port_off_mask |= (1 << port);
			}
		}

		hub->gpio_reset = of_get_named_gpio(np, "reset-gpios", 0);
		if (hub->gpio_reset == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		of_property_read_u32(np, "initial-mode", &mode);
		hub->mode = mode;
	}

	if (hub->port_off_mask && !hub->regmap)
		dev_err(dev, "Ports disabled with no control interface\n");

	if (gpio_is_valid(hub->gpio_reset)) {
		err = devm_gpio_request_one(dev, hub->gpio_reset,
				GPIOF_OUT_INIT_LOW, "usb2514 reset");
		if (err) {
			dev_err(dev,
				"unable to request GPIO %d as reset pin (%d)\n",
				hub->gpio_reset, err);
			return err;
		}
	}

	err = usb2514_switch_mode(hub, hub->mode);
	if (err) {
		dev_err(dev, "unable to configure USB2514 hub.\n");
	} else {
		dev_info(dev, "%s: probed in %s mode\n", __func__,
			(hub->mode == USB2514_MODE_HUB) ? "hub" : "standby");
	}
	return err;
}

static int usb2514_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct usb2514 *hub;
	int err;

	if (usb_hub == NULL) {
		hub = devm_kzalloc(&i2c->dev, sizeof(struct usb2514), GFP_KERNEL);
		if (!hub)
			return -ENOMEM;
	} else {
		hub = usb_hub;
	}

	i2c_set_clientdata(i2c, hub);
	hub->regmap = devm_regmap_init_i2c(i2c, &usb2514_regmap_config);
	if (IS_ERR(hub->regmap)) {
		err = PTR_ERR(hub->regmap);
		dev_err(&i2c->dev, "Failed to initialise regmap: %d\n", err);
		return err;
	}

	if (usb_hub == NULL) {
		usb_hub = hub;
		hub->dev = &i2c->dev;
		hub->i2c = &i2c->dev;
	} else {
		usb_hub->i2c = &i2c->dev;
		return usb2514_probe(usb_hub);
	}
	/* Wait for platform probe to get all the requiered information */
	return 0;
}

static int usb2514_platform_probe(struct platform_device *pdev)
{
	struct usb2514 *hub;

	hub = devm_kzalloc(&pdev->dev, sizeof(struct usb2514), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	if (usb_hub == NULL) {
		usb_hub = hub;
		hub->dev = &pdev->dev;
		hub->platform = &pdev->dev;
	} else {
		usb_hub->platform = &pdev->dev;
		return usb2514_probe(usb_hub);
	}
	/* Wait for i2c probe to get all the requiered information */
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int usb2514_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct usb2514 *hub = i2c_get_clientdata(client);

	usb2514_switch_mode(hub, USB2514_MODE_STANDBY);

	return 0;
}

static int usb2514_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct usb2514 *hub = i2c_get_clientdata(client);

	usb2514_switch_mode(hub, hub->mode);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(usb2514_i2c_pm_ops, usb2514_i2c_suspend,
		usb2514_i2c_resume);

static const struct i2c_device_id usb2514_id[] = {
	{ USB2514_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, usb2514_id);

#ifdef CONFIG_OF
static const struct of_device_id usb2514_of_match[] = {
	{ .compatible = "smsc,usb2514", },
	{},
};
MODULE_DEVICE_TABLE(of, usb2514_of_match);
#endif

static struct i2c_driver usb2514_i2c_driver = {
	.driver = {
		.name = USB2514_I2C_NAME,
		.pm = &usb2514_i2c_pm_ops,
		.of_match_table = of_match_ptr(usb2514_of_match),
	},
	.probe		= usb2514_i2c_probe,
	.id_table	= usb2514_id,
};

static struct platform_driver usb2514_platform_driver = {
	.driver = {
		.name = USB2514_I2C_NAME,
		.of_match_table = of_match_ptr(usb2514_of_match),
	},
	.probe		= usb2514_platform_probe,
};

static int __init usb2514_init(void)
{
	int err;

	err = i2c_add_driver(&usb2514_i2c_driver);
	if (err != 0)
		pr_err("usb2514: Failed to register I2C driver: %d\n", err);

	err = platform_driver_register(&usb2514_platform_driver);
	if (err != 0)
		pr_err("usb2514: Failed to register platform driver: %d\n", err);

	return 0;
}
module_init(usb2514_init);

static void __exit usb2514_exit(void)
{
	platform_driver_unregister(&usb2514_platform_driver);
	i2c_del_driver(&usb2514_i2c_driver);
}
module_exit(usb2514_exit);

MODULE_AUTHOR("Nathael Pajani <nathael.pajani@xxxxxxx>");
MODULE_DESCRIPTION("USB2514 USB HUB driver");
MODULE_LICENSE("GPL");
