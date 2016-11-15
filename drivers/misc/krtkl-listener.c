/**
 *  Copyright (c) 2016 krtkl inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/of.h>

#define KRTKL_LSTNR_DEVICE_NAME "krtkl-listener"

/* Request and ready bits are the same bit in the same register */
#define KRTKL_LSTNR_CMD_REQ_VAL  0x0001
#define KRTKL_LSTNR_CMD_RDY_VAL  0x0001

/* Waiting for data limits */
#define KRTKL_LSTNR_MAX_RD_TIME  500
#define KRTKL_LSTNR_RD_INVERVAL  100

struct krtkl_lstnr_regs {
	union {
		uint32_t slvreg0;
		uint32_t ctrl;
		uint32_t status;
	};
	union {
		uint32_t slvreg1;
		uint32_t data_size;
	};
	union {
		uint32_t slvreg2;
		uint32_t fifo_cnt;
	};
	union {
		uint32_t slvreg3;
		uint32_t max_read_size;
	};
};

struct krtkl_lstnr {
	dev_t char_dev_t;
	int device_open;
	struct class *char_class;
	struct cdev *char_cdev;
	struct mutex lock;

	struct krtkl_lstnr_regs __iomem *regs;
	void __iomem *data;
};

static struct krtkl_lstnr krtkl_lstnr;

static int krtkl_lstnr_open(struct inode *inode, struct file *file)
{
	if (krtkl_lstnr.device_open)
		return -EBUSY;

	mutex_lock(&krtkl_lstnr.lock);
	krtkl_lstnr.device_open++;
	mutex_unlock(&krtkl_lstnr.lock);

	try_module_get(THIS_MODULE);

	return 0;
}

static int krtkl_lstnr_release(struct inode *inode, struct file *file)
{
	mutex_lock(&krtkl_lstnr.lock);
	krtkl_lstnr.device_open--;
	mutex_unlock(&krtkl_lstnr.lock);

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t krtkl_lstnr_read(struct file *file,
				char __user *buffer,
				size_t length,
				loff_t *offset)
{
	uint32_t pos;
	uint32_t size;
	uint32_t total_slept;
	uint8_t  cp_padding;

	/* Set reading offset */
	pos = *offset;

	/* Check if request is aligned to 4B */
	if (pos % 4 || length % 4)
		return -EFAULT;

	/* Check if it wants to read anything */
	if (!length)
		return 0;

	/* Check if there is enough data in the fifo if call is non-blocking */
	if (file->f_flags & O_NONBLOCK) {
		if (krtkl_lstnr.regs->fifo_cnt < length)
			return -EWOULDBLOCK;

		if (length > krtkl_lstnr.regs->max_read_size)
			return -EINVAL;
	}

	/* Perform request and BRAM read if needed */
	if (pos == 0) {
		/* Send FIFO to BRAM transfer request */
		krtkl_lstnr.regs->ctrl |= KRTKL_LSTNR_CMD_REQ_VAL;

		/* Wait for the transfer to finish */
		total_slept = 0;
		while (!(krtkl_lstnr.regs->status & KRTKL_LSTNR_CMD_RDY_VAL)) {
			if (total_slept <= KRTKL_LSTNR_MAX_RD_TIME) {
				msleep(KRTKL_LSTNR_RD_INVERVAL);
				total_slept += KRTKL_LSTNR_RD_INVERVAL;
			} else {
				return -ETIMEDOUT;
			}
		}
	}

	/* Check how much data there is yet to read */
	size = krtkl_lstnr.regs->data_size;

	if (size == 0)
		return -ENODATA;

	/* Check if anything is left to be read */
	if (pos >= size)
		return 0;

	/* Check read limits */
	if ((pos + length) >= size)
		length = (size - pos);

	/* Make sure it is aligned to 4B */
	cp_padding = 0;
	while (length % 4) {
		length++;
		cp_padding++;
	}

	/* Copy the data to userspace */
	if (copy_to_user(buffer, krtkl_lstnr.data + pos, length))
		return -EFAULT;

	/* Keep the offset aligned to 4B in case it's the last iteration */
	pos += length;
	*offset = pos;

	/* But return the number bytes that are actually valid */
	return length - cp_padding;
}

static loff_t krtkl_lstnr_llseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;

	mutex_lock(&file_inode(file)->i_mutex);
	switch (orig) {
	case SEEK_CUR:
		offset += file->f_pos;
	case SEEK_SET:
		/* To avoid userland mistaking f_pos=-9 as -EBADF=-9 */
		if (IS_ERR_VALUE((unsigned long long)offset)) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&file_inode(file)->i_mutex);

	return ret;
}

const struct file_operations krtkl_lstnr_fops = {
	.open    = krtkl_lstnr_open,
	.read    = krtkl_lstnr_read,
	.release = krtkl_lstnr_release,
	.llseek  = krtkl_lstnr_llseek,
};

static int krtkl_lstnr_probe(struct platform_device *pdev)
{
	struct device_node *devnode = pdev->dev.of_node;
	int result;
	uint32_t phys_addr;
	uint32_t phys_len;

	/* Map AXI control interface */
	result = of_property_read_u32(devnode,
				      "krtkl,cmd_if_addr",
				      &phys_addr);
	if (result) {
		dev_err(&pdev->dev, "Could not get cmd interface address\n");
		return -ENODEV;
	}
	if (!devm_request_mem_region(&pdev->dev,
				     phys_addr,
				     sizeof(struct krtkl_lstnr_regs),
				     "krtkl,regs")) {
		dev_err(&pdev->dev, "Could not request cmd mem region\n");
		return -ENXIO;
	}
	krtkl_lstnr.regs = devm_ioremap(&pdev->dev,
					phys_addr,
					sizeof(struct krtkl_lstnr_regs));

	if (IS_ERR(krtkl_lstnr.regs)) {
		dev_err(&pdev->dev, "Could not map registers\n");
		return -ENXIO;
	}

	/* Map AXI data interface */
	phys_len = 0x10000; /* FIXME */
	result = of_property_read_u32(devnode,
				      "krtkl,data_if_addr",
				      &phys_addr);
	if (result) {
		dev_err(&pdev->dev, "Could not get data interface address\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev,
					 phys_addr,
					 phys_len,
					 "krtkl,data")) {
		dev_err(&pdev->dev, "Could not request data mem region\n");
		return -ENXIO;
	}

	krtkl_lstnr.data = devm_ioremap(&pdev->dev, phys_addr, phys_len);

	if (IS_ERR(krtkl_lstnr.data)) {
		dev_err(&pdev->dev, "Could not map data interface\n");
		return -ENODEV;
	}

	/* Init chardev helpers */
	krtkl_lstnr.device_open = 0;
	mutex_init(&krtkl_lstnr.lock);

	/* Create char device node */
	result = alloc_chrdev_region(&krtkl_lstnr.char_dev_t,
				     0, 1,
				     KRTKL_LSTNR_DEVICE_NAME "-reg");

	if (result < 0) {
		dev_err(&pdev->dev, "Failed to alloc chrdev region\n");
		goto fail_alloc_chrdev_region;
	}
	krtkl_lstnr.char_cdev = cdev_alloc();
	if (!krtkl_lstnr.char_cdev) {
		result = -ENOMEM;
		dev_err(&pdev->dev, "Failed to alloc cdev\n");
		goto fail_alloc_cdev;
	}
	cdev_init(krtkl_lstnr.char_cdev, &krtkl_lstnr_fops);
	result = cdev_add(krtkl_lstnr.char_cdev,
			  krtkl_lstnr.char_dev_t, 1);
	if (result < 0) {
		dev_err(&pdev->dev, "Failed to add cdev\n");
		goto fail_add_cdev;
	}

	krtkl_lstnr.char_class =
		class_create(THIS_MODULE,
			     KRTKL_LSTNR_DEVICE_NAME "-class");

	if (!krtkl_lstnr.char_class) {
		result = -EEXIST;
		dev_err(&pdev->dev, "Failed to create cdev class\n");
		goto fail_create_class;
	}

	if (!device_create(krtkl_lstnr.char_class, NULL,
			  krtkl_lstnr.char_dev_t, NULL,
			  KRTKL_LSTNR_DEVICE_NAME "%d",
			  MINOR(krtkl_lstnr.char_dev_t))) {

		result = -EINVAL;
		dev_err(&pdev->dev, "Failed to create device node\n");
		goto fail_create_device;
	}
	return 0;

fail_create_device:
	class_destroy(krtkl_lstnr.char_class);
fail_create_class:
	cdev_del(krtkl_lstnr.char_cdev);
fail_add_cdev:
fail_alloc_cdev:
	unregister_chrdev_region(krtkl_lstnr.char_dev_t, 1);
fail_alloc_chrdev_region:
	return result;
}

static int krtkl_lstnr_remove(struct platform_device *pdev)
{
	device_destroy(krtkl_lstnr.char_class,
		       krtkl_lstnr.char_dev_t);
	class_destroy(krtkl_lstnr.char_class);
	cdev_del(krtkl_lstnr.char_cdev);
	unregister_chrdev_region(krtkl_lstnr.char_dev_t, 1);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id krtkl_lstnr_of_match[] = {
	{
		.compatible = "krtkl,lstnr",
	},
	{}
};
MODULE_DEVICE_TABLE(of, krtkl_lstnr_of_match);

static struct platform_driver krtkl_lstnr_platform_driver = {
	.probe  = krtkl_lstnr_probe,
	.remove = krtkl_lstnr_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = KRTKL_LSTNR_DEVICE_NAME,
		.of_match_table = krtkl_lstnr_of_match,
	},
};

static int __init krtkl_lstnr_init(void)
{
	return platform_driver_register(&krtkl_lstnr_platform_driver);
}

static void __exit krtkl_lstnr_exit(void)
{
	platform_driver_unregister(&krtkl_lstnr_platform_driver);
}

module_init(krtkl_lstnr_init);
module_exit(krtkl_lstnr_exit);

MODULE_AUTHOR("Tomasz Gorochowik");
MODULE_DESCRIPTION("krtkl Listener Driver");
MODULE_LICENSE("GPL v2");
