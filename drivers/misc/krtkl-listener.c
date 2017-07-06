/*
*  Copyright (C) 2016 krtkl inc.
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/completion.h>

#define KRTKL_LSTNR_DEVICE_NAME "krtkl-listener"

/* Request and ready bits are the same bit in the same register */
#define KRTKL_LSTNR_CMD_REQ_VAL  		(1<<0)
#define KRTKL_LSTNR_CMD_RDY_VAL  		(1<<0)

#define KRTKL_LSTNR_THRESH_INTERRUPT_ENABLE 	(1<<4)
#define KRTKL_LSTNR_FRAME_INTERRUPT_ENABLE 	(1<<5)
#define KRTKL_LSTNR_GLOBAL_INTERRUPT_ENABLE 	(1<<6)

#define KRTKL_LSTNR_FIFO_THRESHOLD_SHIFT		16

#define KRTKL_LSTNR_THRESH_INTERRUPT		(1<<16)
#define KRTKL_LSTNR_FRAME_INTERRUPT		(1<<17)

#define KRTKL_LSTNR_BRAM_SIZE    		0x800

/* Waiting for data definitions */
#define KRTKL_LSTNR_RD_INTERVAL  		10

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
		uint32_t interrupt_status;
	};
};

struct krtkl_lstnr {
	dev_t char_dev_t;
	int device_open;
	struct class *char_class;
	struct cdev *char_cdev;
	struct mutex lock;

	uint32_t bram_available;
	uint32_t bram_read;
	struct completion data_available;

	int irq;
	struct krtkl_lstnr_regs __iomem *regs;
	void __iomem *data;
};

static struct krtkl_lstnr krtkl_lstnr;

static void krtkl_lstnr_irq_enable(void)
{
	uint32_t reg;
	reg = krtkl_lstnr.regs->ctrl;
	reg &= ~(KRTKL_LSTNR_CMD_REQ_VAL);
        reg |= KRTKL_LSTNR_THRESH_INTERRUPT_ENABLE | 	\
		KRTKL_LSTNR_FRAME_INTERRUPT_ENABLE | 	\
		KRTKL_LSTNR_GLOBAL_INTERRUPT_ENABLE;
	krtkl_lstnr.regs->ctrl = reg;
	/* clear any pending interrupt */
	krtkl_lstnr.regs->interrupt_status |= KRTKL_LSTNR_THRESH_INTERRUPT | \
					KRTKL_LSTNR_FRAME_INTERRUPT;
}

static void krtkl_lstnr_irq_disable(void)
{
	uint32_t reg;
	reg = krtkl_lstnr.regs->ctrl;
	reg &= ~(KRTKL_LSTNR_CMD_REQ_VAL);
        reg &= ~(KRTKL_LSTNR_THRESH_INTERRUPT_ENABLE | 	\
		 KRTKL_LSTNR_FRAME_INTERRUPT_ENABLE | 	\
		 KRTKL_LSTNR_GLOBAL_INTERRUPT_ENABLE);
	krtkl_lstnr.regs->ctrl = reg;
}

static irqreturn_t krtkl_lstnr_irq_handler(int irq, void *data)
{
	struct krtkl_lstnr *priv = (struct krtkl_lstnr*) data;
	int interrupt_reg;

	interrupt_reg = priv->regs->interrupt_status;

	if(interrupt_reg & KRTKL_LSTNR_THRESH_INTERRUPT)
		priv->regs->interrupt_status |= KRTKL_LSTNR_THRESH_INTERRUPT;

	if(interrupt_reg & KRTKL_LSTNR_FRAME_INTERRUPT)
		priv->regs->interrupt_status |= KRTKL_LSTNR_FRAME_INTERRUPT;

	complete(&priv->data_available);

	return IRQ_HANDLED;
}

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
	uint32_t bram_left;
	uint32_t cp_size;
	int ret;

	/* Always ignore the offset */
	*offset = 0;

	/* Calculate leftovers from the previous transfer */
	bram_left = krtkl_lstnr.bram_available - krtkl_lstnr.bram_read;

	/* Copy what's available (at least 4 bytes) */
	if (bram_left > 0) {
		cp_size = (bram_left >= length) ? length : bram_left;

		/* Copy the data to userspace */
		if (copy_to_user(buffer,
				 krtkl_lstnr.data + krtkl_lstnr.bram_read,
				 cp_size))
			return -EFAULT;

		krtkl_lstnr.bram_read += cp_size;

		/* Return success if copied anything */
		return cp_size;
	}

	/* New request is needed - check if new data is available */
	if (krtkl_lstnr.regs->fifo_cnt < 4) {
		/* Fail if there is no new data and the read is non-blocking */
		if (file->f_flags & O_NONBLOCK)
			return 0;

		/* Otherwise wait for new data (at least 4 bytes)  */
		krtkl_lstnr_irq_enable();
		ret = wait_for_completion_killable(&krtkl_lstnr.data_available);
		krtkl_lstnr_irq_disable();

		/* ir we were interrupted, return 0 */
		if (ret) return 0;
	}

	/* Trigger a FIFO->BRAM transfer request */
	krtkl_lstnr.regs->ctrl |= KRTKL_LSTNR_CMD_REQ_VAL;

	/* Wait for the ready bit.
	   This shold not take long, so we can simple poll for the bit
	   instead of implementing it as an interrupt */
	while (!(krtkl_lstnr.regs->status & KRTKL_LSTNR_CMD_RDY_VAL))
		msleep(KRTKL_LSTNR_RD_INTERVAL);
	krtkl_lstnr.bram_available = krtkl_lstnr.regs->data_size;
	krtkl_lstnr.bram_read = 0;

	bram_left = krtkl_lstnr.bram_available;

	cp_size = (bram_left >= length) ? length : bram_left;
	/* Align to 4 bytes */
	while (cp_size % 4)
		cp_size--;

	krtkl_lstnr.bram_read += cp_size;

	/* Copy the data to user */
	if (copy_to_user(buffer, krtkl_lstnr.data, cp_size))
		return -EFAULT;

	return cp_size;
}

const struct file_operations krtkl_lstnr_fops = {
	.open    = krtkl_lstnr_open,
	.read    = krtkl_lstnr_read,
	.release = krtkl_lstnr_release,
};

static int krtkl_lstnr_probe(struct platform_device *pdev)
{
	struct device_node *devnode = pdev->dev.of_node;
	int result;
	int err;
	uint32_t reg;
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
	phys_len = KRTKL_LSTNR_BRAM_SIZE;
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

	/* Get the interrupt */
	krtkl_lstnr.irq = irq_of_parse_and_map(devnode, 0);

	err = request_irq(krtkl_lstnr.irq, krtkl_lstnr_irq_handler, IRQF_SHARED,
				"krtkl-listener", &krtkl_lstnr);

	if (err) {
		dev_err(&pdev->dev, "unable to request IRQ %d \n", krtkl_lstnr.irq);
		return err;
	}

	init_completion(&krtkl_lstnr.data_available);

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

	/* Set the default threshold*/
	reg = krtkl_lstnr.regs->ctrl;
	reg &= ~(KRTKL_LSTNR_CMD_REQ_VAL);
	reg |= (KRTKL_LSTNR_BRAM_SIZE-1) << KRTKL_LSTNR_FIFO_THRESHOLD_SHIFT;
	krtkl_lstnr.regs->ctrl = reg;

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
MODULE_DESCRIPTION("Krtkl Listener Driver");
MODULE_LICENSE("GPL v2");
