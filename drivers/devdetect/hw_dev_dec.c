/*
 * Copyright (C) huawei company
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */
#include <linux/uaccess.h>
#include <mach/gpio.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/slab.h> 
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/crc-ccitt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/hw_dev_dec.h>
#include <linux/hw_dev_array.h>
#include <linux/err.h>
#ifndef TYPEDEF_UINT8
typedef unsigned char	uint8;
#endif

#ifndef TYPEDEF_UINT16
typedef unsigned short	uint16;
#endif

#ifndef TYPEDEF_UINT32
typedef unsigned int	uint32;
#endif

#ifndef TYPEDEF_UINT64
typedef unsigned long long	uint64;
#endif

#ifndef false
#define false 0
#endif
#ifndef true
#define true 1
#endif

struct dev_flag_device{
	const char    *name;
	struct device    *dev;
	int        index;
};
static uint64 dev_flag_long = 0;

/* set device flag */
void set_hw_dev_flag( int dev_id )
{
	if( (dev_id >= 0) && ( dev_id < DEV_I2C_MAX ) )
	{
		dev_flag_long = dev_flag_long | (1 << dev_id);
	}
	else
	{
		printk("Device %s  is unknown in list \n",hw_dec_device_array[dev_id].devices_name);
	}
}
/* show device flag */
static ssize_t dev_flag_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (dev == NULL) {
		pr_err("dev_flag dev is null\n");
		return -EINVAL;
	}
	return sprintf((char *)buf, "%llx\n",dev_flag_long);
}
static DEVICE_ATTR(dev_flag, S_IRUGO,
					dev_flag_show, NULL);

static struct attribute *dev_dct_attributes[] = {
	&dev_attr_dev_flag.attr,
	NULL
};

static const struct attribute_group dev_dct_attr_group= {
	.attrs = dev_dct_attributes,
};



static int __devinit dev_dct_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk( " -------dev_dct_probe\n " );
	/* creat attr node */
	ret = sysfs_create_group(&pdev->dev.kobj, &dev_dct_attr_group);
	
	if( ret < 0 )
	{
		printk("%s: set hw dev detect flag failed.\n", __func__ );
	}
	
	return 0;
}

static struct of_device_id cyttsp4_i2c_of_match[] = 
{
	{ .compatible = "huawei,hw-dev-detect", },
	{ }
};

static struct platform_driver dev_dct_driver = {
	.driver	= {
		.name	= "hw-dev-detect",
		.owner  = THIS_MODULE,
		.of_match_table = cyttsp4_i2c_of_match,
	},
	.probe		= dev_dct_probe,
	.remove		= NULL,
};

static int __init hw_dev_dct_init(void)
{
	return platform_driver_register(&dev_dct_driver);
}

static void __exit hw_dev_dct_exit(void)
{
	platform_driver_unregister(&dev_dct_driver);
}

late_initcall_sync(hw_dev_dct_init);

module_exit(hw_dev_dct_exit);

MODULE_AUTHOR("sunlibin@huawei.com");
MODULE_DESCRIPTION("Device Detect Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dev_dct");
