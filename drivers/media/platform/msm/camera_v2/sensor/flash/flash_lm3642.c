//qcom tmp patch for camera flash
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#define FLASH_NAME "led-flash"

//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(x...) printk(KERN_ERR "[LM3642]" x)
#else
#define CDBG(x...)
#endif

#define ENABLE_REGISTER 0x0A
#define MODE_BIT_MASK 0x03
#define MODE_BIT_STANDBY 0x00
#define MODE_BIT_INDICATOR 0x01
#define MODE_BIT_TORCH 0x02
#define MODE_BIT_FLASH 0x03
#define ENABLE_BIT_FLASH 0x20
#define ENABLE_BIT_TORCH 0x10

#define CURRENT_REGISTER 0x09
#define CURRENT_TORCH_MASK 0x70
#define CURRENT_TORCH_SHIFT 4
#define CURRENT_FLASH_MASK 0x0F
#define CURRENT_FLASH_SHIFT 0

#define FLASH_FEATURE_REGISTER 0x08
#define FLASH_TIMEOUT_MASK 0x07
#define FLASH_TIMEOUT_SHIFT 0

#define FLASH_CHIP_ID_MASK 0x07
#define FLASH_CHIP_ID 0x0

// Default flash high current, 10 corresponds to 1031.25mA.
#define FLASH_HIGH_CURRENT 9 


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3642_i2c_driver;

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY},
};

//delete "set current action"
static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{ENABLE_REGISTER, MODE_BIT_STANDBY},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	{CURRENT_REGISTER, 0x40},//234mA
	{ENABLE_REGISTER, MODE_BIT_TORCH},
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	{CURRENT_REGISTER, 0x0A},//1030mA
	// The duration of high flash, 0x05 corresponds to 600ms.
	{FLASH_FEATURE_REGISTER, 0x05},
	{ENABLE_REGISTER, MODE_BIT_FLASH | ENABLE_BIT_FLASH},
};

#ifdef CONFIG_HUAWEI_KERNEL
static struct msm_camera_i2c_reg_array lm3642_torch_array[] = {
	{CURRENT_REGISTER, 0x20},//140mA
	{ENABLE_REGISTER, MODE_BIT_TORCH},
};
#endif

static void __exit msm_flash_lm3642_i2c_remove(void)
{
	msm_flash_free_led_data(&fctrl);
	i2c_del_driver(&lm3642_i2c_driver);
	return;
}


static const struct i2c_device_id lm3642_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm3642_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	CDBG("%s entry \n", __func__);

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm3642_i2c_driver = {
	.id_table = lm3642_i2c_id,
	.probe  = msm_flash_lm3642_i2c_probe,
	.remove = __exit_p(msm_flash_lm3642_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
	},
};

static int __init msm_flash_lm3642_i2c_add_driver(void)
{
	CDBG("%s entry\n", __func__);
	return i2c_add_driver(&lm3642_i2c_driver);
}

static struct msm_camera_i2c_client lm3642_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

#ifdef CONFIG_HUAWEI_KERNEL
static struct msm_camera_i2c_reg_setting lm3642_torch_setting = {
	.reg_setting = lm3642_torch_array,
	.size = ARRAY_SIZE(lm3642_torch_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
#endif

static struct msm_led_flash_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
#ifdef CONFIG_HUAWEI_KERNEL
	.torch_setting = &lm3642_torch_setting,
#endif
};

static struct msm_flash_fn_t lm3642_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
#ifdef CONFIG_HUAWEI_KERNEL
	.torch_led_on = msm_torch_led_on,
#endif
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3642_i2c_client,
	.reg_setting = &lm3642_regs,
	.func_tbl = &lm3642_func_tbl,
	// Set default flash high current.
	.flash_high_current = FLASH_HIGH_CURRENT,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm3642_i2c_add_driver);
module_exit(msm_flash_lm3642_i2c_remove);
MODULE_DESCRIPTION("lm3642 FLASH");
MODULE_LICENSE("GPL v2");
