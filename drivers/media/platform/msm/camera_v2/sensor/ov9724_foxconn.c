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
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>

#include <misc/app_info.h>
#define OV9724_FOXCONN_SENSOR_NAME "ov9724_foxconn"
DEFINE_MSM_MUTEX(ov9724_foxconn_mut);

static struct msm_sensor_ctrl_t ov9724_foxconn_s_ctrl;

static struct msm_sensor_power_setting ov9724_foxconn_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = MSM_SENSOR_MCLK_24HZ,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov9724_foxconn_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov9724_foxconn_i2c_id[] = {
	{OV9724_FOXCONN_SENSOR_NAME, (kernel_ulong_t)&ov9724_foxconn_s_ctrl},
	{ }
};

static int32_t msm_ov9724_foxconn_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov9724_foxconn_s_ctrl);
}

static struct i2c_driver ov9724_foxconn_i2c_driver = {
	.id_table = ov9724_foxconn_i2c_id,
	.probe  = msm_ov9724_foxconn_i2c_probe,
	.driver = {
		.name = OV9724_FOXCONN_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov9724_foxconn_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov9724_foxconn_dt_match[] = {
	{.compatible = "qcom,ov9724_foxconn", .data = &ov9724_foxconn_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov9724_foxconn_dt_match);

static struct platform_driver ov9724_foxconn_platform_driver = {
	.driver = {
		.name = "qcom,ov9724_foxconn",
		.owner = THIS_MODULE,
		.of_match_table = ov9724_foxconn_dt_match,
	},
};

static int32_t ov9724_foxconn_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	
	pr_err("%s: %d\n", __func__, __LINE__);
	match = of_match_device(ov9724_foxconn_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	
	return rc;
}

static int __init ov9724_foxconn_init_module(void)
{
	int32_t rc = 0;
	
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov9724_foxconn_platform_driver,
		ov9724_foxconn_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	
	return i2c_add_driver(&ov9724_foxconn_i2c_driver);
}

static void __exit ov9724_foxconn_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov9724_foxconn_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov9724_foxconn_s_ctrl);
		platform_driver_unregister(&ov9724_foxconn_platform_driver);
	} else
		i2c_del_driver(&ov9724_foxconn_i2c_driver);
	return;
}

/****************************************************************************
* FunctionName: ov9724_foxconn_add_project_name;
* Description : add the project name and app_info display;
***************************************************************************/
static int ov9724_foxconn_add_project_name(struct msm_sensor_ctrl_t *s_ctrl)
{
    /*Todo: check the module before cp project name, when we use two sensors with the same IC*/

    /*add project name for the project menu*/
    strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060133FF-OV-F", strlen("23060133FF-OV-F")+1);

    pr_info("%s %d : ov9724_foxconn_add_project_name sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_info->sensor_project_name);

    /*add the app_info*/
    app_info_set("camera_main", "ov9724_foxconn");
    
    pr_info("ov9724_foxconn_add_project_name OK\n");

    return 0;
}

static struct msm_sensor_fn_t ov9724_foxconn_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
    .sensor_add_project_name = ov9724_foxconn_add_project_name,
#endif
};

static struct msm_sensor_ctrl_t ov9724_foxconn_s_ctrl = {
	.sensor_i2c_client = &ov9724_foxconn_sensor_i2c_client,
	.power_setting_array.power_setting = ov9724_foxconn_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov9724_foxconn_power_setting),
	.msm_sensor_mutex = &ov9724_foxconn_mut,
	.sensor_v4l2_subdev_info = ov9724_foxconn_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov9724_foxconn_subdev_info),
	.func_tbl = &ov9724_foxconn_sensor_func_tbl,
};

module_init(ov9724_foxconn_init_module);
module_exit(ov9724_foxconn_exit_module);
MODULE_DESCRIPTION("OVimison 1M Bayer sensor");
MODULE_LICENSE("GPL v2");

