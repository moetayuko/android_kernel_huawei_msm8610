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

#define OV5648_SUNNY_P5V18G_SENSOR_NAME "ov5648_sunny_p5v18g"
DEFINE_MSM_MUTEX(ov5648_sunny_p5v18g_mut);

#define GPIO_CAM_VCM_PWDN 79

#undef CDBG
#define OV5648_SUNNY_P5V18G_DEBUG
#ifdef OV5648_SUNNY_P5V18G_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


static struct msm_sensor_ctrl_t ov5648_sunny_p5v18g_s_ctrl;


static struct msm_sensor_power_setting ov5648_sunny_p5v18g_power_setting[] = {
	//exchange the sequence of IOVDD and AVDD
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 40,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov5648_sunny_p5v18g_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov5648_sunny_p5v18g_i2c_id[] = {
	{OV5648_SUNNY_P5V18G_SENSOR_NAME, (kernel_ulong_t)&ov5648_sunny_p5v18g_s_ctrl},
	{ }
};

static int32_t msm_ov5648_sunny_p5v18g_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov5648_sunny_p5v18g_s_ctrl);
}

static struct i2c_driver ov5648_sunny_p5v18g_i2c_driver = {
	.id_table = ov5648_sunny_p5v18g_i2c_id,
	.probe  = msm_ov5648_sunny_p5v18g_i2c_probe,
	.driver = {
		.name = OV5648_SUNNY_P5V18G_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov5648_sunny_p5v18g_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov5648_sunny_p5v18g_dt_match[] = {
	{.compatible = "qcom,ov5648_sunny_p5v18g", .data = &ov5648_sunny_p5v18g_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov5648_sunny_p5v18g_dt_match);

static struct platform_driver ov5648_sunny_p5v18g_platform_driver = {
	.driver = {
		.name = "qcom,ov5648_sunny_p5v18g",
		.owner = THIS_MODULE,
		.of_match_table = ov5648_sunny_p5v18g_dt_match,
	},
};

static int32_t ov5648_sunny_p5v18g_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov5648_sunny_p5v18g_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov5648_sunny_p5v18g_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov5648_sunny_p5v18g_platform_driver,
		ov5648_sunny_p5v18g_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov5648_sunny_p5v18g_i2c_driver);
}

static void __exit ov5648_sunny_p5v18g_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov5648_sunny_p5v18g_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5648_sunny_p5v18g_s_ctrl);
		platform_driver_unregister(&ov5648_sunny_p5v18g_platform_driver);
	} else
		i2c_del_driver(&ov5648_sunny_p5v18g_i2c_driver);
	return;
}


static int32_t ov5648_sunny_p5v18g_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t ov5648_sunny_p5v18g_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}


int32_t ov5648_sunny_p5v18g_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	if (data->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			data->gpio_conf->cam_gpiomux_conf_tbl,
			data->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	rc = msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}
	for (index = 0; index < power_setting_array->size; index++) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			if (power_setting->seq_val >= s_ctrl->clk_info_size) {
				pr_err("%s clk index %d >= max %d\n", __func__,
					power_setting->seq_val,
					s_ctrl->clk_info_size);
				goto power_up_failed;
			}
			if (power_setting->config_val)
				s_ctrl->clk_info[power_setting->seq_val].
					clk_rate = power_setting->config_val;

			rc = msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				1);
			if (rc < 0) {
				pr_err("%s: clk enable failed\n",
					__func__);
				goto power_up_failed;
			}
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= SENSOR_GPIO_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
			pr_debug("%s:%d gpio set val %d\n", __func__, __LINE__,
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val]);
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val],
				power_setting->config_val);		
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				goto power_up_failed;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);

			gpio_set_value_cansleep(GPIO_CAM_VCM_PWDN,1); //disable vcm_pwdn
			
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				ov5648_sunny_p5v18g_sensor_enable_i2c_mux(data->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			goto power_up_failed;
		}
	}

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0) {
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
		goto power_up_failed;
	}

	CDBG("%s exit\n", __func__);
	return 0;
power_up_failed:
	pr_err("%s:%d failed\n", __func__, __LINE__);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index--; index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				ov5648_sunny_p5v18g_sensor_disable_i2c_mux(data->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);
	return rc;
}

int32_t ov5648_sunny_p5v18g_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index = (power_setting_array->size - 1); index >= 0; index--) {
		CDBG("%s index %d\n", __func__, index);
		power_setting = &power_setting_array->power_setting[index];
		CDBG("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_GPIO:
			if (power_setting->seq_val >= SENSOR_GPIO_MAX ||
				!data->gpio_conf->gpio_num_info) {
				pr_err("%s gpio index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				continue;
			}
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], GPIOF_OUT_INIT_LOW);
			break;
		case SENSOR_VREG:
			if (power_setting->seq_val >= CAM_VREG_MAX) {
				pr_err("%s vreg index %d >= max %d\n", __func__,
					power_setting->seq_val,
					SENSOR_GPIO_MAX);
				continue;
			}
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);

			gpio_set_value_cansleep(GPIO_CAM_VCM_PWDN,0); //enable vcm_pwdn
			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				ov5648_sunny_p5v18g_sensor_disable_i2c_mux(data->i2c_conf);
			break;
		default:
			pr_err("%s error power seq type %d\n", __func__,
				power_setting->seq_type);
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);
	CDBG("%s exit\n", __func__);

	return 0;
}

/****************************************************************************
* FunctionName: ov5648_sunny_p5v18g_add_project_name;
* Description : add the project name and app_info display;
***************************************************************************/
static int ov5648_sunny_p5v18g_add_project_name(struct msm_sensor_ctrl_t *s_ctrl)
{
    /*Todo: check the module before cp project name, when we use two sensors with the same IC*/

    /*add project name for the project menu*/
    strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060129FA-OV-S", strlen("23060129FA-OV-S")+1);

    pr_info("%s %d : ov5648_sunny_p5v18g_add_project_name sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_info->sensor_project_name);
    
    /*add the app_info*/
    app_info_set("camera_main", "ov5648_sunny");

    pr_info("ov5648_sunny_p5v18g_add_project_name OK\n");
    return 0;
}

static struct msm_sensor_fn_t ov5648_sunny_p5v18g_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov5648_sunny_p5v18g_sensor_power_up,
	.sensor_power_down = ov5648_sunny_p5v18g_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
    .sensor_add_project_name = ov5648_sunny_p5v18g_add_project_name,
#endif
};


static struct msm_sensor_ctrl_t ov5648_sunny_p5v18g_s_ctrl = {
	.sensor_i2c_client = &ov5648_sunny_p5v18g_sensor_i2c_client,
	.power_setting_array.power_setting = ov5648_sunny_p5v18g_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov5648_sunny_p5v18g_power_setting),
	.msm_sensor_mutex = &ov5648_sunny_p5v18g_mut,
	.sensor_v4l2_subdev_info = ov5648_sunny_p5v18g_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov5648_sunny_p5v18g_subdev_info),
	.func_tbl = &ov5648_sunny_p5v18g_sensor_func_tbl,
};

module_init(ov5648_sunny_p5v18g_init_module);
module_exit(ov5648_sunny_p5v18g_exit_module);
MODULE_DESCRIPTION("ov5648_sunny_p5v18g");
MODULE_LICENSE("GPL v2");
