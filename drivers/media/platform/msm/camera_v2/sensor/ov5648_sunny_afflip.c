
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

#define OV5648_SUNNY_AFFLIP_SENSOR_NAME "ov5648_sunny_afflip"
DEFINE_MSM_MUTEX(ov5648_sunny_afflip_mut);

#define GPIO_CAM_VCM_PWDN 79

#undef CDBG
#define OV5648_SUNNY_AFFLIP_DEBUG
#ifdef OV5648_SUNNY_AFFLIP_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
/* if support OTP or open OTP function, please define it */
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
#define OV5648_OTP_FEATURE
#endif

#ifdef OV5648_OTP_FEATURE

//OV5648 Sunny 0x400 = 1x gain
#define OV5648_OTP_ONE_GAIN 0x400

//OV5648 has two bank
typedef enum ov5648_bank_count{
	BANK_0 = 0,
	BANK_1,
	BANK_MAX
}enum_ov5648_bank;

//Write OTP RGB sequence
enum gain_cfg_status {
	GAIN_CFG_R = 0,
	GAIN_CFG_G,
	GAIN_CFG_B,
	GAIN_CFG_MAX
};

//Write OTP RGB High addr, Low addr and name
typedef struct RGB_GAIN_ADDR{
	int32_t High;
	int32_t Low;
	char* Name;
}st_rgb_gain_addr;

//Bank Addr for bank select
typedef struct BANK_ADDR{
	int32_t High;
	int32_t Low;
}st_bank_addr;

//OTP write RGB addr and name
static const st_rgb_gain_addr RGB_Gain_Addr[GAIN_CFG_MAX] = {{0x5186, 0x5187, "R Gain"}, 
															{0x5188, 0x5189, "G Gain"},
															{0x518a, 0x518b, "B Gain"}};

//Bank Addr from 0~1
static const st_bank_addr OTP_Bank_Select[BANK_MAX] = {{0x00, 0x0f}, {0x10, 0x1f}};

//Golden sensor typical ratio
static int RG_Ratio_Typical = 0x2FC;
static int BG_Ratio_Typical = 0x2B5;

//OTP info struct
typedef struct ov5648_otp_info {
	uint16_t iProduct_Year;
	uint16_t iProduct_Month;
	uint16_t iProduct_Date;
	uint16_t iCamera_Id;
	uint16_t iVersion_Id;
	uint16_t iWB_RG_H;
	uint16_t iWB_RG_L;
	uint16_t iWB_BG_H;
	uint16_t iWB_BG_L;
	uint16_t iWB_GbGr_H;
	uint16_t iWB_GbGr_L;
	uint16_t iVCM_Start;
	uint16_t iVCM_End;
}st_ov5648_otp_info;

//OTP info
static st_ov5648_otp_info g_ov5648_otp = {0};
#endif

static struct msm_sensor_ctrl_t ov5648_sunny_afflip_s_ctrl;

static struct msm_sensor_power_setting ov5648_sunny_afflip_power_setting[] = {
	//exchange the sequence of IOVDD and AVDD
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_sensor_power_setting ov5648_sunny_afflip_power_down_setting[] = {
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
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
		.delay = 20,
	},
};
static struct v4l2_subdev_info ov5648_sunny_afflip_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov5648_sunny_afflip_i2c_id[] = {
	{OV5648_SUNNY_AFFLIP_SENSOR_NAME, (kernel_ulong_t)&ov5648_sunny_afflip_s_ctrl},
	{ }
};

static int32_t msm_ov5648_sunny_afflip_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov5648_sunny_afflip_s_ctrl);
}

static struct i2c_driver ov5648_sunny_afflip_i2c_driver = {
	.id_table = ov5648_sunny_afflip_i2c_id,
	.probe  = msm_ov5648_sunny_afflip_i2c_probe,
	.driver = {
		.name = OV5648_SUNNY_AFFLIP_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov5648_sunny_afflip_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov5648_sunny_afflip_dt_match[] = {
	{.compatible = "qcom,ov5648_sunny_afflip", .data = &ov5648_sunny_afflip_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov5648_sunny_afflip_dt_match);

static struct platform_driver ov5648_sunny_afflip_platform_driver = {
	.driver = {
		.name = "qcom,ov5648_sunny_afflip",
		.owner = THIS_MODULE,
		.of_match_table = ov5648_sunny_afflip_dt_match,
	},
};

static int32_t ov5648_sunny_afflip_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov5648_sunny_afflip_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov5648_sunny_afflip_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov5648_sunny_afflip_platform_driver,
		ov5648_sunny_afflip_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov5648_sunny_afflip_i2c_driver);
}

static void __exit ov5648_sunny_afflip_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov5648_sunny_afflip_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5648_sunny_afflip_s_ctrl);
		platform_driver_unregister(&ov5648_sunny_afflip_platform_driver);
	} else
		i2c_del_driver(&ov5648_sunny_afflip_i2c_driver);
	return;
}

static int32_t ov5648_sunny_afflip_sensor_enable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_INIT, NULL);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
		VIDIOC_MSM_I2C_MUX_CFG, (void *)&i2c_conf->i2c_mux_mode);
	return 0;
}

static int32_t ov5648_sunny_afflip_sensor_disable_i2c_mux(struct msm_camera_i2c_conf *i2c_conf)
{
	struct v4l2_subdev *i2c_mux_sd =
		dev_get_drvdata(&i2c_conf->mux_dev->dev);
	v4l2_subdev_call(i2c_mux_sd, core, ioctl,
				VIDIOC_MSM_I2C_MUX_RELEASE, NULL);
	return 0;
}


int32_t ov5648_sunny_afflip_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;

	struct msm_sensor_power_setting_array *power_down_setting_array = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;
	power_down_setting_array = &s_ctrl->power_down_setting_array;

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

#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
			/*store data[0] for the use of power down*/
			if(s_ctrl->power_down_setting_array.power_setting){
				int32_t i = 0;
				int32_t j = 0;
				struct msm_sensor_power_setting *power_down_setting = NULL;
				for(i=0; i<power_down_setting_array->size; i++){
					power_down_setting = &power_down_setting_array->power_setting[i];
					if(power_setting->seq_val == power_down_setting->seq_val){
						for(j=0; j<s_ctrl->clk_info_size; j++)
						{
							power_down_setting->data[j] = power_setting->data[j];
							CDBG("%s clkptr %p \n", __func__, power_setting->data[j]);
						}
					}
				}
			}
#endif

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

#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
			/*store data[0] for the use of power down*/
			if(s_ctrl->power_down_setting_array.power_setting){
				int32_t i = 0;
				struct msm_sensor_power_setting *power_down_setting = NULL;
				for(i=0;i<power_down_setting_array->size; i++){
					power_down_setting = &power_down_setting_array->power_setting[i];
					if(power_setting->seq_val == power_down_setting->seq_val){
						power_down_setting->data[0] = power_setting->data[0];
					}
				}
			}
#endif

			break;
		case SENSOR_I2C_MUX:
			if (data->i2c_conf && data->i2c_conf->use_i2c_mux)
				ov5648_sunny_afflip_sensor_enable_i2c_mux(data->i2c_conf);
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

	/*power up failed, use the power down array to power down*/
	if(s_ctrl->power_down_setting_array.power_setting){
		power_setting_array = &s_ctrl->power_down_setting_array;
		index = power_setting_array->size;
	}
	for (index = 0; index < power_setting_array->size; index++){
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
				ov5648_sunny_afflip_sensor_disable_i2c_mux(data->i2c_conf);
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

int32_t ov5648_sunny_afflip_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t index = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	s_ctrl->stop_setting_valid = 0;

	CDBG("%s:%d\n", __func__, __LINE__);
	/*use separate power down array, and the positive sequence*/
	power_setting_array = &s_ctrl->power_down_setting_array;

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	for (index = 0; index < power_setting_array->size; index++) {
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
			/*set as the value shown, not all to LOW*/
			gpio_set_value_cansleep(
				data->gpio_conf->gpio_num_info->gpio_num
				[power_setting->seq_val], power_setting->seq_val);
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
				ov5648_sunny_afflip_sensor_disable_i2c_mux(data->i2c_conf);
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
* FunctionName: ov5648_sunny_afflip_add_project_name;
* Description : add the project name and app_info display;
***************************************************************************/
static int ov5648_sunny_afflip_add_project_name(struct msm_sensor_ctrl_t *s_ctrl)
{
    /*Todo: check the module before cp project name, when we use two sensors with the same IC*/

    /*add project name for the project menu*/
    strncpy(s_ctrl->sensordata->sensor_info->sensor_project_name, "23060129FA-OV-S", strlen("23060129FA-OV-S")+1);

    pr_info("%s %d : ov5648_sunny_afflip_add_project_name sensor_project_name=%s \n",  __func__, __LINE__,
            s_ctrl->sensordata->sensor_info->sensor_project_name);
    
    /*add the app_info*/
    app_info_set("camera_main", "ov5648_sunny");

    pr_info("ov5648_sunny_afflip_add_project_name OK\n");
    return 0;
}

#ifdef OV5648_OTP_FEATURE

/****************************************************************************
* FunctionName: ov5648_otp_write_i2c;
* Description : write otp info via i2c;
***************************************************************************/
int32_t ov5648_otp_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, int32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
															addr,
															data,
															MSM_CAMERA_I2C_BYTE_DATA);

	if ( rc < 0 )
	{
		CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, data);
	}

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_read_i2c;
* Description : read otp info via i2c;
***************************************************************************/
int32_t ov5648_otp_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,uint32_t addr, uint16_t *data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
															addr,
															data,
															MSM_CAMERA_I2C_BYTE_DATA);
	if ( rc < 0 )
	{
		CDBG("%s fail, rc = %d! addr = 0x%x, data = 0x%x\n", __func__, rc, addr, *data);
	}
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_clear;
* Description : clear otp buffer 0x3D00~0x3D0F;
***************************************************************************/
int32_t ov5648_otp_clear(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i = 0;
	int32_t rc = 0;
	
	for ( i = 0; i < 16; i++ ) 
	{
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d00 + i, 0x00);
		
		if ( rc < 0 )
		{
			CDBG("%s:%d clear error: rc is %d, i2c addr is 0x3d00 + 0x%x \n", __func__, __LINE__, rc, i);
			break;
		}
	}
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_set_read_mode;
* Description : set sensor otp mode:read;
***************************************************************************/
int32_t ov5648_otp_set_read_mode(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//set otp read mode before reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d84, 0xc0);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_enable_read;
* Description : set sensor could be readed;
***************************************************************************/
int32_t ov5648_otp_enable_read(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//enable otp reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d81, 0x01);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_disable_read;
* Description : set sensor could not be readed;
***************************************************************************/
int32_t ov5648_otp_disable_read(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	//disable otp reading
	rc = ov5648_otp_write_i2c(s_ctrl, 0x3d81, 0x00);
	msleep(10);
	
	return rc;
}

/****************************************************************************
* FunctionName: ov5648_otp_select_bank;
* Description : select otp bank0 or bank1;
***************************************************************************/
int32_t ov5648_otp_select_bank(struct msm_sensor_ctrl_t *s_ctrl, enum_ov5648_bank bank)
{
	int32_t rc = 0;

	if ( ( bank >= BANK_0 ) && ( bank < BANK_MAX ) )
	{
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d85, OTP_Bank_Select[bank].High);
		rc = ov5648_otp_write_i2c(s_ctrl, 0x3d86, OTP_Bank_Select[bank].Low);
	}
	else
	{
		CDBG("%s error bank = %d\n", __func__, bank);
	}
	
	msleep(10);
	
	return rc;
}


/****************************************************************************
* FunctionName: ov5648_check_awb;
* Description : if WB data read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_awb(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( (0 == otp_ptr->iWB_RG_H) && (0 == otp_ptr->iWB_RG_L) )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_module;
* Description : if module info read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_module(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( 0 == otp_ptr->iCamera_Id )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_start_code;
* Description : if VCM start code read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_start_code(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{	
		if ( 0 == otp_ptr->iVCM_Start )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_check_end_code;
* Description : if VCM end code read from otp bank x is valid, return 0;
***************************************************************************/
int32_t ov5648_check_end_code(const st_ov5648_otp_info *otp_ptr)
{
	int32_t ret = 0;

	if ( NULL != otp_ptr )
	{
		if ( 0 == otp_ptr->iVCM_End )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d otp_ptr is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp_bank1;
* Description : Read WB,module info and VCM info from bank1;
* return value:
* 0 means no need to read bank0
*-1 means need to read bank0
***************************************************************************/
int32_t ov5648_read_otp_bank1(struct msm_sensor_ctrl_t *s_ctrl, st_ov5648_otp_info *p_ov5648_otp)
{
	int32_t ret = 0;
	
	if ( NULL != p_ov5648_otp )
	{
		/*read AWB */
		ov5648_otp_read_i2c(s_ctrl, 0x3D07, &p_ov5648_otp->iWB_RG_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D08, &p_ov5648_otp->iWB_RG_L);
		ov5648_otp_read_i2c(s_ctrl, 0x3D09, &p_ov5648_otp->iWB_BG_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0A, &p_ov5648_otp->iWB_BG_L);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0B, &p_ov5648_otp->iWB_GbGr_H);
		ov5648_otp_read_i2c(s_ctrl, 0x3D0C, &p_ov5648_otp->iWB_GbGr_L);

		/*read module*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D02, &p_ov5648_otp->iProduct_Year);
		ov5648_otp_read_i2c(s_ctrl, 0x3D03, &p_ov5648_otp->iProduct_Month);
		ov5648_otp_read_i2c(s_ctrl, 0x3D04, &p_ov5648_otp->iProduct_Date);
		ov5648_otp_read_i2c(s_ctrl, 0x3D05, &p_ov5648_otp->iCamera_Id);
		ov5648_otp_read_i2c(s_ctrl, 0x3D06, &p_ov5648_otp->iVersion_Id);

		/*read VCM start code*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D0D, &p_ov5648_otp->iVCM_Start);
		
		if ( 0 == p_ov5648_otp->iVCM_Start )
		{
			ov5648_otp_read_i2c(s_ctrl, 0x3D00, &p_ov5648_otp->iVCM_Start);
			
			if( 0 == p_ov5648_otp->iVCM_Start )
			{
				CDBG("%s:%d ov5648_otp->iVCM_Start is No Value\n", __func__, __LINE__);
			}
		}

		p_ov5648_otp->iVCM_Start = p_ov5648_otp->iVCM_Start << 2;

		/*read VCM end code*/
		ov5648_otp_read_i2c(s_ctrl, 0x3D0E, &p_ov5648_otp->iVCM_End);
		
		if ( 0 == p_ov5648_otp->iVCM_End )
		{
			ov5648_otp_read_i2c(s_ctrl, 0x3D01, &p_ov5648_otp->iVCM_End);
		
			if ( 0 == p_ov5648_otp->iVCM_End )
			{
				CDBG("%s:%d ov5648_otp->iVCM_End is No Value\n", __func__, __LINE__);
			}
		}

		p_ov5648_otp->iVCM_End = p_ov5648_otp->iVCM_End << 2;
		
		if ( (0 != ov5648_check_awb(p_ov5648_otp)) || (0 != ov5648_check_module(p_ov5648_otp)) )
		{
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d p_ov5648_otp is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp_bank0;
* Description : Read WB and module info from bank0;
* return value:
* 0 means read bank0 OK
* -1 means read bank0 and bank1 all failed
***************************************************************************/
int32_t ov5648_read_otp_bank0(struct msm_sensor_ctrl_t *s_ctrl, st_ov5648_otp_info *p_ov5648_otp)
{
	int32_t ret = 0;

	if ( NULL != p_ov5648_otp )
	{
		if (0 != ov5648_check_awb(p_ov5648_otp))
		{
			/*read AWB */
			ov5648_otp_read_i2c(s_ctrl, 0x3D0A, &p_ov5648_otp->iWB_RG_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0B, &p_ov5648_otp->iWB_RG_L);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0C, &p_ov5648_otp->iWB_BG_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0D, &p_ov5648_otp->iWB_BG_L);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0E, &p_ov5648_otp->iWB_GbGr_H);
			ov5648_otp_read_i2c(s_ctrl, 0x3D0F, &p_ov5648_otp->iWB_GbGr_L);
		}

		if (0 != ov5648_check_module(p_ov5648_otp))
		{
			/*read module*/
			ov5648_otp_read_i2c(s_ctrl, 0x3D05, &p_ov5648_otp->iProduct_Year);
			ov5648_otp_read_i2c(s_ctrl, 0x3D06, &p_ov5648_otp->iProduct_Month);
			ov5648_otp_read_i2c(s_ctrl, 0x3D07, &p_ov5648_otp->iProduct_Date);
			ov5648_otp_read_i2c(s_ctrl, 0x3D08, &p_ov5648_otp->iCamera_Id);
			ov5648_otp_read_i2c(s_ctrl, 0x3D09, &p_ov5648_otp->iVersion_Id);
		}
		
		if ( (0 != ov5648_check_awb(p_ov5648_otp)) || (0 != ov5648_check_module(p_ov5648_otp)) )
		{
			CDBG("%s:%d Read Bank0 failed or there is no valuable data!\n", __func__, __LINE__);
			ret = -1;
		}
	}
	else
	{
		CDBG("%s:%d p_ov5648_otp is NULL\n", __func__, __LINE__);
		ret = -1;
	}
	
	return ret;
}

/****************************************************************************
* FunctionName: ov5648_read_otp;
* Description : read otp info;
* return value:
* 0 means read otp info succeed.
* -1 means read otp info failed, should not write otp.
***************************************************************************/
int32_t ov5648_read_otp(struct msm_sensor_ctrl_t *s_ctrl)	
{
	int32_t ret = 0;
	st_ov5648_otp_info ov5648_otp;
	
	memset(&ov5648_otp, 0, sizeof(ov5648_otp));

	/* first read from bank 1 */
	
	ov5648_otp_set_read_mode(s_ctrl);

	ov5648_otp_clear(s_ctrl);

	ov5648_otp_select_bank(s_ctrl, BANK_1);

	ov5648_otp_enable_read(s_ctrl);

	ret = ov5648_read_otp_bank1(s_ctrl, &ov5648_otp);

	ov5648_otp_disable_read(s_ctrl);

	ov5648_otp_clear(s_ctrl); 

	/*if awb or module is 0, then read bank 0*/
	if ( 0 != ret )
	{
		printk("%s read from bank0\n", __func__);
		
		ov5648_otp_set_read_mode(s_ctrl);

		ov5648_otp_select_bank(s_ctrl, BANK_0);

		ov5648_otp_enable_read(s_ctrl);
		
		ret = ov5648_read_otp_bank0(s_ctrl, &ov5648_otp);
		
		ov5648_otp_disable_read(s_ctrl);

		ov5648_otp_clear(s_ctrl);
	}

	if ( (0 != ret)
	|| (0 != ov5648_check_start_code(&ov5648_otp))
	|| (0 != ov5648_check_end_code(&ov5648_otp)) )
	{
		CDBG("%s:%d ERROR:there is no valuable otp data!\n", __func__, __LINE__);
		ret = -1;
	}
	else
	{
		memcpy(&g_ov5648_otp, &ov5648_otp, sizeof(st_ov5648_otp_info));

		printk("%s:iProduct_Year = %d, \n \
		iProduct_Month = %d, \n \
		iProduct_Date = %d,\n  \
		iCamera_Id = %d, \n \
		iVersion_Id = %d, \n \
		iWB_RG_H = 0x0%X, \n \
		iWB_RG_L = 0x0%X, \n \
		iWB_BG_H = 0x0%X, \n \
		iWB_BG_L = 0x0%X,\n  \
		iWB_GbGr_H = 0x0%X, \n \
		iWB_GbGr_L = 0x0%X, \n \
		iVCM_Start = %d, \n \
		iVCM_End = %d\n ", 
		__func__, g_ov5648_otp.iProduct_Year, g_ov5648_otp.iProduct_Month, 
		g_ov5648_otp.iProduct_Date, g_ov5648_otp.iCamera_Id, g_ov5648_otp.iVersion_Id,	
		g_ov5648_otp.iWB_RG_H, g_ov5648_otp.iWB_RG_L, g_ov5648_otp.iWB_BG_H, 
		g_ov5648_otp.iWB_BG_L, g_ov5648_otp.iWB_GbGr_H, g_ov5648_otp.iWB_GbGr_L, 
		g_ov5648_otp.iVCM_Start, g_ov5648_otp.iVCM_End);
	}

	return ret;
}

/****************************************************************************
* FunctionName: ov5648_update_awb_gain;
* Description : write R_gain,G_gain,B_gain to otp;
* 0x400 =1x Gain
* 0 means write WB info succeed.
* -1 means write WB info failed.
***************************************************************************/
int32_t ov5648_update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl, const int *pGainCfg)
{
	int32_t rc = 0;
	int iGainindex = 0;
	
	if ( NULL != pGainCfg )
	{
		for ( iGainindex = GAIN_CFG_R; iGainindex < GAIN_CFG_MAX; iGainindex++ )
		{
			if ( pGainCfg[iGainindex] > OV5648_OTP_ONE_GAIN ) 
			{
				rc = ov5648_otp_write_i2c(s_ctrl, RGB_Gain_Addr[iGainindex].High, pGainCfg[iGainindex] >> 8);
				
				if ( rc < 0 )
				{
					CDBG("%s:%d write otp %s High failed, rc is %d\n", __func__, __LINE__, RGB_Gain_Addr[iGainindex].Name, rc);
					break;
				}
				
				rc = ov5648_otp_write_i2c(s_ctrl, RGB_Gain_Addr[iGainindex].Low, pGainCfg[iGainindex] & 0x00ff);
				
				if ( rc < 0 )
				{
					CDBG("%s:%d write otp %s Low failed, rc is %d\n", __func__, __LINE__, RGB_Gain_Addr[iGainindex].Name, rc);
					break;
				}

				printk("OV5648 sunny OTP AWB info:%s is 0x%X\n",  RGB_Gain_Addr[iGainindex].Name, pGainCfg[iGainindex]);
			}
			
		}

		if ( rc < 0 )
		{
			CDBG("Set ov5648 sunny OTP AWB Failed!\n");
		}
		else
		{
			CDBG("Set ov5648 sunny OTP AWB Succeed!\n");
		}
	}
	else
	{
		CDBG("%s:%d  ERROR: pGainCfg is NULL!\n", __func__, __LINE__);
	}

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_sunny_get_otp_info;
* Description : get otp info from sensor;
***************************************************************************/
int32_t ov5648_sunny_get_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	
	CDBG("Get ov5648 sunny OTP info Enter\n");

	//set sensor mode:Active
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x1);
	
	rc = ov5648_read_otp(s_ctrl);
	
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CDBG("Get ov5648 sunny OTP info Exit\n");

	return rc;
}

/****************************************************************************
* FunctionName: ov5648_sunny_set_otp_info;
* Description : set otp data to sensor;
* call this function after OV5648 initialization 
***************************************************************************/
static int32_t ov5648_sunny_set_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int GainCfg[GAIN_CFG_MAX] = {0};
	int rg, bg;
	int32_t rc = 0;

	static int i_read_otp = 0;
	
	//Get otp info on the first time
	if ( 0 == i_read_otp )
	{		
		rc = ov5648_sunny_get_otp_info(s_ctrl);
		
		if ( rc < 0 )
		{
			CDBG("%s:%d otp read failed.\n", __func__, __LINE__);
			return rc;
		}
		else
		{
			i_read_otp = 1;
		}
	}

	CDBG("Set ov5648 sunny OTP info Enter\n");
	
	rg = (g_ov5648_otp.iWB_RG_H << 8) + g_ov5648_otp.iWB_RG_L;
	bg = (g_ov5648_otp.iWB_BG_H << 8) + g_ov5648_otp.iWB_BG_L;

	//calculate G gain
	//0x400 = 1x gain
	if ( bg < BG_Ratio_Typical ) 
	{
		if ( rg< RG_Ratio_Typical ) 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = OV5648_OTP_ONE_GAIN;
			B_gain = OV5648_OTP_ONE_GAIN * BG_Ratio_Typical / bg;
			R_gain = OV5648_OTP_ONE_GAIN * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = OV5648_OTP_ONE_GAIN;
			G_gain = OV5648_OTP_ONE_GAIN * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else 
	{
		if ( rg < RG_Ratio_Typical ) 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = OV5648_OTP_ONE_GAIN;
			G_gain = OV5648_OTP_ONE_GAIN * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		}
		else 
		{
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = OV5648_OTP_ONE_GAIN * bg / BG_Ratio_Typical;
			G_gain_R = OV5648_OTP_ONE_GAIN * rg / RG_Ratio_Typical;

			if ( G_gain_B > G_gain_R ) 
			{
				B_gain = OV5648_OTP_ONE_GAIN;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			}
			else 
			{
				R_gain = OV5648_OTP_ONE_GAIN;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}

	GainCfg[GAIN_CFG_R] = R_gain;
	GainCfg[GAIN_CFG_G] = G_gain;
	GainCfg[GAIN_CFG_B] = B_gain;

	//set sensor mode:Active
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x1);
	
	//Set the otp info to sensor
	rc = ov5648_update_awb_gain(s_ctrl, GainCfg);
	
	usleep_range(5000, 6000);

	//set sensor mode:Standby
	ov5648_otp_write_i2c(s_ctrl, 0x100, 0x0);

	CDBG("Set ov5648 sunny OTP info Exit\n");
	
	return rc;
}

#endif

static struct msm_sensor_fn_t ov5648_sunny_afflip_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov5648_sunny_afflip_sensor_power_up,
	.sensor_power_down = ov5648_sunny_afflip_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
    .sensor_add_project_name = ov5648_sunny_afflip_add_project_name,
#ifdef OV5648_OTP_FEATURE
	//add otp function
	.sensor_set_otp_info = ov5648_sunny_set_otp_info,
#endif
#endif
};

static struct msm_sensor_ctrl_t ov5648_sunny_afflip_s_ctrl = {
	.sensor_i2c_client = &ov5648_sunny_afflip_sensor_i2c_client,
	.power_setting_array.power_setting = ov5648_sunny_afflip_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov5648_sunny_afflip_power_setting),
	.msm_sensor_mutex = &ov5648_sunny_afflip_mut,
	.sensor_v4l2_subdev_info = ov5648_sunny_afflip_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov5648_sunny_afflip_subdev_info),
	.func_tbl = &ov5648_sunny_afflip_sensor_func_tbl,
#ifdef CONFIG_HUAWEI_KERNEL_CAMERA
	.power_down_setting_array.power_setting = ov5648_sunny_afflip_power_down_setting,
	.power_down_setting_array.size = ARRAY_SIZE(ov5648_sunny_afflip_power_down_setting),
#endif
};

module_init(ov5648_sunny_afflip_init_module);
module_exit(ov5648_sunny_afflip_exit_module);
MODULE_DESCRIPTION("ov5648_sunny_afflip");
MODULE_LICENSE("GPL v2");

