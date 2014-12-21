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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <mach/gpiomux.h>
#include <linux/module.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include "msm_led_flash.h"
#include <linux/debugfs.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#define FLASH_CHIP_ID_MASK 0x07
#define FLASH_CHIP_ID 0x0

//#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(x...) printk(KERN_ERR "[LED-I2C]" x)
#else
#define CDBG(x...)
#endif

//lm3642 flash flag register
#define FLASH_FLAG_REGISTER 0x0B

#ifdef CONFIG_HUAWEI_KERNEL
#define TEMPERATUE_NORMAL 1  //normal
#define TEMPERATUE_ABNORMAL 0 //abnormal
static bool led_temperature = TEMPERATUE_NORMAL; //led temperature status
#endif

int32_t msm_led_i2c_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	
	CDBG("%s entry\n", __func__);
	
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	*subdev_id = fctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	return 0;
}

int32_t msm_led_i2c_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;

	CDBG("%s entry,cfg->cfgtype=%d\n", __func__,cfg->cfgtype);

	if (!fctrl->func_tbl) {
		pr_err("failed\n");
		return -EINVAL;
	}

    #ifdef CONFIG_HUAWEI_KERNEL
    //if led status is off and led status abnormal close the led
	if((TEMPERATUE_ABNORMAL == led_temperature) && (MSM_CAMERA_LED_TORCH_POWER_NORMAL != cfg->cfgtype))
	{
		cfg->cfgtype = MSM_CAMERA_LED_OFF;
		pr_err("flash can not work.\n");
	}
    #endif

	switch (cfg->cfgtype) {

	case MSM_CAMERA_LED_INIT:
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		break;

	case MSM_CAMERA_LED_RELEASE:
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->
				flash_led_release(fctrl);
		break;

	case MSM_CAMERA_LED_OFF:
		if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:
		if (fctrl->func_tbl->flash_led_low)
			rc = fctrl->func_tbl->flash_led_low(fctrl);
		break;

	case MSM_CAMERA_LED_HIGH:
		if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);
		break;
#ifdef CONFIG_HUAWEI_KERNEL
	case MSM_CAMERA_LED_TORCH:
		if (fctrl->func_tbl->torch_led_on){
			msleep(100);    //have to sleep to solve the flash problem of torch app
			rc = fctrl->func_tbl->torch_led_on(fctrl);
		}
		break;
#endif

#ifdef CONFIG_HUAWEI_KERNEL
	//normal
	case MSM_CAMERA_LED_TORCH_POWER_NORMAL:
		pr_err("resume the flash.\n");
		led_temperature = TEMPERATUE_NORMAL;
		break;
	//abnormal
	case MSM_CAMERA_LED_TORCH_POWER_ABNORMAL: 
		//need run MSM_CAMERA_LED_OFF to take off the led
		pr_err("tunn off the flash.\n");
		led_temperature = TEMPERATUE_ABNORMAL;
		//close flash
		if (fctrl->func_tbl->flash_led_off)
		{
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		}
#endif
    
	default:
		rc = -EFAULT;
		break;
	}
	CDBG("%s leave, rc=%d\n", __func__,rc);
	return rc;
}

/****************************************************************************
* FunctionName: msm_flash_clear_err_and_unlock;
* Description : clear the error and unlock the IC ;
* NOTE: this funtion must be called before register is read and write
***************************************************************************/
int msm_flash_clear_err_and_unlock(struct msm_led_flash_ctrl_t *fctrl){

        int rc = 0;
        uint16_t reg_value=0;

        CDBG("%s entry\n", __func__);

        if (!fctrl) {
                pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
                return -EINVAL;
        }


        if (fctrl->flash_i2c_client) {
                rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                        fctrl->flash_i2c_client,
                        FLASH_FLAG_REGISTER,&reg_value, MSM_CAMERA_I2C_BYTE_DATA);
                if (rc < 0){
                        pr_err("clear err and unlock %s:%d failed\n", __func__, __LINE__);
                }

                CDBG("clear err and unlock success:%02x\n",reg_value);
        }else{
                pr_err("%s:%d flash_i2c_client NULL\n", __func__, __LINE__);
                return -EINVAL;
        }


       return 0;

}

int msm_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	CDBG("%s entry\n", __func__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_direction_output(fctrl->flash_en, 0);

	return rc;
}

int msm_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s entry\n", __func__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_direction_output(fctrl->flash_en, 0);

	return rc;
}

int msm_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s entry\n", __func__);
	
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	gpio_direction_output(fctrl->flash_en, 0);

	return rc;
}

int msm_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s entry,fctrl->flash_en=%d\n", __func__,fctrl->flash_en);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	gpio_direction_output(fctrl->flash_en, 1);

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("%s entry,fctrl->flash_en=%d\n", __func__,fctrl->flash_en);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	gpio_direction_output(fctrl->flash_en, 1);

	//clear the err and unlock IC, this function must be called before read and write register
	msm_flash_clear_err_and_unlock(fctrl);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

#ifdef CONFIG_HUAWEI_KERNEL
/*===========================================================================
 * FUNCTION    msm_torch_led_on
 *
 * DESCRIPTION: used for torch and mmi application
 *==========================================================================*/
int msm_torch_led_on(struct msm_led_flash_ctrl_t *fctrl)
{
    int rc = 0;

    if (!fctrl) {
        pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
        return -EINVAL;
    }

    gpio_direction_output(fctrl->flash_en, 1);

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_clear_err_and_unlock(fctrl);

    if (fctrl->flash_i2c_client && fctrl->reg_setting) {
    rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
            fctrl->flash_i2c_client,
            fctrl->reg_setting->torch_setting);
        if (rc < 0)
            pr_err("%s:%d failed\n", __func__, __LINE__);
    }

    return rc;
}
#endif

/*===========================================================================
 * FUNCTION    - msm_flash_free_led_data -
 *
 * DESCRIPTION: free space 
 *==========================================================================*/
int32_t msm_flash_free_led_data(struct msm_led_flash_ctrl_t *s_ctrl)
{
        if(s_ctrl->flashdata->slave_info){
	    kfree(s_ctrl->flashdata->slave_info);
        }

        if(s_ctrl->flashdata->gpio_conf){
	    kfree(s_ctrl->flashdata->gpio_conf);
        }

        if(s_ctrl->flashdata){
	    kfree(s_ctrl->flashdata);
        }
	return 0;
}

static int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct device_node *flash_src_node = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3] = {0};

	CDBG("%s entry\n", __func__);

	if (!of_node) {
		pr_err("%s of_node null %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	fctrl->flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->flashdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl->flashdata;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("get subdev id failed\n");
		kfree(fctrl->flashdata);
		return rc;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	// Get the flash high current from .dtsi file. If failed to get the value of current,
	// set register as the default value 1031.25mA.
	rc = of_property_read_u32(of_node, "qcom,flash-high-current", &fctrl->flash_high_current);
	if (rc < 0) {
		pr_err("get flash_high_current failed\n");
	}

	fctrl->reg_setting->high_setting->reg_setting[0].reg_data = fctrl->flash_high_current;
	CDBG("flash_high_current %d\n", fctrl->flash_high_current);

	rc = of_property_read_string(of_node, "qcom,flash-name",
		&flashdata->sensor_name);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		kfree(fctrl->flashdata);
		return rc;
	}

	CDBG("%s qcom,flash-name %s, rc %d\n", __func__,flashdata->sensor_name, rc);

	/*Handle LED Flash Ctrl by flash-source and trigger*/
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("failed\n");
			rc =  -EINVAL;
			kfree(fctrl->flashdata);
			return rc;
		}
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				 fctrl->flash_trigger_name[i]);

			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("failed rc %d\n", rc);
				of_node_put(flash_src_node);
				continue;
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}

	} else /*Handle LED Flash Ctrl by GPIO*/
	{
		flashdata->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!flashdata->gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			kfree(fctrl->flashdata);
			return rc;
		}
		gconf = flashdata->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CDBG("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				kfree(fctrl->flashdata->gpio_conf);
				kfree(fctrl->flashdata);
				return rc;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CDBG("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			fctrl->flash_en = gpio_array[0];
			rc = gpio_request(fctrl->flash_en, "FLASH_EN");
			if (rc) {
				pr_err("%s gpio_request failed %d\n", __func__, __LINE__);
				kfree(gpio_array);
				kfree(fctrl->flashdata->gpio_conf);
				kfree(fctrl->flashdata);
				return rc;

			}
			gpio_tlmm_config(GPIO_CFG(fctrl->flash_en, 0,
				  GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				  GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			gpio_direction_output(fctrl->flash_en, 0);

			kfree(gpio_array);

		}

		flashdata->slave_info =
			kzalloc(sizeof(struct msm_camera_slave_info),
				GFP_KERNEL);
		if (!flashdata->slave_info) {
			CDBG("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			kfree(fctrl->flashdata->gpio_conf);
			kfree(fctrl->flashdata);
			return rc;
		}

		rc = of_property_read_u32_array(of_node, "qcom,slave-id",id_info, 3);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			kfree(fctrl->flashdata->slave_info);
			kfree(fctrl->flashdata->gpio_conf);
			kfree(fctrl->flashdata);
			return rc;
		}
		fctrl->flashdata->slave_info->sensor_slave_addr = id_info[0];
		fctrl->flashdata->slave_info->sensor_id_reg_addr = id_info[1];
		fctrl->flashdata->slave_info->sensor_id = id_info[2];

		CDBG("sensor_slave_addr=%x,sensor_id_reg_addr=%x,sensor_id=%x\n", id_info[0],id_info[1],id_info[2]);

	}

	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

#ifdef CONFIG_DEBUG_FS
static int set_led_status(void *data, u64 val)
{
	struct msm_led_flash_ctrl_t *fctrl =
		 (struct msm_led_flash_ctrl_t *)data;
	int rc = -1;
	pr_debug("set_led_status: Enter val: %llu", val);
	if (!fctrl) {
		pr_err("set_led_status: fctrl is NULL");
		return rc;
	}
	if (!fctrl->func_tbl) {
		pr_err("set_led_status: fctrl->func_tbl is NULL");
		return rc;
	}
	if (val == 0) {
		pr_debug("set_led_status: val is disable");
		rc = msm_flash_led_off(fctrl);
	} else {
		pr_debug("set_led_status: val is enable");
		rc = msm_flash_led_low(fctrl);
	}

	return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(ledflashdbg_fops,
	NULL, set_led_status, "%llu\n");
#endif

int msm_flash_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    uint16_t tmp_data = 0;
#endif

	CDBG("%s entry\n", __func__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed I2C_FUNC_I2C\n");
		goto probe_failure;
	}

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
	{
	    pr_err("i2c_check_functionality failed I2C_FUNC_SMBUS_I2C_BLOCK\n");
		goto probe_failure;
	}

	fctrl = (struct msm_led_flash_ctrl_t *)(id->driver_data);
	if (fctrl->flash_i2c_client)
		fctrl->flash_i2c_client->client = client;
	/* Set device type as I2C */
	fctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
		"%s", id->name);

	rc = msm_led_get_dt_data(client->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	
	if (fctrl->flash_i2c_client != NULL) {
		fctrl->flash_i2c_client->client = client;
		if (fctrl->flashdata->slave_info->sensor_slave_addr)
			fctrl->flash_i2c_client->client->addr =
				fctrl->flashdata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	fctrl->flash_i2c_client->client->addr = fctrl->flash_i2c_client->client->addr<<1;

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* read chip id */

    //clear the err and unlock IC, this function must be called before read and write register
    msm_flash_clear_err_and_unlock(fctrl);

    //tmp_data = i2c_smbus_read_byte_data(client, 0x00);
    rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
                fctrl->flash_i2c_client,0x00,&tmp_data, MSM_CAMERA_I2C_BYTE_DATA);
    if(rc < 0)
    {
        pr_err("%s: FLASHCHIP READ I2C error!\n", __func__);
        goto probe_failure;
    }

    if ( FLASH_CHIP_ID == (tmp_data & FLASH_CHIP_ID_MASK) )
    {
        pr_err("%s : Read chip id ok!Chip ID is %d.\n", __func__, tmp_data);
        /* detect current device successful, set the flag as present */
        set_hw_dev_flag(DEV_I2C_FLASH);
        pr_err("%s : LM3642 probe succeed!\n", __func__);
    }
    else
    {
        pr_err("%s : read chip id error!Chip ID is %d.\n", __func__, tmp_data);
        rc = -ENODEV;
        goto probe_failure;
    }
#endif

	rc = msm_led_i2c_flash_create_v4lsubdev(fctrl);
#ifdef CONFIG_DEBUG_FS
	dentry = debugfs_create_file("ledflash", S_IRUGO, NULL, (void *)fctrl,
		&ledflashdbg_fops);
	if (!dentry)
		pr_err("Failed to create the debugfs ledflash file");
#endif
	CDBG("sensor_slave_addr=%x\n", fctrl->flash_i2c_client->client->addr);
	return 0;

probe_failure:
	pr_err("%s:%d probe failed\n", __func__, __LINE__);
	return rc;
}
