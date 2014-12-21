/********************************************************************************
* Device driver for monitoring ambient light intensity (lux) and proximity
* detection for the TAOS TSL2x7x and TMD2x7x family of devices.
*
* Copyright (c) 2012, TAOS Corporation.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later vers ion.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA      02110-1301, USA.
********************************************************************************/

/********************************************************************************
*										*
*   File Name:    tmd27713.c							*
*   Description:   Linux device driver for Taos ambient light and		*
*   proximity sensors.								*
   Author:         John Koshi							*
*   History:	09/16/2009 - Initial creation					*
*				10/09/2009 - Triton version			*
*				12/21/2009 - Probe/remove mode			*
*				02/07/2010 - Add proximity			*
*										*
********************************************************************************/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/delay.h>
#include <linux/taos_common.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
//#include	<linux/regulator/consumer.h>
#include <misc/app_info.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

// device name/id/address/counts
#define TAOS_SENSOR_TMD2771_NAME	"tmd27713"
#define TAOS_DEVICE_NAME                TAOS_SENSOR_TMD2771_NAME
#define TAOS_DEVICE_ID                  TAOS_SENSOR_TMD2771_NAME
#define TAOS_INPUT_NAME			TAOS_SENSOR_TMD2771_NAME
#define TAOS_SENSOR_INFO		"1.0"

#define TAOS_ID_NAME_SIZE               10
#define TAOS_MAX_NUM_DEVICES            1
#define TAOS_MAX_DEVICE_REGS            32
#define I2C_MAX_ADAPTERS                9

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_TEST_STATUS         0x1F

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG		0X80
#define TAOS_TRITON_CMD_AUTO		0x10
#define TAOS_TRITON_CMD_BYTE_RW		0x00
#define TAOS_TRITON_CMD_SPL_FN		0x60
#define TAOS_TRITON_CMD_PROX_INTCLR	0X05
#define TAOS_TRITON_CMD_ALS_INTCLR	0X06
#define TAOS_TRITON_CMD_INTCLR		0X07

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL	0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL	0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL	0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL	0X04
#define TAOS_TRITON_CNTL_SENS_ENBL	0x0F
#define TAOS_TRITON_CNTL_ADC_ENBL	0x02
#define TAOS_TRITON_CNTL_PWRON		0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID	0x01

// lux constants
#define TAOS_MAX_LUX			10000
#define TAOS_FILTER_DEPTH		3
/*use this to contrl the tmd27713 debug message*/
static int tmd27713_debug_mask = 0;
module_param_named(tmd27713_debug, tmd27713_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
    
#define TMD27713_DEBUG(x...) do { \
    if (tmd27713_debug_mask) { \
        printk(KERN_DEBUG x); \
    } \
} while (0)


/*use to enable/disable als or prox or the ic*/
#define  SENSOR_POWER 0
#define  ALS_ENABLE   1
#define  PROX_ENABLE  2
// forward declarations
struct taos_data;

static int taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp);
static int taos_remove(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void taos_early_suspend(struct early_suspend *h);
static void taos_late_resume(struct early_suspend *h);
#endif
/*delete the device note fops, we use sys interface*/
static int taos_get_lux(void);
static int taos_device_name(unsigned char *bufp, char **device_name);
static int taos_prox_poll(struct taos_prox_info *prxp);

static int taos_als_threshold_set(void);
static int taos_prox_threshold_set(void);
static int taos_als_get_data(void);

#define PS_INT_CLR	0
#define ALS_INT_CLR	1
/*use to clear als and ps a time*/
#define PA_ALS_INT_CLR 3
static int taos_ps_als_int_clear(int type);
/*delete the old interface, we use new interface for enable/disable als and prox*/

static int taos_suspend(struct i2c_client *client, pm_message_t mesg);
static int taos_resume(struct i2c_client *client);

DECLARE_WAIT_QUEUE_HEAD(waitqueue_read);

/*don't need the struct and vars,  delete them*/


// module device table
static struct i2c_device_id taos_idtable[] = {
	{TAOS_DEVICE_ID, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, taos_idtable);

// client and device
struct i2c_client *my_clientp;
static char pro_buf[4];
static int mcount = 0;
static char als_buf[4];
u16 status = 0;
/*don't use the vars now, delete them*/
/*remove the not use var, and use sensor_on_flag to mask the seneor is on?*/
static int sensor_on_flag  =0;
static u8 reg_ctrl_value = 0x38;
static int init_ok = 0;
#ifdef CONFIG_OF
static struct of_device_id goodix_match_table[] = {
                { .compatible = "taos,tmd27713",},
                { },
        };
#else
#define goodix_match_table NULL
#endif
// driver definition
static struct i2c_driver taos_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = TAOS_DEVICE_NAME,
		.of_match_table = goodix_match_table,
	},
	.id_table = taos_idtable,
	.probe = taos_probe,
	.remove = __devexit_p(taos_remove),
	.suspend = taos_suspend,
    .resume = taos_resume,
};

// per-device data
struct taos_data {
	struct i2c_client *client;
	struct cdev cdev;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct work_struct work;
	struct mutex date_lock;
	char taos_name[TAOS_ID_NAME_SIZE];
	struct semaphore update_lock;
	struct wake_lock taos_wake_lock;
	struct early_suspend early_suspend;
	int working;
	struct tmd2771x_platform_data *pdata;
	int ps_enable;
	int als_enable;
};

static struct taos_data *taos_datap;

// file operations
/*don't use the fops, delete it*/

// device configuration
struct taos_cfg *taos_cfgp;
static u32 calibrate_target_param = 300000;
static u16 als_time_param = 200;
static u16 scale_factor_param = 1;
static u16 gain_trim_param = 512;
static u8 filter_history_param = 3;
static u8 gain_param = 0;

//static u16 prox_threshold_hi_param = 820;
//static u16 prox_threshold_lo_param = 550;

static u16 prox_threshold_hi_param = 500;
static u16 prox_threshold_lo_param = 400;

static u16 als_threshold_hi_param = 3000;
static u16 als_threshold_lo_param = 10;
static u8 prox_int_time_param = 0xEE;		//50ms
static u8 prox_adc_time_param = 0xFF;
static u8 prox_wait_time_param = 0xEE;
static u8 prox_intr_filter_param = 0x13;
static u8 prox_config_param = 0x00;
static u8 prox_pulse_cnt_param = 0x02;	//set led pulse count
static u8 prox_gain_param = 0x20;

// prox info
struct taos_prox_info prox_cal_info[20];
struct taos_prox_info prox_cur_info;
struct taos_prox_info *prox_cur_infop = &prox_cur_info;
/*don't use these vars, delete them*/
static u16 sat_als = 0;
static u16 sat_prox = 0;

// device reg init values
u8 taos_triton_reg_init[16] = { 0x00, 0xFF, 0XFF, 0XFF, 0X00, 0X00, 0XFF, 0XFF,
	0X00, 0X00, 0XFF, 0XFF, 0X00, 0X00, 0X00, 0X00
};

// lux time scale
struct time_scale_factor {
	u16 numerator;
	u16 denominator;
	u16 saturation;
};
struct time_scale_factor TritonTime = { 1, 0, 0 };

struct time_scale_factor *lux_timep = &TritonTime;

// gain table
u8 taos_triton_gain_table[] = { 1, 8, 16, 120 };

// lux data
struct lux_data {
	u16 ratio;
	u16 clear;
	u16 ir;
};

struct lux_data TritonFN_lux_data[] = {
	{9830, 8320, 15360},
	{12452, 10554, 22797},
	{14746, 6234, 11430},
	{17695, 3968, 6400},
	{0, 0, 0}
};

struct lux_data *lux_tablep = TritonFN_lux_data;

static int lux_history[TAOS_FILTER_DEPTH] = { -ENODATA, -ENODATA, -ENODATA };
/*add init reg, copy from 8x25Q's aps-9900 driver */
struct inti_regdata_s{
      int reg;
      uint8_t data;
};

static struct inti_regdata_s taos_init_regdata[]=
{
    {TAOS_TRITON_CNTRL, 0x0},
    {TAOS_TRITON_ALS_TIME,   0xdb},
    {TAOS_TRITON_PRX_TIME,   0xff},
    /*modify the value*/
    {TAOS_TRITON_WAIT_TIME,  0xb6},
    /* modify the ppcount from 8 to 4 */
    {TAOS_TRITON_PRX_COUNT, 0x04},
    {TAOS_TRITON_GAIN, 0x60},
    {TAOS_TRITON_CNTRL, 0x38},
    {TAOS_TRITON_INTERRUPT, 0x12}
};

static struct workqueue_struct *aps_wq = NULL;
/*don't use config regulator, we use ldo19 on 8x12*/

static int taos_read_byte(struct i2c_client *client, u8 reg)
{
	s32 ret;

	reg &= ~TAOS_TRITON_CMD_SPL_FN;
	reg |= TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_BYTE_RW;
	/*lock for i2c read and write*/
    mutex_lock(&taos_datap->date_lock);
	ret = i2c_smbus_read_byte_data(client, reg);
    mutex_unlock(&taos_datap->date_lock);
	return ret;
}

static int taos_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	s32 ret;

	reg &= ~TAOS_TRITON_CMD_SPL_FN;
	reg |= TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_BYTE_RW;
	/*lock for i2c read and write*/
    mutex_lock(&taos_datap->date_lock);
	ret = i2c_smbus_write_byte_data(client, reg, data);
    mutex_unlock(&taos_datap->date_lock);
	return (int)ret;
}

static int taos_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = -1;
    pr_info("%s: enter reg_ctrl_value = %d\n",__func__,reg_ctrl_value);
	/*if als or prox enable, then disable irq*/
    if(sensor_on_flag)
    {
	    disable_irq(taos_datap->client->irq);
    }
    
    ret = cancel_work_sync(&taos_datap->work);

	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL,0)) < 0) {
		printk(KERN_ERR "%s: write byte failed in taos_suspend ret = %d\n",__func__,ret);
	}
    return 0;
}
static int taos_resume(struct i2c_client *client)
{
	int ret = -1;
    pr_info("%s: enter reg_ctrl_value = %d\n",__func__,reg_ctrl_value);
	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL, reg_ctrl_value)) < 0) {
		printk(KERN_ERR "%s: write byte_data failed ret = %d reg_ctrl_value=0x%02x\n",__func__, ret,reg_ctrl_value);
	}
	
	/*if als or prox enable, then enable irq*/
    if(sensor_on_flag)
    {
        enable_irq(taos_datap->client->irq);
    }
    return 0;
}

#define MAX_TRY_COUNT 1
/*the functioun use to init sensor regs when probe success*/
static int taos_regs_init(struct taos_data *pdata)
{
    int i = 0;
    int try = 0;
    int ret = 0;

    
    for(try = 0; try < MAX_TRY_COUNT; try++)
    {
        for (i=0; i< ARRAY_SIZE(taos_init_regdata);i++)
        {
            ret = taos_write_byte(pdata->client, taos_init_regdata[i].reg, taos_init_regdata[i].data);
            if (ret < 0)
            {
                pr_err("Ret of init taos regs(%d - %d) is %d\n", taos_init_regdata[i].reg, taos_init_regdata[i].data, taos_read_byte(pdata->client, taos_init_regdata[i].reg));
                break;
            }
        }

        if(ret < 0)
        {
            pr_err("%s: try count = %d\n",__func__,try);
            msleep(100);
        }
        else
        {
            break;
        }
    }
    
    return ret;
}
/*use this function to control als or prox or the total ic enable/disable*/
static int taos_reg_enable(struct taos_data *pdata, int type, int enable)
{
    int value = 0;
    int ret = 0;
    u8 reg0 = 0;
    /*first: read the current control value*/
    /*
    ret = taos_read_byte(pdata->client,TAOS_TRITON_CNTRL);
    if(ret < 0)
    {
        TMD27713_DEBUG("%s: i2c read TAOS_TRITON_CNTRL fail\n",__func__);
        return ret;
    }
    */
    
    reg0 = reg_ctrl_value;

    if(enable)
    {
        /*second: check the type is als or prox or the total ic? then set the reg value*/
        switch(type)
        {
            case SENSOR_POWER:
                value = reg0 | (1 << 0);
                break;
            case ALS_ENABLE:
                value = reg0 | (1 << 1);
                break;
            case PROX_ENABLE:
                value = reg0 | (1 << 2);
                break;
            default:
                TMD27713_DEBUG("%s: type fail\n",__func__);
                break;
        }
        
    }
    else
    {
        /*second: check the type is als or prox or the total ic? then set the reg value*/
        switch(type)
        {
            case SENSOR_POWER:
                value = reg0 & 0xfe;
                break;
            case ALS_ENABLE:
                value = reg0 & 0xfd;
                break;
            case PROX_ENABLE:
                value = reg0 & 0xfb;
                break;
            default:
                TMD27713_DEBUG("%s: type fail\n",__func__);
                break;
        }
        
    }
    /*third: write the new value to the control reg*/
    ret = taos_write_byte(pdata->client,TAOS_TRITON_CNTRL,value);
    TMD27713_DEBUG("%s: control = 0x%02x\n",__func__,value);
    if(ret < 0)
    {
        TMD27713_DEBUG("%s: i2c write TAOS_TRITON_CNTRL fail\n",__func__);
    }

    reg_ctrl_value = value;

    return ret;
}
static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	/*disable irq first, then wakeup the work*/
	disable_irq_nosync(taos_datap->client->irq);
	queue_work(aps_wq, &taos_datap->work);

	return IRQ_HANDLED;
}

static int taos_ps_als_int_clear(int type)
{
	int ret = 0;
	u8 cl_irq = 0;
	/*add clear prox and als int at a time*/
	if (type == PS_INT_CLR)
		cl_irq = 0x05;
	else if(type == ALS_INT_CLR)
		cl_irq = 0x06;
	else if(type == PA_ALS_INT_CLR)
		cl_irq = 0x07;

	if ((ret = (i2c_smbus_write_byte(taos_datap->client,
			(TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_SPL_FN | cl_irq)))) < 0) {
				printk(KERN_ERR "TAOS: clear interrupt failed in %s\n",
		       __func__);
		return (ret);
	}

	return ret;
}

static int taos_get_data(void)
{
	int ret = 0;

	if ((status = taos_read_byte(taos_datap->client, TAOS_TRITON_STATUS)) < 0) {
		printk(KERN_ERR "%s: read the chip status is failed\n",
		       __func__);
		return ret;
	}

    TMD27713_DEBUG("%s: status = 0x%02x\n",__func__,(u8)status);
	/*check if triger als int*/
	if ((status & 0x10) == 0x10) {
        TMD27713_DEBUG("%s: als irq\n",__func__);
		/*check the als data is valid?*/
        if((status & 0x01) != 0x01)
        {
            taos_ps_als_int_clear(ALS_INT_CLR);
            TMD27713_DEBUG("%s:als data avalid! just return!\n",__func__);
            return ret;
        }
		if(!taos_datap->als_enable)
		{
			/*if als disable, just clear int and return*/
            TMD27713_DEBUG("%s: als is disable but receive als int! just return!\n",__func__);
            taos_ps_als_int_clear(ALS_INT_CLR);
            return ret;
		}
		/*get als data and report it*/
		taos_als_get_data();
        taos_als_threshold_set();
	}
	if ((status & 0x20) == 0x20) {
        TMD27713_DEBUG("%s: prox irq\n",__func__);
        if(!taos_datap->ps_enable)
        {
			/*if prox disable, just clear int and return*/
            taos_ps_als_int_clear(PS_INT_CLR);
            TMD27713_DEBUG("%s: prox is disable but receive prox int! just return!\n",__func__);
            return ret;
        }
		/*get prox data and report it, and change the threshold value*/
		ret = taos_prox_threshold_set();
	}
	/*clear als and prox int*/
    taos_ps_als_int_clear(PA_ALS_INT_CLR);
	return ret;
}

static void taos_work_func(struct work_struct *work)
{
	taos_get_data();

    /*enable the irq after process prox and als data*/
    if(taos_datap->client->irq)
    {
        enable_irq(taos_datap->client->irq);
        TMD27713_DEBUG("%s: enable irq\n",__func__);
    }
}

static int taos_als_get_data(void)
{
	int ret = 0;
	int lux_val = 0;

	/*don't need check control reg and status, delete them*/

	if ((lux_val = taos_get_lux()) < 0)
		printk(KERN_ERR
		       "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n",
		       lux_val);
	input_report_abs(taos_datap->als_input_dev, ABS_MISC, lux_val);
	input_sync(taos_datap->als_input_dev);

	return ret;
}
static int taos_als_threshold_set(void)
{
	int i, ret = 0;
	u8 chdata[2];
	u16 ch0;

	for (i = 0; i < 2; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_ALS_CHAN0LO + i);
	}

	ch0 = chdata[0] + chdata[1] * 256;
	als_threshold_hi_param = (12 * ch0) / 10;
	if (als_threshold_hi_param >= 65535)
		als_threshold_hi_param = 65535;
	als_threshold_lo_param = (8 * ch0) / 10;
	als_buf[0] = als_threshold_lo_param & 0x0ff;
	als_buf[1] = als_threshold_lo_param >> 8;
	als_buf[2] = als_threshold_hi_param & 0x0ff;
	als_buf[3] = als_threshold_hi_param >> 8;

	for (mcount = 0; mcount < 4; mcount++) {
		if ((ret = taos_write_byte(taos_datap->client,
				     TAOS_TRITON_ALS_MINTHRESHLO + mcount, als_buf[mcount])) < 0) {
			printk(KERN_ERR
			       "TAOS: write als failed in taos als threshold set\n");
			return (ret);
		}
	}
	return ret;
}
static int taos_prox_threshold_set(void)
{
	int i, ret = 0;
	u8 chdata[6];
	u16 proxdata = 0;
	int data = 0;
	for (i = 0; i < 6; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_ALS_CHAN0LO + i);
	}
	proxdata = chdata[4] | (chdata[5] << 8);
	/*don't use prox_on var, delete it*/
	if (proxdata < taos_cfgp->prox_threshold_lo) {
/*set prox_threshold_hi = 500*/
		pro_buf[0] = 0x00;
		pro_buf[1] = 0x00;
		pro_buf[2] = 0xF4;
		pro_buf[3] = 0x01;
		data = 1;
		input_report_abs(taos_datap->ps_input_dev, ABS_DISTANCE, data);
		input_sync(taos_datap->ps_input_dev);
	} else if (proxdata > taos_cfgp->prox_threshold_hi) {
/*set prox_threshold_lo = 400*/
		pro_buf[0] = 0x90;
		pro_buf[1] = 0x01;
		pro_buf[2] = 0xff;
		pro_buf[3] = 0xff;
		data = 0;
		input_report_abs(taos_datap->ps_input_dev, ABS_DISTANCE, data);
		input_sync(taos_datap->ps_input_dev);
	}

	for (mcount = 0; mcount < 4; mcount++) {
		if ((ret = taos_write_byte(taos_datap->client,
				     TAOS_TRITON_PRX_MINTHRESHLO + mcount, pro_buf[mcount])) < 0) {
			printk(KERN_ERR"%s: wirte the proximity threshold is faild\n",
			       __func__);
			return (ret);
		}
	}

	/*don't use the var, delete it*/
	return ret;
}

/* ATTR  */
static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Chip: TAOS %s\nVersion: %s\n",
		       TAOS_SENSOR_TMD2771_NAME, TAOS_SENSOR_INFO);
}

static DEVICE_ATTR(info, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   info_show, NULL);

static ssize_t enable_als_store(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	if (strncmp(buf, "1", 1) == 0) {
        if(taos_datap->als_enable)
        {
			/*if als already enable, just return*/
            TMD27713_DEBUG("%s: als already enable, just return\n",__func__);
            return count;
        }
        TMD27713_DEBUG("%s: als enable before sensor_on_flag = %d\n",__func__,sensor_on_flag);
		if (sensor_on_flag == 0) {
            init_ok = taos_regs_init(taos_datap);
			/*if the first enable, should enable irq and enable the ic*/
			enable_irq(taos_datap->client->irq);
            taos_reg_enable(taos_datap, SENSOR_POWER, 1);
            mdelay(5);
		}
        else
        {
            if(init_ok < 0)
            {
                init_ok = taos_regs_init(taos_datap);
	            taos_reg_enable(taos_datap, SENSOR_POWER, 1);
	            mdelay(5);
            }
        }
		/*enable the als reg*/
		taos_reg_enable(taos_datap, ALS_ENABLE, 1);
        sensor_on_flag++;
        taos_datap->als_enable = 1;
	} 
    else 
	{
  		if(!taos_datap->als_enable)
		{
			/*if als is already disable, just return*/
            TMD27713_DEBUG("%s: als already disable, just return\n",__func__);
            return count;
		}
        
        TMD27713_DEBUG("%s: als disable before sensor_on_flag = %d\n",__func__,sensor_on_flag);
	    sensor_on_flag--;
		/*disable the als reg*/
        taos_reg_enable(taos_datap, ALS_ENABLE, 0);
        taos_datap->als_enable = 0;
        
		if (sensor_on_flag == 0) {
			/*if no user for the ic, we disable irq and ic*/
			  disable_irq(taos_datap->client->irq);
              taos_reg_enable(taos_datap, SENSOR_POWER, 0);
		}
	}

	return count;
}

static ssize_t enable_als_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->als_enable);
}

static DEVICE_ATTR(enable_als_sensor, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   enable_als_show, enable_als_store);

static ssize_t enable_ps_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (strncmp(buf, "1", 1) == 0) {
        if(taos_datap->ps_enable)
        {
			/*if prox already enable,just return*/
           TMD27713_DEBUG("%s: prox already enable, just return\n",__func__);
            return count;
        }
        TMD27713_DEBUG("%s: prox enable before sensor_on_flag = %d\n",__func__,sensor_on_flag);
        if (sensor_on_flag == 0) {
            init_ok = taos_regs_init(taos_datap);
			/*if the first open ic, should enable irq and ic*/
			enable_irq(taos_datap->client->irq);
            taos_reg_enable(taos_datap, SENSOR_POWER, 1);
            mdelay(5);
		}
        else
        {
            if(init_ok < 0)
            {
                init_ok = taos_regs_init(taos_datap);
	            taos_reg_enable(taos_datap, SENSOR_POWER, 1);
	            mdelay(5);
            }
        }
		/*enable prox reg*/
		taos_reg_enable(taos_datap, PROX_ENABLE, 1);
        sensor_on_flag++;
        taos_datap->ps_enable = 1;
	} else {
		if(!taos_datap->ps_enable)
		{
			/*if prox already disable, just return*/
            TMD27713_DEBUG("%s: prox alwaredy disable, just return\n",__func__);
            return count;
		}
        TMD27713_DEBUG("%s: close prox before sensor_on_flag = %d\n",__func__,sensor_on_flag);
	    sensor_on_flag--;

		/*disable prox reg*/
        taos_reg_enable(taos_datap, PROX_ENABLE, 0);
        taos_datap->ps_enable = 0;
        
		if (sensor_on_flag == 0) {
			/*if als already disable, we can disable irq and ic*/
			  disable_irq(taos_datap->client->irq);
              taos_reg_enable(taos_datap, SENSOR_POWER, 0);
		}
	}
	return count;
}

static ssize_t enable_ps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->ps_enable);
}

static DEVICE_ATTR(enable_ps_sensor, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		   enable_ps_show, enable_ps_store);

static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct taos_prox_info prxp;

	memset(&prxp, 0, sizeof(prxp));
	taos_prox_poll(&prxp);

	return sprintf(buf, "%d\n", prxp.prox_data);
}

static DEVICE_ATTR(raw_adc, S_IRUGO, ps_adc_show, NULL);
/*add interface to show Hi and Lo value, and control and status reg value for prox*/
static ssize_t ps_Hi_Lo_s_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int Hi,Lo,status,ctrl;
    u8 reg = 0;

    reg = taos_read_byte(taos_datap->client, TAOS_TRITON_PRX_MINTHRESHLO);
    Lo = reg;
    reg = taos_read_byte(taos_datap->client, TAOS_TRITON_PRX_MINTHRESHHI);
    Lo = Lo | (reg << 8);
    
    reg = taos_read_byte(taos_datap->client, TAOS_TRITON_PRX_MAXTHRESHLO);
    Hi = reg;
    reg = taos_read_byte(taos_datap->client, TAOS_TRITON_PRX_MAXTHRESHHI);
    Hi = Hi | (reg << 8);

    status = taos_read_byte(taos_datap->client,TAOS_TRITON_STATUS);
    ctrl = taos_read_byte(taos_datap->client,TAOS_TRITON_CNTRL);

	return sprintf(buf, "status=0x%02x, control=0x%02x, Hi=%d, Lo=%d\n", status,ctrl,Hi,Lo);
}

static DEVICE_ATTR(hi_lo_s, S_IRUGO, ps_Hi_Lo_s_show, NULL);


static ssize_t als_lux_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int lux;

	lux = taos_get_lux();

	return sprintf(buf, "%d\n", lux);
}

static DEVICE_ATTR(lux_adc, S_IRUGO, als_lux_show, NULL);

static struct attribute *tmd2771_als_attributes[] = {
	&dev_attr_info.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_lux_adc.attr,
	NULL
};

static struct attribute_group tmd2771_als_attribute_group = {
	.attrs = tmd2771_als_attributes,
};

static struct attribute *tmd2771_ps_attributes[] = {
	&dev_attr_info.attr,
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_raw_adc.attr,
	&dev_attr_hi_lo_s.attr,
	NULL
};
static struct attribute_group tmd2771_ps_attribute_group = {
	.attrs = tmd2771_ps_attributes,
};

/* ATTR end */
// driver init
static int __init taos_init(void)
{
/*don't use device node, only use sys interface, delete those code*/
return i2c_add_driver(&taos_driver);
}

// driver exit
static void __exit taos_exit(void)
{
/*don't use device node, only use sys interface, delete those code*/
    i2c_del_driver(&taos_driver);
}
static int tmd_parse_dt(struct device *dev,
				struct tmd2771x_platform_data *tmd_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	rc = of_property_read_u32(np, "taos,pdrive", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read min-interval\n");
		return rc;
	} else {
		tmd_pdata->pdrive = (u8)temp_val;
        printk("wxl pdrive = %d\n",tmd_pdata->pdrive);
	}

	rc = of_property_read_u32(np, "taos,ppcount", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		tmd_pdata->ppcount = (u8)temp_val;
	}

	rc = of_property_read_u32(np, "taos,irq_gpio", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read axis-map_x\n");
		return rc;
	} else {
		tmd_pdata->irq_gpio = temp_val;
	}

	rc = of_property_read_u32(np, "taos,real_i2c_addr", &temp_val);
	if (rc && (rc != -EINVAL)) {
		tmd_pdata->real_i2c_addr = 0;
	} else {
		tmd_pdata->real_i2c_addr = (u16)temp_val;
	}

	return 0;
}
/*
struct tmd2771x_platform_data tmd_pdata = {
	.pdrive = 0x03,
	.ppcount = 0x08,
	.irq_gpio = 80,
	.setup_resources = NULL,
	.release_resources = NULL,
};
*/
// client probe
static int taos_probe(struct i2c_client *clientp,
		      const struct i2c_device_id *idp)
{
	int ret = 0;
	int i = 0;
	unsigned char buf[TAOS_MAX_DEVICE_REGS];
	char *device_name;
	unsigned short backup_org_i2c_adr = clientp->addr;
	pr_info("%s:%d tmd27713 probe start\n", __func__, __LINE__);

    taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos_datap) {
		printk(KERN_ERR
		       "TAOS: kmalloc for struct taos_data failed in taos_init()\n");
		ret = -ENOMEM;
		return ret;
	}
	memset(taos_datap, 0, sizeof(struct taos_data));

	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus byte data functions unsupported\n");
        kfree(taos_datap);
		return -EOPNOTSUPP;
	}
	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus word data functions unsupported\n");
	}
	if (!i2c_check_functionality
	    (clientp->adapter, I2C_FUNC_SMBUS_BLOCK_DATA)) {
		printk(KERN_ERR
		       "TAOS: taos_probe() - i2c smbus block data functions unsupported\n");
	}
	taos_datap->client = clientp;
	i2c_set_clientdata(clientp, taos_datap);

    mutex_init(&taos_datap->date_lock);

    taos_datap->pdata = kzalloc(sizeof(*(taos_datap->pdata)), GFP_KERNEL);
    if (!taos_datap->pdata){
        dev_err(&clientp->dev, "failed to allocate memory for module data\n");
        kfree(taos_datap);
        return -ENOMEM;
    }

    if (clientp->dev.of_node) {
		memset(taos_datap->pdata, 0 , sizeof(*(taos_datap->pdata)));
		ret = tmd_parse_dt(&clientp->dev, taos_datap->pdata);
		if (ret) {
			dev_err(&clientp->dev,
				"Unable to parse platfrom data err=%d\n", ret);
			goto err_parse_dt;
		}
    }

    //check i2c addr
    if((taos_datap->pdata->real_i2c_addr != 0) && taos_datap->pdata->real_i2c_addr != clientp->addr)
    {   
        pr_info("%s: change i2c addr from 0x%02x to 0x%02x\n", __func__, clientp->addr, taos_datap->pdata->real_i2c_addr);  
        /*
          we change the addr to the slave's real addr. 
          if probe fail, we don't change the clientp->addr.
        */
        clientp->addr = taos_datap->pdata->real_i2c_addr;
    }

    /*read chip regs info from i2c*/
    for (i = 0; i < TAOS_MAX_DEVICE_REGS; i++) {
		if ((buf[i] = taos_read_byte(clientp, TAOS_TRITON_CNTRL + i)) < 0) {
			printk("%s: read the every reg is faild\n", __func__);
			goto err_chip_id;
		}
	}
	
   /*compare chip-id*/
	if ((ret = taos_device_name(buf, &device_name)) == 0) {
		printk(KERN_ERR
		       "TAOS: chip id that was read found mismatched by taos_device_name(), in taos_probe()\n");
		ret = -ENODEV;
		goto err_chip_id;
	}

	if (strcmp(device_name, TAOS_DEVICE_ID)) {
		printk(KERN_ERR
		       "TAOS: chip id that was read does not match expected id in taos_probe()\n");
		ret = -ENODEV;
		goto err_chip_id;
	} else {
		pr_info("TAOS: chip id of %s that was read matches expected id in taos_probe()\n",
		     device_name);
	}

/*add i2c detect flag and app info when read chip id success.*/
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
		set_hw_dev_flag(DEV_I2C_APS);
		set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
		ret = app_info_set("LP-Sensor", TAOS_SENSOR_TMD2771_NAME);
		if (ret < 0) {/*failed to add app_info*/
		
			dev_err(&clientp->dev, "%s %d:faile to add app_info\n", __func__, __LINE__);
		}

	INIT_WORK(&(taos_datap->work), taos_work_func);
	taos_datap->als_input_dev = input_allocate_device();
	if (taos_datap->als_input_dev == NULL) {
		printk("[tmd27713 als_input_dev] probe error\n");
		goto err_chip_id;
	}
	taos_datap->ps_input_dev = input_allocate_device();
	if (taos_datap->ps_input_dev == NULL) {
		printk("[tmd27713 ps_input_dev] probe error\n");
		goto err_chip_id;
	}

	taos_datap->als_input_dev->name = "lightsensor-level";
	taos_datap->ps_input_dev->name = "proximity";
	taos_datap->als_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, taos_datap->als_input_dev->evbit);
	taos_datap->ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, taos_datap->ps_input_dev->evbit);
	input_set_capability(taos_datap->ps_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_capability(taos_datap->als_input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(taos_datap->als_input_dev, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(taos_datap->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	//check i2c addr
        //we change the addr to the slave's real addr. if probe fail, we don't change the clientp->addr.    
	
	
/*compare chip-id*/

	ret = input_register_device(taos_datap->als_input_dev);
	ret = input_register_device(taos_datap->ps_input_dev);
	/*remove check chip id code up to input devices register*/

	strlcpy(clientp->name, TAOS_DEVICE_ID, I2C_NAME_SIZE);
	strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);


	if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL))) {
		printk(KERN_ERR
		       "TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
		ret = -ENOMEM;
		goto err_unregister_input_dev;
	}

	taos_cfgp->calibrate_target = calibrate_target_param;
	taos_cfgp->als_time = als_time_param;
	taos_cfgp->scale_factor = scale_factor_param;
	taos_cfgp->gain_trim = gain_trim_param;
	taos_cfgp->filter_history = filter_history_param;
	taos_cfgp->gain = gain_param;
	taos_cfgp->als_threshold_hi = als_threshold_hi_param;
	taos_cfgp->als_threshold_lo = als_threshold_lo_param;
	taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
	taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
	taos_cfgp->prox_int_time = prox_int_time_param;
	taos_cfgp->prox_adc_time = prox_adc_time_param;
	taos_cfgp->prox_wait_time = prox_wait_time_param;
	taos_cfgp->prox_intr_filter = prox_intr_filter_param;
	taos_cfgp->prox_config = prox_config_param;
	taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
	taos_cfgp->prox_gain = prox_gain_param;
	sat_als = (256 - taos_cfgp->prox_int_time) << 10;
	sat_prox = (256 - taos_cfgp->prox_adc_time) << 10;

	/*init regs*/
	taos_regs_init(taos_datap);

	printk("[ tmd27713]clientp->irq = %d\n", clientp->irq);

	ret = gpio_request(taos_datap->pdata->irq_gpio, "taos_irq");
	if (ret) {
		printk(KERN_ALERT "%s: gp2ap request gpio failed.\n",
		       __func__);
		goto err_taos_cfgp_kfree;
	}
	gpio_tlmm_config(GPIO_CFG(taos_datap->pdata->irq_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
			GPIO_CFG_ENABLE);
	ret = request_irq(clientp->irq, taos_irq_handler, IRQ_TYPE_EDGE_FALLING,
			"taos_irq", taos_datap);
	printk("[ tmd27713]request_irq ret = %d\n", ret);

	if (ret != 0) {
		printk("\nrequest tmd27713 irq : %d failed\n", clientp->irq);
		goto err_taos_cfgp_kfree;
	}

	printk("request tmd27713 irq : %d succeed\n", clientp->irq);
	disable_irq(clientp->irq);

	if (taos_datap == NULL) {
		printk("[tmd27713] taos_datap == NULL\n");
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	taos_datap->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	taos_datap->early_suspend.suspend = taos_early_suspend;
	taos_datap->early_suspend.resume = taos_late_resume;
	register_early_suspend(&taos_datap->early_suspend);
#endif

	ret = sysfs_create_group(&taos_datap->als_input_dev->dev.kobj,
			       &tmd2771_als_attribute_group);
	ret = sysfs_create_group(&taos_datap->ps_input_dev->dev.kobj,
			       &tmd2771_ps_attribute_group);
	/*create a single workqueue like 8930 and 8x25Q*/
    aps_wq = create_singlethread_workqueue("aps_wq");
    if(!aps_wq)
    {
        ret = -ENOMEM;
        goto err_taos_cfgp_kfree;
    }
	if (!ret)
		goto exit;

 err_taos_cfgp_kfree:
	kfree(taos_cfgp);
 err_unregister_input_dev:
	input_unregister_device(taos_datap->als_input_dev);
	input_unregister_device(taos_datap->ps_input_dev);
 err_chip_id:
    clientp->addr = backup_org_i2c_adr;
 err_parse_dt:
    kfree(taos_datap->pdata);
    kfree(taos_datap);
 exit:
	pr_info("%s: probe end ret = %d\n",__func__, ret);
	return ret;
}

// client remove
static int __devexit taos_remove(struct i2c_client *client)
{
    if(taos_cfgp)
    {
	    kfree(taos_cfgp);
    }
    
	unregister_early_suspend(&taos_datap->early_suspend);
	input_unregister_device(taos_datap->als_input_dev);
	sysfs_remove_group(&taos_datap->als_input_dev->dev.kobj,
			   &tmd2771_als_attribute_group);
	input_unregister_device(taos_datap->ps_input_dev);
	sysfs_remove_group(&taos_datap->ps_input_dev->dev.kobj,
			   &tmd2771_ps_attribute_group);
	/*don't need config regulator, delete it*/
	if(taos_datap && taos_datap->pdata)
	{
	    kfree(taos_datap->pdata);
	}

    if(taos_datap)
    {
	    kfree(taos_datap);
    }
	/*remove workqueue*/
    if (aps_wq)
    {
        destroy_workqueue(aps_wq);
    }
	return 0;
}

//resume
#ifdef CONFIG_HAS_EARLYSUSPEND
static u8 reg_store = 0;
static void taos_late_resume(struct early_suspend *h)
{
	u8 reg_val = 0;
	int ret = -1;

	if ((reg_val = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk("TAOS: read byte is failed in resume\n");
		return;
	}
	if (taos_datap->working == 1) {
		taos_datap->working = 0;
		if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL, reg_store)) < 0) {	//reg_cntrl)) < 0) {
			printk(KERN_ERR
			       "TAOS: write byte_data failed in ioctl als_off\n");
			return;
		}
	}
	/*if als or prox enable, then enable irq*/
    if(sensor_on_flag)
    {
        enable_irq(taos_datap->client->irq);
    }
}

//suspend
static void taos_early_suspend(struct early_suspend *h)
{
	u8 reg_val = 0;
	int ret = -1;
	/*if als or prox enable, then disable irq*/
    if(sensor_on_flag)
    {
	    disable_irq(taos_datap->client->irq);
    }
    taos_datap->working = 1;
    
	if ((reg_store = taos_read_byte(taos_datap->client, TAOS_TRITON_CNTRL)) < 0) {
		printk("TAOS: read byte is failed in suspend\n");
		return;
	}
	if ((ret = taos_write_byte(taos_datap->client, TAOS_TRITON_CNTRL,
			     reg_val)) < 0) {
		printk(KERN_ERR "TAOS: write byte failed in taos_suspend\n");
		return;
	}
}
#endif
/*don't use the device fops interface, we use sys interface, delete these functions*/

// read/calculate lux value
static int taos_get_lux(void)
{
	u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0;
	u32 ratio = 0;
	u8 dev_gain = 0;
	u16 Tint = 0;
	struct lux_data *p;
	int ret = 0;
	u8 chdata[4];
	int tmp = 0, i = 0;

	for (i = 0; i < 4; i++) {
		if ((chdata[i] = (taos_read_byte
		      (taos_datap->client, TAOS_TRITON_ALS_CHAN0LO + i))) < 0) {
			printk(KERN_ERR
			       "TAOS: read chan0lo/li failed in taos_get_lux()\n");
			return (ret);
		}
		TMD27713_DEBUG("--GGG--chdata is %d\n",chdata[i]);
	}

	/*if atime =100  tmp = (atime+25)/50=2.5   tine = 2.7*(256-atime)=  412.5*/
	tmp = (taos_cfgp->als_time + 25) / 50;
	TritonTime.numerator = 1;
	TritonTime.denominator = tmp;

	//tmp = 300*atime  400
	tmp = 300 * taos_cfgp->als_time;
	if (tmp > 65535)
		tmp = 65535;
	TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir = chdata[3];
	raw_ir <<= 8;
	raw_ir |= chdata[2];

	raw_clear *= (taos_cfgp->scale_factor);
	raw_ir *= (taos_cfgp->scale_factor);

	if (raw_ir > raw_clear) {
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}
	dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
	if (raw_clear >= lux_timep->saturation)
		return (TAOS_MAX_LUX);
	if (raw_ir >= lux_timep->saturation)
		return (TAOS_MAX_LUX);
	if (raw_clear == 0)
		return (0);
	if (dev_gain == 0 || dev_gain > 127) {
		printk(KERN_ERR
		       "TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
		return -1;
	}
	if (lux_timep->denominator == 0) {
		printk(KERN_ERR
		       "TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
		return -1;
	}
	ratio = (raw_ir << 15) / raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++) ;
	if (!p->ratio) {
		if (lux_history[0] < 0)
			return 0;
		else
			return lux_history[0];
	}
	Tint = taos_cfgp->als_time;
	raw_clear = ((raw_clear * 400 + (dev_gain >> 1)) / dev_gain +
			(Tint >> 1)) / Tint;
	raw_ir = ((raw_ir * 400 + (dev_gain >> 1)) / dev_gain + (Tint >> 1)) / Tint;
	lux = ((raw_clear * (p->clear)) - (raw_ir * (p->ir)));
	lux = (lux + 32000) / 64000;
	if (lux > TAOS_MAX_LUX) {
		lux = TAOS_MAX_LUX;
	}
	return (lux);
}

// verify device
static int taos_device_name(unsigned char *bufp, char **device_name)
{
	if ((bufp[0x12] & 0xf0) == 0x00)
		return (0);
	if (bufp[0x10] | bufp[0x1a] | bufp[0x1b] | bufp[0x1c] | bufp[0x1d] |
	    bufp[0x1e])
		return (0);
	if (bufp[0x13] & 0x0c)
		return (0);
	*device_name = "tmd27713";
	return (1);
}

// proximity poll
static int taos_prox_poll(struct taos_prox_info *prxp)
{
	int i = 0, ret = 0;
	u8 chdata[6];
	for (i = 0; i < 6; i++) {
		chdata[i] = taos_read_byte(taos_datap->client,
				   TAOS_TRITON_CMD_AUTO |
				   (TAOS_TRITON_ALS_CHAN0LO + i));
		if (chdata[i] < 0) {
			printk("ERROR : taos_read_byte chdata[%d]\n", i);
		}
	}
	prxp->prox_clear = chdata[1];
	prxp->prox_clear <<= 8;
	prxp->prox_clear |= chdata[0];
	prxp->prox_data = chdata[5];
	prxp->prox_data <<= 8;
	prxp->prox_data |= chdata[4];

	return (ret);
}

MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(taos_init);
module_exit(taos_exit);
