/* drivers/input/misc/aps-12d.c
 *
 * Copyright (C) 2010 HUAWEI, Inc.
 * Author: Benjamin Gao <gaohuajiang@huawei.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/aps_avago_9930.h>
#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/light.h>
#include <linux/sensors.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <misc/app_info.h>
#include <linux/gpio.h>

#undef PROXIMITY_DB
#ifdef PROXIMITY_DB
#define PROXIMITY_DEBUG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define PROXIMITY_DEBUG(fmt, args...)
#endif

static struct workqueue_struct *aps_wq;

struct aps_data {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct mutex  mlock;
    struct hrtimer timer;
    struct work_struct  work;
	/* delete user_irq */
    int (*power)(int on);
};

/*code unitary*/
/*lsensor is for all the product,different phone has different table in .h */
/*change uint16_t to int*/
static int lsensor_adc_table[LSENSOR_MAX_LEVEL] = {
	8, 25, 110, 400, 750, 1200, 3000
};
/* add the macro of log */
static int aps9930_debug_mask;
module_param_named(aps9930_debug, aps9930_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* the two parameters are in /sys/module/aps_9930/parameters */
static int apds_9930_pwindows_value = 200;
static int apds_9930_pwave_value = 100; 
static int apds_9930_fall_prop = SENSOR_PROP_NONE;
module_param_named(apds_9930_pwindows_value, apds_9930_pwindows_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(apds_9930_pwave_value, apds_9930_pwave_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define APS9930_DBG(x...) do {\
    if (aps9930_debug_mask) \
        printk(KERN_DEBUG x);\
    } while (0)

struct aps_init_regdata{
      int reg;
      uint8_t data;
};
static struct aps_data  *this_aps_data;

//delete p_h and p_l
struct input_dev *sensor_9930_dev=NULL;
static int aps_9930_delay = 1000;     /*1s*/
/* delete this part */
static int aps_first_read = 1;
/* use this to make sure which device is open and make a wake lcok*/
static int aps_open_flag=0;
/* decline the default min_proximity_value */
static int origin_prox = 822;
static int min_proximity_value;
static int light_device_minor = 0;
static int proximity_device_minor = 0;
static struct wake_lock proximity_wake_lock;
static atomic_t l_flag;
static atomic_t p_flag;
/* to be same with ICS, move the values up */
static int proximity_data_value = 0;
static int light_data_value = 0;
static int irdata = 0;/* ch1 data  */
static int sunlight = 0;
/*init the value of reg 0*/
static u8  reg0_value = 0x38; 

/*default value,75% of maximum range*/
#define PX_SUNLIGHT_DEFAULT 30000

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7
static char light_device_id[] = "AVAGO-TAOS-9930";
static atomic_t px_status_flag;
/* an arithmometer to mark how much times the aps device opened*/
static int open_count = 0;
/*modify the value*/
/* removed some lines. */

static struct aps_init_regdata aps9930_init_regdata[]=
{
    {APDS9930_ENABLE_REG, 0x0},
    {APDS9930_ATIME_REG,   0xdb},
    {APDS9930_PTIME_REG,   0xff},
    /*modify the value*/
    {APDS9930_WTIME_REG,  0xb6},
    /* modify the ppcount from 8 to 4 */
	/* change value frome 0x04 to 0x0C */
	{APDS9930_PPCOUNT_REG, 0xC},
    {APDS9930_CONTROL_REG, 0x60},
    {APDS9930_ENABLE_REG, 0x38},
    {APDS9930_PERS_REG, 0x12}
};

/* Coefficients in open air: 
  * GA:Glass (or Lens) Attenuation Factor
  * DF:Device Factor
  * alsGain: ALS Gain
  * aTime: ALS Timing
  * ALSIT = 2.72ms * (256 ¨C ATIME) = 2.72ms * (256-0xDB) =  100ms
  */
static int aTime = 0xDB; 
static int alsGain = 1;
/* modify the parameter by FAE */
/*
static int ga=48;
static int coe_b=223;
static int coe_c=7;
static int coe_d=142;
*/
/* modify the parameter */
static int ga=515;
static int coe_b=1948;
static int coe_c=613;
static int coe_d=1163;
static int DF=52;


static int  set_9930_register(struct aps_data  *aps, u8 reg, u16 value, int flag)
{
    int ret;

    mutex_lock(&aps->mlock);
    /*  flag=1 means reading the world value */
    if (flag)
    {
        ret = i2c_smbus_write_word_data(aps->client, CMD_WORD | reg, value);
    }
    /*  flag=0 means reading the byte value */
    else
    {
        ret = i2c_smbus_write_byte_data(aps->client, CMD_BYTE | reg, (u8)value);
    }

    mutex_unlock(&aps->mlock);
	/* delete some lines */

    return ret;
}
static int get_9930_register(struct aps_data  *aps, u8 reg, int flag)
{
    int ret;

    mutex_lock(&aps->mlock);
    if (flag)
    {
        ret = i2c_smbus_read_word_data(aps->client, CMD_WORD | reg);
    }
    else
    {
        ret = i2c_smbus_read_byte_data(aps->client, CMD_BYTE | reg);
    }
    mutex_unlock(&aps->mlock);
    if (ret < 0)
    {
        pr_err("%s, line %d: read register fail!(reg=0x%x, flag=%d, ret=0x%x)\n", __func__, __LINE__, reg, flag, ret);
    }
    return ret;
}
static int aps_9930_open(struct inode *inode, struct file *file)
{ 
    int ret = 0;
    int i = 0;
    /* when the device is open use this if light open report -1 when proximity open then lock it*/
    if( light_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s:light sensor open\n", __func__);
		/*it's not necessary to set this flag everytime*/
        /*aps_first_read = 1;*/
    }
    if( proximity_device_minor == iminor(inode) )
    {
        if((PRXIMITY_OPEN_INIT == apds_9930_fall_prop)&&(PROXIMITY_SUSPEND == atomic_read(&px_status_flag)))
        {
            /*Re-init the register for avoid hardware drop experiment*/
            for ( i=0; i< ARRAY_SIZE(aps9930_init_regdata);i++)
            {
                ret = set_9930_register(this_aps_data, CMD_BYTE|aps9930_init_regdata[i].reg, aps9930_init_regdata[i].data, 0);
                if (ret < 0)
                {
                    pr_err("%s,%d: Ret of init aps9930 regs(%d - %d) is %d\n", __func__, __LINE__, aps9930_init_regdata[i].reg, aps9930_init_regdata[i].data, get_9930_register(this_aps_data, aps9930_init_regdata[i].reg, 0));
                    break;
                }
            }
            PROXIMITY_DEBUG("%s:RE-init register happened\n", __func__);
        }
        /*When device working,Init register will make device down,
          So we use Atomic flag to avoid this happened*/
        atomic_set(&px_status_flag, PROXIMITY_RESUME);
        PROXIMITY_DEBUG("%s:proximity_device_minor == iminor(inode),get wake_lock\n", __func__);
        wake_lock( &proximity_wake_lock);
        /* 0 is close, 1 is far */
        input_report_abs(this_aps_data->input_dev, ABS_DISTANCE, 1);
        input_sync(this_aps_data->input_dev);
    }
    /* when open_count come to max, the aps device reset the value of min_proximity_value*/
    if( OPEN_COUNT_MAX == open_count )
    {
        min_proximity_value = origin_prox;
        open_count = 0;
    }
    open_count ++;
    APS9930_DBG(KERN_ERR "%s:flag is %d,open_count = %d\n", __func__,aps_open_flag,open_count);
    if (!aps_open_flag)
    {
        u8 value_reg0;
		/*set bit 0 of reg 0 ==1*/
		value_reg0 = reg0_value;
        /* if power on ,will not set PON=1 again */
        if (APDS9930_POWER_OFF == (value_reg0 & APDS9930_POWER_MASK))
        {
            value_reg0 = value_reg0 | APDS9930_POWER_ON;
            ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0 , 0);
            if (ret)
            {
                pr_err("%s,%d: set_9930_register is error(%d)!\n", __func__, __LINE__, ret);
            }
		     else
		     {
				reg0_value = value_reg0;
			 }
        }
        if (this_aps_data->client->irq)   
        {
            enable_irq(this_aps_data->client->irq);
        }
    }
    aps_open_flag++;

	return nonseekable_open(inode, file);
}

static int aps_9930_release(struct inode *inode, struct file *file)
{
    int ret;
    aps_open_flag--;
    aps_9930_delay = 1000;//1s
    if( proximity_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s: proximity_device_minor == iminor(inode),free wake_lock\n", __func__);
        wake_unlock( &proximity_wake_lock);
    }
    if( light_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s: light_device_minor == iminor(inode)\n", __func__);
    }
    if (!aps_open_flag)
    {
        int value_reg0;
		/*set bit 0 of reg 0 ==0*/
		value_reg0 = reg0_value & APDS9930_REG0_POWER_OFF;
        ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0, 0);
        if (ret)
        {
            pr_err("%s,%d:set_9930_register is error(%d)\n", __func__, __LINE__, ret);
        }
		else
		{
			reg0_value = value_reg0;
		}
        if (this_aps_data->client->irq) 
        {
            disable_irq(this_aps_data->client->irq);
        }
    }
    if(proximity_device_minor == iminor(inode))
        atomic_set(&px_status_flag, PROXIMITY_SUSPEND);
    return 0;
}
static long
aps_9930_ioctl(struct file *file, unsigned int cmd,
     unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag;
    int value_reg0;
    int set_flag;
    int ret;
	/*delete one line */
    PROXIMITY_DEBUG("%s:enter in aps_9930_ioctl function cmd = %d arg = %ld\n", __func__, cmd, arg);
    switch (cmd) 
    {
        case ECS_IOCTL_APP_SET_LFLAG:   /* app set  light sensor flag */
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                pr_err("%s,%d: copy_from_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            atomic_set(&l_flag, flag);
            set_flag = atomic_read(&l_flag) ? 1 : 0;

			/*set bit 1 of reg 0 by set_flag */
            /* set AEN,if l_flag=1 then enable ALS.if l_flag=0 then disable ALS */
			if (set_flag)
			{
				value_reg0 = reg0_value | (set_flag << APDS9930_AEN_BIT_SHIFT);
           		ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0, 0);
			}
			else
			{
				value_reg0 = reg0_value & APDS9930_REG0_AEN_OFF;
           		ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0, 0);
			}
            if (ret)
            {
                pr_err("%s,%d: set ECS_IOCTL_APP_SET_LFLAG flag is error(%d)\n", __func__, __LINE__, ret);
            }
			else
			{
				reg0_value = value_reg0;
			}
            break;
        }
        case ECS_IOCTL_APP_GET_LFLAG:  /*app  get open light sensor flag*/
        {
            flag = atomic_read(&l_flag);
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
        case ECS_IOCTL_APP_SET_PFLAG:
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                pr_err("%s,%d: copy_from_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            atomic_set(&p_flag, flag);
            set_flag = atomic_read(&p_flag) ? 1 : 0;
			/*set bit 1 of reg 0 by set_flag */
            /* set PEN,if p_flag=1 then enable proximity.if p_flag=0 then disable proximity */
			if (set_flag)
			{
				value_reg0 = reg0_value | (set_flag << APDS9930_PEN_BIT_SHIFT);
          		ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0, 0);
			}
			else
			{
				value_reg0 = reg0_value & APDS9930_REG0_PEN_OFF;
           		ret = set_9930_register(this_aps_data, APDS9930_ENABLE_REG, value_reg0, 0);
			}
            if (ret)
            {
                pr_err("%s,%d:set ECS_IOCTL_APP_SET_PFLAG flag is error(%d)\n", __func__, __LINE__, ret);
            }	
			else
			{
				reg0_value = value_reg0;
			}
            break;
        }
        case ECS_IOCTL_APP_GET_PFLAG:  /*get open acceleration sensor flag*/
        {
            flag = atomic_read(&p_flag);
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
        case ECS_IOCTL_APP_SET_DELAY:
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                pr_err("%s,%d: copy_from_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            if(flag)
                aps_9930_delay = flag;
            else
                aps_9930_delay = 20;   /*20ms*/
            break;
        }
        case ECS_IOCTL_APP_GET_DELAY:
        {
            flag = aps_9930_delay;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
		/*get value of proximity and light*/
		case ECS_IOCTL_APP_GET_PDATA_VALVE:
        {
            flag = proximity_data_value;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
		case ECS_IOCTL_APP_GET_LDATA_VALVE:
        {
            flag = light_data_value;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
		case ECS_IOCTL_APP_GET_APSID:
        {
            if (copy_to_user(argp, light_device_id, strlen(light_device_id)+1))
            {
                pr_err("%s,%d: copy_to_user function fails\n", __func__, __LINE__);
                return -EFAULT;
            }
            break;
        }
        default:
        {
            break;
        }
		
    }
    return 0;
}

static struct file_operations aps_9930_fops = {
    .owner = THIS_MODULE,
    .open = aps_9930_open,
    .release = aps_9930_release,
    .unlocked_ioctl = aps_9930_ioctl,
};

static struct miscdevice light_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "light",
    .fops = &aps_9930_fops,
};

static struct miscdevice proximity_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "proximity",
    .fops = &aps_9930_fops,
};

static int luxcalculation(int cdata, int irdata)
{
    int luxValue=0;
	int first_half, sec_half;
    int iac1=0;
    int iac2=0;
    int iac=0;
    PROXIMITY_DEBUG("%s:enter in luxcalculation function cdata = %d ,irdata = %d\n", __func__, cdata, irdata);
    /*
     *Lux Equation:
     *       IAC1 = CH0DATA -B * CH1DATA                (IAC:IR Adjusted Count)
     *       IAC2 = C * CH0DATA - D * CH1DATA
     *       IAC = Max (IAC1, IAC2, 0)
     *       LPC = GA * DF / (ALSIT * AGAIN)               (LPC:Lux per Count)
     *       Lux = IAC * LPC
     *Coefficients in open air:
     *       GA = 0.48
     *       B = 2.23
     *       C = 0.7
     *       D = 1.42
    */
	/*adjust data by parameter*/
    iac1 = (int) (cdata - (coe_b*irdata/1000));
    iac2 = (int) ((coe_c*cdata/1000) - (coe_d*irdata/1000));
    
    if (iac1 > iac2)
    {
        iac = iac1;
    }
    else if (iac1 <= iac2)
    {
        iac = iac2;
    }
    else
    {
        iac = 0;
    }
	
    first_half = iac*ga*DF/100;
    sec_half = ((272*(256-aTime))*alsGain)/100;
    luxValue = first_half/sec_half;
	 APS9930_DBG("first_half====%d  ,sec_half ===%d,iac===%d\n", first_half,sec_half,iac);
    return luxValue;
}

static void aps_9930_work_func(struct work_struct *work)
{
    int pdata = 0;/* proximity data*/
    /* change irdate to static, cdate to dynamic */
    int cdata = 0;/* ch0 data  */
    /*make irdata static*/
    int cdata_high = 0, cdata_low = 0;
    int lux; 
    int status;
    int ret;
    int als_level = 0;
    int i = 0;
    struct aps_data *aps = container_of(work, struct aps_data, work);

    status = get_9930_register(aps,APDS9930_STATUS_REG,0);
    /* proximity flag is open and the interrupt belongs to proximity */
    if (atomic_read(&p_flag) && (status & APDS9930_STATUS_PROXIMITY_BIT))
    {
        int pthreshold_h=0, pthreshold_l;
        /* read the proximity data  */
        APS9930_DBG("Into prox init! \n");
        pdata = get_9930_register(aps, APDS9930_PDATAL_REG, 1);
        /* protect the proximity fuction when the sunlight is very strong */
        if( pdata < 0 )
        {
            /* the number "200" is a value to make sure there is a valid value */
            pdata = 200 ;
            pr_err("%s, line %d: pdate<0, reset to %d\n", __func__, __LINE__, pdata);
        }
        /* delete this part ,because modify sunlight problem bring a new problem,so we should delete it*/
        /* add the arithmetic of setting the proximity thresholds automatically */

        if ((pdata + apds_9930_pwave_value) < min_proximity_value)
        {
            min_proximity_value = pdata + apds_9930_pwave_value;
            ret = set_9930_register(aps, APDS9930_PILTL_REG, min_proximity_value, 1);
            ret |= set_9930_register(aps, APDS9930_PIHTL_REG, (min_proximity_value + apds_9930_pwindows_value), 1);
            if (ret)
            {
                pr_err("%s, line %d: set APDS9930_PILTL_REG register is error(min=%d, window=%d, ret=%d)\n", \
                       __func__, __LINE__, min_proximity_value, apds_9930_pwindows_value, ret);
            }
            APS9930_DBG("%s:min_proximity_value=%d\n", __func__, min_proximity_value);
        }
        pthreshold_h = get_9930_register(aps, APDS9930_PIHTL_REG, 1);
        pthreshold_l = get_9930_register(aps, APDS9930_PILTL_REG, 1);
        /* add some logs */
        APS9930_DBG("%s:pdata=%d pthreshold_h=%d pthreshold_l=%d\n", __func__, pdata, pthreshold_h, pthreshold_l);
        /* clear proximity interrupt bit */
        ret = i2c_smbus_write_byte(aps->client,CMD_CLR_PS_INT);
        if (ret)
        {
            pr_err("%s, line %d: set_9930_register is error(%d),clear failed!\n", __func__, __LINE__, ret);
        }
		/*get value of proximity*/
         proximity_data_value = pdata;

        /* if more than the value of  proximity high threshold we set*/
        if (pdata >= pthreshold_h) 
        {
            /*move sunlight avoid to this part */
            APS9930_DBG("pdata = %d,irdata = %d\n",pdata,irdata);
            if(atomic_read(&l_flag))
            {
                if (( irdata > sunlight ))
                {
                    ret = i2c_smbus_write_byte(aps->client,CMD_CLR_PS_INT);
                    if (ret)
                    {
                        pr_err("%s,%d:set_9930_register is error(%d),clear failed!\n",__func__, __LINE__, ret);
                    }
                    else
                    {
                        PROXIMITY_DEBUG("%s:the sunlight is so strong that the proximity is influence, we filtrate it\n", __func__);
                    }
                    goto Light_init;
                }
            }
            ret = set_9930_register(aps, APDS9930_PILTL_REG, min_proximity_value, 1);
            ret |= set_9930_register(aps, APDS9930_PIHTL_REG, (MAX_ADC_PROX_VALUE + 1), 1);
            if (ret)
            {
                pr_err("%s, line %d: set APDS9930_PILTL_REG register is error(min=%d, ret=%d)!\n", __func__, __LINE__, min_proximity_value, ret);
            }
            input_report_abs(aps->input_dev, ABS_DISTANCE, 0);
            input_sync(aps->input_dev);
        }
        /* if less than the value of  proximity low threshold we set*/
        /* the condition of pdata==pthreshold_l is valid */
        else if (pdata <=  pthreshold_l)
        {
            ret = set_9930_register(aps, APDS9930_PIHTL_REG, (min_proximity_value + apds_9930_pwindows_value), 1);
            /*Warning,this scheme is not tested by batch production*/
            //ret = set_9930_register(aps, APDS9930_PILTL_REG, 0, 1);
            if (ret)
            {
                pr_err("%s, line %d: set APDS9930_PILTL_REGs register is error(%d)!\n", __func__, __LINE__, ret);
            }
            input_report_abs(aps->input_dev, ABS_DISTANCE, 1);
            input_sync(aps->input_dev);
        }
        /*on 27a platform ,bug info is a lot*/
        else
        {
            pr_err("%s, line %d: Wrong status!\n",  __func__, __LINE__);
            ret = set_9930_register(aps, APDS9930_PILTL_REG, min_proximity_value, 1);
            if (ret)
            {
                pr_err("%s, line %d: set APDS9930_PILTL_REG register is error(%d)!\n", __func__, __LINE__, ret);
            }
        }
        pthreshold_h = get_9930_register(aps, APDS9930_PIHTL_REG, 1);
        pthreshold_l = get_9930_register(aps, APDS9930_PILTL_REG, 1);
        //delete p_h and p_l
        APS9930_DBG("%s:min = %d,prox_window = %d\n",__func__,min_proximity_value,apds_9930_pwindows_value);
        APS9930_DBG("%s:after reset the pdata=%d pthreshold_h=%d pthreshold_l=%d\n", __func__, pdata, pthreshold_h, pthreshold_l);
    }
    /* p_flag is close, and no proximity interrupt: normal, just add for debug */ 
    else if ( !atomic_read(&p_flag) && !(status & APDS9930_STATUS_PROXIMITY_BIT) )
    {
        APS9930_DBG("%s, line %d: [APS_OK]p_flag is close and no prox interrupt(status=0x%x).\n", __func__, __LINE__, status);
    }
    /* p_flag is open, but no proximity interrupt: show registers value */
    else if (atomic_read(&p_flag) && !(status & APDS9930_STATUS_PROXIMITY_BIT) )
    {
        int pthreshold_h = 0;
        int pthreshold_l = 0;
        
        /* get pdata, p_h, p_l value from registers */
        pdata        = get_9930_register(aps, APDS9930_PDATAL_REG, 1);
        pthreshold_h = get_9930_register(aps, APDS9930_PIHTL_REG, 1);
        pthreshold_l = get_9930_register(aps, APDS9930_PILTL_REG, 1);
        
        /* normal */
        if ( (0 == pthreshold_l && pdata < pthreshold_h)       /* near, but less than pthreshold_h  */
            || (pthreshold_l > 0 && pdata > pthreshold_l) )    /* far, but bigger than pthreshold_l */
        {
            APS9930_DBG("%s, line %d: [APS_OK]p_flag is open, but no prox int(STATUS=0x%x,ENABLE=0x%x,PDATA=%d,PILT=%d,PIHT=%d)\n", \
                        __func__, __LINE__, status, get_9930_register(aps,APDS9930_ENABLE_REG,0), pdata, pthreshold_l, pthreshold_h);
        }
        /* abnormal */
        else
        {
            int reg_enable = get_9930_register(aps,APDS9930_ENABLE_REG,0);
            pr_err("%s, line %d: [APS_ERR]p_flag is open, but no prox int(STATUS=0x%x,ENABLE=0x%x,PDATA=%d,PILT=%d,PIHT=%d)\n", \
                   __func__, __LINE__, status, reg_enable, pdata, pthreshold_l, pthreshold_h);
        }
    }
    /* p_flag is close, but raise proximity interrupt: abnormal, clear the prox interrupt bit */ 
    else if ( !atomic_read(&p_flag) && (status & APDS9930_STATUS_PROXIMITY_BIT) )
    {
        /* clear proximity interrupt bit */
        ret = i2c_smbus_write_byte(aps->client,CMD_CLR_PS_INT);
        if (ret)
        {
            pr_err("%s, line %d: clear proximity interrupt bit failed(%d)!\n", __func__, __LINE__, ret);
        }
        else
        {
            pr_err("%s, line %d: p_flag is close, but raise prox interrupt, clear prox interrupt bit.\n", __func__, __LINE__);
        }
    }
Light_init:
    /* ALS flag is open and the interrupt belongs to ALS */
    if (atomic_read(&l_flag) && (status & APDS9930_STATUS_ALS_BIT)) 
    {
        /* read the CH0 data and CH1 data  */
        APS9930_DBG("into light_init!!\n");
        cdata = get_9930_register(aps, APDS9930_CDATAL_REG, 1);
        irdata = get_9930_register(aps, APDS9930_IRDATAL_REG, 1);
        /* set ALS high threshold = ch0(cdata) + 20%,low threshold = ch0(cdata) - 20% */
        cdata_high = (cdata *  600)/500;
        cdata_low = (cdata *  400)/500;
        /* the max value of cdata_high == 0xffff */
        if(0xffff <= cdata_high )
        {
            cdata_high=0xffff;
            pr_err("%s, line %d: 0xffff <= cdata_high, reset to 0x%x!\n",  __func__, __LINE__, cdata_high);
        }
        /* clear als interrupt bit */
        ret = i2c_smbus_write_byte(aps->client,CMD_CLR_ALS_INT);
        ret |= set_9930_register(aps, APDS9930_AILTL_REG, cdata_low, 1);
        ret |= set_9930_register(aps, APDS9930_AIHTL_REG, cdata_high, 1);
        if (ret)
        {
            pr_err("%s, line %d: set APDS9930_AILTL_REG register is error(cdata_low=%d,cdata_high=%d,ret=%d)!\n", __func__, __LINE__, cdata_low, cdata_high, ret);
        }
        /* convert the raw pdata and irdata to the value in units of lux */
        lux = luxcalculation(cdata, irdata);
        APS9930_DBG("%s:cdata=%d irdata=%d lux=%d\n",__func__, cdata, irdata, lux);
        if (lux >= 0) 
        {
            /*get value of light*/
             light_data_value = lux;
            /* lux=0 is valid */
            als_level = LSENSOR_MAX_LEVEL - 1;
            for (i = 0; i < ARRAY_SIZE(lsensor_adc_table); i++)
            {
                if (lux < lsensor_adc_table[i])
                {
                    als_level = i;
                    break;
                }
            }
            APS9930_DBG("%s:cdata=%d irdata=%d lux=%d,als_level==%d\n",__func__, cdata, irdata, lux,als_level);

            if(aps_first_read)
            {
                aps_first_read = 0;
                input_report_abs(aps->input_dev, ABS_LIGHT, -1);
                input_sync(aps->input_dev);
            }
            else
            {
                input_report_abs(aps->input_dev, ABS_LIGHT, als_level);
                input_sync(aps->input_dev);
            }
        }
        /* if lux<0,we need to change the gain which we can set register 0x0f */
        else {
                pr_err("%s, line %d: Need to change gain(lux=%2d)\n", __func__, __LINE__, lux);
                /*We should change gain,but we must input this status to framework*/
                light_data_value = abs(lux);
                input_report_abs(aps->input_dev, ABS_LIGHT, 6);
                input_sync(aps->input_dev);
        }
    }   
    /* l_flag is close, but raise als interrupt: abnormal, clear the als interrupt bit */
    else if ((status & APDS9930_STATUS_ALS_BIT) && (!atomic_read(&l_flag)))
    {
        /* clear als interrupt bit */
        ret = i2c_smbus_write_byte(aps->client,CMD_CLR_ALS_INT);
        if (ret)
        {
            pr_err("%s, line %d: clear als interrupt bit failed(%d)!\n", __func__, __LINE__, ret);
        }
        else
        {
            pr_err("%s, line %d: l_flag is close, but raise als interrupt, clear als interrupt bit.\n", __func__, __LINE__);
        }
    }

    /* delete the condition */
    if (aps->client->irq)
    {
        enable_irq(aps->client->irq);
    }
}
	/* delete some lines */

static inline int aps_i2c_reg_init(struct aps_data *aps)
{
    int ret;
    int i;
    int revid;
    int id;
    PROXIMITY_DEBUG("%s:enter in aps_i2c_reg_init function\n", __func__);
    for (i=0; i< ARRAY_SIZE(aps9930_init_regdata);i++)
    {
        ret = set_9930_register(aps, CMD_BYTE|aps9930_init_regdata[i].reg, aps9930_init_regdata[i].data, 0);
        if (ret < 0)
        {
            pr_err("%s,%d:Ret of init aps9930 regs(%d - %d) is %d\n", __func__,  __LINE__, aps9930_init_regdata[i].reg, aps9930_init_regdata[i].data, get_9930_register(aps, aps9930_init_regdata[i].reg, 0));
            break;
        }
    }
    /* ID check ,if not equal, return -1 */
    revid = get_9930_register(aps, APDS9930_REV_REG, 0);
    id = get_9930_register(aps, APDS9930_ID_REG, 0);
    if ((APDS_9900_REV_ID == revid)  || (APDS_9900_ID == id))
    {
        pr_info("%s %d:Alps-sensor read id is APS9900/TMD27713 \n", __func__, __LINE__);
        ret = app_info_set("LP-Sensor", "APS9900/TMD27713");
        if (ret < 0)/*failed to add app_info*/
        {
            pr_err("%s %d:faile to add app_info\n", __func__, __LINE__);
        }
    }
    else if ((APDS_9930_REV_ID == revid)  || (APDS_9930_ID == id))
    {
        pr_info("%s %d:Alps-sensor read id is APS9930/TMD27723 \n", __func__, __LINE__);
        ret = app_info_set("LP-Sensor", "APS9930/TMD27723");
        if (ret < 0)/*failed to add app_info*/
        {
            pr_err("%s %d:faile to add app_info\n", __func__, __LINE__);
        }
    }
    else
    {
        pr_err("%s,%d:The ID of checking failed!(revid=%.2x id=%.2x)\n", __func__, __LINE__, revid, id);
        ret = -1;
    }

    return ret;
}

irqreturn_t aps_irq_handler(int irq, void *dev_id)
{
    struct aps_data *aps = dev_id;
    PROXIMITY_DEBUG("%s:enter in aps_irq_handler function\n", __func__);
    disable_irq_nosync(aps->client->irq);
    queue_work(aps_wq, &aps->work);

    return IRQ_HANDLED;
}

static u16 *create_and_get_u16_array(struct device_node *dev_node,
		const char *name, int *size)
{
	const __be32 *values;
	u16 *val_array;
	int len;
	int sz;
	int rc;
	int i;

	values = of_get_property(dev_node, name, &len);
	if (values == NULL)
		return NULL;

	sz = len / sizeof(u32);
	pr_debug("%s: %s size:%d\n", __func__, name, sz);

	val_array = kzalloc(sz * sizeof(u16), GFP_KERNEL);
	if (val_array == NULL) {
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < sz; i++)
		val_array[i] = (u16)be32_to_cpup(values++);

	*size = sz;

	return val_array;

fail:
	return ERR_PTR(rc);
}

static u16 *create_and_get_als_adc_array(struct device_node *dev_node)
{
	u16 *adc;
	int size;
	int rc;

	adc = create_and_get_u16_array(dev_node, "aps,adc_array", &size);
	if (IS_ERR_OR_NULL(adc))
		return (void *)adc;//sunlibin  do not understand??

	/* Check for valid abs size */
	if (size % LSENSOR_MAX_LEVEL) {
		rc = -EINVAL;
		goto fail_free_adc;
	}

	return adc;

fail_free_adc:
	kfree(adc);

	return ERR_PTR(rc);
}

/* some param undefine(rc == -EINVAL) will cause error */
static int aps_parse_dt(struct device *dev,
				struct aps9930_hw_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "aps,int_gpio", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read int_gpio\n");
		return rc;
	}
	else
	{
		pdata->int_gpio = (int)temp_val;
	}

	rc = of_property_read_u32(np, "aps,window", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read window\n");
		return rc;
	}
	else
	{
		pdata->window = (int)temp_val;
	}

	rc = of_property_read_u32(np, "aps,wave", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read wave\n");
		return rc;
	}
	else
	{
		pdata->wave = (int)temp_val;
	}

	rc = of_property_read_u32(np, "aps,fall", &temp_val);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read fall\n");
		return rc;
	}
	else if (rc == -EINVAL)
	{
		dev_err(dev, "get fall value wrong,return default value\n");
		pdata->fall = 0;
	}
	else
	{
		pdata->fall = (int)temp_val;
	}

	rc = of_property_read_u32(np, "aps,sunlight", &temp_val);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read sunlight\n");
		return rc;
	}
	else if (rc == -EINVAL)
	{
		dev_err(dev, "get sunlight value wrong,return default value\n");
		pdata->sunlight = PX_SUNLIGHT_DEFAULT;
	}
	else
	{
		pdata->sunlight = (int)temp_val;
	}

	pdata->adc_array = create_and_get_als_adc_array(np);
	if (pdata->adc_array == NULL) {
		rc = -EINVAL;
		dev_err(dev, "Unable to read adc_array,  rc = %d \n",rc);
		return rc;
	} else if (IS_ERR(pdata->adc_array)) {
		rc = PTR_ERR(pdata->adc_array);
		dev_err(dev, "Unable to read adc_array,  rc = %d \n",rc);
		return rc;
	}

	return 0;
}

/* delete aps_timer_func function */
static int aps_9930_probe(
    struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct aps_data *aps;
    int i = 0;
    struct aps9930_hw_platform_data *platform_data = NULL;

	platform_data = kzalloc(sizeof(struct aps9930_hw_platform_data), GFP_KERNEL);
	if (!platform_data) 
	{
		ret = -ENOMEM;
		goto err_exit;
	}

	if (client->dev.of_node)
	{
		memset(platform_data, 0 , sizeof(struct aps9930_hw_platform_data));
		ret = aps_parse_dt(&client->dev, platform_data);
		if (ret) 
		{
				pr_err("%s,%d:Unable to parse platfrom data err=%d\n", __func__, __LINE__, ret);
			goto err_exit;
		}
	}  
	else 
	{
		if (client->dev.platform_data) 
		{
			platform_data = (struct aps9930_hw_platform_data *)client->dev.platform_data;
		} 
		else 
		{
        pr_err("%s,%d:platform data is NULL. exiting.\n", __func__, __LINE__);
			ret = -ENODEV;
			goto err_exit;
		}
	}

    apds_9930_pwindows_value = platform_data->window;
    apds_9930_pwave_value = platform_data->wave; 
    apds_9930_fall_prop = platform_data->fall; 
    sunlight = platform_data->sunlight;
    //get_light_value(p);
    for(i = 0;i < ARRAY_SIZE(lsensor_adc_table) ; i++ )
    {
		lsensor_adc_table[i] = platform_data->adc_array[i];
		printk("lux[%d] = %d ",i,lsensor_adc_table[i]);
    }
    printk("\n");
    origin_prox = MAX_ADC_PROX_VALUE - apds_9930_pwindows_value;
    min_proximity_value = origin_prox;

    //platform_data = client->dev.platform_data;
/*8930 mate aps power*/
    if (platform_data->aps9930_power)
    {
        ret = platform_data->aps9930_power(IC_PM_ON);
        if (ret < 0)
        {
            pr_err("%s,%d:aps9930 power on error!\n", __func__, __LINE__);
            goto err_exit ;
        }
    }
    mdelay(5);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s, line %d: need I2C_FUNC_I2C\n", __func__, __LINE__);
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
    /* delete some lines */

    aps = kzalloc(sizeof(*aps), GFP_KERNEL);
    if (aps == NULL) {
        pr_err("%s, line %d: kzalloc fail!\n", __func__, __LINE__);
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    mutex_init(&aps->mlock);
    INIT_WORK(&aps->work, aps_9930_work_func);
    aps->client = client;
    i2c_set_clientdata(client, aps);

    PROXIMITY_DEBUG(KERN_INFO "ghj aps_9930_probe send command 2\n ");

    ret = aps_i2c_reg_init(aps);
    if (ret <0)
    {
        pr_err("%s,%d:aps_i2c_reg_init: Failed to init aps_i2c_reg_init!(%d)\n", __func__, __LINE__,ret);
        goto err_detect_failed;
    }
    /* delete mdelay(12) */
    client->irq = gpio_to_irq(platform_data->int_gpio);
    if (client->irq < 0) {
        ret = -EINVAL;
        goto err_gpio_config_failed;
    }
	
    if (client->irq) 
    {
        if (platform_data->aps9930_gpio_config_interrupt)
        {
			ret = gpio_request(platform_data->int_gpio,"aps9930_irq_gpio");
		if (ret)
		{
			pr_err("%s, line %d:unable to request gpio [%d]\n", __func__, __LINE__,platform_data->int_gpio);
			goto err_gpio_config_failed;
		} 
		else
		{
			ret = gpio_tlmm_config(GPIO_CFG(platform_data->int_gpio,0,
				                                 GPIO_CFG_INPUT,GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if(ret < 0) 
			{
				pr_err("%s, line %d:Fail set gpio as no pull=%d\n", __func__, __LINE__,platform_data->int_gpio);
				goto err_gpio_config_failed;
			}
		}
        }
        
        if (request_irq(client->irq, aps_irq_handler,IRQF_TRIGGER_LOW, client->name, aps) >= 0) 
        {
            PROXIMITY_DEBUG("Received IRQ!\n");
    	    disable_irq(aps->client->irq);
            #if 0
            if (set_irq_wake(client->irq, 1) < 0)
            {
                pr_err("%s, line %d:failed to set IRQ wake\n", __func__, __LINE__);
            }
            #endif
        }
        else 
        {
            pr_err("%s, line %d:Failed to request IRQ!\n", __func__, __LINE__);
        }
         /* set the threshold of proximity and ALS */
         ret = set_9930_register(aps, APDS9930_AILTL_REG, 0, 1);
         ret |= set_9930_register(aps, APDS9930_AIHTL_REG, 0, 1);
         ret |= set_9930_register(aps, APDS9930_PILTL_REG, 0, 1);
         /* modify the high proximity threshold from 500 to 800 */
         /*modify the value*/
         ret |= set_9930_register(aps, APDS9930_PIHTL_REG, 0x3c0, 1);
		 /* set the low thresthold of 1023 to make sure make an interrupt */
         /*modify the value*/
         ret |= set_9930_register(aps, APDS9930_PILTL_REG, 0x3bf, 1);
         if (ret)
         {
             pr_err("%s,%d:set the threshold of proximity and ALS failed(%d)!\n", __func__, __LINE__, ret);
         }
		 
    }
    /* if not define irq,then error */
    else
    {
        pr_err("%s, line %d: please set the irq num!\n", __func__, __LINE__);
        goto err_detect_failed;
    }
    if (sensor_9930_dev == NULL) 
    {
         aps->input_dev = input_allocate_device();
         if (aps->input_dev == NULL) {
         ret = -ENOMEM;
         pr_err("%s, line %d: Failed to allocate input device\n", __func__, __LINE__);
         goto err_input_dev_alloc_failed;
         }
        aps->input_dev->name = "sensors_aps";
        aps->input_dev->id.bustype = BUS_I2C;
        input_set_drvdata(aps->input_dev, aps);
        
        ret = input_register_device(aps->input_dev);
        if (ret) {
            pr_err("%s, line %d:aps_9930_probe: Unable to register %s input device\n", __func__, __LINE__, aps->input_dev->name);
            goto err_input_register_device_failed;
        }
        sensor_9930_dev = aps->input_dev;
    } else {
        pr_err("%s, line %d:sensor_dev is not null\n", __func__, __LINE__);
        aps->input_dev = sensor_9930_dev;
    }
    set_bit(EV_ABS, aps->input_dev->evbit);
    input_set_abs_params(aps->input_dev, ABS_LIGHT, 0, 10240, 0, 0);
    input_set_abs_params(aps->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = misc_register(&light_device);
    if (ret) {
        pr_err("%s, line %d:aps_9930_probe: light_device register failed\n", __func__, __LINE__);
        goto err_light_misc_device_register_failed;
    }

    ret = misc_register(&proximity_device);
    if (ret) {
        pr_err("%s, line %d:aps_9930_probe: proximity_device register failed\n", __func__, __LINE__);
        goto err_proximity_misc_device_register_failed;
    }

    if( light_device.minor != MISC_DYNAMIC_MINOR ){
        light_device_minor = light_device.minor;
    }


    if( proximity_device.minor != MISC_DYNAMIC_MINOR ){
        proximity_device_minor = proximity_device.minor ;
    }

    wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");

    aps_wq = create_singlethread_workqueue("aps_wq");

    if (!aps_wq) 
    {
        ret = -ENOMEM;
        goto err_create_workqueue_failed;
    }

    this_aps_data =aps;
    atomic_set(&px_status_flag, PROXIMITY_SUSPEND);
    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_APS);
    set_hw_dev_flag(DEV_I2C_L_SENSOR);
    #endif
    ret = set_sensor_input(PS, aps->input_dev->dev.kobj.name);
    if (ret) {
        dev_err(&client->dev, "%s set_sensor_input failed\n", __func__);
        goto err_create_workqueue_failed;
    }
    ret = set_sensor_input(ALS, aps->input_dev->dev.kobj.name);
    if (ret) {
        dev_err(&client->dev, "%s set_sensor_input failed\n", __func__);
        goto err_create_workqueue_failed;
    }
    pr_info("%s, line %d: aps_9930_probe: Start Proximity Sensor APS-9930\n", __func__, __LINE__);

    //set_sensors_list(L_SENSOR + P_SENSOR);
    return 0;

err_create_workqueue_failed:
    misc_deregister(&proximity_device);
err_proximity_misc_device_register_failed:
    misc_deregister(&light_device);
err_light_misc_device_register_failed:
err_input_register_device_failed:
    input_free_device(aps->input_dev);
err_input_dev_alloc_failed:
err_gpio_config_failed:
err_detect_failed:
    kfree(aps);
err_alloc_data_failed:
err_check_functionality_failed:
/*add 8930 compass exception process*/
err_exit:
    return ret;
  
}
static int aps_9930_remove(struct i2c_client *client)
{
    struct aps_data *aps = i2c_get_clientdata(client);

    PROXIMITY_DEBUG("ghj aps_9930_remove enter\n ");
    if (aps->client->irq)
    {
        disable_irq(aps->client->irq);
    }
    free_irq(client->irq, aps);
    misc_deregister(&light_device);
    misc_deregister(&proximity_device);
    input_unregister_device(aps->input_dev);

    kfree(aps);
    return 0;
}

/*set  reg 0 */
static int aps_9930_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct aps_data *aps = i2c_get_clientdata(client);

    PROXIMITY_DEBUG("aps_9930_suspend enter\n ");
    if (aps->client->irq)
    {
        disable_irq(aps->client->irq);
    }
    ret = cancel_work_sync(&aps->work);

    /* set [PON] bit =0 ,meaning disables the oscillator */
	/*reconfig reg before supspend*/
    ret = set_9930_register(aps, APDS9930_ENABLE_REG,  APDS9930_POWER_OFF,0);
    if (ret)
    {
        pr_err("%s,%d: set APDS9930_ENABLE_REG register[PON=OFF] failed(%d)!\n", __func__, __LINE__, ret);
    }
    return 0;
}
static int aps_9930_resume(struct i2c_client *client)
{
    int ret;
    struct aps_data *aps = i2c_get_clientdata(client);

    PROXIMITY_DEBUG("aps_9930_resume enter\n ");
    /* Command 0 register: set [PON] bit =1 */
    ret = set_9930_register(aps, APDS9930_ENABLE_REG, reg0_value,0);
    if (ret)
    {
        pr_err("%s, %d: set APDS9930_ENABLE_REG register[PON=ON] failed(%d)!\n", __func__, __LINE__,ret);
    }
    if (aps->client->irq)
    {
        enable_irq(aps->client->irq);
    }
    return 0;
}

static const struct i2c_device_id aps_id[] = {
    { "aps-9930", 0 },
    { }
};

static struct of_device_id taos_match_table[] = {
	{ .compatible = "tmd277x,aps-9930", },
	{ },
};


static struct i2c_driver aps_driver = {
    .probe      = aps_9930_probe,
    .remove     = aps_9930_remove,
    .suspend    = aps_9930_suspend,
    .resume     = aps_9930_resume,
    .id_table   = aps_id,
    .driver = {
        .name   ="aps-9930",
	.of_match_table = taos_match_table,
    },
};

static int __devinit aps_9930_init(void)
{
    return i2c_add_driver(&aps_driver);
}

static void __exit aps_9930_exit(void)
{
    i2c_del_driver(&aps_driver);
    if (aps_wq)
    {
        destroy_workqueue(aps_wq);
    }
}

device_initcall_sync(aps_9930_init);
module_exit(aps_9930_exit);
MODULE_DESCRIPTION("Proximity Driver");
MODULE_LICENSE("GPL");
