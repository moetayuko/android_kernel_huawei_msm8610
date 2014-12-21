/* Isl29044 ps do not work,als data too low */
/* drivers/staging/taos/tmd277x.c
 *
 * Copyright (C) 2011-2013 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/i2c/tmd277x.h>
#include <linux/module.h>
#include <misc/app_info.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

//#include <mach/regs-gpio.h>
#include <mach/irqs.h>
#include <linux/irq.h>

#ifdef CONFIG_HUAWEI_KERNEL
#define CMD_BYTE      0x80
#define CMD_WORD      0xA0
#define MAX_ADC_PROX_VALUE 1022
#define CMD_CLR_PS_INT		0xE5
int prox_wave = 100;
int prox_window = 200;
u16 prox_max;
u16 prox_min =  MAX_ADC_PROX_VALUE - 200;
/*add workqueue and some var for suspend/resume*/
static struct workqueue_struct *aps_wq = NULL;
static int prox_enable_suspend = 0;
static int als_enable_suspend = 0;
/*use this to contrl the al sensor debug message*/
static int als_debug_mask;
module_param_named(als_debug, als_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define ALS_DEBUG_MASK(x...) do {\
	if (als_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)
	
/*use this to contrl the pr sensor debug message*/
static int ps_debug_mask;
module_param_named(ps_debug, ps_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define PS_DEBUG_MASK(x...) do {\
	if (ps_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)
#endif
	

enum tmd277x_regs {
	TMD277X_ENABLE,
	TMD277X_ALS_TIME,
	TMD277X_PRX_TIME,
	TMD277X_WAIT_TIME,
	TMD277X_ALS_MINTHRESHLO,
	TMD277X_ALS_MINTHRESHHI,
	TMD277X_ALS_MAXTHRESHLO,
	TMD277X_ALS_MAXTHRESHHI,
	TMD277X_PRX_MINTHRESHLO,
	TMD277X_PRX_MINTHRESHHI,
	TMD277X_PRX_MAXTHRESHLO,
	TMD277X_PRX_MAXTHRESHHI,
	TMD277X_PERSISTENCE,
	TMD277X_CONFIG,
	TMD277X_PRX_PULSE_COUNT,
	TMD277X_CONTROL,

	TMD277X_REVID = 0x11,
	TMD277X_CHIPID,
	TMD277X_STATUS,
	TMD277X_ALS_CHAN0LO,
	TMD277X_ALS_CHAN0HI,
	TMD277X_ALS_CHAN1LO,
	TMD277X_ALS_CHAN1HI,
	TMD277X_PRX_LO,
	TMD277X_PRX_HI,

	TMD277X_REG_PRX_OFFS = 0x1e,
	TMD277X_REG_MAX,
};

enum tmd277x_cmd_reg {
	TMD277X_CMD_REG           = (1 << 7),
	TMD277X_CMD_INCR          = (0x1 << 5),
	TMD277X_CMD_SPL_FN        = (0x3 << 5),
	TMD277X_CMD_PROX_INT_CLR  = (0x5 << 0),
	TMD277X_CMD_ALS_INT_CLR   = (0x6 << 0),
};

enum tmd277x_en_reg {
	TMD277X_EN_PWR_ON   = (1 << 0),
	TMD277X_EN_ALS      = (1 << 1),
	TMD277X_EN_PRX      = (1 << 2),
	TMD277X_EN_WAIT     = (1 << 3),
	TMD277X_EN_ALS_IRQ  = (1 << 4),
	TMD277X_EN_PRX_IRQ  = (1 << 5),
	TMD277X_EN_SAI      = (1 << 6),
};

enum tmd277x_status {
	TMD277X_ST_ALS_VALID  = (1 << 0),
	TMD277X_ST_PRX_VALID  = (1 << 1),
	TMD277X_ST_ALS_IRQ    = (1 << 4),
	TMD277X_ST_PRX_IRQ    = (1 << 5),
	TMD277X_ST_PRX_SAT    = (1 << 6),
};

enum {
	TMD277X_ALS_GAIN_MASK = (3 << 0),
	TMD277X_ALS_AGL_MASK  = (1 << 2),
	TMD277X_ALS_AGL_SHIFT = 2,
	TMD277X_ATIME_PER_100 = 273,
	TMD277X_ATIME_DEFAULT_MS = 50,
	SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 10,
	GAIN_SWITCH_LEVEL = 100,
	GAIN_AUTO_INIT_VALUE = 16,
};

static u8 const tmd277x_ids[] = {
	0x39,
	0x30,
};

static char const *tmd277x_names[] = {
	"tmd27723 / tmd27727",
	"tmd27721 / tmd27725",
};

static u8 const restorable_regs[] = {
	TMD277X_ALS_TIME,
	TMD277X_PRX_TIME,
	TMD277X_WAIT_TIME,
	TMD277X_PERSISTENCE,
	TMD277X_CONFIG,
	TMD277X_PRX_PULSE_COUNT,
	TMD277X_CONTROL,
	TMD277X_REG_PRX_OFFS,
};

static u8 const als_gains[] = {
	1,
	8,
	16,
	120
};

struct taos_als_info {
	int ch0;
	int ch1;
	u32 cpl;
	u32 saturation;
	int lux;
};

struct taos_prox_info {
	int raw;
	int detected;
};

static struct lux_segment segment_default[] = {
	{
		.ratio = (435 << RATIO_SHIFT) / 1000,
		.k0 = (46516 << SCALE_SHIFT) / 1000,
		.k1 = (95381 << SCALE_SHIFT) / 1000,
	},
	{
		.ratio = (551 << RATIO_SHIFT) / 1000,
		.k0 = (23740 << SCALE_SHIFT) / 1000,
		.k1 = (43044 << SCALE_SHIFT) / 1000,
	},
};

struct tmd2772_chip {
	struct mutex lock;
	struct work_struct work;
	struct i2c_client *client;
	struct taos_prox_info prx_inf;
	struct taos_als_info als_inf;
	struct taos_parameters params;
	struct tmd2772_i2c_platform_data *pdata;
	u8 shadow[TMD277X_REG_MAX];
	struct input_dev *p_idev;
	struct input_dev *a_idev;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;
	bool prx_enabled;
	struct lux_segment *segment;
	int segment_num;
	int seg_num_max;
	bool als_gain_auto;
};

static int taos_i2c_read(struct tmd2772_chip *chip, u8 reg, u8 *val)
{
	int ret;
	s32 read;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte(client, (TMD277X_CMD_REG | reg));
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
		return ret;
	}
	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		dev_err(&client->dev, "%s: failed to read from register %x\n",
				__func__, reg);
		return ret;
	}
	*val = read;
	return read;
}

static int taos_i2c_blk_read(struct tmd2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_read_i2c_block_data(client,
			TMD277X_CMD_REG | TMD277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int taos_i2c_write(struct tmd2772_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = i2c_smbus_write_byte_data(client, TMD277X_CMD_REG | reg, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed to write register %x\n",
				__func__, reg);
	return ret;
}

static int taos_i2c_blk_write(struct tmd2772_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;

	ret =  i2c_smbus_write_i2c_block_data(client,
			TMD277X_CMD_REG | TMD277X_CMD_INCR | reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}

static int set_segment_table(struct tmd2772_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) {
		dev_dbg(dev, "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			dev_err(dev, "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	dev_dbg(dev, "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		dev_dbg(dev, "segment %d: ratio %6u, k0 %6u, k1 %6u\n",
				i, chip->segment[i].ratio,
				chip->segment[i].k0, chip->segment[i].k1);
	return 0;
}

static void taos_calc_cpl(struct tmd2772_chip *chip) 
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMD277X_ALS_TIME];
	u8 agl = (chip->shadow[TMD277X_CONFIG] & TMD277X_ALS_AGL_MASK)  
			>> TMD277X_ALS_AGL_SHIFT;			


	u32 time_scale = (256 - atime ) * 2730 / 200;   //2730 =2.73*1000  

/*20120830 johnny
	u32 time_scale = ((256 - atime) << SCALE_SHIFT) *		//atime = TMD277X_ALS_TIME 
		TMD277X_ATIME_PER_100 / (TMD277X_ATIME_DEFAULT_MS * 100); // TMD277X_ATIME_PER_100 = 273,TMD277X_ATIME_DEFAULT_MS = 50
*/
										//time_scale = ((256-238)<<11)*273/(50*100) = 2012
	cpl = time_scale * chip->params.als_gain;				// cpl = 2012 * 8 = 16096
	if (agl)   							
		cpl = cpl * 16 / 1000;						// cpl = 16096*16/1000 = 257
	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10); 	//  (256-238)<<10 = 18432,min_t(u32,X,u32,Y)
	sat = sat * 8 / 10;					  	
	dev_dbg(&chip->client->dev,//12
			"%s: cpl = %u [time_scale %u, gain %u, agl %u], "
			"saturation %u\n", __func__, cpl, time_scale,
			chip->params.als_gain, agl, sat);
	chip->als_inf.cpl = cpl;					//CPL = chip->als_inf.cpl
	chip->als_inf.saturation = sat;					//sat = chip->als_inf.saturation
}

static int set_als_gain(struct tmd2772_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg  = chip->shadow[TMD277X_CONTROL] & ~TMD277X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 8:
		ctrl_reg |= AGAIN_8;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 120:
		ctrl_reg |= AGAIN_120;
		break;
	default:
		dev_err(&chip->client->dev, "%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}
	rc = taos_i2c_write(chip, TMD277X_CONTROL, ctrl_reg);
	if (!rc) {
		chip->shadow[TMD277X_CONTROL] = ctrl_reg;
		chip->params.als_gain = gain;
		dev_dbg(&chip->client->dev, "%s: new gain %d\n",
				__func__, gain);
	}
	return rc;
}

static int taos_get_lux(struct tmd2772_chip *chip)
{
	unsigned i;
	int ret = 0;
	struct device *dev = &chip->client->dev;	
	struct lux_segment *s = chip->segment;		
	u32 c0 = chip->als_inf.ch0;
	u32 c1 = chip->als_inf.ch1;
	u32 sat = chip->als_inf.saturation;
	u32 ratio;
	u64 lux_0, lux_1;
	u32 cpl = chip->als_inf.cpl;
	u32 lux, k0 = 0, k1 = 0;

	if (!chip->als_gain_auto) {  			
									
		if (c0 <= MIN_ALS_VALUE) {  				//MIN_ALS_VALUE = 10
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {							//chip->als_gain_auto = "AUTO"
		u8 gain = chip->params.als_gain;			// auto gain , 1x , 16x , 120x
		int rc = -EIO;

		if (gain == 16 && c0 >= sat) {
			rc = set_als_gain(chip, 1);
		} else if (gain == 16 && c0 < GAIN_SWITCH_LEVEL) {	//GAIN_SWITCH_LEVEL = 100
			rc = set_als_gain(chip, 120);
		} else if ((gain == 120 && c0 >= sat) ||
				(gain == 1 && c0 < GAIN_SWITCH_LEVEL)) {
			rc = set_als_gain(chip, 16);
		}
		if (!rc) {							
			dev_dbg(dev, "%s: gain adjusted, skip\n", __func__);	
			taos_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (c0 <= MIN_ALS_VALUE) {
			dev_dbg(dev, "%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (c0 >= sat) {
			dev_dbg(dev, "%s: saturation, keep lux\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	//atime = 50ms, gain = 8x , ch0 = 3891 , ch1 = 424

	ratio = (c1 << RATIO_SHIFT) / c0;						//ratio = c1 * 1024 / c0     //ratio = 111
	for (i = 0; i < chip->segment_num; i++, s++) {
		if (ratio <= s->ratio) {
			dev_dbg(&chip->client->dev, "%s: ratio %u segment %u "
					"[r %u, k0 %u, k1 %u]\n", __func__,
					ratio, i, s->ratio, s->k0, s->k1);
			k0 = s->k0;
			k1 = s->k1;
			break;
		}
	}
	if (i >= chip->segment_num) {
		dev_dbg(&chip->client->dev, "%s: ratio %u - darkness\n",
				__func__, ratio);
		lux = 0;
		goto exit;
	}

	lux_0 = ( ( ( c0 * 100 ) - ( c1 * 175 ) ) * 10 ) / cpl;
	lux_1 = ( ( ( c0 *  63 ) - ( c1 * 100 ) ) * 10 ) / cpl; 				//20120830	lux

	
	//snprintf( buf, "lux_0 = 0x%16llx , lux_1 = 0x%16llx , c0 = 0x%16llx , c1 = 0x%16llx ", lux_0 , lux_1 , c0 , c1 );

	lux = max(lux_0, lux_1);							//20120830	lux
	lux = max(lux , (u32)0);							//20120830	lux

exit:
#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s: lux %u (%u x %u - %u x %u) / %u\n",
		__func__, lux, k0, c0, k1, c1, cpl);
	//lux = lux / 4;
#else
	dev_dbg(&chip->client->dev, "%s: lux %u (%u x %u - %u x %u) / %u\n",
		__func__, lux, k0, c0, k1, c1, cpl);
	lux = lux / 4;
#endif
	chip->als_inf.lux = lux;
	return ret;
}
#ifndef CONFIG_HUAWEI_KERNEL
static int pltf_power_on(struct tmd2772_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_ON);
		msleep(10);
	}
	chip->unpowered = rc != 0;
	return rc;
}

static int pltf_power_off(struct tmd2772_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	return rc;
}
#endif
static int taos_irq_clr(struct tmd2772_chip *chip, u8 bits)
{
	int ret = i2c_smbus_write_byte(chip->client, TMD277X_CMD_REG |
			TMD277X_CMD_SPL_FN | bits);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: failed, bits %x\n",
				__func__, bits);
	return ret;
}

static void taos_get_als(struct tmd2772_chip *chip)
{
	u32 ch0, ch1;
	u8 *buf = &chip->shadow[TMD277X_ALS_CHAN0LO];

	ch0 = le16_to_cpup((const __le16 *)&buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&buf[2]);
	chip->als_inf.ch0 = ch0;
	chip->als_inf.ch1 = ch1;
	dev_dbg(&chip->client->dev, "%s: ch0 %u, ch1 %u\n", __func__, ch0, ch1);
}


#ifdef CONFIG_HUAWEI_KERNEL
static int  set_tmd2772_register(struct tmd2772_chip  *aps, u8 reg, u16 value, int flag)
{
    int ret;

   // mutex_lock(&aps->mlock);
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

    //mutex_unlock(&aps->mlock);

    return ret;
}
static int get_tmd2772_register(struct tmd2772_chip *chip, u8 reg, int flag)
{
    int ret;

    //mutex_lock(&chip->lock);
    if (flag)
    {
        ret = i2c_smbus_read_word_data(chip->client, CMD_WORD | reg);
    }
    else
    {
        ret = i2c_smbus_read_byte_data(chip->client, CMD_BYTE | reg);
    }
    //mutex_unlock(&chip->lock);
    if (ret < 0)
    {
        printk(KERN_ERR "%s, line %d: read register fail!(reg=0x%x, flag=%d, ret=0x%x)", __func__, __LINE__, reg, flag, ret);
    }

    return ret;
}
#endif

static void taos_get_prox(struct tmd2772_chip *chip)
{
	u8 *buf = &chip->shadow[TMD277X_PRX_LO];
	bool d = chip->prx_inf.detected;
#ifdef CONFIG_HUAWEI_KERNEL
	int pdata = 0;
	u8 *thrd = &chip->shadow[TMD277X_PRX_MINTHRESHLO];
	int ret = 0;
#endif

	chip->prx_inf.raw = (buf[1] << 8) | buf[0];
	chip->prx_inf.detected =
			(d && (chip->prx_inf.raw > chip->params.prox_th_min)) ||
			(!d && (chip->prx_inf.raw > chip->params.prox_th_max));
#ifdef CONFIG_HUAWEI_KERNEL
	pdata = chip->prx_inf.raw;
	PS_DEBUG_MASK("%s: ps_raw_data = %d, detected = %d\n", __func__,pdata, chip->prx_inf.detected);
	if((pdata+prox_wave) < prox_min)
	{
		prox_min = pdata+prox_wave;
		prox_max = prox_min + prox_window;
		*thrd++ = prox_min & 0xff;
		*thrd++ = prox_min >> 8;
		*thrd++ = prox_max & 0xff;
		*thrd++ = prox_max >> 8;
		ret = taos_i2c_blk_write(chip, TMD277X_PRX_MINTHRESHLO,
				&chip->shadow[TMD277X_PRX_MINTHRESHLO],
				TMD277X_PRX_MAXTHRESHHI - TMD277X_PRX_MINTHRESHLO + 1);
		if(ret < 0)
		{
			dev_dbg(&chip->client->dev, "%s, line %d:i2c_blk_write (ret = %d) fail!\n", __func__, __LINE__,ret);
		}
		else
		{
			PS_DEBUG_MASK("%s(p+wave<min): max = %d, min = %d\n", __func__,prox_max,prox_min);
		}
	}

	prox_min = get_tmd2772_register(chip, TMD277X_PRX_MINTHRESHLO, 1);
	prox_max = get_tmd2772_register(chip, TMD277X_PRX_MAXTHRESHLO, 1);
	PS_DEBUG_MASK("%s: The threshold before compare is:max = %d, min = %d\n", __func__,prox_max,prox_min);
	
	ret = i2c_smbus_write_byte(chip->client,CMD_CLR_PS_INT);
	/* clear interupt register first */
	if (ret)
	{
		dev_dbg(&chip->client->dev, "%s, line %d: set_tmd2772_register is error(%d),clear int failed!", __func__, __LINE__, ret);
	}
	
	if(pdata > prox_max)
	{
		prox_max = (MAX_ADC_PROX_VALUE + 1);
		thrd = &chip->shadow[TMD277X_PRX_MINTHRESHLO];
		*thrd++ = prox_min & 0xff;
		*thrd++ = prox_min >> 8;
		*thrd++ = prox_max & 0xff;
		*thrd++ = prox_max >> 8;
		ret = taos_i2c_blk_write(chip, TMD277X_PRX_MINTHRESHLO,
				&chip->shadow[TMD277X_PRX_MINTHRESHLO],
				TMD277X_PRX_MAXTHRESHHI - TMD277X_PRX_MINTHRESHLO + 1);
		if(ret < 0)
		{
			dev_dbg(&chip->client->dev, "%s, line %d:i2c_blk_write (ret = %d) fail!\n", __func__, __LINE__,ret);
		}
		else
		{
			PS_DEBUG_MASK("%s(p>max): max = %d, min = %d\n", __func__,prox_max,prox_min);
		}
		input_report_abs(chip->p_idev, ABS_DISTANCE,0);
		input_sync(chip->p_idev);
	}
	else if(pdata < prox_min)
	{
		prox_max = (prox_min + prox_window);
		/*if prox far, we need modify the MAX thresh value*/
		thrd = &chip->shadow[TMD277X_PRX_MAXTHRESHLO];
		*thrd++ = prox_max & 0xff;
		*thrd++ = prox_max >> 8;
		ret = taos_i2c_blk_write(chip, TMD277X_PRX_MAXTHRESHLO,
				&chip->shadow[TMD277X_PRX_MAXTHRESHLO],
				TMD277X_PRX_MAXTHRESHHI - TMD277X_PRX_MAXTHRESHLO + 1);
		if(ret < 0)
		{
			dev_dbg(&chip->client->dev, "%s, line %d:i2c_blk_write (ret = %d) fail!\n", __func__, __LINE__,ret);
		}
		else
		{
			PS_DEBUG_MASK("%s(p<min): max = %d, min = %d \n", __func__,prox_max,prox_min);
		}
		input_report_abs(chip->p_idev, ABS_DISTANCE,1);
		input_sync(chip->p_idev);
	}
	else
	{
		dev_dbg(&chip->client->dev, "%s, line %d: Wrong status!\n",  __func__, __LINE__);
		ret = set_tmd2772_register(chip, TMD277X_PRX_MINTHRESHLO, prox_min, 1);
		if (ret)
		{
			dev_dbg(&chip->client->dev, "%s, line %d: set tmd2772_PILTL_REG register is error(%d)!", __func__, __LINE__, ret);
		}
		else
		{
			PS_DEBUG_MASK("%s(error): max = %d, min = %d \n", __func__,prox_max,prox_min);
		}
	}

	prox_min = get_tmd2772_register(chip, TMD277X_PRX_MINTHRESHLO, 1);
	prox_max = get_tmd2772_register(chip, TMD277X_PRX_MAXTHRESHLO, 1);
	PS_DEBUG_MASK("%s: The threshold after compare is:max = %d, min = %d\n", __func__,prox_max,prox_min);
#else
	dev_dbg(&chip->client->dev, "%s: raw %d, detected %d\n", __func__,
			chip->prx_inf.raw, chip->prx_inf.detected);
#endif
}

static int taos_read_all(struct tmd2772_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	ret = taos_i2c_blk_read(chip, TMD277X_STATUS,
			&chip->shadow[TMD277X_STATUS],
			TMD277X_PRX_HI - TMD277X_STATUS + 1);
	return (ret < 0) ? ret : 0;
}

static int update_prox_thresh(struct tmd2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TMD277X_PRX_MINTHRESHLO];
	u16 from, to;

	if (on_enable) {
		/* zero gate to force irq */
#ifdef CONFIG_HUAWEI_KERNEL
		from = prox_min;
		to = prox_max;
#else
		from = to = 0;
#endif
	} else {
		if (chip->prx_inf.detected) {
			from = chip->params.prox_th_min;
			to = 0xffff;
		} else {
			from = 0;
			to = chip->params.prox_th_max;
		}
	}
	dev_dbg(&chip->client->dev, "%s: %u - %u\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TMD277X_PRX_MINTHRESHLO,
			&chip->shadow[TMD277X_PRX_MINTHRESHLO],
			TMD277X_PRX_MAXTHRESHHI - TMD277X_PRX_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

static int update_als_thres(struct tmd2772_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TMD277X_ALS_MINTHRESHLO];
	u16 gate = chip->params.als_gate;
	u16 from, to, cur;

	cur = chip->als_inf.ch0;
	if (on_enable) {
		/* zero gate far away form current position to force an irq */
		from = to = cur > 0xffff / 2 ? 0 : 0xffff;
	} else {
		gate = cur * gate / 100;
		if (!gate)
			gate = 1;
		if (cur > gate)
			from = cur - gate;
		else
			from = 0;
		if (cur < (0xffff - gate))
			to = cur + gate;
		else
			to = 0xffff;
	}
	dev_dbg(&chip->client->dev, "%s: [%u - %u]\n", __func__, from, to);
	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = taos_i2c_blk_write(chip, TMD277X_ALS_MINTHRESHLO,
			&chip->shadow[TMD277X_ALS_MINTHRESHLO],
			TMD277X_ALS_MAXTHRESHHI - TMD277X_ALS_MINTHRESHLO + 1);
	return (ret < 0) ? ret : 0;
}

#ifndef CONFIG_HUAWEI_KERNEL
static void report_prox(struct tmd2772_chip *chip)
{
	if (chip->p_idev) {
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 1);
		input_sync(chip->p_idev);
	}
}
#endif

static void report_als(struct tmd2772_chip *chip)
{
	if (chip->a_idev) {
		int rc = taos_get_lux(chip);
		if (!rc) {
			int lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
			update_als_thres(chip, 0);
		} else {
			update_als_thres(chip, 1);
		}
	}
}

static int taos_check_and_report(struct tmd2772_chip *chip)
{
	u8 status;
#ifdef CONFIG_HUAWEI_KERNEL
	u8 *buf = &chip->shadow[TMD277X_PRX_LO];
	u16 max = 0;
	u16 min = 0;
	u32 ch0, ch1;

#endif
	int ret = taos_read_all(chip);
	if (ret)
		goto exit_clr;
	
#ifdef CONFIG_HUAWEI_KERNEL
	chip->prx_inf.raw = (buf[1] << 8) | buf[0];
	PS_DEBUG_MASK("%s: ps_raw_data = %d \n", __func__,chip->prx_inf.raw);
	
	min = get_tmd2772_register(chip, TMD277X_PRX_MINTHRESHLO, 1);
	max = get_tmd2772_register(chip, TMD277X_PRX_MAXTHRESHLO, 1);
	PS_DEBUG_MASK("%s: ps_max = %d, ps_min = %d\n", __func__,max,min);
	
	ch0 = le16_to_cpup((const __le16 *)&buf[0]);
	ch1 = le16_to_cpup((const __le16 *)&buf[2]);
	ALS_DEBUG_MASK("%s: als_ch0 = %u, als_ch1 = %u\n", __func__, ch0, ch1);
#endif

	status = chip->shadow[TMD277X_STATUS];
#ifdef CONFIG_HUAWEI_KERNEL
	PS_DEBUG_MASK("%s: status = 0x%02x\n", __func__, status);
	ALS_DEBUG_MASK("%s: status = 0x%02x\n", __func__, status);
#else
	dev_dbg(&chip->client->dev, "%s: status 0x%02x\n", __func__, status);
#endif
	if ((status & (TMD277X_ST_PRX_VALID | TMD277X_ST_PRX_IRQ)) ==
			(TMD277X_ST_PRX_VALID | TMD277X_ST_PRX_IRQ)) {
#ifdef CONFIG_HUAWEI_KERNEL
		PS_DEBUG_MASK("ps\n");
#else
		printk("ps\n");
#endif
		taos_get_prox(chip);
#ifndef CONFIG_HUAWEI_KERNEL
		report_prox(chip);
		update_prox_thresh(chip, 0);
#endif
	}

	if ((status & (TMD277X_ST_ALS_VALID | TMD277X_ST_ALS_IRQ)) ==
			(TMD277X_ST_ALS_VALID | TMD277X_ST_ALS_IRQ)) {
#ifdef CONFIG_HUAWEI_KERNEL
		ALS_DEBUG_MASK("als\n");
#else
		printk("als\n");
#endif
		taos_get_als(chip);
		report_als(chip);
	}
exit_clr:
	taos_irq_clr(chip, TMD277X_CMD_PROX_INT_CLR | TMD277X_CMD_ALS_INT_CLR);
	return ret;
}

static irqreturn_t taos_irq(int irq, void *handle)
{
	struct tmd2772_chip *chip = handle;
#ifndef CONFIG_HUAWEI_KERNEL
	struct device *dev = &chip->client->dev;
	mutex_lock(&chip->lock);
	if (chip->in_suspend) {
		dev_dbg(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		disable_irq_nosync(chip->client->irq);
		goto bypass;
	}
#ifdef CONFIG_HUAWEI_KERNEL
	PS_DEBUG_MASK("%s\n", __func__);
	ALS_DEBUG_MASK("%s\n", __func__);
#else
	dev_dbg(dev, "%s\n", __func__);
#endif
	(void)taos_check_and_report(chip);
bypass:
	mutex_unlock(&chip->lock);
#else
    /*disable irq first, then wakeup the work*/
    disable_irq_nosync(chip->client->irq);
    queue_work(aps_wq, &chip->work);
#endif
	return IRQ_HANDLED;
}

static void set_pltf_settings(struct tmd2772_chip *chip)
{
	struct taos_raw_settings const *s = chip->pdata->raw_settings;
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	if (s) {
		dev_dbg(dev, "%s: form pltf data\n", __func__);
		sh[TMD277X_ALS_TIME] = s->als_time;
		sh[TMD277X_PRX_TIME] = s->prx_time;
		sh[TMD277X_WAIT_TIME] = s->wait_time;
		sh[TMD277X_PERSISTENCE] = s->persist;
		sh[TMD277X_CONFIG] = s->cfg_reg;
		sh[TMD277X_PRX_PULSE_COUNT] = s->prox_pulse_cnt;
		sh[TMD277X_CONTROL] = s->ctrl_reg;
		sh[TMD277X_REG_PRX_OFFS] = s->prox_offs;
	} else {
		dev_dbg(dev, "%s: use defaults\n", __func__);
		sh[TMD277X_ALS_TIME] = 238; /* ~50 ms */
		sh[TMD277X_PRX_TIME] = 255;
		sh[TMD277X_WAIT_TIME] = 0;
		sh[TMD277X_PERSISTENCE] = PRX_PERSIST(1) | ALS_PERSIST(3);
		sh[TMD277X_CONFIG] = 0;
		sh[TMD277X_PRX_PULSE_COUNT] = 12;
		sh[TMD277X_CONTROL] = AGAIN_8 | PGAIN_4 |
				PDIOD_CH0 | PDRIVE_30MA;
		sh[TMD277X_REG_PRX_OFFS] = 0;
	}
	chip->params.als_gate = chip->pdata->parameters.als_gate;
	chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
	chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
	chip->params.als_gain = chip->pdata->parameters.als_gain;
	if (chip->pdata->parameters.als_gain) {
		chip->params.als_gain = chip->pdata->parameters.als_gain;
	} else {
		chip->als_gain_auto = true;
		chip->params.als_gain = GAIN_AUTO_INIT_VALUE;
		dev_dbg(&chip->client->dev, "%s: auto als gain.\n", __func__);
	}
	(void)set_als_gain(chip, chip->params.als_gain);
	taos_calc_cpl(chip);
}

static int flush_regs(struct tmd2772_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	dev_dbg(&chip->client->dev, "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = taos_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}
	return rc;
}

static int update_enable_reg(struct tmd2772_chip *chip)
{
	dev_dbg(&chip->client->dev, "%s: %02x\n", __func__,
			chip->shadow[TMD277X_ENABLE]);
	return taos_i2c_write(chip, TMD277X_ENABLE,
			chip->shadow[TMD277X_ENABLE]);
}

static int taos_prox_enable(struct tmd2772_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		taos_irq_clr(chip, TMD277X_CMD_PROX_INT_CLR);
		update_prox_thresh(chip, 1);
		chip->shadow[TMD277X_ENABLE] |=
				(TMD277X_EN_PWR_ON | TMD277X_EN_PRX |
				TMD277X_EN_PRX_IRQ);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(3);
	} else {
		chip->shadow[TMD277X_ENABLE] &=
				~(TMD277X_EN_PRX_IRQ | TMD277X_EN_PRX);
		if (!(chip->shadow[TMD277X_ENABLE] & TMD277X_EN_ALS))
			chip->shadow[TMD277X_ENABLE] &= ~TMD277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		taos_irq_clr(chip, TMD277X_CMD_PROX_INT_CLR);
	}
	if (!rc)
		chip->prx_enabled = on;
	return rc;
}

static int taos_als_enable(struct tmd2772_chip *chip, int on)
{
	int rc;

	dev_dbg(&chip->client->dev, "%s: on = %d\n", __func__, on);
	printk("%s: on = %d\n", __func__, on);
	if (on) {
		taos_irq_clr(chip, TMD277X_CMD_ALS_INT_CLR);
		update_als_thres(chip, 1);
		chip->shadow[TMD277X_ENABLE] |=
				(TMD277X_EN_PWR_ON | TMD277X_EN_ALS |
				TMD277X_EN_ALS_IRQ);
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		msleep(3);
	} else {
		chip->shadow[TMD277X_ENABLE] &=
				~(TMD277X_EN_ALS_IRQ | TMD277X_EN_ALS);
		if (!(chip->shadow[TMD277X_ENABLE] & TMD277X_EN_PRX))
			chip->shadow[TMD277X_ENABLE] &= ~TMD277X_EN_PWR_ON;
		rc = update_enable_reg(chip);
		if (rc)
			return rc;
		taos_irq_clr(chip, TMD277X_CMD_ALS_INT_CLR);
	}
	if (!rc)
		chip->als_enabled = on;
	return rc;
}

static ssize_t taos_device_als_ch0(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch0);
}

static ssize_t taos_device_als_ch1(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.ch1);
}

static ssize_t taos_device_als_cpl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t taos_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t taos_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k, "%d:%u,%u,%u\n", i,
				(s[i].ratio * 1000) >> RATIO_SHIFT,
				(s[i].k0 * 1000) >> SCALE_SHIFT,
				(s[i].k1 * 1000) >> SCALE_SHIFT);
	return k;
}

static ssize_t taos_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int i;
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	u32 ratio, k0, k1;

	if (4 != sscanf(buf, "%10d:%10u,%10u,%10u", &i, &ratio, &k0, &k1))
		return -EINVAL;
	if (i >= chip->segment_num)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->segment[i].ratio = (ratio << RATIO_SHIFT) / 1000;
	chip->segment[i].k0 = (k0 << SCALE_SHIFT) / 1000;
	chip->segment[i].k1 = (k1 << SCALE_SHIFT) / 1000;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmd2772_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmd2772_als_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		taos_als_enable(chip,1);
	else
		taos_als_enable(chip,0);

	return size;
}

static ssize_t tmd2772_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmd2772_prox_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		taos_prox_enable(chip,1);
	else
		taos_prox_enable(chip,0);

	return size;
}

static ssize_t taos_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n", chip->params.als_gain,
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t taos_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int rc;
	struct tmd2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gain);
	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 8 && gain != 16 && gain != 120)
		return -EINVAL;
	mutex_lock(&chip->lock);
	if (gain) {
		chip->als_gain_auto = false;
		rc = set_als_gain(chip, gain);
		if (!rc)
			taos_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return rc ? rc : size;
}

static ssize_t taos_als_gate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (in %%)\n", chip->params.als_gate);
}

static ssize_t taos_als_gate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gate;
	int rc;
	struct tmd2772_chip *chip = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, 10, &gate);
	if (rc || gate > 100)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.als_gate = gate;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t taos_device_prox_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 *bufff;
	int rawww;
	struct tmd2772_chip *chip = dev_get_drvdata(dev);

	int ret = taos_read_all(chip);
	if (ret)
		goto exit_clr;

	bufff = &chip->shadow[TMD277X_PRX_LO];

	rawww = (bufff[1] << 8) | bufff[0];
	return snprintf(buf, PAGE_SIZE, "%d\n", rawww);
exit_clr:
	taos_irq_clr(chip, TMD277X_CMD_PROX_INT_CLR | TMD277X_CMD_ALS_INT_CLR);
	return ret;
}

static ssize_t taos_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t taos_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t read_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = chip->client;

	char *after;
	int i;
	u8 reg;

	after = buf;
	reg = 0x01;

	for(i = 0; i < 15; i++) {
		i2c_smbus_write_byte(client, (TMD277X_CMD_REG | reg));
		after += sprintf(after,"0x%02x\t",i2c_smbus_read_byte(client));
		reg++;
	}
	after += sprintf(after, "\n\t");

	reg = 0x12;
	for(i = 0; i < 8; i++) {
		i2c_smbus_write_byte(client, (TMD277X_CMD_REG | reg));
		after += sprintf(after,"0x%02x\t",i2c_smbus_read_byte(client));
		reg++;
	}
	i2c_smbus_write_byte(client, (TMD277X_CMD_REG | 0x1e));
	after += sprintf(after,"0x%02x\t",i2c_smbus_read_byte(client));
	after += sprintf(after, "\n");
	return (after - buf);
}

static struct device_attribute prox_attrs[] = {
	__ATTR(prox_raw, 0444, taos_device_prox_raw, NULL),
	__ATTR(prx_raw, 0444, taos_device_prx_raw, NULL),
	__ATTR(prx_detect, 0444, taos_device_prx_detected, NULL),
	__ATTR(enable_ps_sensor, 0666, tmd2772_prox_enable_show, tmd2772_prox_enable),
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_ch0, 0444, taos_device_als_ch0, NULL),
	__ATTR(als_ch1, 0444, taos_device_als_ch1, NULL),
	__ATTR(als_cpl, 0444, taos_device_als_cpl, NULL),
	__ATTR(als_chlux, 0444, taos_device_als_lux, NULL),
	__ATTR(als_gain, 0644, taos_als_gain_show, taos_als_gain_store),
	__ATTR(als_gate, 0644, taos_als_gate_show, taos_als_gate_store),
	__ATTR(lux_table, 0644, taos_lux_table_show, taos_lux_table_store),
	__ATTR(enable_als_sensor, 0666, tmd2772_als_enable_show, tmd2772_als_enable),
	__ATTR(get_reg,0644,read_reg,NULL),
};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int taos_get_id(struct tmd2772_chip *chip, u8 *id, u8 *rev)
{
	int rc = taos_i2c_read(chip, TMD277X_REVID, rev);
	if (rc < 0)
		return rc;
	return taos_i2c_read(chip, TMD277X_CHIPID, id);
}
/*don't use the function, delete it*/
#ifndef CONFIG_HUAWEI_KERNEL
static int power_on(struct tmd2772_chip *chip)
{
	int rc;
	rc = pltf_power_on(chip);
	if (rc)
		return rc;
	dev_dbg(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return flush_regs(chip);
}
#endif
#if 0
static int prox_idev_open(struct input_dev *idev)
{
	struct tmd2772_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool als = chip->a_idev && chip->a_idev->users;
printk("%s\n",__func__);
	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = taos_prox_enable(chip, 1);
	if (rc && !als)
		pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void prox_idev_close(struct input_dev *idev)
{
	struct tmd2772_chip *chip = dev_get_drvdata(&idev->dev);
printk("%s\n",__func__);
	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	taos_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int als_idev_open(struct input_dev *idev)
{
	struct tmd2772_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	bool prox = chip->p_idev && chip->p_idev->users;
printk("%s\n",__func__);
	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		rc = power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = taos_als_enable(chip, 1);
	if (rc && !prox)
		pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return rc;
}

static void als_idev_close(struct input_dev *idev)
{
	struct tmd2772_chip *chip = dev_get_drvdata(&idev->dev);
printk("%s\n",__func__);
	dev_dbg(&idev->dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	taos_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}
#endif
/*use work queue instead of irq function process*/
static void tmd27723_work_func(struct work_struct *work)
{
    int ret = 0;
    struct tmd2772_chip *chip = container_of(work, struct tmd2772_chip, work);
    if(chip == NULL)
    {
        pr_err("%s: chip is NULL\n",__func__);
        return;
    }

    mutex_lock(&chip->lock);
    if(chip->in_suspend == 0)
    {
        ret = taos_check_and_report(chip);
        enable_irq(chip->client->irq);
    }
    mutex_unlock(&chip->lock);
}
static int taos_parse_dt(struct device *dev,
				struct tmd2772_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "tmd2772,int_gpio", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read int_gpio\n");
		return rc;
	} else {
		pdata->int_gpio = (int)temp_val;
	}
	pdata->als_can_wake = of_property_read_bool(np, "tmd2772,als_can_wake");
	pdata->proximity_can_wake = of_property_read_bool(np, "tmd2772,proximity_can_wake");
	rc = of_property_read_u32(np, "tmd2772,prox_th_min", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_th_min\n");
		return rc;
	} else {
		pdata->parameters.prox_th_min = (u16)temp_val;
	}
	rc = of_property_read_u32(np, "tmd2772,prox_th_max", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read prox_th_max\n");
		return rc;
	} else {
			pdata->parameters.prox_th_max = (u16)temp_val;
		}
	rc = of_property_read_u32(np, "tmd2772,als_gate", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read als_gate\n");
		return rc;
	} else {
			pdata->parameters.als_gate = (u16)temp_val;
		}
	pdata->raw_settings = NULL;
	return 0;
}

static int __devinit taos_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev;
	static struct tmd2772_chip *chip;
	struct tmd2772_i2c_platform_data *pdata;
	bool powered = 0;
    int device_id_index = 0;/* the location of name in tmd277x_names  */
	printk("tmd2772\n");
	dev = &client->dev;
	pdata = kzalloc(sizeof(struct tmd2772_i2c_platform_data), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	if (client->dev.of_node) {
		memset(pdata, 0 , sizeof(struct tmd2772_i2c_platform_data));
		ret = taos_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev,
				"Unable to parse platfrom data err=%d\n", ret);
			return ret;
		}
	}  else {
		if (client->dev.platform_data) {
			pdata = (struct tmd2772_i2c_platform_data *)client->dev.platform_data;
		} else {
			dev_err(&client->dev, "platform data is NULL. Abort.\n");
			return -EINVAL;
		}
	}

#ifdef CONFIG_HUAWEI_KERNEL
	prox_min =	MAX_ADC_PROX_VALUE - 200;
#endif
	
	pdata->prox_name = "proximity";
	pdata->als_name = "lightsensor-level";
	dev_info(&client->dev, "%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		dev_err(&client->dev, "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		dev_err(&client->dev, "%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (pdata->platform_init) {
		ret = pdata->platform_init(dev);
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(&client->dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		msleep(10);
	}
	chip = kzalloc(sizeof(struct tmd2772_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = set_segment_table(chip, chip->pdata->segment,
			chip->pdata->segment_num);
	else
		ret =  set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = taos_get_id(chip, &id, &rev);
	if (ret < 0)
	{
		goto id_failed;
	}
	
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    set_hw_dev_flag(DEV_I2C_APS);
    set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	for (i = 0; i < ARRAY_SIZE(tmd277x_ids); i++) {
		if (id == tmd277x_ids[i]){
            device_id_index = i;
			break;
         }
	}
	if (i < ARRAY_SIZE(tmd277x_names)) {
		dev_info(&client->dev, "%s: '%s rev. %d' detected\n", __func__, 
		tmd277x_names[i], rev);
		
		ret = app_info_set("LP-Sensor", tmd277x_names[device_id_index]);
		if (ret < 0) {/*failed to add app_info*/
		
			dev_err(&client->dev, "%s %d:faile to add app_info\n", __func__, __LINE__);
		}
	} else {
		dev_err(&client->dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}
	mutex_init(&chip->lock);
	set_pltf_settings(chip);
	ret = flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		dev_err(&client->dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	//chip->p_idev->open = prox_idev_open;
	//chip->p_idev->close = prox_idev_close;
	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		dev_err(&client->dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
bypass_prox_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		dev_err(&client->dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	//chip->a_idev->open = als_idev_open;
	//chip->a_idev->close = als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		dev_err(&client->dev, "%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_alloc_failed;
	}
	ret = add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:
printk("int_gpio=%d\n",chip->pdata->int_gpio);
	gpio_tlmm_config(GPIO_CFG(chip->pdata->int_gpio, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
				GPIO_CFG_ENABLE);
	ret = request_threaded_irq(client->irq, NULL, taos_irq,
		      IRQ_TYPE_EDGE_FALLING,
		      "taos_irq", chip);
	if (ret) {
		dev_info(&client->dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}
    INIT_WORK(&(chip->work), tmd27723_work_func);
    /*create a single workqueue like 8930 and 8x25Q*/
    aps_wq = create_singlethread_workqueue("aps_wq");
    if(!aps_wq)
    {
        ret = -ENOMEM;
        goto irq_register_fail;
    }
	dev_info(&client->dev, "Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(&client->dev, "Probe failed.\n");
	return ret;
}

static int taos_suspend(struct device *dev)
{

	struct tmd2772_chip *chip = dev_get_drvdata(dev);
#ifndef CONFIG_HUAWEI_KERNEL
#ifdef CONFIG_HUAWEI_KERNEL
	struct tmd2772_i2c_platform_data *pdata = chip->pdata;
#else
    struct tmd2772_i2c_platform_data *pdata = dev->platform_data;
#endif
	

	dev_err(dev, "%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;
	if (chip->p_idev && chip->p_idev->users) {
		if (pdata->proximity_can_wake) {
			dev_dbg(dev, "set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "proximity off\n");
			taos_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			dev_dbg(dev, "set wake on als\n");
			chip->wake_irq = 1;
		} else {
			dev_dbg(dev, "als off\n");
			taos_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_dbg(dev, "powering off\n");
		pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);
#else
    PS_DEBUG_MASK("%s: enter\n",__func__);
    chip->in_suspend = 1;
    mutex_lock(&chip->lock);
	/*restore prox and als status*/
    prox_enable_suspend = chip->prx_enabled;
    als_enable_suspend = chip->als_enabled;
    
    disable_irq(chip->client->irq);

    cancel_work_sync(&chip->work);
    
    if(chip->prx_enabled)
    {
        taos_prox_enable(chip, 0);
    }
    if(chip->als_enabled)
    {
        taos_als_enable(chip, 0);
    }
    mutex_unlock(&chip->lock);
    PS_DEBUG_MASK("%s: exit\n",__func__);
#endif
	return 0;
}

static int taos_resume(struct device *dev)
{
	struct tmd2772_chip *chip = dev_get_drvdata(dev);
#ifndef CONFIG_HUAWEI_KERNEL
	bool als_on, prx_on;
	int rc = 0;
	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;
	dev_dbg(dev, "%s: powerd %d, als: needed %d  enabled %d,"
			" prox: needed %d  enabled %d\n", __func__,
			!chip->unpowered, als_on, chip->als_enabled,
			prx_on, chip->prx_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		dev_dbg(dev, "powering on\n");
		rc = power_on(chip);
		if (rc)
			goto err_power;
	}
	if (prx_on && !chip->prx_enabled)
		(void)taos_prox_enable(chip, 1);
	if (als_on && !chip->als_enabled)
		(void)taos_als_enable(chip, 1);
	if (chip->irq_pending) {
		dev_dbg(dev, "%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)taos_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);
#else
    PS_DEBUG_MASK("%s: enter\n",__func__);
    chip->in_suspend = 0;
    mutex_lock(&chip->lock);
	/*recover prox and als state*/
    if(prox_enable_suspend)
    {
        taos_prox_enable(chip, 1);
    }
    if(als_enable_suspend)
    {
        taos_als_enable(chip, 1);
    }

    enable_irq(chip->client->irq);
    mutex_unlock(&chip->lock);
    PS_DEBUG_MASK("%s: exit\n",__func__);
#endif
	return 0;
}

static int __devexit taos_remove(struct i2c_client *client)
{
	struct tmd2772_chip *chip = i2c_get_clientdata(client);
	free_irq(client->irq, chip);
	if (chip->a_idev) {
		remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	kfree(chip->segment);
	kfree(chip);
    if (aps_wq)
    {
        destroy_workqueue(aps_wq);
    }
	return 0;
}

static struct i2c_device_id taos_idtable[] = {
	{ "tmd2772", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

static struct of_device_id taos_match_table[] = {
	{ .compatible = "tmd2772,tmd2772", },
	{ },
};

static const struct dev_pm_ops taos_pm_ops = {
	.suspend = taos_suspend,
	.resume  = taos_resume,
};

static struct i2c_driver taos_driver = {
	.driver = {
		.name = "tmd2772",
		.pm = &taos_pm_ops,
		.of_match_table = taos_match_table,
	},
	.id_table = taos_idtable,
	.probe = taos_probe,
	.remove = __devexit_p(taos_remove),
};

static int __init taos_init(void)
{
	return i2c_add_driver(&taos_driver);
}

static void __exit taos_exit(void)
{
	i2c_del_driver(&taos_driver);
}

module_init(taos_init);
module_exit(taos_exit);

MODULE_AUTHOR("Aleksej Makarov <aleksej.makarov@sonyericsson.com>");
MODULE_DESCRIPTION("TAOS tmd2772 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
