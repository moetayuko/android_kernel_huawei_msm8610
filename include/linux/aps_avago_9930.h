/* drivers/input/misc_hw/aps_avago_9930.c
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

#ifndef _LINUX_APS_AVAGO_9930_H
#define _LINUX_APS_AVAGO_9930_H

#define ECS_IOCTL_APP_SET_DELAY	    _IOW(0xA1, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY       _IOR(0xA1, 0x30, short)
#define ECS_IOCTL_APP_SET_LFLAG		_IOW(0xA1, 0x1C, short)
#define ECS_IOCTL_APP_GET_LFLAG		_IOR(0xA1, 0x1B, short)
#define ECS_IOCTL_APP_SET_PFLAG		_IOW(0xA1, 0x1E, short)
#define ECS_IOCTL_APP_GET_PFLAG		_IOR(0xA1, 0x1D, short)
#define ECS_IOCTL_APP_GET_PDATA_VALVE	_IOR(0xA1, 0x32, short)
#define ECS_IOCTL_APP_GET_LDATA_VALVE	_IOR(0xA1, 0x33, short)

#define VREG_GP4_NAME	"gp4"
#define VREG_GP4_VOLTAGE_VALUE	2700

#define CMD_BYTE      0x80
#define CMD_WORD      0xA0
#define CMD_SPECIAL   0xE0

#define APDS9930_ENABLE_REG  0x00
#define APDS9930_ATIME_REG   0x01
#define APDS9930_PTIME_REG   0x02
#define APDS9930_WTIME_REG   0x03
#define APDS9930_AILTL_REG   0x04
#define APDS9930_AILTH_REG   0x05
#define APDS9930_AIHTL_REG   0x06
#define APDS9930_AIHTH_REG   0x07
#define APDS9930_PILTL_REG   0x08
#define APDS9930_PILTH_REG   0x09
#define APDS9930_PIHTL_REG   0x0A
#define APDS9930_PIHTH_REG   0x0B
#define APDS9930_PERS_REG    0x0C
#define APDS9930_CONFIG_REG  0x0D
#define APDS9930_PPCOUNT_REG 0x0E
#define APDS9930_CONTROL_REG 0x0F
#define APDS9930_REV_REG     0x11
#define APDS9930_ID_REG      0x12
#define APDS9930_STATUS_REG  0x13
#define APDS9930_CDATAL_REG  0x14
#define APDS9930_CDATAH_REG  0x15
#define APDS9930_IRDATAL_REG 0x16
#define APDS9930_IRDATAH_REG 0x17
#define APDS9930_PDATAL_REG  0x18
#define APDS9930_PDATAH_REG  0x19

#define DETECTION_THRESHOLD	500

#define APDS9930_POWER_ON 1     /* set the APDS9930_ENABLE_REG's PON=1,Writing a 1 activates the APDS9930 */
#define APDS9930_POWER_OFF 0    /* set the APDS9930_ENABLE_REG's PON=1,Writing a 0 disables the APDS9930 */
/*reconfig reg after resume*/
#define APDS9930_ENABLE 0x3F    /* set the APDS9930_ENABLE_REG's*/
#define APDS9930_POWER_MASK (1<<0)
#define APDS9930_STATUS_PROXIMITY_BIT (1<<5)
#define APDS9930_STATUS_ALS_BIT (1<<4)
#define APDS9930_PEN_BIT_SHIFT 2
#define APDS9930_AEN_BIT_SHIFT 1

#define APDS_9901_ID  0x20 /* APDS-9901 */
#define APDS_9900_ID  0x29 /* APDS-9900 */
#define APDS_9900_REV_ID 0x01
#define APDS_9930_ID  0x39 /* APDS-9930 */
#define APDS_9930_REV_ID 0x02
#define APDS9930_REG0_POWER_OFF	0xfe
#define APDS9930_REG0_AEN_OFF	0xfd
#define APDS9930_REG0_PEN_OFF	0xfb

/* add lsensor level*/
#define LSENSOR_MAX_LEVEL 7
/* The max value of the  variable open_count */
#define OPEN_COUNT_MAX (10)
#define MAX_ADC_PROX_VALUE 1022
/* delete useless dts number */

typedef enum//for Y340 fail
{
	SENSOR_PROP_NONE = 0,
	PRXIMITY_OPEN_INIT = 1,
	/*force to use 32 bit for compile to improve running speed*/
	SENSOR_SPECIAL_MAX = 0x7FFFFFFF,
} hw_sensor_prop;
#define PROXIMITY_SUSPEND  0
#define PROXIMITY_RESUME   1

#define IC_PM_ON   1
#define IC_PM_OFF  0
#define MAX_DEVICES_LENGTH 64
#define PX_WAVE_DEFAULT 100
#define PX_WINDOW_DEFAULT 200
/*this 7 should be align with aps-9930.h*/
#define LUX_NUMBER 7

#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_AVAGO_9930
struct aps9930_hw_platform_data {
    int (*aps9930_power)(int on);
    int (*aps9930_gpio_config_interrupt)(void);
	int int_gpio;
	int window;
	int wave;
	int fall;
	int sunlight;
	u16 *adc_array;
};
#endif
#endif /* _LINUX_APS_AVAGO_9930_H */
