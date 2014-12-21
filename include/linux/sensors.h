/************************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  FileName: board_sensors.h
  Author: hantao(00185954)       Version : 0.1      Date:  2011-07-11
  Description:	.h file for sensors
  Version:
  Function List:
  History:
  <author>  <time>   <version >   <desc>
***********************************************************/
/*==============================================================================
History

Problem NO.         Name        Time         Reason

==============================================================================*/

#ifndef	__BOARD_SENSORS_H__
#define	__BOARD_SENSORS_H__

/*sunlibin added*/
#define GS_SUSPEND  0
#define GS_RESUME   1
#define IC_PM_ON   1
#define IC_PM_OFF  0

#define WHOAMI_LIS3DH_ACC	0x33	/* St Expected content for WAI */

/******************************************************************************
 * Rohm Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define KIONIX_ACCEL_WHO_AM_I_KXTE9 		0x00
#define KIONIX_ACCEL_WHO_AM_I_KXTF9 		0x01
#define KIONIX_ACCEL_WHO_AM_I_KXTI9_1001 	0x04
#define KIONIX_ACCEL_WHO_AM_I_KXTIK_1004 	0x05
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005 	0x07
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007 	0x08
#define KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008 	0x0A
#define KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009 	0x09
#define KIONIX_ACCEL_WHO_AM_I_KXCJK_1013 	0x11
#define KIONIX_ACCEL_WHO_AM_I_KX023		0x15


enum input_name {
	ACC,
	AKM,
	GYRO,
	ALS,
	PS,
	SENSOR_MAX
};

/*sunlibin added*/
typedef enum
{
	COMPASS_TOP_GS_TOP 			=0,
	COMPASS_TOP_GS_BOTTOM 		=1,
	COMPASS_BOTTOM_GS_TOP 		=2,
	COMPASS_BOTTOM_GS_BOTTOM	=3,
	COMPASS_NONE_GS_BOTTOM		=4,
	COMPASS_NONE_GS_TOP			=5,
	COMPASS_GS_POSITION_MAX,
}compass_gs_position_type;

typedef enum
{
	GS_ADIX345 	= 0x01,
	GS_ST35DE	= 0x02,
	GS_ST303DLH = 0X03,
	GS_MMA8452  = 0x04,
	GS_BMA250   = 0x05,
	GS_STLIS3XH	= 0x06,
	GS_ADI346   = 0x07,
	GS_KXTIK1004= 0x08,
}hw_gs_type;

/*sunlibin added*/
/* Use this variable to control the number of
 * effective bits of the accelerometer output.
 * Use the macro definition to select the desired
 * number of effective bits. */
#define KIONIX_ACCEL_RES_12BIT	0
#define KIONIX_ACCEL_RES_8BIT	1
#define KIONIX_ACCEL_RES_6BIT	2
#define KIONIX_ACCEL_RES_16BIT	3	//KX023

/* Use this variable to control the G range of
 * the accelerometer output. Use the macro definition
 * to select the desired G range.*/
#define KIONIX_ACCEL_G_2G		0
#define KIONIX_ACCEL_G_4G		1
#define KIONIX_ACCEL_G_6G		2
#define KIONIX_ACCEL_G_8G		3

struct gs_platform_data {
	int (*adapt_fn)(void);	/* fucntion is suported in some product */
	int slave_addr;     /*I2C slave address*/
	int dev_id;         /*who am I*/
	int *init_flag;     /*Init*/
	int (*gs_power)(int on);
	int int1_gpio;
	int int2_gpio;
	int compass_gs_position;
	u8 accel_res;
	u8 accel_g_range;
};



int set_sensor_input(enum input_name name, const char *input_num);
#endif
