/* Optimize code */
/*
 * include/linux/touch_platfrom_config.h - platform data structure for touchscreen
 *
 * Copyright (C) 2010 Google, Inc.
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
#ifndef _TOUCH_PLATFORM_CONFIG_H
#define _TOUCH_PLATFORM_CONFIG_H
#include <linux/atomic.h>

#define LCD_ALL_FWVGA_45INCHTP     922  
#define LCD_X_FWVGA 480
#define LCD_Y_FWVGA 854

#define SYN_CONFIG_SIZE 32 * 16
#define CURRENT_PR_VERSION  1191601
//#define CURRENT_PR_VERSION  1294018








typedef enum
{
    TP_COF = 0x0,
    TP_COB = 0x1,
    TP_MAX = 0xF,
}tp_type;
typedef enum
{
    NOT_NEED_UPDATE_FW = 0x0,
    NEED_UPDATE_FW = 0x1,
    UPDATE_MAX = 0xF,
}tp_update_type;

extern tp_type get_touch_type(void);

extern tp_update_type is_need_update_fw(void);

extern uint8_t* get_tp_lockdown_config(void);
extern int get_tp_id(void);
extern tp_type get_touch_type(void);
extern uint8_t *get_tp_version_config(int module_id,u16 ic_type);
extern tp_update_type is_need_update_fw(void);

extern struct touch_hw_platform_data touch_pdata;
extern atomic_t touch_detected_yet; 

struct tp_resolution_conversion{
	int lcd_x;
	int lcd_y;
	int lcd_all;
};
struct touch_hw_platform_data {

	void (*set_touch_probe_flag)(int detected);/*we use this to detect the probe is detected*/
	int (*read_touch_probe_flag)(void);/*when the touch is find we return a value*/
	int (*get_touch_resolution)(struct tp_resolution_conversion *tp_resolution_type);/*add this function for judge the tp type*/
};
#endif /*_TOUCH_PLATFORM_CONFIG_H */
