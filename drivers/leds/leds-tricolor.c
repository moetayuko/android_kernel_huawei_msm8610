/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <mach/pmic.h>
#include <linux/gpio.h>
#include <mach/socinfo.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <mach/rpm-smd.h>

#include <linux/debugfs.h>

/*use this to contrl the tricolor debug and info message*/
#define TRI_COLOR_LOG_ERR 1
#define TRI_COLOR_LOG_INFO 2
#define TRI_COLOR_LOG_DEBUG 3

static int tri_color_log_mask = TRI_COLOR_LOG_INFO;

#define TRI_COLOR_DEBUG(x...) do { \
    if (tri_color_log_mask >= TRI_COLOR_LOG_DEBUG) { \
        printk(KERN_ERR x); \
    } \
} while (0) 

#define TRI_COLOR_INFO(x...) do { \
    if (tri_color_log_mask >= TRI_COLOR_LOG_INFO) { \
        printk(KERN_ERR x); \
    } \
} while (0) 

/* error print, no control */
#define TRI_COLOR_ERROR(x...) do { \
    if (tri_color_log_mask >= TRI_COLOR_LOG_ERR) { \
        printk(KERN_ERR x); \
    }\
} while (0) 

#define DEBUG_TRICOLOR_LED 0
#define LED_ON 1
#define LED_OFF 0
static int red_gpio = 96;
static int green_gpio = 95;
static int blue_gpio = 94;
static void tricolor_led_on(int gpio);
static void tricolor_led_off(int gpio);

#define LED_RED_ON() 	tricolor_led_on(red_gpio)
#define LED_BLUE_ON() 	tricolor_led_on(blue_gpio)
#define LED_GREEN_ON() 	tricolor_led_on(green_gpio)

#define LED_RED_OFF() 	tricolor_led_off(red_gpio)
#define LED_BLUE_OFF() 	tricolor_led_off(blue_gpio)
#define LED_GREEN_OFF() tricolor_led_off(green_gpio)

static bool red_led_status=LED_OFF;
static bool blue_led_status=LED_OFF;
static bool green_led_status=LED_OFF;
static int qrd7_led_flash_en = 96;

/*delete the blink_time instead of xxx_blink_time to control on/off time for red/green/blue led*/
static int red_blink_time_on = 0;
static int red_blink_time_off = 0;
static int blue_blink_time_on = 0;
static int blue_blink_time_off = 0;
static int green_blink_time_on = 0;
static int green_blink_time_off = 0;

enum tri_color_led_color {
	LED_COLOR_RED,
	LED_COLOR_GREEN,
	LED_COLOR_BLUE,
	LED_COLOR_MAX
};

enum tri_led_status{
	ALL_OFF,
	ALL_ON,
	BLUE_ON,
	BLUE_OFF,
	RED_ON,
	RED_OFF,
	GREEN_ON,
	GREEN_OFF,
	BLUE_BLINK,
	RED_BLINK,
	GREEN_BLINK,
	BLUE_BLINK_OFF,
	RED_BLINK_OFF,
	GREEN_BLINK_OFF,
	LED_MAX
};

struct tricolor_led_data {
	spinlock_t led_lock;
	int led_data[4];
	struct led_classdev leds[4];	
};

static struct timer_list red_led_blink_timer;
static struct timer_list blue_led_blink_timer;
static struct timer_list green_led_blink_timer;
/*
  function: turn on led
  input :gpio gpio pin
  output : NONE
  return : NONE
 */
static void tricolor_led_on(int gpio)
{
    if (!gpio_is_valid(gpio))				/* check gpio if have requested */
    {
        pr_info("%s: gpio%d is not valid\n", __func__, gpio);
        return ;
    }
	
    gpio_set_value(gpio, LED_ON);			/* set gpio to turn on led */
    pr_info("%s: gpio%d value is %d\n",__func__,gpio,gpio_get_value(gpio));
}

/*
  function: turn off led
  input :gpio gpio pin
  output : NONE
  return : NONE
 */
static void tricolor_led_off(int gpio)
{
    gpio_set_value(gpio, LED_OFF);/* set gpio to turn off led */
    
    TRI_COLOR_DEBUG("%s %d:gpio%d value is %d\n", __func__, __LINE__, gpio, gpio_get_value(gpio));
}
/*control led can use the different time for on and off*/
static void led_red_blink_cb(unsigned long data)
{
  TRI_COLOR_DEBUG("%s %d:red_led_blink data = %lu\n", __func__, __LINE__, data);
  
  if(red_led_status == LED_ON )
  {
   LED_RED_ON();
   red_led_status = LED_OFF;
   red_led_blink_timer.expires=jiffies + msecs_to_jiffies(red_blink_time_on);
	
   TRI_COLOR_DEBUG("%s %d:red_led_status = %d\n", __func__, __LINE__, red_led_status);
  }
  else
  {
    LED_RED_OFF();
	red_led_status = LED_ON;
    red_led_blink_timer.expires=jiffies + msecs_to_jiffies(red_blink_time_off);

    TRI_COLOR_DEBUG("%s %d:red_led_status = %d\n", __func__, __LINE__, red_led_status);
  }
  add_timer(&red_led_blink_timer);
}
static void led_blue_blink_cb(unsigned long data) 
{
  TRI_COLOR_DEBUG("%s %d:blue_led_blink data = %lu\n", __func__, __LINE__, data);
  
  if(blue_led_status == LED_ON )
	{
	  LED_BLUE_ON();
	  blue_led_status = LED_OFF;
      blue_led_blink_timer.expires=jiffies + msecs_to_jiffies(blue_blink_time_on);
	  
      TRI_COLOR_DEBUG("%s %d:blue_led_status = %d\n", __func__, __LINE__, blue_led_status);
	}
	else
	{
	  LED_BLUE_OFF();
	  blue_led_status = LED_ON;
      blue_led_blink_timer.expires=jiffies + msecs_to_jiffies(blue_blink_time_off);

      TRI_COLOR_DEBUG("%s %d:blue_led_status = %d\n", __func__, __LINE__, blue_led_status);
	}
    add_timer(&blue_led_blink_timer);
}
static void led_green_blink_cb(unsigned long data)
{
  TRI_COLOR_DEBUG("%s %d:green_led_blink data = %lu\n", __func__, __LINE__, data);
 
  if(green_led_status == LED_ON )
  	{
  	  LED_GREEN_ON();
	  green_led_status = LED_OFF;
      green_led_blink_timer.expires=jiffies + msecs_to_jiffies(green_blink_time_on);

      TRI_COLOR_DEBUG("%s %d:green_led_status = %d\n", __func__, __LINE__, blue_led_status);
	}
  else
	{
	LED_GREEN_OFF();
	green_led_status = LED_ON;
    green_led_blink_timer.expires=jiffies + msecs_to_jiffies(green_blink_time_off);

    TRI_COLOR_DEBUG("%s %d:green_led_status = %d\n", __func__, __LINE__, blue_led_status);
	}
  
  add_timer(&green_led_blink_timer);
}
static void led_red_blink_off(void)
{
  del_timer(&red_led_blink_timer);
  LED_RED_OFF();
  
  TRI_COLOR_DEBUG("%s %d:red led blink off\n", __func__, __LINE__);
}
static void led_green_blink_off(void)
{
  del_timer(&green_led_blink_timer);
  LED_GREEN_OFF();

  TRI_COLOR_DEBUG("%s %d:green led blink off\n", __func__, __LINE__);
}
static void led_blue_blink_off(void)
{
  del_timer(&blue_led_blink_timer);
  LED_BLUE_OFF();
  
  TRI_COLOR_DEBUG("%s %d:blue led blink off\n", __func__, __LINE__);
}
/*when echo blink to on led, should enable led immediate and start timer to control on and off*/
static void led_red_blink_on(void)
{
  TRI_COLOR_DEBUG("%s %d:red led blink on\n", __func__, __LINE__);
  
  LED_RED_ON();
  red_led_status = LED_OFF;
  
  init_timer(&red_led_blink_timer);
  red_led_blink_timer.function=led_red_blink_cb;
  red_led_blink_timer.expires=jiffies + msecs_to_jiffies(red_blink_time_on);
  add_timer(&red_led_blink_timer);
}

static void led_blue_blink_on(void)
{
  TRI_COLOR_DEBUG("%s %d:blue led blink on\n", __func__, __LINE__);
 
  LED_BLUE_ON();
  blue_led_status = LED_OFF;
  
  init_timer(&blue_led_blink_timer);
  blue_led_blink_timer.function=led_blue_blink_cb;
  blue_led_blink_timer.expires=jiffies + msecs_to_jiffies(blue_blink_time_on);
  add_timer(&blue_led_blink_timer);
}

static void led_green_blink_on(void)
{
  TRI_COLOR_DEBUG("%s %d:green led blink on\n", __func__, __LINE__);
  
  LED_GREEN_ON();
  green_led_status = LED_OFF;
  
  init_timer(&green_led_blink_timer);
  green_led_blink_timer.function=led_green_blink_cb;
  green_led_blink_timer.expires=jiffies + msecs_to_jiffies(green_blink_time_on);
  add_timer(&green_led_blink_timer);
}
static ssize_t led_blink_solid_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
  ssize_t ret = 0;
  /*show red or green or blue led on and off time*/
  int on_time = 0;
  int off_time = 0;
  enum tri_color_led_color color = LED_COLOR_MAX;
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct tricolor_led_data *tricolor_led = NULL;
  
  TRI_COLOR_DEBUG("%s %d:led_blink_solid_show color = %s\n", __func__, __LINE__, led_cdev->name);
  
  if (!strcmp(led_cdev->name, "red")) {
  	color = LED_COLOR_RED;
    on_time = red_blink_time_on;
    off_time = red_blink_time_off;
  } else if (!strcmp(led_cdev->name, "green")) {
  	color = LED_COLOR_GREEN;
    on_time = green_blink_time_on;
    off_time = green_blink_time_off;
  } else {
  	color = LED_COLOR_BLUE;
    on_time = blue_blink_time_on;
    off_time = blue_blink_time_off;
  }
  tricolor_led = container_of(led_cdev, struct tricolor_led_data, leds[color]);
  if(!tricolor_led)
  	printk(KERN_ERR "%s tricolor_led is NULL ",__func__);
  ret = snprintf(buf,200,"onMS = %d offMS = %d\n"
                "COMMAND:echo [onMS] [offMS] > /sys/class/leds/[color]/blink\n",on_time,off_time);
  return ret;
}
struct msm_rpm_request	*handle_tricolors;
static u32 rpm_vreg_string_to_int(const u8 *str)
{
	int i, len;
	u32 output = 0;

	len = strnlen(str, sizeof(u32));
	for (i = 0; i < len; i++)
		output |= str[i] << (i * 8);

	return output;
}
static ssize_t led_blink_solid_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
  int blink = 0;
  unsigned long flags = 0;
  int on_time = 0;
  int off_time = 0;
  unsigned char blink_flag=false;
  enum tri_color_led_color color = LED_COLOR_MAX;
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct tricolor_led_data *tricolor_led = NULL;
  if (!strcmp(led_cdev->name, "red")) {
  	color = LED_COLOR_RED;
  } else if (!strcmp(led_cdev->name, "green")) {
  	color = LED_COLOR_GREEN;
  } else {
  	color = LED_COLOR_BLUE;
  }
  
  TRI_COLOR_DEBUG("%s %d:store %s blink time \n", __func__, __LINE__, led_cdev->name);
  tricolor_led = container_of(led_cdev, struct tricolor_led_data, leds[color]);
  if(!tricolor_led)
  	printk(KERN_ERR "%s tricolor_led is NULL ",__func__);
  sscanf(buf, "%d %d", &on_time, &off_time);
  #if DEBUG_TRICOLOR_LED
  	printk("tricolor %s is on:%d off:%d\n",led_cdev->name, on_time, off_time);
  #endif
	//set the blink_flag
	if((on_time>0)&&(off_time>0))
		blink_flag=true;
	else
		blink_flag=false;
	msm_rpm_add_kvp_data(handle_tricolors, rpm_vreg_string_to_int("blk"),
					    (u8 *)&blink_flag, 1);
    //send blink_flag to rpm, then disable freeze_io or enable freeze_io
	msm_rpm_wait_for_ack(msm_rpm_send_request(handle_tricolors));
	return size;
  
  spin_lock_irqsave(&tricolor_led->led_lock, flags);
  /*if both on and off time > 0, then need enable blink for led*/
  if(on_time > 0 && off_time > 0){
    TRI_COLOR_DEBUG("%s %d:(on_time > 0 && off_time > 0) = true\n", __func__, __LINE__);
  	switch(color) {
  		case LED_COLOR_RED:
            {
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_RED\n", __func__, __LINE__);
                red_blink_time_on = on_time;
                red_blink_time_off = off_time;
                /*if the led blink enable already, just modify the time*/
                if(!tricolor_led->led_data[color])
                {
  			        led_red_blink_on();
                }
  		    }
  			break;
  		case LED_COLOR_GREEN:
            {
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_GREEN\n", __func__, __LINE__);
                green_blink_time_on = on_time;
                green_blink_time_off = off_time;
                /*if the led blink enable already, just modify the time*/
                if(!tricolor_led->led_data[color])
                {
                    led_green_blink_on();
                }  
  		    }
  			break;
  		case LED_COLOR_BLUE:
            {
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_BLUE\n", __func__, __LINE__);
                blue_blink_time_on = on_time;
                blue_blink_time_off = off_time;
                /*if the led blink enable already, just modify the time*/
                if(!tricolor_led->led_data[color])
                {
  			        led_blue_blink_on();
                }
  		    }
  			break;
  		default:
  			break;
  	}

    blink = 1;
  } else {
    /*only the led blink enable already, then off it.*/
    TRI_COLOR_DEBUG("%s %d:(on_time > 0 && off_time > 0) = false\n", __func__, __LINE__);
    if(tricolor_led->led_data[color])
    {
        switch(color) {
            case LED_COLOR_RED:
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_RED\n", __func__, __LINE__);
		        led_red_blink_off();
  	        break;
	        case LED_COLOR_GREEN:
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_GREEN\n", __func__, __LINE__);
  		        led_green_blink_off();
  		    break;
  	        case LED_COLOR_BLUE:
                TRI_COLOR_DEBUG("%s %d:case LED_COLOR_BLUE\n", __func__, __LINE__);
  		        led_blue_blink_off();
  		    break;
  	        default:
  		    break;
  	    }
    }
  }
  tricolor_led->led_data[color] = blink;
  spin_unlock_irqrestore(&tricolor_led->led_lock, flags);
  return size;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);
static char hw_version[10]="2.0";

static ssize_t led_hw_version_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	ret=snprintf(buf,sizeof(hw_version),"%s",hw_version);
	TRI_COLOR_DEBUG("%s %d:hardware design version = %s\n", __func__, __LINE__, hw_version);
	return ret;
}
				     
static ssize_t led_hw_version_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	if(size > sizeof(hw_version)){
		TRI_COLOR_ERROR("%s %d:out of memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	sscanf(buf, "%s", hw_version);
	TRI_COLOR_DEBUG("%s %d:hardware design version = %s\n", __func__, __LINE__, hw_version);
	return size;
}
static DEVICE_ATTR(hardware_version, 0644, led_hw_version_show, led_hw_version_store);
static void led_brightness_set_tricolor(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
  struct tricolor_led_data *tricolor_led = NULL;
  enum tri_color_led_color color = LED_COLOR_MAX;
  unsigned long flags = 0;
  
  TRI_COLOR_DEBUG("%s %d:brightness set color = %s\n", __func__, __LINE__, led_cdev->name);
  
  if (!strcmp(led_cdev->name, "red")) {
  	color = LED_COLOR_RED;
  } else if (!strcmp(led_cdev->name, "green")) {
  	color = LED_COLOR_GREEN;
  } else {
  	color = LED_COLOR_BLUE;
  }
  tricolor_led = container_of(led_cdev, struct tricolor_led_data, leds[color]);
  if(!tricolor_led)
  	printk(KERN_ERR "%s tricolor_led is NULL ",__func__);
  
  spin_lock_irqsave(&tricolor_led->led_lock, flags);
  
  if(brightness){
    TRI_COLOR_DEBUG("%s %d:brightness = true, color = %d\n", __func__, __LINE__, color);
  	switch(color) {
  		case LED_COLOR_RED:
  			LED_RED_ON();
  			break;
  		case LED_COLOR_GREEN:
  			LED_GREEN_ON();
  			break;
  		case LED_COLOR_BLUE:
  			LED_BLUE_ON();
  			break;
  		default:
  			break;
  	}
  } else {
    TRI_COLOR_DEBUG("%s %d:brightness = false, color = %d\n", __func__, __LINE__, color);
  	switch(color) {
  		case LED_COLOR_RED:
  			LED_RED_OFF();
  			break;
  		case LED_COLOR_GREEN:
  			LED_GREEN_OFF();
  			break;
  		case LED_COLOR_BLUE:
  			LED_BLUE_OFF();
  			break;
  		default:
  			break;
  	}
}
spin_unlock_irqrestore(&tricolor_led->led_lock, flags);
}

static void led_brightness_set_flash(struct led_classdev *led_cdev,
				     enum led_brightness brightness)
{
  if(brightness){
  		gpio_set_value(qrd7_led_flash_en, 1);
  } else {
  		gpio_set_value(qrd7_led_flash_en, 0);
  }
  
  TRI_COLOR_DEBUG("%s %d:led_brightness_set_flash finish.\n", __func__, __LINE__);
}

/*************************************************
  Function:        debug_mask_set
  Description:     this func is to set the value to control log enable or disable
  Input:           data:address of register
                   val:to control log enable or disable
  Output:          none
  Return:          0
*************************************************/
static int debug_log_set(void *data, u64 val)
{
    tri_color_log_mask = (val >= 1) ? (int)val : 1;
    return 0;
}

/*************************************************
  Function:        debug_mask_get
  Description:     this func is to get the value of control log enable or disable
  Input:           data:address of register
  Output:          *val:to control log enable or disable
  Return:          0
*************************************************/
static int debug_log_get(void *data, u64 *val)
{
    *val = (u64)tri_color_log_mask;
    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_mask_fops,debug_log_get, debug_log_set, "%llu\n");

static int tricolor_led_probe(struct platform_device *pdev)
{
  int ret = 0;
  int i, j;
  struct tricolor_led_data *tricolor_led;

  /* used to create director before create file node that used to control debug log */
  struct dentry *dentry_trilog = NULL; 
  struct dentry *dentry_file = NULL;

  struct device_node *np = pdev->dev.of_node;
  unsigned gpio_config = 0;
  printk(KERN_ERR "tricolor leds and flashlight: probe init \n");

  red_gpio= of_get_named_gpio(np,"tricolor,red-gpio", 0);   /* get led gpio pin from dtsi file  */
  /* if red_gpio is 96 ,means the board is old hw design. new design is 98 */  
  if(96 == red_gpio )
  	strncpy(hw_version,"1.0",sizeof(hw_version));
  else
  	strncpy(hw_version,"2.0",sizeof(hw_version));
  ret = gpio_request(red_gpio, "tricolor red");        		/* request gpio popedom */
  if(ret < 0)												/* if request failed */
  {
      pr_err("%s:%d gpio_request %d fail\n",__func__,__LINE__,red_gpio);
  }
  else														/* if request success, config gpio */
  {
      /*parameter 1:gpio pin, 2:gpio work type, 3:input or out put, 4:pull up/down or no, 5: electricity 2MA */
      gpio_config = GPIO_CFG(red_gpio,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);/*  gpio config */
      ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);                               
      if(ret < 0) 											/* faild to config gpio */
	  {
          pr_err("%s:%d gpio_tlmm_config %d fail\n",__func__,__LINE__,red_gpio);
	  }
  }
  
  green_gpio = of_get_named_gpio(np, "tricolor,green-gpio", 0);/* get led gpio pin from dtsi file  */
  ret = gpio_request(green_gpio, "tricolor green");            /* request gpio popedom */
  if(ret < 0)												   /* if request failed */
  {
    pr_err("%s:%d gpio_request %d fail\n",__func__,__LINE__,green_gpio);
  }
  else														   /* if request success, config gpio */
  {
    gpio_config = GPIO_CFG(green_gpio,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);/*  gpio config */
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    if(ret < 0)												   /* faild to config gpio */
	{
        pr_err("%s:%d gpio_tlmm_config %d fail\n",__func__,__LINE__,green_gpio);
	}
  }
  
  blue_gpio = of_get_named_gpio(np,"tricolor,blue-gpio", 0);   /* config blue led gpio */
  ret = gpio_request(blue_gpio, "tricolor blue");
  if(ret < 0)												   /* if request failed */
  {
    pr_err("%s:%d gpio_request %d fail\n",__func__,__LINE__,blue_gpio);
  }
  else                                                         /* if request success, config gpio */
  {
    gpio_config = GPIO_CFG(blue_gpio,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);/*  gpio config */
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    if(ret < 0)
	{
        pr_err("%s:%d gpio_tlmm_config %d fail\n",__func__,__LINE__,blue_gpio);
    }
  }
  tricolor_led = kzalloc(sizeof(struct tricolor_led_data), GFP_KERNEL);
  if (tricolor_led == NULL) {
  	printk(KERN_ERR "tricolor_led_probe: no memory for device\n");
  	ret = -ENOMEM;
  	goto err;
  }
  memset(tricolor_led, 0, sizeof(struct tricolor_led_data));
  
  spin_lock_init(&tricolor_led->led_lock);
  
  tricolor_led->leds[0].name = "red";
  tricolor_led->leds[0].brightness_set = led_brightness_set_tricolor;
  
  tricolor_led->leds[1].name = "green";
  tricolor_led->leds[1].brightness_set = led_brightness_set_tricolor;
  
  tricolor_led->leds[2].name = "blue";
  tricolor_led->leds[2].brightness_set = led_brightness_set_tricolor;
  
  tricolor_led->leds[3].name = "flashlight_";
  tricolor_led->leds[3].brightness_set = led_brightness_set_flash;
  
  for (i = 0; i < 4; i++) {	/* red, green, blue, flashlight */
  	ret = led_classdev_register(&pdev->dev, &tricolor_led->leds[i]);
  	printk(KERN_ERR "led_classdev_register tricolor_leds init\n");
  	if (ret) {
  		printk(KERN_ERR
  		       "tricolor_led: led_classdev_register failed\n");
  		goto err_led_classdev_register_failed;
  	}
  }
  
  for (i = 0; i < 4; i++) {
  	ret = device_create_file(tricolor_led->leds[i].dev, &dev_attr_blink);
  	if (ret) {
  		printk(KERN_ERR
  		       "tricolor_led: device_create_file failed\n");
  		goto err_out_attr_blink;
  	}
  }
  for (i = 0; i < 4; i++) {
  	ret = device_create_file(tricolor_led->leds[i].dev, &dev_attr_hardware_version);
  	if (ret) {
		TRI_COLOR_ERROR("%s %d:device_create_file failed \n", __func__, __LINE__);
  		goto err_out_attr_hardware_version;
  	}
  }
  dev_set_drvdata(&pdev->dev, tricolor_led);

  /* create directory "hw_tri_color_log" */
  dentry_trilog = debugfs_create_dir("hw_tri_color_log", NULL);
  if(dentry_trilog)
  {
      /* create file named debug_mask at /sys/kernel/debug/hw_tri_color_log */
      dentry_file = debugfs_create_file("debug_mask", 0664, dentry_trilog, NULL, &debug_mask_fops);
      if (!dentry_file)
      {
          TRI_COLOR_ERROR("%s %d:Create log file error! \n", __func__, __LINE__);
          debugfs_remove(dentry_trilog);
          /* if can not create file, no need to return, because the device can run well */
      }
  }
  else
  {
      TRI_COLOR_ERROR("%s %d:Create log director error! \n", __func__, __LINE__);
  }
  
  handle_tricolors = msm_rpm_create_request(MSM_RPM_CTX_ACTIVE_SET,
      rpm_vreg_string_to_int("leds"), 0, 1);
  return 0;
err_out_attr_hardware_version:
  for (j = 0; j < i; j++)
  	device_remove_file(tricolor_led->leds[j].dev, &dev_attr_hardware_version);
  i = 4;
err_out_attr_blink:
  for (j = 0; j < i; j++)
  	device_remove_file(tricolor_led->leds[j].dev, &dev_attr_blink);
  i = 4;

err_led_classdev_register_failed:
  for (j = 0; j < i; j++)
	led_classdev_unregister(&tricolor_led->leds[j]);
err:
  return ret;
}

static int __devexit tricolor_led_remove(struct platform_device *pdev)
{
	struct tricolor_led_data *tricolor_led;
	int i;
	printk(KERN_ERR "tricolor_led_remove: remove\n");

	tricolor_led = platform_get_drvdata(pdev);

	for (i = 0; i < 4; i++) {
		device_remove_file(tricolor_led->leds[i].dev, &dev_attr_blink);
		led_classdev_unregister(&tricolor_led->leds[i]);
	}
	
	kfree(tricolor_led);
	return 0;
}
static struct of_device_id tricolor_match_table[] = {
	{	.compatible = "huawei,tricolor_led",
	}
};
static struct platform_driver tricolor_led_driver = {
	.probe = tricolor_led_probe,
	.remove = __devexit_p(tricolor_led_remove),
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = "tricolor leds and flashlight",
           .of_match_table = tricolor_match_table,
		   .owner = THIS_MODULE,
		   },
};
static int __init tricolor_led_init(void)
{
	printk(KERN_ERR "tricolor_leds_backlight_init: module init\n");
	return platform_driver_register(&tricolor_led_driver);
}
static void __exit tricolor_led_exit(void)
{
	printk(KERN_ERR "tricolor_leds_backlight_exit: module exit\n");
	if (gpio_is_valid(red_gpio))/* check red_led gpio if have requested */
	{
		gpio_free(red_gpio);
	}
	
	if (gpio_is_valid(blue_gpio))/* check blue_led gpio if have requested */
	{
    	gpio_free(blue_gpio);
	}
    
	if (gpio_is_valid(green_gpio))/* check green_led gpio if have requested */
	{
		gpio_free(green_gpio);
	}
	platform_driver_unregister(&tricolor_led_driver);
}

MODULE_AUTHOR("Glory Liao");
MODULE_DESCRIPTION("tricolor leds and flashlight driver");
MODULE_LICENSE("GPL");

module_init(tricolor_led_init);
module_exit(tricolor_led_exit);
