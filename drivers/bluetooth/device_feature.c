/*

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.


   Copyright (C) 2011-2012  Huawei Corporation
*/

#include <linux/module.h>    /* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/param.h>
#include <linux/termios.h>

#include <net/bluetooth/bluetooth.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <media/v4l2-dev.h>
#include<media/radio-iris.h>
//#include <hsad/config_interface.h>

#define BT_DEVICE_DBG
#ifndef BT_DEVICE_DBG
#define BT_DBG(fmt, arg...)
#endif

#define LOG_TAG "FeatureTransfer"
/*
 * Defines
 */

#define VERSION            "1.0"
#define PROC_DIR           "device_feature"
#define MAX_BT_FEATURE_LEN 20
#define BT_FIRMWARE_UNKOWN    "Unkown"
#define BT_POWER_DEFAULT    "default"
struct proc_dir_entry *device_dir, *bt_dir, *fm_dir;
typedef enum
{
    FM_SINR_5 = 5,
    FM_SINR_6 = 6,
    FM_SINR_7 = 7,
    FM_SINR_8 = 8,
    FM_SINR_MAX = 0xff
}fm_sinr_value;
typedef enum 
{
    FM_SINR_SAMPLES_9 = 9,
    FM_SINR_SAMPLES_10 = 10,
    FM_SINR_SAMPLES_MAX = 0xff
}fm_sinr_samples;
static int g_sinr_threshold = FM_SINR_MAX;
static int g_sinr_samples = FM_SINR_SAMPLES_MAX;
struct fm_device
{
    char *chip_type;
    char *dev_name;
};
const struct fm_device fm_device_array[] = 
{
    { "BT_FM_QUALCOMM_WCN3620", "2.2" },
    { "BT_FM_UNKNOWN_DEVICE", "Unknown" }
};
    const void *get_bt_fm_device_type(void)
{
           int chip_type_len;
      struct device_node *dp = NULL;
      dp = of_find_node_by_path("/huawei_fm_info");
       if(!of_device_is_available(dp))
      {
            FMDERR("device is not available!\n");
            return NULL;
      }
       
       return  of_get_property(dp,"fm,chiptype", &chip_type_len);
}
static char *get_bt_fm_device_name(void)
{  
    int i = 0;
    int arry_size = sizeof(fm_device_array)/sizeof(fm_device_array[0]);
         const char *bt_fm_chip_type = "BT_FM_UNKNOWN_DEVICE";

    /* get bt/fm chip type from the device feature configuration (.xml file) */
         bt_fm_chip_type = get_bt_fm_device_type();
    if(NULL == bt_fm_chip_type)
    {
        FMDERR("BT-FM, Get chip type fail.\n");
        return fm_device_array[arry_size - 1].dev_name;
    }
    
    /* lookup bt_device_model in bt_device_array[] */
    for(i = 0; i < arry_size; i++)
    {
        if(0 == strncmp(bt_fm_chip_type,fm_device_array[i].chip_type,sizeof(fm_device_array[i].chip_type)))
        {
            break;
        }
    }
    /* If we get invalid type name, return "Unknown".*/
    if( i == arry_size)
    {
        FMDERR("BT-FM, Get chip type fail.\n");
        return fm_device_array[arry_size - 1].dev_name;
    }

    return fm_device_array[i].dev_name;
}

        int get_fm_sinr_threshold_string(int *sinr)
   {
           int sinr_threshold_len;
      int sinr_threshold;
      const char *psinr_threshold = NULL;
      struct device_node *dp = NULL;
      if(NULL == sinr)
        {
                FMDERR("invalid argument !\n");
           return -1;
        }
      dp = of_find_node_by_path("/huawei_fm_info");
      if(!of_device_is_available(dp))
      {
            FMDERR("device is not available!\n");
            return -1;
      }  
           psinr_threshold = of_get_property(dp,"fm,sinr_threshold", &sinr_threshold_len);
           if (NULL == psinr_threshold)
        {
              FMDERR("get sinr threshold value failed .\n");
         return -1;
        }
           sinr_threshold = simple_strtol(psinr_threshold,NULL,10);
      *sinr = sinr_threshold;
           return 0;
    }
    

       int get_fm_sinr_samples(unsigned int *samples)
{
           int ret;
      struct device_node *dp = NULL;
      if (NULL == samples)
       {
                 FMDERR("get sinr samples failed !\n");
            return -1;
       }
       dp = of_find_node_by_path("/huawei_fm_info");
      if(!of_device_is_available(dp))
      {
                FMDERR("device is not available!\n");
                return -1;
      }
            ret = of_property_read_u32(dp,"fm,sinr_samples", samples);
     if (ret < 0)
     {
                 FMDERR("get sinr samples failed .\n");
            return -1;
      }
      return  0;
}
static int featuretransfer_remove(struct platform_device *pdev)
{
    FMDBG("devicefeature removed.");
    return 0;
}

static struct platform_driver featuretransfer_driver = {
    .remove = featuretransfer_remove,
    .driver = {
        .name = "featuretransfer",
        .owner = THIS_MODULE,
    },
};

static int devicefeature_read_proc_sinr_threshold(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
          
     int sinr_threshold = FM_SINR_MAX;
     int ret ;
    //null pointer protection
    if(NULL == page ||NULL == eof)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
     *eof = 1;
     ret = get_fm_sinr_threshold_string(&sinr_threshold);
     if(-1 == ret)
     {
         // 7 is the default value
         FMDERR("Get FM Sinr Threshold failed and will use default value 7.\n");
         sinr_threshold = FM_SINR_7;
     }
        
     /* if we changed the sinr value, get the new value. used for debug */
     if(FM_SINR_MAX != g_sinr_threshold)
     {
          sinr_threshold = g_sinr_threshold;
     }

          FMDBG("in the proc file systerm ,sinr_threshold = %d .\n",sinr_threshold);
          return sprintf(page, "%d\n",sinr_threshold);
}

/**
 * Write the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int devicefeature_write_proc_sinr_threshold(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{
    /* devicefeature_write_proc_sinr_threshold function is not used now.*/
    char sinr_threshold[10];

    FMDERR("sinr threshold write called.\n");

    //null pointer protection
    if(NULL == buffer)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
    if( 10 < count)
     return -1;
    if(copy_from_user(sinr_threshold, buffer,count))
     return -2;

    g_sinr_threshold = simple_strtol(sinr_threshold,NULL,10); //simple_strtol: same as atoi() in userspace
    return count;

}

static int devicefeature_read_proc_sinr_samples(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
         
    unsigned int sinr_samples = FM_SINR_SAMPLES_MAX;
    int ret;
    //null pointer protection
    if(NULL == page ||NULL == eof)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
    *eof = 1;
    ret = get_fm_sinr_samples(&sinr_samples);
    if(-1 == ret)
    {
        // 100 is the default value
        FMDERR("Get FM Sinr Samples failed and will use default value 10.\n");
        sinr_samples = FM_SINR_SAMPLES_10;
    }
          if(FM_SINR_SAMPLES_MAX != g_sinr_samples)
     {
          sinr_samples = g_sinr_samples;
     }
          FMDBG("in the proc file systerm ,sinr_samples = %d .\n",sinr_samples);
    return sprintf(page, "%d\n",sinr_samples);
}

static int devicefeature_write_proc_sinr_samples(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{
    /* devicefeature_write_proc_sinr_samples function is not used now.*/
    char sinr_samples[10];

    FMDBG("sinr samples write called.\n");

    //null pointer protection
    if(NULL == buffer)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
    if( 10 < count)
     return -1;
    if(copy_from_user(sinr_samples, buffer,count))
     return -2;

    g_sinr_samples = simple_strtol(sinr_samples,NULL,10); //simple_strtol: same as atoi() in userspace
    return count;

}
static int devicefeature_read_proc_chiptype(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
    const char *bt_fm_device_name = NULL;
    //null pointer protection
    if(NULL == page ||NULL == eof)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
    *eof = 1;
    
    bt_fm_device_name = get_bt_fm_device_name();
    /* Only the chip type info such as 1.2 or 2.2 is recorded. */
    return sprintf(page, "%s", bt_fm_device_name);
}

static int devicefeature_write_proc_chiptype(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{
    /* the function is not used now.  */
    return 0;
}

/**
 * Read the data via the proc interface.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int devicefeature_read_proc_bt_fm_fw_ver(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
    const char *bt_fm_fw_ver = NULL;
    int bt_fm_fw_ver_len = 0;
    struct device_node *dp = NULL;
    //null pointer protection
    if(NULL == page ||NULL == eof)
    {
        FMDERR("[%s]: Invlid value.line:%d\n",__FUNCTION__,__LINE__);
        return -1;
    }
    *eof = 1;

    dp = of_find_node_by_path("/huawei_bt_info");
    if (!of_device_is_available(dp))
    {
        FMDERR("device is not available!\n");
        return sprintf(page, "%s", BT_FIRMWARE_UNKOWN);
    }

    bt_fm_fw_ver = of_get_property(dp, "bt,fw_ver", &bt_fm_fw_ver_len);
    if(NULL == bt_fm_fw_ver)
    {
        FMDERR("Get Bt fw_ver  failed.\n");
        return sprintf(page, "%s", BT_FIRMWARE_UNKOWN);
    }

    return sprintf(page, "%s", bt_fm_fw_ver);
}
/**
 * Read the data via the proc interface.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int devicefeature_read_proc_bt_power_level(char *page, char **start, off_t offset,
                    int count, int *eof, void *data)
{
    const char *bt_power_level = NULL;
    int len = 0;
    struct device_node *dp = NULL;

    *eof = 1;

    dp = of_find_node_by_path("/huawei_bt_info");
    if (!of_device_is_available(dp))
    {
        FMDERR("device is not available!\n");
        return sprintf(page, "%s", BT_POWER_DEFAULT);
    }

    bt_power_level = of_get_property(dp, "bt,power_level", &len);
    if (NULL == bt_power_level)
    {
        FMDERR("Get Bt power_level failed.\n");
        return sprintf(page, "%s", BT_POWER_DEFAULT);
    }

    return sprintf(page, "%s", bt_power_level);
}

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init featuretransfer_init(void)
{
    int retval = 0;
    struct proc_dir_entry *ent = NULL;

    FMDBG("BT DEVICE FEATURE VERSION: %s", VERSION);

    /* Driver Register */
    retval = platform_driver_register(&featuretransfer_driver);
    if (0 != retval)
    {
        FMDERR("[%s],featurntransfer driver register fail.",LOG_TAG);
        return retval;
    }

    /* create device_feature directory for bt chip info */
    device_dir = proc_mkdir("device_feature", NULL);
    if (NULL == device_dir)
    {
        FMDERR("Unable to create /proc/device_feature directory");
        return -ENOMEM;
    }

    /* create bt_feature for bluetooth feature */
    bt_dir = proc_mkdir("bt_feature", device_dir);
    if (NULL == bt_dir)
    {
        FMDERR("Unable to create /proc/%s directory", PROC_DIR);
        return -ENOMEM;
    }

    /* Creating read/write "chiptype" entry for bluetooth chip type*/
    ent = create_proc_entry("chiptype", 0, bt_dir);
    if (NULL == ent) 
    {
        FMDERR("Unable to create /proc/%s/chiptype entry", PROC_DIR);
        retval = -ENOMEM;
        goto fail;
    }
    ent->read_proc = devicefeature_read_proc_chiptype;
    ent->write_proc = devicefeature_write_proc_chiptype;
    /* Creating read/write "bt_fm_fw_ver" entry */
    ent = create_proc_entry("bt_fm_fw_ver", 0, bt_dir);
    if (NULL == ent) 
    {
        FMDERR("Unable to create /proc/%s/bt_fm_fw_ver entry", PROC_DIR);
        retval = -ENOMEM;
        goto fail;
    }
    ent->read_proc = devicefeature_read_proc_bt_fm_fw_ver;
    /* Creating read/write "bt_power_level" entry */
    ent = create_proc_entry("power_level", 0, bt_dir);
    if (NULL == ent)
    {
        FMDERR("Unable to create /proc/%s/power_level entry", PROC_DIR);
        retval = -ENOMEM;
        goto fail;
    }
    ent->read_proc = devicefeature_read_proc_bt_power_level;
    /* create fm_feature for fm feature */
    fm_dir = proc_mkdir("fm_feature", device_dir);
    if (NULL == fm_dir)
    {
        FMDERR("Unable to create /proc/%s directory", PROC_DIR);
        return -ENOMEM;
    }

    /* Creating read/write "sinr" entry for bluetooth chip type*/
    ent = create_proc_entry("sinr_threshold", 0, fm_dir);
    if (NULL == ent) 
    {
        FMDERR("Unable to create /proc/%s/sinr_threshold entry", PROC_DIR);
        retval = -ENOMEM;
        goto fail;
    }
    ent->read_proc = devicefeature_read_proc_sinr_threshold;
    ent->write_proc = devicefeature_write_proc_sinr_threshold;


    /* Creating read/write "rssi" entry for bcm4330 fm*/
    ent = create_proc_entry("sinr_samples", 0, fm_dir);
    if (NULL == ent) 
    {
        FMDERR("Unable to create /proc/%s/sinr_samples entry", PROC_DIR);
        retval = -ENOMEM;
        goto fail;
    }
    ent->read_proc = devicefeature_read_proc_sinr_samples;
    ent->write_proc = devicefeature_write_proc_sinr_samples;

    
    return 0;

fail:
    remove_proc_entry("chiptype", bt_dir);
    remove_proc_entry("bt_fm_fw_ver", bt_dir);
    remove_proc_entry("power_level", bt_dir);
    remove_proc_entry("sinr_threshold", fm_dir);
    remove_proc_entry("sinr_samples", fm_dir);
    remove_proc_entry("bt_feature", device_dir);
    remove_proc_entry("fm_feature", device_dir);
    remove_proc_entry("device_feature", 0);
    return retval;
}


/**
 * Cleans up the module.
 */
static void __exit featuretransfer_exit(void)
{
    platform_driver_unregister(&featuretransfer_driver);
    remove_proc_entry("chiptype", bt_dir);
    remove_proc_entry("bt_fm_fw_ver", bt_dir);
    remove_proc_entry("power_level", bt_dir);
    remove_proc_entry("sinr_threshold", fm_dir);
    remove_proc_entry("sinr_samples", fm_dir);
    remove_proc_entry("bt_feature", device_dir);
    remove_proc_entry("fm_feature", device_dir);
    remove_proc_entry("device_feature", 0);
}



module_init(featuretransfer_init);
module_exit(featuretransfer_exit);

MODULE_DESCRIPTION("BT DEVICE FEATURE VERSION: %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
