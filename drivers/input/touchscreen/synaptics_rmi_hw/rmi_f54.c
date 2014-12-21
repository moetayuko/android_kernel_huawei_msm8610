/*
 * Copyright (c) 2011 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/delay.h>
#include "rmi_driver.h"

#include "rmi_config.h"
#include <linux/hw_tp_config.h>

#define RX_NUMBER 44  //f01 control_base_addr + 44
#define TX_NUMBER 45  //f01 control_base_addr + 45

/* Set this to 1 for raw hex dump of returned data. */
#define RAW_HEX 0
/* Set this to 1 for human readable dump of returned data. */
#define HUMAN_READABLE 0
/* The watchdog timer can be useful when debugging certain firmware related
 * issues.
 */
#define F54_WATCHDOG 1

static struct completion mmi_comp;
static struct completion mmi_sync;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)
#define KERNEL_VERSION_ABOVE_2_6_32 1
#endif

/* define fn $54 commands */
#define GET_REPORT                1
#define FORCE_CAL                 2

/* status */
#define BUSY 1
#define IDLE 0

/* Offsets for data */
#define RMI_F54_REPORT_DATA_OFFSET	3
#define RMI_F54_FIFO_OFFSET		1
#define RMI_F54_NUM_TX_OFFSET		1
#define RMI_F54_NUM_RX_OFFSET		0

/* Fixed sizes of reports */
#define RMI_54_FULL_RAW_CAP_MIN_MAX_SIZE 4
#define RMI_54_HIGH_RESISTANCE_SIZE 6

extern int synaptics_module_id;

/* definitions for F54 Query Registers in ultra-portable unionstruct form */
struct f54_ad_query {
	/* query 0 */
	u8 number_of_receiver_electrodes;

	/* query 1 */
	u8 number_of_transmitter_electrodes;

	union {
		struct {
			/* query2 */
			u8 f54_ad_query2_b0__1:2;
			u8 has_baseline:1;
			u8 has_image8:1;
			u8 f54_ad_query2_b4__5:2;
			u8 has_image16:1;
			u8 f54_ad_query2_b7:1;
		};
		u8 f54_ad_query2;
	};

	/* query 3.0 and 3.1 */
	u16 clock_rate;

	/* query 4 */
	u8 touch_controller_family;

	/* query 5 */
	union {
		struct {
			u8 has_pixel_touch_threshold_adjustment:1;
			u8 f54_ad_query5_b1__7:7;
		};
		u8 f54_ad_query5;
	};

	/* query 6 */
	union {
		struct {
		u8 has_sensor_assignment:1;
		u8 has_interference_metric:1;
		u8 has_sense_frequency_control:1;
		u8 has_firmware_noise_mitigation:1;
		u8 f54_ad_query6_b4:1;
		u8 has_two_byte_report_rate:1;
		u8 has_one_byte_report_rate:1;
		u8 has_relaxation_control:1;
		};
		u8 f54_ad_query6;
	};

	/* query 7 */
	union {
		struct {
			u8 curve_compensation_mode:2;
			u8 f54_ad_query7_b2__7:6;
		};
		u8 f54_ad_query7;
	};

	/* query 8 */
	union {
		struct {
		u8 f54_ad_query2_b0:1;
		u8 has_iir_filter:1;
		u8 has_cmn_removal:1;
		u8 has_cmn_maximum:1;
		u8 has_pixel_threshold_hysteresis:1;
		u8 has_edge_compensation:1;
		u8 has_perf_frequency_noisecontrol:1;
		u8 f54_ad_query8_b7:1;
		};
		u8 f54_ad_query8;
	};

	u8 f54_ad_query9;
	u8 f54_ad_query10;
	u8 f54_ad_query11;

	/* query 12 */
	union {
		struct {
			u8 number_of_sensing_frequencies:4;
			u8 f54_ad_query12_b4__7:4;
		};
		u8 f54_ad_query12;
	};
};

/* define report types */
enum f54_report_types {
	/* The numbering should follow automatically, here for clarity */
	F54_8BIT_IMAGE = 1,
	F54_16BIT_IMAGE = 2,
	F54_RAW_16BIT_IMAGE = 3,
	F54_HIGH_RESISTANCE = 4,
	F54_TX_TO_TX_SHORT = 5,
	F54_RX_TO_RX1 = 7,
	F54_TRUE_BASELINE = 9,
	F54_FULL_RAW_CAP_MIN_MAX = 13,
	F54_RX_OPENS1 = 14,
	F54_TX_OPEN = 15,
	F54_TX_TO_GROUND = 16,
	F54_RX_TO_RX2 = 17,
	F54_RX_OPENS2 = 18,
	F54_FULL_RAW_CAP = 19,
	F54_FULL_RAW_CAP_RX_COUPLING_COMP = 20
};

/* data specific to fn $54 that needs to be kept around */
struct rmi_fn_54_data {
	struct f54_ad_query query;
	u8 cmd;
	enum f54_report_types report_type;
	u16 fifoindex;
	signed char status;
	bool no_auto_cal;
	/*
	 * May need to do something to make sure this reflects what is currently
	 * in data.
	 */
	unsigned int report_size;
	unsigned char *report_data;
	unsigned int bufsize;
	struct mutex data_mutex;
	struct lock_class_key data_key;
	struct mutex status_mutex;
	struct lock_class_key status_key;
	struct mutex test_mutex; //Phoenix
	struct lock_class_key test_key;
#if F54_WATCHDOG
	struct hrtimer watchdog;
#endif
	struct rmi_function_container *fc;
	struct work_struct work;
};


static char buf_f54test_result[50] = {0};
#define MAX_F54_RAW_DATA_BUF_LEN 2024
static char g_buf_f54raw_data[MAX_F54_RAW_DATA_BUF_LEN] = {0};
static char g_highresistance_report[50] = {0};
static char g_maxmincapacitance_report[50] = {0};
static char g_RxtoRxshort_report[20 * 20 * 7] = {0};
static u8 g_tx_max = 0;
static char mmi_buf[500] = {0};
static int mmidata_size = 0;
static u8 tx = 0;
static u8 rx = 0;


/* sysfs functions */
static ssize_t rmi_fn_54_report_type_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_report_type_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static ssize_t rmi_fn_54_get_report_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_fn_54_force_cal_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_fn_54_status_show(struct device *dev,
				struct device_attribute *attr, char *buf);


static ssize_t rmi_fn_54_status_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count);

#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_54_data_read(struct file *data_file, struct kobject *kobj,
#else
static ssize_t rmi_fn_54_data_read(struct kobject *kobj,
#endif
					struct bin_attribute *attributes,
					char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_54_num_rx_electrodes_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_num_tx_electrodes_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_image16_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_image8_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_baseline_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_clock_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf);


static ssize_t rmi_fn_54_touch_controller_family_show(struct device *dev,
				struct device_attribute *attr, char *buf);


static ssize_t rmi_fn_54_has_pixel_touch_threshold_adjustment_show(
		struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_sensor_assignment_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_interference_metric_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_sense_frequency_control_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_firmware_noise_mitigation_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_two_byte_report_rate_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_one_byte_report_rate_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_relaxation_control_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_curve_compensation_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_iir_filter_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_cmn_removal_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_cmn_maximum_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_pixel_threshold_hysteresis_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_edge_compensation_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_has_perf_frequency_noisecontrol_show(
		struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_number_of_sensing_frequencies_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_no_auto_cal_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_no_auto_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static ssize_t rmi_fn_54_fifoindex_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_fifoindex_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);


/*create node to test  */
static ssize_t rmi_f54_mmi_test_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_f54_mmi_test_show(struct device *dev,
                   struct device_attribute *attr,char *buf);

static int rmi_f54_alloc_memory(struct rmi_function_container *fc);

static void rmi_f54_free_memory(struct rmi_function_container *fc);

static int rmi_f54_initialize(struct rmi_function_container *fc);

static int rmi_f54_config(struct rmi_function_container *fc);

static int rmi_f54_reset(struct rmi_function_container *fc);

static int rmi_f54_create_sysfs(struct rmi_function_container *fc);

static void set_report_size(struct rmi_fn_54_data *data);

static int f54_rawimage_report(void);

static int f54_txtotx_short_report (void);
static int f54_txtoground_short_report(void);
static int f54_RxtoRxshort_report (struct rmi_fn_54_data *data,size_t count);
static int f54_highresistance_report(void);
static int f54_maxmincapacitance_report(void);

static void mmi_rawcapacitance_test(struct device *dev,size_t count);
static void mmi_rxtorxshort_test(struct device *dev,size_t count);
static void mmi_txtotxshort_test(struct device *dev,size_t count);
static void mmi_txtogroundshort_test(struct device *dev,size_t count);
static void mmi_highresistance_test(struct device *dev,size_t count);
static void mmi_maxmin_test(struct device *dev,size_t count);
static int write_to_f54_register(struct rmi_function_container *fc,unsigned char report_type,size_t count);
static struct device_attribute attrs[] = {
	__ATTR(report_type, RMI_RW_ATTR,
		rmi_fn_54_report_type_show, rmi_fn_54_report_type_store),
	__ATTR(get_report, RMI_WU_ATTR,
		rmi_show_error, rmi_fn_54_get_report_store),
	__ATTR(force_cal, RMI_WU_ATTR,
		   rmi_show_error, rmi_fn_54_force_cal_store),
	__ATTR(status, RMI_RW_ATTR,
		rmi_fn_54_status_show, rmi_fn_54_status_store),
	__ATTR(num_rx_electrodes, RMI_RO_ATTR,
		rmi_fn_54_num_rx_electrodes_show, rmi_store_error),
	__ATTR(num_tx_electrodes, RMI_RO_ATTR,
		rmi_fn_54_num_tx_electrodes_show, rmi_store_error),
	__ATTR(has_image16, RMI_RO_ATTR,
		rmi_fn_54_has_image16_show, rmi_store_error),
	__ATTR(has_image8, RMI_RO_ATTR,
		rmi_fn_54_has_image8_show, rmi_store_error),
	__ATTR(has_baseline, RMI_RO_ATTR,
		rmi_fn_54_has_baseline_show, rmi_store_error),
	__ATTR(clock_rate, RMI_RO_ATTR,
		rmi_fn_54_clock_rate_show, rmi_store_error),
	__ATTR(touch_controller_family, RMI_RO_ATTR,
		rmi_fn_54_touch_controller_family_show, rmi_store_error),
	__ATTR(has_pixel_touch_threshold_adjustment, RMI_RO_ATTR,
		rmi_fn_54_has_pixel_touch_threshold_adjustment_show
							, rmi_store_error),
	__ATTR(has_sensor_assignment, RMI_RO_ATTR,
		rmi_fn_54_has_sensor_assignment_show, rmi_store_error),
	__ATTR(has_interference_metric, RMI_RO_ATTR,
		rmi_fn_54_has_interference_metric_show, rmi_store_error),
	__ATTR(has_sense_frequency_control, RMI_RO_ATTR,
		rmi_fn_54_has_sense_frequency_control_show, rmi_store_error),
	__ATTR(has_firmware_noise_mitigation, RMI_RO_ATTR,
		rmi_fn_54_has_firmware_noise_mitigation_show, rmi_store_error),
	__ATTR(has_two_byte_report_rate, RMI_RO_ATTR,
		rmi_fn_54_has_two_byte_report_rate_show, rmi_store_error),
	__ATTR(has_one_byte_report_rate, RMI_RO_ATTR,
		rmi_fn_54_has_one_byte_report_rate_show, rmi_store_error),
	__ATTR(has_relaxation_control, RMI_RO_ATTR,
		rmi_fn_54_has_relaxation_control_show, rmi_store_error),
	__ATTR(curve_compensation_mode, RMI_RO_ATTR,
		rmi_fn_54_curve_compensation_mode_show, rmi_store_error),
	__ATTR(has_iir_filter, RMI_RO_ATTR,
		rmi_fn_54_has_iir_filter_show, rmi_store_error),
	__ATTR(has_cmn_removal, RMI_RO_ATTR,
		rmi_fn_54_has_cmn_removal_show, rmi_store_error),
	__ATTR(has_cmn_maximum, RMI_RO_ATTR,
		rmi_fn_54_has_cmn_maximum_show, rmi_store_error),
	__ATTR(has_pixel_threshold_hysteresis, RMI_RO_ATTR,
		rmi_fn_54_has_pixel_threshold_hysteresis_show, rmi_store_error),
	__ATTR(has_edge_compensation, RMI_RO_ATTR,
		rmi_fn_54_has_edge_compensation_show, rmi_store_error),
	__ATTR(has_perf_frequency_noisecontrol, RMI_RO_ATTR,
	      rmi_fn_54_has_perf_frequency_noisecontrol_show, rmi_store_error),
	__ATTR(number_of_sensing_frequencies, RMI_RO_ATTR,
		rmi_fn_54_number_of_sensing_frequencies_show, rmi_store_error),
	__ATTR(no_auto_cal, RMI_RW_ATTR,
		rmi_fn_54_no_auto_cal_show, rmi_fn_54_no_auto_cal_store),
	__ATTR(fifoindex, RMI_RW_ATTR,
		rmi_fn_54_fifoindex_show, rmi_fn_54_fifoindex_store),
};
struct bin_attribute dev_rep_data = {
	.attr = {
		 .name = "rep_data",
		 .mode = RMI_RO_ATTR},
	.size = 0,
	.read = rmi_fn_54_data_read,
};

#if F54_WATCHDOG
static enum hrtimer_restart clear_status(struct hrtimer *timer);

static void clear_status_worker(struct work_struct *work);
#endif

/*create file node*/
static struct device_attribute f54_mmi_test_att = {
	.attr = {
		.name = "mmi_test",
		.mode = RMI_RW_ATTR,
	},
	.store = rmi_f54_mmi_test_store,

};
static struct device_attribute f54_mmi_test_data_att = {
	.attr = {
		.name = "mmi_test_result",
		.mode = RMI_RO_ATTR,
	},
	.show = rmi_f54_mmi_test_show,
};
#define CBC_OFFSET 8
#define DEFAULT_TIMEOUT 10
#define CDM_OFFSET 0x51
static bool  rmi_f54_disable_cbc(struct device * dev)
{

    unsigned char command = 0;
	unsigned char data  = 0;
	int count = 0;
	struct rmi_device *rmi_dev = NULL;
	struct rmi_function_container *fc = NULL; 
	struct rmi_function_container *f01_fc = NULL;
	struct rmi_driver_data *driver_data = NULL;

    fc = to_rmi_function_container(dev); 
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc =driver_data->f01_container;

	// Turn off CBC.
	data = 0;
	rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr + CBC_OFFSET, &data, 1);
	rmi_read_block(fc->rmi_dev, fc->fd.control_base_addr + CDM_OFFSET, &data, 1);
	data = data | 0x01;
	rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr + CDM_OFFSET, &data, 1);

	// Apply ForceUpdate.
	rmi_read_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);
	data = data | 0x04;
	rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);
	
	// Wait complete
	do 
	{
		rmi_read_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);
		msleep(50);//50ms
		count++;
	} while ((0x00 != data) && (count < DEFAULT_TIMEOUT));
	
	if(count >= DEFAULT_TIMEOUT)
	{
		printk("%s %d :Timeout -- ForceUpdate can not complete\n", __func__, __LINE__);
		rmi_write_block(fc->rmi_dev, f01_fc->fd.command_base_addr,&command, 1);
		return false;
	}

	// Apply ForceCal.
	rmi_read_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);
	data = data | 0x02;
	rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);

	// Wait complete
	count = 0;
	do
	{
		rmi_read_block(fc->rmi_dev, fc->fd.command_base_addr, &data, 1);
		msleep(50);//50ms
		count++;
	} while ((0x00 != data) && (count < DEFAULT_TIMEOUT));

	if(count >= DEFAULT_TIMEOUT)
	{
		 printk("%s %d :Timeout -- ForceCal can not complete\n", __func__, __LINE__);

		 /*reset TP*/
		 rmi_write_block(fc->rmi_dev, f01_fc->fd.command_base_addr,&command, 1);
		return false;
	}
	return true;
}
/* add by synaptics Phoenix to modify test >*/
/*the entrance of function is used to mmi test */
static ssize_t rmi_f54_mmi_test_store(struct device *dev,struct device_attribute *attr,
				   const char *buf, size_t count)
{		
    unsigned long val;	
	int result;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *driver_data;
	struct rmi_function_container *fc;
	struct rmi_function_container *f01_fc;
	struct rmi_fn_54_data* data;//Phoenix
	unsigned char command;
						
	fc = to_rmi_function_container(dev);
	data = fc->data;
	mutex_lock(&data->test_mutex); //Phoenix
	rmi_dev = fc->rmi_dev;
	driver_data = rmi_get_driverdata(rmi_dev);
	f01_fc =driver_data->f01_container;
	//reset
	command = 1;//Phoenix
	result = rmi_write_block(fc->rmi_dev, f01_fc->fd.command_base_addr,&command, 1);
	if (result < 0)
	{
	    printk("failed to write command to f01 reset! \n");
		goto MMI_TEST_FAILED;
	}
	/*TODO: check is 2 sec required here*/
	msleep (2000);
	/* clear all the buf before test */
	memset(buf_f54test_result,0,sizeof(buf_f54test_result));
	memset(g_buf_f54raw_data, 0, sizeof(g_buf_f54raw_data));
	memset(g_highresistance_report, 0, sizeof(g_highresistance_report));
	memset(g_maxmincapacitance_report, 0, sizeof(g_maxmincapacitance_report));
	memset(g_RxtoRxshort_report, 0, sizeof(g_RxtoRxshort_report));
	
	result = strict_strtoul(buf,10,&val);		
	if (result)
	{
		mutex_unlock(&data->test_mutex);  //Phoenix
	    return result;
	}

	result = rmi_read_block(fc->rmi_dev, fc->fd.data_base_addr, &command, 1);
	
	
	if (result < 0)
	{   
		printk("%s query data_base_addr Failed\n",__func__);
		goto MMI_TEST_FAILED;	
	}

	/* before test ,we should disabel CBC */
	if(false == rmi_f54_disable_cbc(dev))
	{   
		printk("%s failed to rmi_f54_disable_cbc\n", __func__);
		goto MMI_TEST_FAILED;
	}
		
	memcpy(buf_f54test_result, "0P-", (strlen("0P-")+1));
	
		mmi_highresistance_test(dev,count); 
		
		INIT_COMPLETION(mmi_sync);
		wait_for_completion(&mmi_sync);
		mmi_maxmin_test(dev,count);

		INIT_COMPLETION(mmi_sync);
		wait_for_completion(&mmi_sync);
		mmi_rawcapacitance_test(dev,count);
		
		INIT_COMPLETION(mmi_sync);
		wait_for_completion(&mmi_sync);
		mmi_rxtorxshort_test(dev,count);

		
		INIT_COMPLETION(mmi_sync);
		wait_for_completion(&mmi_sync);
		mmi_txtogroundshort_test(dev,count);

		INIT_COMPLETION(mmi_sync);
		wait_for_completion(&mmi_sync);
		mmi_txtotxshort_test(dev,count);
		
	command = 1;
	result = rmi_write_block(fc->rmi_dev, f01_fc->fd.command_base_addr,&command, 1);
	if (result < 0)
	{
	    printk("failed to write command to f01 reset! \n");
		mutex_unlock(&data->test_mutex); //Phoenix
		return result;
	}
	/*TODO: check is 2 sec required here*/
	msleep (2000);
	
	goto MMI_TEST_SUCCEED;

MMI_TEST_FAILED:
		memcpy(buf_f54test_result,"0F-1F-2F-3F-4F-5F-6F",(strlen("0F-1F-2F-3F-4F-5F-6F")+1));

MMI_TEST_SUCCEED:
	mutex_unlock(&data->test_mutex); //Phoenix
	return count;    
}

/*it is used to show the test result */
static ssize_t rmi_f54_mmi_test_show(struct device * dev,struct device_attribute * attr,char * buf)
{
	
	int len = strlen(buf_f54test_result);	
	memcpy (buf,buf_f54test_result,len+1);
	strcat(buf,"\n");
	strcat(buf, g_buf_f54raw_data);
	strcat(buf, g_highresistance_report);
	strcat(buf, g_maxmincapacitance_report);
	strcat(buf, g_RxtoRxshort_report);
	strcat(buf,"\0");	
	len=strlen(buf_f54test_result)+strlen(g_buf_f54raw_data)+1
		+strlen(g_highresistance_report)+ strlen(g_maxmincapacitance_report)
		+strlen(g_RxtoRxshort_report);
	return len;	
}

static void mmi_rawcapacitance_test(struct device *dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *rawimage_data;
	unsigned char command;
	int result = 0;
    
	fc = to_rmi_function_container(dev);
	rawimage_data = fc->data;
	rawimage_data->report_type = F54_FULL_RAW_CAP_RX_COUPLING_COMP;
	command = (unsigned char) F54_FULL_RAW_CAP_RX_COUPLING_COMP;
	
	write_to_f54_register(fc,command,count);
	result = f54_rawimage_report();
	if (1 == result)
	{
	    strcat(buf_f54test_result,"1P-");
	}
	else
	{
	    strcat(buf_f54test_result,"1F-");
	}	
}

static void mmi_txtotxshort_test(struct device * dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *txtotx_data;
	unsigned char command;
	int result;

	fc = to_rmi_function_container(dev);
	txtotx_data = fc->data;
	txtotx_data->report_type = F54_TX_TO_TX_SHORT;
	command = (unsigned char) F54_TX_TO_TX_SHORT;
	
	write_to_f54_register(fc,command,count);

	result = f54_txtotx_short_report();

	if (1 == result)
	{
	    strcat(buf_f54test_result,"2P-");
	}
	else
	{
	    strcat(buf_f54test_result,"2F-");
	}
}
static void mmi_txtogroundshort_test(struct device * dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *txtoground_data;
	unsigned char command;
	int result;

	fc = to_rmi_function_container(dev);
	txtoground_data = fc->data;
	txtoground_data->report_type = F54_TX_TO_GROUND;
	command = (unsigned char) F54_TX_TO_GROUND;
	
	write_to_f54_register(fc,command,count);
	result = f54_txtoground_short_report();

	if (1 == result)
	{
	    strcat(buf_f54test_result,"3P-");
	}
	else
	{
	    strcat(buf_f54test_result,"3F-");		
	}
}

static void mmi_rxtorxshort_test(struct device * dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *rawimage_data;
	unsigned char command;
	int result;

	fc = to_rmi_function_container(dev);
	rawimage_data = fc->data;
	rawimage_data->report_type = F54_RX_TO_RX1;
	command = (unsigned char) F54_RX_TO_RX1;
	
	if (rawimage_data->status != BUSY)
	{
	    result = 
			rmi_write_block(fc->rmi_dev, fc->fd.data_base_addr,&command, 1); //report_type
	
	    mutex_unlock(&rawimage_data->status_mutex);
	    if (result < 0) 
	    {
		    dev_err(&fc->dev, "%s : Could not write report type to"
			    " 0x%x\n", __func__, fc->fd.data_base_addr);
		    return ;
	    }
	}
    command = 4;		
	mutex_lock(&rawimage_data->status_mutex);
	result = rmi_write_block(fc->rmi_dev, fc->fd.query_base_addr + 9,&command, 1);
	printk("query_base_addr = %0x \n", fc->fd.query_base_addr);
    mutex_unlock(&rawimage_data->status_mutex);
	if (result < 0)
	    return ;
	
	command = 1;		
	mutex_lock(&rawimage_data->status_mutex);
	result = rmi_write_block(fc->rmi_dev,0x015E,&command, 1);
	printk("control_base_addr = %0x \n", fc->fd.control_base_addr);
    mutex_unlock(&rawimage_data->status_mutex);
	if (result < 0)
	    return ;
	
	command = 4;
	mutex_lock(&rawimage_data->status_mutex);
	result = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,&command, 1);		
    mutex_unlock(&rawimage_data->status_mutex);
	if (result < 0)	
	    return ;
	
	do {
		mdelay(100); //wait 1ms
		result = rmi_read_block(fc->rmi_dev,fc->fd.command_base_addr,&command, 1);
	} while (command != 0x00);	
	
	command = 2;
	mutex_lock(&rawimage_data->status_mutex);
	result = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,&command, 1);		
    mutex_unlock(&rawimage_data->status_mutex);
	if (result < 0)
	    return ;
	
	do {
		mdelay(100); //wait 1ms
		result = rmi_read_block(fc->rmi_dev,fc->fd.command_base_addr,&command, 1);
	} while (command != 0x00);	
	
	command = (unsigned char) F54_RX_TO_RX1;
	write_to_f54_register(fc,command,count);

	result = f54_RxtoRxshort_report(rawimage_data,count);

	if (1 == result)
	{
	    strcat(buf_f54test_result,"4P-");
	}
	else
	{
	    strcat(buf_f54test_result,"4F-");
	}
}

static void mmi_maxmin_test(struct device * dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *maxmin_data;
	unsigned char command;
	int result;

	fc = to_rmi_function_container(dev);
	maxmin_data = fc->data;
	maxmin_data->report_type = F54_FULL_RAW_CAP_MIN_MAX;
	command = (unsigned char) F54_FULL_RAW_CAP_MIN_MAX;
	
	write_to_f54_register(fc,command,count);
	result = f54_maxmincapacitance_report();

	if (1 == result)
	{
	    strcat(buf_f54test_result,"5P-");		
	}
	else
	{
	    strcat(buf_f54test_result,"5F-");
	}
}
static void mmi_highresistance_test(struct device * dev,size_t count)
{
    struct rmi_function_container *fc;
	struct rmi_fn_54_data *highresistance_data;
	unsigned char command;
	int result;

	fc = to_rmi_function_container(dev);
	highresistance_data = fc->data;
	highresistance_data->report_type = F54_HIGH_RESISTANCE;
	command = (unsigned char) F54_HIGH_RESISTANCE;
	
	write_to_f54_register(fc,command,count);
	result = f54_highresistance_report();

	if (1 == result)
	{
	    strcat(buf_f54test_result,"6P-");		
	}
	else
	{
	    strcat(buf_f54test_result,"6F-");
	}
	strcat(buf_f54test_result,"\0");
}

static int write_to_f54_register(struct rmi_function_container *fc,unsigned char report_type,size_t count)
{ 
    struct rmi_fn_54_data *data = fc->data;
	struct rmi_driver *driver;
	unsigned char command;
	int result;

	command = report_type;
	driver = fc->rmi_dev->driver;
	if (F54_RX_TO_RX1 != command)
	{
	    mutex_lock(&data->status_mutex);
	    if (data->status != BUSY)
	    {
	        result = 
			    rmi_write_block(fc->rmi_dev, fc->fd.data_base_addr,&command, 1);
	
	        mutex_unlock(&data->status_mutex);
	        if (result < 0) 
	        {
		        dev_err(&fc->dev, "%s : Could not write report type to"
			        " 0x%x\n", __func__, fc->fd.data_base_addr);
		        return result;
	        }
	    }
	}
	command = (unsigned char)GET_REPORT;
	
	mutex_lock(&data->status_mutex);
	if (data->status != IDLE)
	{
		if (data->status != BUSY)
		{
		    dev_err(&fc->dev, "F54 status is in an abnormal state: 0x%x \n",
						data->status);
		}
		mutex_unlock(&data->status_mutex);
		return count;
	}
	
	mdelay(2);
	
	if (driver->store_irq_mask)
		driver->store_irq_mask(fc->rmi_dev,fc->irq_mask);
	else
		dev_err(&fc->dev, "No way to store interupts!\n");			
	data->status = BUSY;
	result = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,&command, 1);
	msleep(20); //Phoenix
	mutex_unlock(&data->status_mutex);

#if F54_WATCHDOG
/* start watchdog timer */
	hrtimer_start(&data->watchdog, ktime_set(0, 700000000),HRTIMER_MODE_REL);
#endif
	return result;	
}

/* when the report type is  3 or 9, we call this function to  to find open
* transmitter electrodes, open receiver electrodes, transmitter-to-ground 
* shorts, receiver-to-ground shorts, and transmitter-to-receiver shorts. */
static int f54_rawimage_report ()
{
	short Rawimage;
	char buf_Rawimage[50]={0};
	short Result = 0;
	int i,k;
	u16 ic_name;
	int moudle_id;
	u16 *raw_cap_uplimit = NULL;
	u16 *raw_cap_lowlimit = NULL;
	
    ic_name = get_touch_ic();
	moudle_id = synaptics_module_id;
	
    printk("f54_rawimage_report moudle id is %d",moudle_id);
	/*the max uplimit value */
	raw_cap_uplimit = get_f54_raw_cap_uplimit(moudle_id,ic_name);
	if(NULL == raw_cap_uplimit)
	{
		return 0;
	}
	/*the min lowlimit value */
	raw_cap_lowlimit = get_f54_raw_cap_lowlimit(moudle_id,ic_name);
	if(NULL == raw_cap_lowlimit)
	{
		return 0;
	}
	
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	k = 0;

	/* add a mark for data type */
	memset(g_buf_f54raw_data, 0, sizeof(g_buf_f54raw_data));
	sprintf(g_buf_f54raw_data, "RawImageData:\n");
	for ( i = 0; i < mmidata_size; i+=2)
	{
	    Rawimage = (mmi_buf[i]) | (mmi_buf[i+1] << 8);		
	    memset(buf_Rawimage,0,50);
		if(0 == (i+2)%40)
		{
		  	sprintf(buf_Rawimage,"%d \n",Rawimage);
		}
		else
		{
			sprintf(buf_Rawimage,"%d ",Rawimage);
		}
	    strcat(g_buf_f54raw_data,buf_Rawimage);
		//printk("value in rawimage, value[%d] = %d , value[%d] = %d\n",i,mmi_buf[i],i+1,mmi_buf[i+1]);
		if ((Rawimage > raw_cap_lowlimit[k])&& (Rawimage < raw_cap_uplimit[k]))
		{
			Result++;
			k++;
		}
	}	
	
	if (Result == (mmidata_size/2))
	{	   
		return 1;
	}
	else
	{
	    return 0;
	}
  
}

/* when the report type is 7 ,this function is used to find RX to Rx short.*/
static int f54_RxtoRxshort_report(struct rmi_fn_54_data *data,size_t count)
{
	unsigned char command;
	int Result=0;
	/* change threshold get from vn1 */
	int DiagonalUpperLimit = 1335;
	int DiagonalLowerLimit = 693;
	int OthersUpperLimit = 253; 
	int i,j,k;
	short ImageArray;
	char   buf[7] = {0};
	
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	k = 0;

	memset(g_RxtoRxshort_report, 0, sizeof(g_RxtoRxshort_report));
	sprintf(g_RxtoRxshort_report, "RxtoRxshort:\n");
	for (i = 0; i < tx; i++)
	{
		for (j = 0; j < rx; j++)
		{
	        ImageArray = mmi_buf[k]|(mmi_buf[k+1] << 8);
			k = k + 2;
			//printk("%5d", ImageArray);
			sprintf(buf, "%5d\t", ImageArray);
			strncat(g_RxtoRxshort_report, buf, sizeof(g_RxtoRxshort_report)-1); 
			if (i == j)
	        {
	           if((ImageArray <= DiagonalUpperLimit) && (ImageArray >= DiagonalLowerLimit))
			        Result++; 
	        }
	        else
            {
		        if(ImageArray <= OthersUpperLimit)
			        Result++; 
	        }  
			
		}
		strncat(g_RxtoRxshort_report, "\n", sizeof(g_RxtoRxshort_report)-1);
		printk("\n");
	 }

	data->report_type = F54_RX_TO_RX2;	
	command = (unsigned char)F54_RX_TO_RX2;
	
	write_to_f54_register(data->fc,command,count);
	
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	
	k = 0;
	for (i = 0; i < (rx-tx); i++)
	{
		for (j = 0; j < rx; j++)
		{
	        ImageArray = mmi_buf[k]|(mmi_buf[k+1] << 8);
			k = k + 2;
			//printk("%5d", ImageArray);
			sprintf(buf, "%5d\t", ImageArray);
			strncat(g_RxtoRxshort_report, buf, sizeof(g_RxtoRxshort_report)-1); 
			if ((i + tx) == j)
	        {
	           if((ImageArray <= DiagonalUpperLimit) && (ImageArray >= DiagonalLowerLimit))
			        Result++; 
	        }
	        else
            {
		        if(ImageArray <= OthersUpperLimit)
			        Result++; 
	        }  
			
		}
		strncat(g_RxtoRxshort_report, "\n", sizeof(g_RxtoRxshort_report)-1);
		printk("\n");
	 }
	
	 if(Result == (rx * rx)) 
	    return 1; 
	 else 
	 	return 0; 
}

/* when the report type is 5, we call this function to 
*  catche Tx-to-Tx shorts and Tx-to-Vdd shorts.
*  If the bits of the report_data is '0', no short existed. */
static int f54_txtotx_short_report (void)
{    
	char Txstatus;
	int result = 0;
	int i,j,k;
	k = 0;
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	for (i = 0;i < mmidata_size;i++)
	{
	    
		printk("value in txtotxshort, value[%d] = %d\n",i,mmi_buf[i]);
	    for (j = 0;j < 8;j++)
	    {
	        if (g_tx_max == k)
				break;
			Txstatus = (mmi_buf[i] & (1 << j)) >> j;
			if (0 == Txstatus)
				result++;
			k++;
	    }		
	}
	if (g_tx_max == result)
	{
	    return 1;
	}
	else
	{
	    return 0;
	}

}

/* when the report type is 16, this function used to catche Tx-to-ground shorts,
*  If the bits of the report_data is '1', no short existed. */
static int f54_txtoground_short_report (void)
{
	char Txstatus;
	int result = 0;
	int i,j;

	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);

	for (i = 0;i < mmidata_size;i++)
	{
        printk("value in txtoground, value[%d] = %d\n",i,mmi_buf[i]);
		for (j = 0;j < 8;j++)
	    {			
	        Txstatus = (mmi_buf[i] & (1 << j)) >> j;
			if (1 == Txstatus)
				result++;	
	    }
		
	}

	if ((tx) == result)
	{
	    return 1;
	}
	else
	{
	    return 0;
	}

}
static int f54_maxmincapacitance_report(void)
{   
	/* change threshold get from vn1 */
	/* change type 13 max limt from 5500 to 5600 */
    short maxcapacitance = 3525;
	short mincapacitance = 514;
	short value[2] = {0};
	int i,k;
	k = 0;
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	for (i = 0; i < mmidata_size/2; i++)
	{
	    value[i] = (mmi_buf[k])|(mmi_buf[k+1] << 8);	
		printk("value in maxmincapactance, value[%d] = %d\n",i,value[i]);
		k=k+2;
	}

	memset(g_maxmincapacitance_report, 0, sizeof(g_maxmincapacitance_report));
	sprintf(g_maxmincapacitance_report,"maxmincapacitance:\n%d	%d\n",value[0], value[1]);
	if ((value[0] < maxcapacitance)&& (value[1] > mincapacitance))
	    return 1;
	else
		return 0;
    
}
/* when the report type is 4, this function used to catch broken Tx/Rx channels */
static int f54_highresistance_report (void)
{
	/* change threshold get from vn1 */
	short ReceiverMax = 209;
	short TransmiterMax = 128;
	short ReceiverMin = 13;
	/* change type 4 min limt from -1500 to -2000 */
	short PixelMin = -1836;
	short PixelMax = -640;
	short value[3] = {0};
	int i,k;
	k = 0;
	INIT_COMPLETION(mmi_comp);
	wait_for_completion(&mmi_comp);
	for (i = 0; i < mmidata_size/2; i++)
	{
	    value[i] = (mmi_buf[k])|(mmi_buf[k+1] << 8);
		k=k+2;
		printk("value in highresistance, value[%d] = %d\n",i,value[i]);
	}

	memset(g_highresistance_report, 0, sizeof(g_highresistance_report));
	sprintf(g_highresistance_report, "highresistance:\n%d	%d	%d\n",value[0], value[1],value[2]);
	if (((value[0] < ReceiverMax)&& (value[0] > ReceiverMin))
		&& ((value[1] < TransmiterMax)&& (value[1] > ReceiverMin))
		&& ((value[2] > PixelMin)&& (value[2] < PixelMax)))
	    return 1;
	else
		return 0;
}

static int rmi_f54_init(struct rmi_function_container *fc)
{
	int retval = 0;

	dev_info(&fc->dev, "Intializing F54.");
	init_completion(&mmi_comp); 
	init_completion(&mmi_sync);
	retval = rmi_f54_alloc_memory(fc);
	if (retval < 0)
		goto error_exit;

	retval = rmi_f54_initialize(fc);
	if (retval < 0)
		goto error_exit;

	retval = rmi_f54_create_sysfs(fc);
	if (retval < 0)
		goto error_exit;

	return retval;

error_exit:
	rmi_f54_free_memory(fc);

	return retval;
}



static int rmi_f54_alloc_memory(struct rmi_function_container *fc)
{
	struct rmi_fn_54_data *f54;

	f54 = kzalloc(sizeof(struct rmi_fn_54_data), GFP_KERNEL);
	if (!f54) {
		dev_err(&fc->dev, "Failed to allocate rmi_fn_54_data.\n");
		return -ENOMEM;
	}
	fc->data = f54;

	return 0;
}


static void rmi_f54_free_memory(struct rmi_function_container *fc)
{
	struct rmi_fn_54_data *f54 = fc->data;

	if (f54) {
		kfree(f54->report_data);
		kfree(f54);
		fc->data = NULL;
	}
}


static int rmi_f54_initialize(struct rmi_function_container *fc)
{
	struct rmi_fn_54_data *instance_data = fc->data;
	int retval = 0;

	instance_data->fc = fc;

#if F54_WATCHDOG
	/* Set up watchdog timer to catch unanswered get_report commands */
	hrtimer_init(&instance_data->watchdog, CLOCK_MONOTONIC,
							HRTIMER_MODE_REL);
	instance_data->watchdog.function = clear_status;

	/* work function to do unlocking */
	INIT_WORK(&instance_data->work, clear_status_worker);
#endif

	/* Read F54 Query Data */
	retval = rmi_read_block(fc->rmi_dev, fc->fd.query_base_addr,
		(u8 *)&instance_data->query, sizeof(instance_data->query));
	if (retval < 0) {
		dev_err(&fc->dev, "Could not read query registers"
			" from 0x%04x\n", fc->fd.query_base_addr);
		return retval;
	}

	__mutex_init(&instance_data->data_mutex, "data_mutex",
		     &instance_data->data_key);
	__mutex_init(&instance_data->test_mutex, "test_mutex", //Phoenix
		     &instance_data->test_key);
	__mutex_init(&instance_data->status_mutex, "status_mutex",
		     &instance_data->status_key);

	instance_data->status = IDLE;

	return 0;
}


static int rmi_f54_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;

	dev_dbg(&fc->dev, "Creating sysfs files.");

	/* Binary sysfs file to report the data back */
	rc = sysfs_create_bin_file(&fc->dev.kobj, &dev_rep_data);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to create sysfs file for F54 data "
					"(error = %d).\n", rc);
		return -ENODEV;
	}

	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for %s.",
			     attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}

	/*Creat test and test result node*/
	rc = sysfs_create_file(&fc->dev.kobj,&f54_mmi_test_att.attr);
		if (rc < 0) {
			dev_err(&fc->dev, "Failed to create sysfs file for F54 mmi test "
						"(error = %d).\n", rc);
			return -ENODEV;
		}
	
	rc = sysfs_create_file(&fc->dev.kobj,&f54_mmi_test_data_att.attr);
	if (rc < 0) {
		dev_err(&fc->dev, "Failed to create sysfs file for F54 mmi test data "
					"(error = %d).\n", rc);
		return -ENODEV;
	}
	return 0;

err_remove_sysfs:
	sysfs_remove_bin_file(&fc->dev.kobj, &dev_rep_data);

	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	sysfs_remove_file(&fc->dev.kobj, &f54_mmi_test_att.attr);
	sysfs_remove_file(&fc->dev.kobj, &f54_mmi_test_data_att.attr);
	return rc;
}


static int rmi_f54_config(struct rmi_function_container *fc)
{
	/*we shouldn't do anything here.
	 * (reset should write a 1 to a "beenreset" sysfs file)
     */
	return 0;
}



static int rmi_f54_reset(struct rmi_function_container *fc)
{
	struct rmi_fn_54_data *data = fc->data;
	struct rmi_driver *driver = fc->rmi_dev->driver;

#if F54_WATCHDOG
	hrtimer_cancel(&data->watchdog);
#endif

	mutex_lock(&data->status_mutex);
	if (driver->restore_irq_mask) {
		dev_dbg(&fc->dev, "Restoring interupts!\n");
		driver->restore_irq_mask(fc->rmi_dev);
	} else {
		dev_err(&fc->dev, "No way to restore interrupts!\n");
	}
	data->status = -ECONNRESET;
	mutex_unlock(&data->status_mutex);

	return 0;
}



static void rmi_f54_remove(struct rmi_function_container *fc)
{
	struct rmi_fn_54_data *data = fc->data;
	int attr_count = 0;

	dev_info(&fc->dev, "Removing F54.");

	#if F54_WATCHDOG
	/* Stop timer */
	hrtimer_cancel(&data->watchdog);
	#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(&fc->dev.kobj, &attrs[attr_count].attr);

	sysfs_remove_bin_file(&fc->dev.kobj, &dev_rep_data);

	rmi_f54_free_memory(fc);
}


static void set_report_size(struct rmi_fn_54_data *data)
{
	/*delete*/
	switch (data->report_type) {
	case F54_8BIT_IMAGE:
		data->report_size = rx * tx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		data->report_size = 2 * rx * tx;
		break;
	case F54_HIGH_RESISTANCE:
		data->report_size = RMI_54_HIGH_RESISTANCE_SIZE;
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		data->report_size = RMI_54_FULL_RAW_CAP_MIN_MAX_SIZE;
		break;
	case F54_TX_TO_TX_SHORT:
		data->report_size =  (g_tx_max + 7) / 8;
		break;
	case F54_TX_OPEN:
		data->report_size =  (tx + 7) / 8;
		break;	//edw
	case F54_TX_TO_GROUND:
		data->report_size =  3; //edw S2202 uses 13 tx in general... need to check with Platform (tx + 7) / 8
		break;
	case F54_RX_TO_RX1:
//edw
		if (rx < tx)
			data->report_size = 2 * rx * rx;
		else
			data->report_size = 2 * rx * tx;
		break;
//edw
	case F54_RX_OPENS1:
		if (rx < tx)
			data->report_size = 2 * rx * rx;
		else
			data->report_size = 2 * rx * tx;
		break;
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
		if (rx <= tx)
			data->report_size = 0;
		else
			data->report_size = 2 * rx * (rx - tx);
		break;
	default:
		data->report_size = 0;
	}
}

#define F55_control_base_address 0x0300
#define MAX(a, b) (a) > (b) ? (a):(b)
/* Tx physical pin numbers in FW Function 55 Control 02.
 * rmi_read_block(fc->rmi_dev, F55 control base address + 2, &data, tx count) 
 * Then compare the array to find out the maximum number.
 */
static int mmi_get_tx_max_num(struct rmi_function_container *fc)
{   
	u8 *data    = NULL;
	u8  maximum = 0;
	u8  index   = 0;
	data =  kzalloc(sizeof(u8) * tx, GFP_KERNEL);
	if(!data)
	{   
	    dev_err(&fc->dev, "Failed to allocate max pin number buf!\n");
		return tx;
	}
	memset(data, 0, tx);
	
	rmi_read_block(fc->rmi_dev,F55_control_base_address + 2,data, tx);
	
	for(index = 0; index < tx; index++)
	{   
		maximum = MAX(data[index],maximum);
	}
	
	kfree(data);
	
	return maximum;
}
int rmi_f54_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_driver *driver = fc->rmi_dev->driver;
	char fifo[2];
	struct rmi_fn_54_data *data = fc->data;
	int error = 0;
	int l;
    struct rmi_driver_data *driver_data = rmi_get_driverdata(fc->rmi_dev);
	struct rmi_function_container *f01_fc = driver_data->f01_container; 

    /*get tx and rx value by read register */
  	error = rmi_read_block(fc->rmi_dev,f01_fc->fd.control_base_addr+ RX_NUMBER,
			&rx, 1);
	if (error < 0)
	{
		dev_err(&fc->dev, "Could not read RX value "
			"from 0x%04x\n", f01_fc->fd.control_base_addr + RX_NUMBER);	
		goto error_exit;	
	}
	
	error = rmi_read_block(fc->rmi_dev,f01_fc->fd.control_base_addr + TX_NUMBER,
			&tx, 1);
	if (error < 0)
	{
		dev_err(&fc->dev, "Could not read TX value "
			"from 0x%04x\n", f01_fc->fd.control_base_addr + TX_NUMBER);	
		goto error_exit;	
	}	
	if(F54_TX_TO_TX_SHORT == data->report_type)
	{
		g_tx_max = mmi_get_tx_max_num(fc);
		
		printk("%s g_tx_max=%d\n", __func__, g_tx_max);
	}
	
	set_report_size(data);
	if (data->report_size == 0) {
		dev_err(&fc->dev, "Invalid report type set in %s. "
				"This should never happen.\n", __func__);
		error = -EINVAL;
		goto error_exit;
		
	}
	/*
	 * We need to ensure the buffer is big enough. A Buffer size of 0 means
	 * that the buffer has not been allocated.
	 */
	if (data->bufsize < data->report_size) {
		mutex_lock(&data->data_mutex);
		if (data->bufsize > 0)
			kfree(data->report_data);
		data->report_data = kzalloc(data->report_size, GFP_KERNEL);
		if (!data->report_data) {
			dev_err(&fc->dev, "Failed to allocate report_data.\n");
			error = -ENOMEM;
			data->bufsize = 0;
			mutex_unlock(&data->data_mutex);
			goto error_exit;
		}
		data->bufsize = data->report_size;
		mutex_unlock(&data->data_mutex);
	}
	dev_vdbg(&fc->dev, "F54 Interrupt handler is running.\nSize: %d\n",
		 data->report_size);
	/* Write 0 to fifohi and fifolo. */
	fifo[0] = 0;
	fifo[1] = 0;
	error = rmi_write_block(fc->rmi_dev, fc->fd.data_base_addr
				+ RMI_F54_FIFO_OFFSET, fifo,	sizeof(fifo));
	if (error < 0)
		dev_err(&fc->dev, "Failed to write fifo to zero!\n");
	else
		error = rmi_read_block(fc->rmi_dev,
			fc->fd.data_base_addr + RMI_F54_REPORT_DATA_OFFSET,
			data->report_data, data->report_size);
	if (error < 0)
		dev_err(&fc->dev, "F54 data read failed. Code: %d.\n", error);
	else if (error != data->report_size) {
		error = -EINVAL;
		goto error_exit;
	}
	/*get report data */
	mmidata_size = data->report_size;
	for (l = 0; l < data->report_size; l += 2)
	{
	    mmi_buf[l] = data->report_data[l];
		mmi_buf[l+1] = data->report_data[l+1];
	}
	
#if RAW_HEX
	//int l;
	/* Debugging: Print out the file in hex. */

	for (l = 0; l < data->report_size; l += 2) {
		pr_info("%03d: 0x%02x%02x\n", l/2,
			data->report_data[l+1], data->report_data[l]);
	}
#endif
#if HUMAN_READABLE
	/* Debugging: Print out file in human understandable image */
	switch (data->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		pr_info("Report data (Image):\n");
		int i, j, k;
		char c[2];
		short s;
		k = 0;
		for (i = 0; i < data->query.number_of_transmitter_electrodes;
									i++) {
			for (j = 0; j <
			     data->query.number_of_receiver_electrodes; j++) {
				c[0] = data->report_data[k];
				c[1] = data->report_data[k+1];
				memcpy(&s, &c, 2);
				if (s < -64)
					printk(".");
				else if (s < 0)
					printk("-");
				else if (s > 64)
					printk("*");
				else if (s > 0)
					printk("+");
				else
					printk("0");
				k += 2;
			}
			pr_info("\n");
		}
		pr_info("EOF\n");
		break;
	default:
		pr_info("Report type %d debug image not supported",
							data->report_type);
	}
#endif
	error = IDLE;
error_exit:
	mutex_lock(&data->status_mutex);
	/* Turn back on other interupts, if it
	 * appears that we turned them off. */
	if (driver->restore_irq_mask) {
		dev_dbg(&fc->dev, "Restoring interupts!\n");
		driver->restore_irq_mask(fc->rmi_dev);
	} else {
		dev_err(&fc->dev, "No way to restore interrupts!\n");
	}
	data->status = error;
	mutex_unlock(&data->status_mutex);
	complete(&mmi_comp);
	return data->status;
}


#if F54_WATCHDOG
static void clear_status_worker(struct work_struct *work)
{
	struct rmi_fn_54_data *data = container_of(work,
					struct rmi_fn_54_data, work);
	struct rmi_function_container *fc = data->fc;
	struct rmi_driver *driver = fc->rmi_dev->driver;
	char command;
	int result;

	mutex_lock(&data->status_mutex);
	if (data->status == BUSY) {
		pr_debug("F54 Timout Occured: Determining status.\n");
		result = rmi_read_block(fc->rmi_dev, fc->fd.command_base_addr,
								&command, 1);
		if (result < 0) {
			dev_err(&fc->dev, "Could not read get_report register "
				"from 0x%04x\n", fc->fd.command_base_addr);
			data->status = -ETIMEDOUT;
		} else {
			if (command & GET_REPORT) {
				dev_warn(&fc->dev, "Report type unsupported!");
				data->status = -EINVAL;
			} else {
				data->status = -ETIMEDOUT;
			}
		}
		if (driver->restore_irq_mask) {
			dev_dbg(&fc->dev, "Restoring interupts!\n");
			driver->restore_irq_mask(fc->rmi_dev);
		} else {
			dev_err(&fc->dev, "No way to restore interrupts!\n");
		}
	}
	mutex_unlock(&data->status_mutex);
	complete(&mmi_sync);
}

static enum hrtimer_restart clear_status(struct hrtimer *timer)
{
	struct rmi_fn_54_data *data = container_of(timer,
					struct rmi_fn_54_data, watchdog);
	schedule_work(&(data->work));
	return HRTIMER_NORESTART;
}
#endif

/* Check if report_type is valid */
static bool is_report_type_valid(enum f54_report_types reptype)
{
	/* Basic checks on report_type to ensure we write a valid type
	 * to the sensor.
	 * TODO: Check Query3 to see if some specific reports are
	 * available. This is currently listed as a reserved register.
	 */
	switch (reptype) {
	case F54_8BIT_IMAGE:
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORT:
	case F54_RX_TO_RX1:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS1:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		return true;
		break;
	default:
		return false;
	}
}

/* SYSFS file show/store functions */
static ssize_t rmi_fn_54_report_type_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->report_type);
}

static ssize_t rmi_fn_54_report_type_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	int result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;
	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* need to convert the string data to an actual value */
	result = strict_strtoul(buf, 10, &val);
	if (result)
		return result;
	if (!is_report_type_valid(val)) {
		dev_err(dev, "%s : Report type %d is invalid.\n",
					__func__, (u8) val);
		return -EINVAL;
	}
	mutex_lock(&instance_data->status_mutex);
	if (instance_data->status != BUSY) {
		instance_data->report_type = (enum f54_report_types)val;
		data = (char)val;
		/* Write the Report Type back to the first Block
		 * Data registers (F54_AD_Data0). */
		result =
		    rmi_write_block(fc->rmi_dev, fc->fd.data_base_addr,
								&data, 1);
		mutex_unlock(&instance_data->status_mutex);
		if (result < 0) {
			dev_err(dev, "%s : Could not write report type to"
				" 0x%x\n", __func__, fc->fd.data_base_addr);
			return result;
		}
		return count;
	} else {
		dev_err(dev, "%s : Report type cannot be changed in the middle"
				" of command.\n", __func__);
		mutex_unlock(&instance_data->status_mutex);
		return -EINVAL;
	}
}

static ssize_t rmi_fn_54_get_report_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count) {
	unsigned long val;
	int error, result;
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;
	struct rmi_driver *driver;
	u8 command;
	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	driver = fc->rmi_dev->driver;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;
	/* Do nothing if not set to 1. This prevents accidental commands. */
	if (val != 1)
		return count;
	command = (unsigned char)GET_REPORT;
	/* Basic checks on report_type to ensure we write a valid type
	 * to the sensor.
	 * TODO: Check Query3 to see if some specific reports are
	 * available. This is currently listed as a reserved register.
	 */
	if (!is_report_type_valid(instance_data->report_type)) {
		dev_err(dev, "%s : Report type %d is invalid.\n",
				__func__, instance_data->report_type);
		return -EINVAL;
	}
	mutex_lock(&instance_data->status_mutex);
	if (instance_data->status != IDLE) {
		if (instance_data->status != BUSY) {
			dev_err(dev, "F54 status is in an abnormal state: 0x%x",
							instance_data->status);
		}
		mutex_unlock(&instance_data->status_mutex);
		return count;
	}
	/* Store interrupts */
	/* Do not exit if we fail to turn off interupts. We are likely
	 * to still get useful data. The report data can, however, be
	 * corrupted, and there may be unexpected behavior.
	 */
	dev_dbg(dev, "Storing and overriding interupts\n");
	if (driver->store_irq_mask)
		driver->store_irq_mask(fc->rmi_dev,
					fc->irq_mask);
	else
		dev_err(dev, "No way to store interupts!\n");
	instance_data->status = BUSY;

	/* small delay to avoid race condition in firmare. This value is a bit
	 * higher than absolutely necessary. Should be removed once issue is
	 * resolved in firmware. */

	mdelay(2);

	/* Write the command to the command register */
	result = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
						&command, 1);
	mutex_unlock(&instance_data->status_mutex);
	if (result < 0) {
		dev_err(dev, "%s : Could not write command to 0x%x\n",
				__func__, fc->fd.command_base_addr);
		return result;
	}
#if F54_WATCHDOG
	/* start watchdog timer */
	hrtimer_start(&instance_data->watchdog, ktime_set(1, 0),
							HRTIMER_MODE_REL);
#endif
	return count;
}


static ssize_t rmi_fn_54_force_cal_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count) {
	unsigned long val;
	int error, result;
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;
	struct rmi_driver *driver;
	u8 command;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	driver = fc->rmi_dev->driver;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;
	/* Do nothing if not set to 1. This prevents accidental commands. */
	if (val != 1)
		return count;

	command = (unsigned char)FORCE_CAL;

	if (instance_data->status == BUSY)
		return -EBUSY;
	/* Write the command to the command register */
	result = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
						&command, 1);
	if (result < 0) {
		dev_err(dev, "%s : Could not write command to 0x%x\n",
				__func__, fc->fd.command_base_addr);
		return result;
	}
	return count;
}

static ssize_t rmi_fn_54_status_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d\n", instance_data->status);
}



static ssize_t rmi_fn_54_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	mutex_lock(&instance_data->status_mutex);
	/* any write to status resets it */
	instance_data->status = 0;
	mutex_unlock(&instance_data->status_mutex);

	return 0;
}


static ssize_t rmi_fn_54_num_rx_electrodes_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	u8 temp_rx;
	struct rmi_function_container *fc;
	struct rmi_driver_data *driver_data;
	struct rmi_function_container *f01_fc;
	fc = to_rmi_function_container(dev);
	driver_data = rmi_get_driverdata(fc->rmi_dev);
	f01_fc = driver_data->f01_container; 
	
    
    
	rmi_read_block(fc->rmi_dev,f01_fc->fd.control_base_addr+ RX_NUMBER,
			&temp_rx, 1);

	return snprintf(buf, PAGE_SIZE, "%u\n",temp_rx);
}

static ssize_t rmi_fn_54_num_tx_electrodes_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	u8 temp_tx;
	struct rmi_function_container *fc;
	struct rmi_driver_data *driver_data;
	struct rmi_function_container *f01_fc;
	fc = to_rmi_function_container(dev);
	driver_data = rmi_get_driverdata(fc->rmi_dev);
	f01_fc = driver_data->f01_container; 
    
    
	rmi_read_block(fc->rmi_dev,f01_fc->fd.control_base_addr+ TX_NUMBER,
			&temp_tx, 1);
			
	return snprintf(buf, PAGE_SIZE, "%u\n",temp_tx);
}

static ssize_t rmi_fn_54_has_image16_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_image16);
}

static ssize_t rmi_fn_54_has_image8_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_image8);
}

static ssize_t rmi_fn_54_has_baseline_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_baseline);
}

static ssize_t rmi_fn_54_clock_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.clock_rate);
}


static ssize_t rmi_fn_54_touch_controller_family_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.touch_controller_family);
}


static ssize_t rmi_fn_54_has_pixel_touch_threshold_adjustment_show(
		struct device *dev, struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_pixel_touch_threshold_adjustment);
}

static ssize_t rmi_fn_54_has_sensor_assignment_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_sensor_assignment);
}

static ssize_t rmi_fn_54_has_interference_metric_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_interference_metric);
}

static ssize_t rmi_fn_54_has_sense_frequency_control_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_sense_frequency_control);
}

static ssize_t rmi_fn_54_has_firmware_noise_mitigation_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_firmware_noise_mitigation);
}

static ssize_t rmi_fn_54_has_two_byte_report_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_two_byte_report_rate);
}

static ssize_t rmi_fn_54_has_one_byte_report_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_one_byte_report_rate);
}

static ssize_t rmi_fn_54_has_relaxation_control_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_relaxation_control);
}

static ssize_t rmi_fn_54_curve_compensation_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.curve_compensation_mode);
}

static ssize_t rmi_fn_54_has_iir_filter_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_iir_filter);
}

static ssize_t rmi_fn_54_has_cmn_removal_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_cmn_removal);
}

static ssize_t rmi_fn_54_has_cmn_maximum_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_cmn_maximum);
}

static ssize_t rmi_fn_54_has_pixel_threshold_hysteresis_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_pixel_threshold_hysteresis);
}

static ssize_t rmi_fn_54_has_edge_compensation_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_edge_compensation);
}

static ssize_t rmi_fn_54_has_perf_frequency_noisecontrol_show(
		struct device *dev, struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.has_perf_frequency_noisecontrol);
}

static ssize_t rmi_fn_54_number_of_sensing_frequencies_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->query.number_of_sensing_frequencies);
}


static ssize_t rmi_fn_54_no_auto_cal_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
				instance_data->no_auto_cal ? 1 : 0);
}

static ssize_t rmi_fn_54_no_auto_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	int result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* need to convert the string data to an actual value */
	result = strict_strtoul(buf, 10, &val);

	/* if an error occured, return it */
	if (result)
		return result;
	/* Do nothing if not 0 or 1. This prevents accidental commands. */
	if (val > 1)
		return count;
	/* Read current control values */
	result =
	    rmi_read_block(fc->rmi_dev, fc->fd.control_base_addr, &data, 1);

	/* if the current control registers are already set as we want them, do
	 * nothing to them */
	if ((data & 1) == val)
		return count;
	/* Write the control back to the control register (F54_AD_Ctrl0)
	 * Ignores everything but bit 0 */
	data = (data & ~1) | (val & 0x01); /* bit mask for lowest bit */
	result =
	    rmi_write_block(fc->rmi_dev, fc->fd.control_base_addr, &data, 1);
	if (result < 0) {
		dev_err(dev, "%s : Could not write control to 0x%x\n",
		       __func__, fc->fd.control_base_addr);
		return result;
	}
	/* update our internal representation iff the write succeeds */
	instance_data->no_auto_cal = (val == 1);
	return count;
}

static ssize_t rmi_fn_54_fifoindex_show(struct device *dev,
				  struct device_attribute *attr, char *buf) {
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;
	struct rmi_driver *driver;
	unsigned char temp_buf[2];
	int retval;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	driver = fc->rmi_dev->driver;

	/* Read fifoindex from device */
	retval = rmi_read_block(fc->rmi_dev,
				fc->fd.data_base_addr + RMI_F54_FIFO_OFFSET,
				temp_buf, ARRAY_SIZE(temp_buf));

	if (retval < 0) {
		dev_err(dev, "Could not read fifoindex from 0x%04x\n",
		       fc->fd.data_base_addr + RMI_F54_FIFO_OFFSET);
		return retval;
	}
	batohs(&instance_data->fifoindex, temp_buf);
	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->fifoindex);
}

static ssize_t rmi_fn_54_fifoindex_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->fifoindex = val;

	/* Write the FifoIndex back to the first data registers. */
	hstoba(data, (unsigned short)val);

	error = rmi_write_block(fc->rmi_dev,
				fc->fd.data_base_addr + RMI_F54_FIFO_OFFSET,
				data,
				ARRAY_SIZE(data));

	if (error < 0) {
		dev_err(dev, "%s : Could not write fifoindex to 0x%x\n",
		       __func__, fc->fd.data_base_addr + RMI_F54_FIFO_OFFSET);
		return error;
	}
	return count;
}

/* Provide access to last report */
#ifdef KERNEL_VERSION_ABOVE_2_6_32
static ssize_t rmi_fn_54_data_read(struct file *data_file, struct kobject *kobj,
#else
static ssize_t rmi_fn_54_data_read(struct kobject *kobj,
#endif
				struct bin_attribute *attributes,
				char *buf, loff_t pos, size_t count)
{
	struct device *dev;
	struct rmi_function_container *fc;
	struct rmi_fn_54_data *instance_data;

	dev = container_of(kobj, struct device, kobj);
	fc = to_rmi_function_container(dev);
	instance_data = fc->data;
	mutex_lock(&instance_data->data_mutex);
	if (count < instance_data->report_size) {
		dev_err(dev,
			"%s: F54 report size too large for buffer: %d."
				" Need at least: %d for Report type: %d.\n",
			__func__, count, instance_data->report_size,
			instance_data->report_type);
		mutex_unlock(&instance_data->data_mutex);
		return -EINVAL;
	}
	if (instance_data->report_data) {
		/* Copy data from instance_data to buffer */
		memcpy(buf, instance_data->report_data,
					instance_data->report_size);
		mutex_unlock(&instance_data->data_mutex);
		dev_dbg(dev, "%s: Presumably successful.", __func__);
		return instance_data->report_size;
	} else {
		dev_err(dev, "%s: F54 report_data does not exist!\n", __func__);
		mutex_unlock(&instance_data->data_mutex);
		return -EINVAL;
	}
}


static struct rmi_function_handler function_handler = {
	.func = 0x54,
	.init = rmi_f54_init,
	.config = rmi_f54_config,
	.reset = rmi_f54_reset,
	.attention = rmi_f54_attention,
	.remove = rmi_f54_remove
};

static int __init rmi_f54_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}
	return 0;
}

static void rmi_f54_module_exit(void)
{
	/* There is no function handler specific data to be freed here.
	 * Function container specific data will be freed as part of the
	 * unregistration process.
	 */
	rmi_unregister_function_driver(&function_handler);
}

module_init(rmi_f54_module_init);
module_exit(rmi_f54_module_exit);

MODULE_AUTHOR("Daniel Rosenberg <daniel.rosenberg@synaptics.com>");
MODULE_DESCRIPTION("RMI F54 module");
MODULE_LICENSE("GPL");
