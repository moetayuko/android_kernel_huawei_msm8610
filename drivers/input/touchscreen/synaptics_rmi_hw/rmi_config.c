/*
 * Copyright (c) 2012 Huawei Device Company
 *
 * This file include Touch fw and config for solving touch performance.
 * 
 * fw be named as followed:
 * PhoneName_ICName_ModuleName_FW
 * for example: C8820_S3200_TPK_FW
 *
 * config be named as followed:
 * PhoneName_ICName_ModuleName_Config
 * for example: C8820_S3200_TPK_Config
 *
 */
#include <asm/mach-types.h>
#include <linux/rmi.h> 
#include "rmi_config.h"
#include <linux/hw_tp_config.h>

/*return C8820 config array for tp performance*/
static unsigned char * get_c8820_config_array(void);


static char touch_info[50] = {0};

unsigned char *C8820_S3200_TPK_Config  = NULL;

unsigned char *C8820_S2202_TPK_Config  = NULL;

unsigned char *M660_S3200_EELY_Config  = NULL;

unsigned char *M660_S2202_EELY_Config  = NULL;

unsigned char *M660_S3200_OFilm_Config = NULL; 

unsigned char *M660_S2202_OFilm_Config = NULL;

unsigned char *C8655_S3200_EELY_Config = NULL;

unsigned char *C8655_S2202_EELY_Config = NULL;

unsigned char *C8655_S3200_OFilm_Config = NULL;

unsigned char *C8655_S2202_OFilm_Config = NULL;

/*prototype*/

/*return config array for tp performance*/
unsigned char * get_config_array()
{
	return get_c8820_config_array();
}

/*return C8820 config array for tp performance*/
static unsigned char * get_c8820_config_array()
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	
	touch_ic = get_touch_ic();
	module_id = get_fn34_module_id();
	if (touch_ic == S2202)
	{
		switch (module_id)
		{
			case TPK:
				return C8820_S2202_TPK_Config;
			default :
				return NULL;
		}
	}
	else if (touch_ic == S3200)
	{
		switch (module_id)
		{
			case TPK:
				return C8820_S3200_TPK_Config;
			default :
				return NULL;
		}
	}
		
	return NULL;
}




/* named Rule
 * 2202 3200 : syanptics-IC-Module.ver
 * for example: syanptics-3200-tpk.2
 *
 * 2000 2100 3000 :syanptics-Module.ver
 * for example: syanptics-tpk.2
 */
char * get_synaptics_touch_info(void)
{
	u16 touch_ic = 0;
	u8 module_id = 0;
	u32 config_id = 0;
	char * module_name = NULL;
	
	/*show normal info of the tp version for the cob code*/
	u8 *temp_configid = NULL;
	
	touch_ic = get_touch_ic();
	if ((touch_ic == S2202) || (touch_ic == S3200))
	{
	    if(TP_COB == get_touch_type())
	    {
	    	temp_configid = get_fn34_cob_config_ver();
	    	if (NULL == temp_configid)
	    	{
	    		return NULL;
	    	}
	    }
	    else
	    {
			module_id = get_fn34_module_id();
		}
	}
	else
	{
		module_id = get_fn01_module_id();
	}

	if(TP_COB == get_touch_type())
	{
	    if (NULL == temp_configid)
    	{
    		return NULL;
    	}
		module_name = get_touch_module_name(temp_configid[1]);
		printk("houming module value is = %s \n",module_name);
		if (module_name == NULL)
		{
			printk("module name is empty pointer\n");
			return NULL;
	    }
		if (touch_ic == S2202)
		{
			sprintf(touch_info,"synaptics-2202-%s.%d",module_name,temp_configid[0]);		
		}
		else if (touch_ic == S3200)
		{
			sprintf(touch_info,"synaptics-3200-%s.%d",module_name,temp_configid[0]);	
		}
		else
		{
			sprintf(touch_info,"synaptics-%s.%d",module_name,temp_configid[0]);	
		}
	}
	else
	{
		module_name = get_touch_module_name(module_id);
		
        if (module_name == NULL)
                return NULL;
            
        if (touch_ic == S2202)
        {
            config_id = get_fn34_config_ver();
            sprintf(touch_info,"synaptics-2202-%s.%d",module_name,config_id);       
        }
        else if (touch_ic == S3200)
        {
            config_id = get_fn34_config_ver();
            sprintf(touch_info,"synaptics-3200-%s.%d",module_name,config_id);   
        }
        else
        {
            config_id = get_fn01_config_ver();
            sprintf(touch_info,"synaptics-%s.%d",module_name,config_id);    
        }
	}
	
	return touch_info;
}

char * get_touch_module_name(u8 module_id)
{
	
    switch(module_id)
    {
        case TP_COB_ID_OFILM:
			 return "ofilm";
        case TP_COB_ID_JUNDA:
			 return "junda";
        case TP_COB_ID_TRULY:
			 return "truly";
        default:
                      return "unknow";
    }
	return NULL;
}
