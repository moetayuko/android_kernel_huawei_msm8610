/*
 * Copyright (C) huawei company
 *
 * This    program    is free    software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public    License    version    2 as
 * published by    the Free Software Foundation.
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/slab.h> 
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/of.h>

#define AUDIO_PROP_BUILTIN_MIC_TYPE_NODE    "builtin-mic-type"
#define AUDIO_PROP_BUILTIN_MIC_TYPE_MASK    0x0000000f
#define AUDIO_PROP_BUILTIN_MIC_TYPE_SINGLE  0x00000001
#define AUDIO_PROP_BUILTIN_MIC_TYPE_DUAL    0x00000002
#define AUDIO_PROP_BUILTIN_MIC_TYPE_DEFAULT AUDIO_PROP_BUILTIN_MIC_TYPE_DUAL

// Use the new strategy of mic chosen
/* Bit4 ~ bit7:
   Actually existing mics on the phone, it's NOT relevant to fluence. Using one bit to denote 
   the existence of one kind of mic, possible mics are:
     master mic: the basic mic used for call and record, if it doesn't exist means the 
                 config or software is wrong.
     secondary mic: auxiliary mic, usually used for fluence or paired with speaker in 
                    handsfree mode, it's possible that this mic exist but fluence isn't enabled.
     error mic: used in handset ANC. */
#define AUDIO_PROP_MASTER_MIC_EXIST_NODE    "builtin-master-mic-exist"
#define AUDIO_PROP_SECONDARY_MIC_EXIST_NODE "builtin-2nd-mic-exist"
#define AUDIO_PROP_ERROR_MIC_EXIST_NODE     "builtin-error-mic-exist"
#define AUDIO_PROP_MASTER_MIC_EXIST_MASK    0x00000010
#define AUDIO_PROP_SECONDARY_MIC_EXIST_MASK 0x00000020
#define AUDIO_PROP_ERROR_MIC_EXIST_MASK     0x00000040

/* Bit12 ~ bit15:
   Denote which mic would be used in hand held mode, please add as needed */
#define AUDIO_PROP_HANDHELD_MASTER_MIC_NODE "hand_held_master_mic_strategy"
#define AUDIO_PROP_HANDHELD_DUAL_MIC_NODE "hand_held_dual_mic_strategy"
#define AUDIO_PROP_HANDHELD_AANC_MIC_NODE "hand_held_aanc_mic_strategy"
#define AUDIO_PROP_HANDHELD_MASTER_MIC 0x00001000
#define AUDIO_PROP_HANDHELD_DUAL_MIC 0x00002000
#define AUDIO_PROP_HANDHELD_AANC_MIC 0x00004000

/* Bit16 ~ bit19:
   Denote which mic would be used in loud speaker mode, please add as needed */
#define AUDIO_PROP_LOUDSPEAKER_MASTER_MIC_NODE "loud_speaker_master_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_SECOND_MIC_NODE "loud_speaker_second_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_ERROR_MIC_NODE "loud_speaker_error_mic_strategy"
#define AUDIO_PROP_LOUDSPEAKER_MASTER_MIC 0x00010000
#define AUDIO_PROP_LOUDSPEAKER_SECOND_MIC 0x00020000
#define AUDIO_PROP_LOUDSPEAKER_ERROR_MIC 0x00040000
#define PRODUCT_IDENTIFIER_NODE             "product-identifier"
#define PRODUCT_IDENTIFIER_BUFF_SIZE        64
/**
Add node aud_param_ver to query the audio para version
*/
#define AUD_PARAM_VER_NODE             "aud_param_ver"
#define AUD_PARAM_VER_BUFF_SIZE        32

#define AUDIO_PROP_BTSCO_NREC_ADAPT_MASK 0xf0000000
#define AUDIO_PROP_BTSCO_NREC_ADAPT_OFF  0x10000000
#define AUDIO_PROP_BTSCO_NREC_ADAPT_ON   0x20000000

#define PRODUCT_NERC_ADAPT_CONFIG    "product-btsco-nrec-adapt"

typedef struct string_value_pair_struct {
    char *string;
    int value;
} string_value_pair_t;

/**
* string value pairs for built-in mic type.
* Must be ended with an element whose member string = NULL, 
* and it's value will be used as default value, when no match is found.
*/
static string_value_pair_t builtin_mic_type_pairs[] = {
    { .string = "single",   AUDIO_PROP_BUILTIN_MIC_TYPE_SINGLE  },
    { .string = "dual",     AUDIO_PROP_BUILTIN_MIC_TYPE_DUAL    },
    { .string = NULL,       AUDIO_PROP_BUILTIN_MIC_TYPE_DEFAULT },
};

/**
* string_to_value - Translate a string to an integer value
* @string: The string needs to be translated;
* @pairs: An array of string_value_pair_t type, must be ended with 
*         an element whose member string = NULL, and it's value will 
*         be used as default value, when no match is found.
*
* The function will search through the array of pairs to find the first 
* total match of string, then return the corresponding value; 
* If no match is found, return the value corresponding to the NULL string, 
* which is considered a default value.
* If any of the input parameters are NULL, return 0;
*/
static int string_to_value(const char *string, string_value_pair_t *pairs)
{
    int i;
    
    if((NULL == string) ||  (NULL == pairs))
    {
        printk(KERN_ERR "huawei_audio: string_to_value failed\n");
        return 0;
    }
    
    for(i = 0; pairs[i].string != NULL; i++)
    {
        if(strcmp(string, pairs[i].string) == 0)
        {
            break;
        }
    }
    
    return pairs[i].value;
}

/**
* audio_property - Product specified audio properties
*/
static unsigned int audio_property = 2;

/**
* product_identifier - Product identifier, used for audio parameter auto-adapt
*/
static char product_identifier[PRODUCT_IDENTIFIER_BUFF_SIZE] = "default";

static char aud_param_ver[AUD_PARAM_VER_BUFF_SIZE] = "default";

static ssize_t audio_property_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    if(NULL != buf)
    {
        /* The size of buf should be one page(PAGE_SIZE), which will be surely 
           enough to hold the string converted from a 32 bit unsigned integer. 
           So omit unnecessary overflow check */
        return sprintf(buf, "%08X\n", audio_property);
    }

    return 0;
}

static struct kobj_attribute audio_property_attribute = 
    __ATTR(audio_property, 0444, audio_property_show, NULL);

static ssize_t product_identifier_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    if(NULL != buf)
    {
        /* Newline is not appended on purpose, for convenience of reader programs */
        snprintf(buf, PAGE_SIZE, "%s", product_identifier);
        return strlen(buf);
    }

    return 0;
}

/**
* audiopara_version_show - used for cat the value of node aud_param_ver
*/
static ssize_t audiopara_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    if(NULL != buf)
    {
        return snprintf(buf, PAGE_SIZE,"%s", aud_param_ver);
    }

    return 0;
}

static struct kobj_attribute product_identifier_attribute = 
    __ATTR(product_identifier, 0444, product_identifier_show, NULL);

static struct kobj_attribute aud_param_ver_attribute =
    __ATTR(aud_param_ver, 0444, audiopara_version_show, NULL);

static struct attribute *audio_attrs[] = {
    &audio_property_attribute.attr,
    &product_identifier_attribute.attr,
    &aud_param_ver_attribute.attr,
    NULL,
};

static struct attribute_group audio_group = {
    .name ="hw_audio_info",
    .attrs = audio_attrs,
}; 

static const struct attribute_group *groups[] = {
    &audio_group,
    NULL,
};

static struct of_device_id audio_info_match_table[] = {
    { .compatible = "huawei,hw_audio_info",},
    { },
};

static int __devinit audio_info_probe(struct platform_device *pdev)
{
    int ret;
    const char *string;
    
    if(NULL == pdev)
    {
        printk(KERN_ERR "huawei_audio: audio_info_probe failed, pdev is NULL\n");
        return 0;
    }
    
    if(NULL == pdev->dev.of_node)
    {
        printk(KERN_ERR "huawei_audio: audio_info_probe failed, of_node is NULL\n");
        return 0;
    }
    
    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, AUDIO_PROP_BUILTIN_MIC_TYPE_NODE, &string);
    if(ret || (NULL == string))
    {
        printk(KERN_ERR "huawei_audio: of_property_read_string builtin-mic-type failed %d\n", ret);
    }
    else
    {
        audio_property |= 
            (string_to_value(string, builtin_mic_type_pairs) & AUDIO_PROP_BUILTIN_MIC_TYPE_MASK);
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_MASTER_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_MASTER_MIC_EXIST_MASK;
    }
    else
    {
        printk(KERN_ERR "huawei_audio: check mic config, no master mic found\n");
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_SECONDARY_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_SECONDARY_MIC_EXIST_MASK;
    }
    
    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_ERROR_MIC_EXIST_NODE))
    {
        audio_property |= AUDIO_PROP_ERROR_MIC_EXIST_MASK;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_MASTER_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_MASTER_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_DUAL_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_DUAL_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_HANDHELD_AANC_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_HANDHELD_AANC_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_MASTER_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_MASTER_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_SECOND_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_SECOND_MIC;
    }

    if(of_property_read_bool(pdev->dev.of_node, AUDIO_PROP_LOUDSPEAKER_ERROR_MIC_NODE))
    {
        audio_property |= AUDIO_PROP_LOUDSPEAKER_ERROR_MIC;
    }
    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, PRODUCT_IDENTIFIER_NODE, &string);
    if(ret || (NULL == string))
    {
        printk(KERN_ERR "huawei_audio: of_property_read_string product-identifier failed %d\n", ret);
    }
    else
    {
        memset(product_identifier, 0, sizeof(product_identifier));
        strncpy(product_identifier, string, sizeof(product_identifier) - 1);
    }

    string = NULL;
    ret = of_property_read_string(pdev->dev.of_node, AUD_PARAM_VER_NODE, &string);
    if(ret || (NULL == string))
    {
        printk(KERN_ERR "huawei_audio: of_property_read_string audiopara-version failed %d\n", ret);
    }
    else
    {
        memset(aud_param_ver, 0, sizeof(aud_param_ver));
        strncpy(aud_param_ver, string, sizeof(aud_param_ver) - 1);
    }

    if(false == of_property_read_bool(pdev->dev.of_node, PRODUCT_NERC_ADAPT_CONFIG))
    {
        printk(KERN_ERR "huawei_audio: of_property_read_bool PRODUCT_NERC_ADAPT_CONFIG failed %d\n", ret);
        audio_property |= (AUDIO_PROP_BTSCO_NREC_ADAPT_OFF & AUDIO_PROP_BTSCO_NREC_ADAPT_MASK);
    }
    else
    {
        audio_property |= ( AUDIO_PROP_BTSCO_NREC_ADAPT_ON & AUDIO_PROP_BTSCO_NREC_ADAPT_MASK);
    }

    return 0;
}

static struct platform_driver audio_info_driver = {
    .driver = {
        .name  = "hw_audio_info",
        .owner  = THIS_MODULE,
        .groups = groups,
        .of_match_table = audio_info_match_table,
    },

    .probe = audio_info_probe,
    .remove = NULL,
};

static int __init audio_info_init(void)
{
    return platform_driver_register(&audio_info_driver);
}

static void __exit audio_info_exit(void)
{
    platform_driver_unregister(&audio_info_driver);
}

module_init(audio_info_init);
module_exit(audio_info_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Huawei audio info");
MODULE_AUTHOR("duhongyan<hongyan.du@huawei.com>");
