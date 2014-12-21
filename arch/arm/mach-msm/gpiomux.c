/* Copyright (c) 2010,2013, The Linux Foundation. All rights reserved.
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
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>

#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/gpio.h>
#include <mach/irqs.h>
#endif

struct msm_gpiomux_rec {
	struct gpiomux_setting *sets[GPIOMUX_NSETTINGS];
	int ref;
};
static DEFINE_SPINLOCK(gpiomux_lock);
static struct msm_gpiomux_rec *msm_gpiomux_recs;
static struct gpiomux_setting *msm_gpiomux_sets;
static unsigned msm_gpiomux_ngpio;

static int msm_gpiomux_store(unsigned gpio, enum msm_gpiomux_setting which,
	struct gpiomux_setting *setting, struct gpiomux_setting *old_setting)
{
	struct msm_gpiomux_rec *rec = msm_gpiomux_recs + gpio;
	unsigned set_slot = gpio * GPIOMUX_NSETTINGS + which;
	unsigned long irq_flags;
	int status = 0;

	if (!msm_gpiomux_recs)
		return -EFAULT;

	if (gpio >= msm_gpiomux_ngpio)
		return -EINVAL;

	spin_lock_irqsave(&gpiomux_lock, irq_flags);

	if (old_setting) {
		if (rec->sets[which] == NULL)
			status = 1;
		else
			*old_setting =  *(rec->sets[which]);
	}

	if (setting) {
		msm_gpiomux_sets[set_slot] = *setting;
		rec->sets[which] = &msm_gpiomux_sets[set_slot];
	} else {
		rec->sets[which] = NULL;
	}

	spin_unlock_irqrestore(&gpiomux_lock, irq_flags);
	return status;
}

int msm_gpiomux_write(unsigned gpio, enum msm_gpiomux_setting which,
	struct gpiomux_setting *setting, struct gpiomux_setting *old_setting)
{
	int ret;
	unsigned long irq_flags;
	struct gpiomux_setting *new_set;
	struct msm_gpiomux_rec *rec = msm_gpiomux_recs + gpio;

	ret = msm_gpiomux_store(gpio, which, setting, old_setting);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&gpiomux_lock, irq_flags);

	new_set = rec->ref ? rec->sets[GPIOMUX_ACTIVE] :
		rec->sets[GPIOMUX_SUSPENDED];
	if (new_set)
		__msm_gpiomux_write(gpio, *new_set);

	spin_unlock_irqrestore(&gpiomux_lock, irq_flags);
	return ret;
}
EXPORT_SYMBOL(msm_gpiomux_write);

int msm_gpiomux_get(unsigned gpio)
{
	struct msm_gpiomux_rec *rec = msm_gpiomux_recs + gpio;
	unsigned long irq_flags;

	if (!msm_gpiomux_recs)
		return -EFAULT;

	if (gpio >= msm_gpiomux_ngpio)
		return -EINVAL;

	spin_lock_irqsave(&gpiomux_lock, irq_flags);
	if (rec->ref++ == 0 && rec->sets[GPIOMUX_ACTIVE])
		__msm_gpiomux_write(gpio, *rec->sets[GPIOMUX_ACTIVE]);
	spin_unlock_irqrestore(&gpiomux_lock, irq_flags);
	return 0;
}
EXPORT_SYMBOL(msm_gpiomux_get);

int msm_gpiomux_put(unsigned gpio)
{
	struct msm_gpiomux_rec *rec = msm_gpiomux_recs + gpio;
	unsigned long irq_flags;

	if (!msm_gpiomux_recs)
		return -EFAULT;

	if (gpio >= msm_gpiomux_ngpio)
		return -EINVAL;

	spin_lock_irqsave(&gpiomux_lock, irq_flags);
	BUG_ON(rec->ref == 0);
	if (--rec->ref == 0 && rec->sets[GPIOMUX_SUSPENDED])
		__msm_gpiomux_write(gpio, *rec->sets[GPIOMUX_SUSPENDED]);
	spin_unlock_irqrestore(&gpiomux_lock, irq_flags);
	return 0;
}
EXPORT_SYMBOL(msm_gpiomux_put);

#ifdef CONFIG_HUAWEI_KERNEL
static const char * const gpiomux_drv_str[] = {
	"DRV_2mA",
	"DRV_4mA",
	"DRV_6mA",
	"DRV_8mA",
	"DRV_10mA",
	"DRV_12mA",
	"DRV_14mA",
	"DRV_16mA",
};

static const char * const gpiomux_func_str[] = {
	"GPIO",
	"Func_1",
	"Func_2",
	"Func_3",
	"Func_4",
	"Func_5",
	"Func_6",
	"Func_7",
	"Func_8",
	"Func_9",
	"Func_a",
	"Func_b",
	"Func_c",
	"Func_d",
	"Func_e",
	"Func_f",
};

static const char * const gpiomux_pull_str[] = {
	"PULL_NONE",
	"PULL_DOWN",
	"PULL_KEEPER",
	"PULL_UP",
};

static const char * const gpiomux_dir_str[] = {
	"IN",
	"OUT_HIGH",
	"OUT_LOW",
};

static const char * const gpiomux_val_str[] = {
	"VAL_LOW",
	"VAL_HIGH",
};

unsigned msm_gpiomux_get_num(void)
{
	int rc;
	unsigned int ngpio;
	struct device_node *of_gpio_node;

	of_gpio_node = of_find_compatible_node(NULL, NULL, "qcom,msm-gpio");
	if (!of_gpio_node) {
		pr_err("%s: Failed to find qcom,msm-gpio node\n", __func__);
		return -ENODEV;
	}

	rc = of_property_read_u32(of_gpio_node, "ngpio", &ngpio);
	if (rc) {
		pr_err("%s: Failed to find ngpio property in msm-gpio device node %d\n"
				, __func__, rc);
		return rc;
	}

	return ngpio;
}

void msm_gpio_print_enabled(void)
{
	unsigned long flags;
	struct gpiomux_setting set;
	unsigned val = 0;
	unsigned gpio;
	unsigned num = 0;

	spin_lock_irqsave(&gpiomux_lock, flags);

	num = msm_gpiomux_get_num();

	for (gpio = 0; gpio < num; ++gpio) {
		msm_gpiomux_read(gpio, &set);
		val = gpio_get_value(gpio);
		printk(KERN_INFO"GPIO[%u] \t%s \t%s \t%s \t%s \t%s\n", gpio,
			gpiomux_func_str[set.func],
			gpiomux_dir_str[set.dir],
			gpiomux_pull_str[set.pull],
			gpiomux_drv_str[set.drv],
			gpiomux_val_str[val]);
	}

	spin_unlock_irqrestore(&gpiomux_lock, flags);
}
#endif

void msm_tlmm_misc_reg_write(enum msm_tlmm_misc_reg misc_reg, int val)
{
	writel_relaxed(val, MSM_TLMM_BASE + misc_reg);
	/* ensure the write completes before returning */
	mb();
}

int msm_gpiomux_init(size_t ngpio)
{
	if (!ngpio)
		return -EINVAL;

	if (msm_gpiomux_recs)
		return -EPERM;

	msm_gpiomux_recs = kzalloc(sizeof(struct msm_gpiomux_rec) * ngpio,
				   GFP_KERNEL);
	if (!msm_gpiomux_recs)
		return -ENOMEM;

	/* There is no need to zero this memory, as clients will be blindly
	 * installing settings on top of it.
	 */
	msm_gpiomux_sets = kmalloc(sizeof(struct gpiomux_setting) * ngpio *
		GPIOMUX_NSETTINGS, GFP_KERNEL);
	if (!msm_gpiomux_sets) {
		kfree(msm_gpiomux_recs);
		msm_gpiomux_recs = NULL;
		return -ENOMEM;
	}

	msm_gpiomux_ngpio = ngpio;

	return 0;
}
EXPORT_SYMBOL(msm_gpiomux_init);

void msm_gpiomux_install_nowrite(struct msm_gpiomux_config *configs,
				unsigned nconfigs)
{
	unsigned c, s;
	int rc;

	for (c = 0; c < nconfigs; ++c) {
		for (s = 0; s < GPIOMUX_NSETTINGS; ++s) {
			rc = msm_gpiomux_store(configs[c].gpio, s,
				configs[c].settings[s], NULL);
			if (rc)
				pr_err("%s: write failure: %d\n", __func__, rc);
		}
	}
}

void msm_gpiomux_install(struct msm_gpiomux_config *configs, unsigned nconfigs)
{
	unsigned c, s;
	int rc;

	for (c = 0; c < nconfigs; ++c) {
		for (s = 0; s < GPIOMUX_NSETTINGS; ++s) {
			rc = msm_gpiomux_write(configs[c].gpio, s,
				configs[c].settings[s], NULL);
			if (rc)
				pr_err("%s: write failure: %d\n", __func__, rc);
		}
	}
}
EXPORT_SYMBOL(msm_gpiomux_install);

int msm_gpiomux_init_dt(void)
{
	int rc;
	unsigned int ngpio;
	struct device_node *of_gpio_node;

	of_gpio_node = of_find_compatible_node(NULL, NULL, "qcom,msm-gpio");
	if (!of_gpio_node) {
		pr_err("%s: Failed to find qcom,msm-gpio node\n", __func__);
		return -ENODEV;
	}

	rc = of_property_read_u32(of_gpio_node, "ngpio", &ngpio);
	if (rc) {
		pr_err("%s: Failed to find ngpio property in msm-gpio device node %d\n"
				, __func__, rc);
		return rc;
	}

	return msm_gpiomux_init(ngpio);
}
EXPORT_SYMBOL(msm_gpiomux_init_dt);
