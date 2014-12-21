/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
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

#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/rmi.h>
#include <linux/touch_platform_config.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/hw_tp_config.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/input/rmi_i2c.h>
#define COMMS_DEBUG 0

#define IRQ_DEBUG 0
#if COMMS_DEBUG || IRQ_DEBUG
#define DEBUG
#endif

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)


static char *phys_proto_name = "i2c";

struct rmi_i2c_data {
	struct mutex page_mutex;
	int page;
	int enabled;
	int irq;
	int irq_flags;
	struct rmi_phys_device *phys;
};

/* add a parameter for the module */
int synaptics_debug_mask = TP_INFO;
module_param_named(synaptics_debug_mask, synaptics_debug_mask, int, 0644);

static irqreturn_t rmi_i2c_irq_thread(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = phys->platform_data;

#if IRQ_DEBUG
	dev_info(phys->dev, "ATTN gpio, value: %d.\n",
			gpio_get_value(pdata->attn_gpio));
#endif
	if (gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity) {
		phys->info.attn_count++;
		if (driver && driver->irq_handler && rmi_dev)
			driver->irq_handler(rmi_dev, irq);
	}

	return IRQ_HANDLED;
}

/*
 * rmi_set_page - Set RMI page
 * @phys: The pointer to the rmi_phys_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_phys_device *phys, unsigned int page)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	char txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

#if COMMS_DEBUG
	dev_dbg(&client->dev, "RMI4 I2C writes 3 bytes: %02x %02x\n",
		txbuf[0], txbuf[1]);
#endif
	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}
	data->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			       int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[len + 1];
	int retval;
#if	COMMS_DEBUG
	int i;
#endif

	txbuf[0] = addr & 0xff;
	memcpy(txbuf + 1, buf, len);

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

#if COMMS_DEBUG
	dev_dbg(&client->dev, "RMI4 I2C writes %d bytes: ", sizeof(txbuf));
	for (i = 0; i < sizeof(txbuf); i++)
		dev_dbg(&client->dev, "%02x ", txbuf[i]);
	dev_dbg(&client->dev, "\n");
#endif

	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval < 0)
		phys->info.tx_errs++;

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int rmi_i2c_write(struct rmi_phys_device *phys, u16 addr, u8 data)
{
	int retval = rmi_i2c_write_block(phys, addr, &data, 1);
	return (retval < 0) ? retval : 0;
}

static int rmi_i2c_read_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			      int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	u8 txbuf[1] = {addr & 0xff};
	int retval;
#if	COMMS_DEBUG
	int i;
#endif

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

#if COMMS_DEBUG
	dev_dbg(&client->dev, "RMI4 I2C writes 1 bytes: %02x\n", txbuf[0]);
#endif
	phys->info.tx_count++;
	phys->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		phys->info.tx_errs++;
		retval = (retval < 0) ? retval : -EIO;
		goto exit;
	}

	retval = i2c_master_recv(client, buf, len);

	phys->info.rx_count++;
	phys->info.rx_bytes += len;
	if (retval < 0)
		phys->info.rx_errs++;
#if COMMS_DEBUG
	else {
		dev_dbg(&client->dev, "RMI4 I2C received %d bytes: ", len);
		for (i = 0; i < len; i++)
			dev_dbg(&client->dev, "%02x ", buf[i]);
		dev_dbg(&client->dev, "\n");
	}
#endif

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int rmi_i2c_read(struct rmi_phys_device *phys, u16 addr, u8 *buf)
{
	int retval = rmi_i2c_read_block(phys, addr, buf, 1);
	return (retval < 0) ? retval : 0;
}


static int acquire_attn_irq(struct rmi_i2c_data *data)
{
	return request_threaded_irq(data->irq, NULL, rmi_i2c_irq_thread,
			data->irq_flags, dev_name(data->phys->dev), data->phys);
}

static int enable_device(struct rmi_phys_device *phys)
{
	int retval = 0;

	struct rmi_i2c_data *data = phys->data;

	if (data->enabled)
		return 0;

	retval = acquire_attn_irq(data);
	if (retval)
		goto error_exit;

	data->enabled = true;
	dev_dbg(phys->dev, "Physical device enabled.\n");
	return 0;

error_exit:
	dev_err(phys->dev, "Failed to enable physical device. Code=%d.\n",
		retval);
	return retval;
}


static void disable_device(struct rmi_phys_device *phys)
{
	struct rmi_i2c_data *data = phys->data;

	if (!data->enabled)
		return;

	disable_irq(data->irq);
	free_irq(data->irq, data->phys);

	dev_dbg(phys->dev, "Physical device disabled.\n");
	data->enabled = false;
}
static int touch_reset(struct rmi_device_platform_data *pdata)
{
	int ret = 0;
       int gpio_config = 0;
	  
	gpio_config = GPIO_CFG(pdata->reset_gpio,0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	if (ret)
	{
		pr_err("%s:touch int gpio config failed\n", __func__);
		return ret;
	}
	ret = gpio_request(pdata->reset_gpio, "TOUCH_RESET");
	if (ret)
	{
		pr_err("%s:touch int gpio request failed\n", __func__);
		return ret;
	}
	
	ret = gpio_direction_output(pdata->reset_gpio, 1);
	mdelay(5);
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	mdelay(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
	ret = gpio_direction_output(pdata->reset_gpio, 1);
	mdelay(50);//must more than 10ms.

	gpio_free(pdata->reset_gpio);
	return ret;
}



struct syna_gpio_data {
	u16 gpio_number;
	char* gpio_name;
};

static struct syna_gpio_data touch_gpiodata ;

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	int retval=0;
	struct syna_gpio_data *data = gpio_data;

	if (configure) {
		retval = gpio_request(data->gpio_number, "rmi4_attn");
		if (retval) {
			pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			return retval;
		}

		retval = gpio_tlmm_config(GPIO_CFG(data->gpio_number, 
						0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, 
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s: Failed to config attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
			return retval;
		}
		retval = gpio_direction_input(data->gpio_number);
		if (retval) {
			pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			       __func__, data->gpio_number, retval);
			gpio_free(data->gpio_number);
		}
	} else {
		pr_warn("%s: No way to deconfigure gpio %d.",
		       __func__, data->gpio_number);
	}

	return retval;
}

static int synaptics_rmi4_power_on(struct device *dev)
{
	char const *power_pin_vdd;
	char const *power_pin_vbus;
	struct regulator *vdd_synaptics;
    struct regulator *vbus_synaptics;
	int rc;

       /*get the power name*/
	rc = of_property_read_string(dev->of_node,"synaptics,vdd", &power_pin_vdd);
	if (rc) {
		pr_err("%s: OF error vdd rc=%d\n", __func__, rc);
	}
	
	rc = of_property_read_string(dev->of_node,"synaptics,vbus", &power_pin_vbus);
	if (rc) {
		pr_err("%s: OF error vbus rc=%d\n", __func__, rc);
	}
	    /* VDD power on */

	vdd_synaptics = regulator_get(dev, power_pin_vdd);
	if (IS_ERR(vdd_synaptics)) {
		pr_err("%s: OF error vbus rc=%d\n", __func__, rc);
		return  -EINVAL;
	}

	rc = regulator_set_voltage(vdd_synaptics,2850000,2850000);
	if(rc < 0){
		pr_err( "%s: failed to set synaptics vdd\n", __func__);
		return  -EINVAL;
	}

	rc = regulator_enable(vdd_synaptics);
	if (rc < 0) {
		pr_err( "%s: failed to set synaptics vdd\n", __func__);
		return -EINVAL;
	}

	/* VBUS power on */
	vbus_synaptics = regulator_get(dev, power_pin_vbus);
	if (IS_ERR(vbus_synaptics)) {
		pr_err( "%s: failed to set synaptics vdd\n", __func__);
		return -EINVAL;
	}

	rc = regulator_set_voltage(vbus_synaptics,1800000,1800000);
	if(rc < 0){
		pr_err( "%s: failed to set synaptics vdd\n", __func__);
		return -EINVAL;
	}
 
	rc = regulator_enable(vbus_synaptics);
	if (rc < 0) {
		pr_err( "%s: failed to set synaptics vdd\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int synaptics_rmi4_parse_dt(struct device *dev,
				struct rmi_device_platform_data *rmi4_pdata)
{
	struct device_node *np = dev->of_node;
	int rc;

	rmi4_pdata->driver_name = "rmi-generic";

	rc = of_property_read_u32(np,"synaptics,irq_gpio_num", &rmi4_pdata->attn_gpio);
	if (rc) {
		pr_err("%s: OF error irq_gpio_num rc=%d\n", __func__, rc);
	}
	rc = of_property_read_u32(np,"synaptics,reset_gpio_num", &rmi4_pdata->reset_gpio);
	if (rc) {
		pr_err("%s: OF error reset_gpio_num rc=%d\n", __func__, rc);
	}
	rc = of_property_read_u32(np,"synaptics,irq_gpio_polarity", &rmi4_pdata->attn_polarity);
	if (rc) {
		pr_err("%s: OF error irq_gpio_polarity rc=%d\n", __func__, rc);
	}
	rc = of_property_read_u32(np,"synaptics,reset_delay_ms", &rmi4_pdata->reset_delay_ms);
	if (rc) {
		pr_err("%s: OF error reset_delay_ms rc=%d\n", __func__, rc);
	}
	touch_gpiodata.gpio_number = rmi4_pdata->attn_gpio;
	touch_gpiodata.gpio_name = "irq_gpio";
		
	rmi4_pdata->gpio_data = &touch_gpiodata;
	rmi4_pdata->gpio_config = synaptics_touchpad_gpio_setup;
	rmi4_pdata->hardware_reset = touch_reset;
	rmi4_pdata->get_phone_version = get_touch_resolution;
	
	return 0;
}

static int __devinit rmi_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct rmi_phys_device *rmi_phys;
	struct rmi_i2c_data *data;
	struct rmi_device_platform_data *pdata;
	int error;

	tp_log_info("enter rmi_i2c_probe\n");
	if (client->dev.of_node) {
		pdata = kzalloc(sizeof(struct rmi_device_platform_data),GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory pdata\n");
			return -ENOMEM;
		}
		error = synaptics_rmi4_parse_dt(&client->dev, pdata);
		if (error)
		{
			goto err_platform_data;
		}
	} else {
	       dev_err(&client->dev, "it is not dts struct\n");
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		return -EINVAL;
	}

	error = synaptics_rmi4_power_on(&client->dev);
	if (error)
	{
		goto err_platform_data;
	}

    error = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!error) {
		dev_err(&client->dev, "i2c_check_functionality error %d.\n",
			error);
        error = -ENODEV;
		return error;
	}

	rmi_phys = kzalloc(sizeof(struct rmi_phys_device), GFP_KERNEL);
	if (!rmi_phys)
	{
		error = -ENOMEM;
		goto err_platform_data;
	}

	data = kzalloc(sizeof(struct rmi_i2c_data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_phys;
	}

	data->enabled = true;	/* We plan to come up enabled. */
	data->irq = gpio_to_irq(pdata->attn_gpio);
    /* change the irq trigger as IRQF_TRIGGER_LOW for solve the tp freeze issue */
	data->irq_flags = (pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH) ?
		IRQF_TRIGGER_HIGH | IRQF_ONESHOT : IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	data->phys = rmi_phys;

	rmi_phys->data = data;
	rmi_phys->dev = &client->dev;
    rmi_phys->platform_data = pdata;
	rmi_phys->write = rmi_i2c_write;
	rmi_phys->write_block = rmi_i2c_write_block;
	rmi_phys->read = rmi_i2c_read;
	rmi_phys->read_block = rmi_i2c_read_block;
	rmi_phys->enable_device = enable_device;
	rmi_phys->disable_device = disable_device;

	rmi_phys->info.proto = phys_proto_name;

	mutex_init(&data->page_mutex);

	mdelay(1000);

	/* Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	error = rmi_set_page(rmi_phys, 0); 
	if (error) {
		dev_err(&client->dev, "Failed to set page select to 0.\n");
		goto err_data;
	}

	if(pdata->hardware_reset)
	{
		error = pdata->hardware_reset(rmi_phys->platform_data);
		if (error < 0) {
		dev_err(&client->dev, "failed to reset tp\n");
		goto err_data;
		}
	}
	// delete some line,config irq gpio before request irq
	error = rmi_register_phys_device(rmi_phys);
	if (error) {
		dev_err(&client->dev,
			"failed to register physical driver at 0x%.2X.\n",
			client->addr);
		goto err_data;
	}
	i2c_set_clientdata(client, rmi_phys);
	if (pdata->gpio_config) {
		error = pdata->gpio_config(pdata->gpio_data, true);
		if (error < 0) {
			dev_err(&client->dev, "failed to setup irq %d\n",
				pdata->attn_gpio);
			goto err_data;
		}
	}
	if (pdata->attn_gpio > 0) {
		error = acquire_attn_irq(data);
		if (error < 0) {
			dev_err(&client->dev,
				"request_threaded_irq failed %d\n",
				pdata->attn_gpio);
			goto err_unregister;
		}
	}

#if defined(CONFIG_RMI4_DEV)
	error = gpio_export(pdata->attn_gpio, false);
	if (error) {
		dev_warn(&client->dev, "%s: WARNING: Failed to "
				 "export ATTN gpio!\n", __func__);
		error = 0;
	} else {
		error = gpio_export_link(&(rmi_phys->rmi_dev->dev), "attn",
					pdata->attn_gpio);
		if (error) {
			dev_warn(&(rmi_phys->rmi_dev->dev), "%s: WARNING: "
				"Failed to symlink ATTN gpio!\n", __func__);
			error = 0;
		} else {
			dev_info(&(rmi_phys->rmi_dev->dev),
				"%s: Exported GPIO %d.", __func__,
				pdata->attn_gpio);
		}
	}
#endif /* CONFIG_RMI4_DEV */

	tp_log_info("registered rmi i2c driver at 0x%.2X. success!\n",
			client->addr);
	return 0;

err_unregister:
	rmi_unregister_phys_device(rmi_phys);
err_data:
	kfree(data);
err_phys:
	kfree(rmi_phys);
err_platform_data:
	kfree(pdata);
	return error;
}

static int __devexit rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_phys_device *phys = i2c_get_clientdata(client);
	struct rmi_device_platform_data *pd = client->dev.platform_data;

	rmi_unregister_phys_device(phys);
	kfree(phys->data);
	kfree(phys);
       kfree(phys->platform_data);
	if (pd->gpio_config)
		pd->gpio_config(&client->dev, false);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id rmi_match_table[] = {
	{ .compatible = "synaptics,rmi4",},
	{ },
};
#else
#define rmi4_match_table NULL
#endif

static const struct i2c_device_id rmi_id[] = {
	{ "rmi", 0 },
	{ "rmi-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi-i2c",
		.of_match_table = rmi_match_table,
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= __devexit_p(rmi_i2c_remove),
};

static int __init rmi_i2c_init(void)
{
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __exit rmi_i2c_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");

module_init(rmi_i2c_init);
module_exit(rmi_i2c_exit);
