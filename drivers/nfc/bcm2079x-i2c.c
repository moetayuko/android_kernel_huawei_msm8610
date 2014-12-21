/*
 * Copyright (C) 2012 Broadcom Corporation.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/nfc/bcm2079x.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>

#define TRUE		1
#define FALSE		0
#define STATE_HIGH	1
#define STATE_LOW	0

/* end of compile options */

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(4)
#define PACKET_HEADER_SIZE_HCI	(3)
#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)

struct bcm2079x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice bcm2079x_device;
	unsigned int wake_gpio;
	unsigned int en_gpio;
	unsigned int irq_gpio;
	unsigned int clk_req_gpio;
	unsigned int clk_out_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	int clock_req_value;
	spinlock_t clock_irq_lock;
	unsigned int error_write;
	unsigned int error_read;
	unsigned int count_read;
	unsigned int count_irq;
    int original_address;
    struct wake_lock wake_locker;
};

/*********************************************************************
 * Initialization checking macros
 *********************************************************************/
#define NULL_CHECK(ctx, x, ret) \
  if ((ctx) == NULL)                  \
  {                              \
      printk("%s: NULL error: %s", __func__, x); \
      ret;                                                        \
  }

static void bcm2079x_init_stat(struct bcm2079x_dev *bcm2079x_dev)
{
    NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return);
	bcm2079x_dev->error_write = 0;
	bcm2079x_dev->error_read = 0;
	bcm2079x_dev->count_read = 0;
	bcm2079x_dev->count_irq = 0;
}

static void bcm2079x_disable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;

	NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return);  
	NULL_CHECK(bcm2079x_dev->client,"bcm2079x_dev->client",return); 
	
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_enabled) {
		disable_irq_nosync(bcm2079x_dev->client->irq);
		bcm2079x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

static void bcm2079x_enable_irq(struct bcm2079x_dev *bcm2079x_dev)
{
	unsigned long flags;

	NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return); 
	NULL_CHECK(bcm2079x_dev->client,"bcm2079x_dev->client",return); 
	
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (!bcm2079x_dev->irq_enabled) {
		bcm2079x_dev->irq_enabled = true;
		enable_irq(bcm2079x_dev->client->irq);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

/*
 The alias address 0x79, when sent as a 7-bit address from the host processor
 will match the first byte (highest 2 bits) of the default client address
 (0x1FA) that is programmed in bcm20791.
 When used together with the first byte (0xFA) of the byte sequence below,
 it can be used to address the bcm20791 in a system that does not support
 10-bit address and change the default address to 0x38.
 the new address can be changed by changing the CLIENT_ADDRESS below if 0x38
 conflicts with other device on the same i2c bus.
 */
#define ALIAS_ADDRESS	  0x79

static void set_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
	struct i2c_client *client;

	NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return); 
	NULL_CHECK(bcm2079x_dev->client,"bcm2079x_dev->client",return); 

	client = bcm2079x_dev->client;
	client->addr = addr;
	if (addr > 0x7F)
		client->flags |= I2C_CLIENT_TEN;
    else
        client->flags &= ~I2C_CLIENT_TEN;

	dev_info(&client->dev,
		"Set client device changed to (0x%04X) flag = %04x\n",
		client->addr, client->flags);
}

static void change_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr)
{
	struct i2c_client *client;
	int ret;
	int i;
	int offset = 1;
	char addr_data[] = {
		0xFA, 0xF2, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2A
	};

	NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return); 
	NULL_CHECK(bcm2079x_dev->client,"bcm2079x_dev->client",return); 

	client = bcm2079x_dev->client;
	if ((client->flags & I2C_CLIENT_TEN) == I2C_CLIENT_TEN) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		offset = 0;
	}

	addr_data[5] = addr & 0xFF;
	ret = 0;
	for (i = 1; i < sizeof(addr_data) - 1; ++i)
		ret += addr_data[i];
	addr_data[sizeof(addr_data) - 1] = (ret & 0xFF);
	dev_info(&client->dev,
		 "Change client device from (0x%04X) flag = "\
		 "%04x, addr_data[%d] = %02x\n",
		 client->addr, client->flags, sizeof(addr_data) - 1,
		 addr_data[sizeof(addr_data) - 1]);
	ret = i2c_master_send(client, addr_data+offset, sizeof(addr_data)-offset);
	if (ret != sizeof(addr_data)-offset) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		dev_info(&client->dev,
			 "Change client device from (0x%04X) flag = "\
			 "%04x, addr_data[%d] = %02x\n",
			 client->addr, client->flags, sizeof(addr_data) - 1,
			 addr_data[sizeof(addr_data) - 1]);
		ret = i2c_master_send(client, addr_data, sizeof(addr_data));
	}
	client->addr = addr_data[5];

	dev_info(&client->dev,
		 "Change client device changed to (0x%04X) flag = %04x, ret = %d\n",
		 client->addr, client->flags, ret);
}

static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *dev_id)
{
	struct bcm2079x_dev *bcm2079x_dev;
	unsigned long flags;

	NULL_CHECK(dev_id,"dev_id",return IRQ_NONE); 

    bcm2079x_dev = dev_id;
	if(!wake_lock_active(&bcm2079x_dev->wake_locker))
	{
		wake_lock_timeout(&bcm2079x_dev->wake_locker, 30*HZ);
	}

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	bcm2079x_dev->count_irq++;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
	wake_up(&bcm2079x_dev->read_wq);

	return IRQ_HANDLED;
}

/*clock request irq handler*/
static irqreturn_t bcm2079x_clkreq_irq_handler(int irq, void *dev_id)
{
    struct bcm2079x_dev *bcm2079x_dev;
    struct clk * nfc_clk=NULL;
    int clk_req_value;
    unsigned long flags;

    NULL_CHECK(dev_id,"dev_id",return IRQ_NONE); 
    bcm2079x_dev = dev_id;
    NULL_CHECK(bcm2079x_dev->client,"bcm2079x_dev->client",return IRQ_NONE); 

    printk("%s:handler, irq=%d\n",__func__,irq);

    clk_req_value=gpio_get_value(bcm2079x_dev->clk_req_gpio);
    printk("%s:clk_req_value=%d\n",__func__,clk_req_value);

    if(clk_req_value == bcm2079x_dev->clock_req_value)
    {
	      printk("%s:irq not change. clk_req_value=%d,\n",__func__,clk_req_value);
	      return IRQ_HANDLED;		  
    }
    disable_irq_nosync(irq);

    nfc_clk = clk_get(&bcm2079x_dev->client->dev, "nfc_clk"); 
    if(nfc_clk == NULL)
    {
              printk("%s:clk_get failed\n",__func__);
              enable_irq(irq);
       	return IRQ_HANDLED;
    }
	   
    if(clk_req_value)
    {		
            printk("%s:clk_enable\n",__func__);
 	     /*enable clock output*/
 	     clk_enable(nfc_clk);

 	     /*modify irq type to low trigger*/
            irq_set_irq_type(irq,IRQF_TRIGGER_LOW | IRQF_ONESHOT);
    }
    else
    {
            printk("%s:clk_disable\n",__func__);
 	     /*disable clock output*/
 	     clk_disable(nfc_clk);//clk_disable and clk_enable should be the same times

 	     /*modify irq type to high trigger*/
            irq_set_irq_type(irq,IRQF_TRIGGER_HIGH | IRQF_ONESHOT);
    }
    	   
    spin_lock_irqsave(&bcm2079x_dev->clock_irq_lock, flags);
    bcm2079x_dev->clock_req_value = clk_req_value;
    spin_unlock_irqrestore(&bcm2079x_dev->clock_irq_lock, flags);
	
    enable_irq(irq);

    return IRQ_HANDLED;
}


static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
	struct bcm2079x_dev *bcm2079x_dev;
	unsigned int mask = 0;
	unsigned long flags;

	NULL_CHECK(filp,"filp",return 0); 
	NULL_CHECK(filp->private_data,"filp->private_data",return 0); 
	NULL_CHECK(wait,"wait",return 0); 

      bcm2079x_dev = filp->private_data;
    if(0 == gpio_get_value(bcm2079x_dev->irq_gpio) ||0 ==  bcm2079x_dev->count_irq)
	{
		//move into bcm2079x_dev_unlocked_ioctl()
	    //wake_unlock(&bcm2079x_dev->wake_locker);
        poll_wait(filp, &bcm2079x_dev->read_wq, wait);
    }
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->count_irq > 0)
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total, len, ret;

	NULL_CHECK(filp,"filp",return -EFAULT); 
	NULL_CHECK(filp->private_data,"filp->private_data",return -EFAULT); 
	NULL_CHECK(buf,"buf",return -EFAULT); 

	total = 0;
	len = 0;
	bcm2079x_dev = filp->private_data;
	NULL_CHECK(bcm2079x_dev->client,"filp->private_data->client",return -EFAULT); 

	if (bcm2079x_dev->count_irq > 0)
		bcm2079x_dev->count_irq--;

	bcm2079x_dev->count_read++;
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bcm2079x_dev->read_mutex);

	/** Read the first 4 bytes to include the length of the NCI or HCI packet.
	**/
	ret = i2c_master_recv(bcm2079x_dev->client, tmp, 4);
	if (ret == 4) {
		total = ret;
		/** First byte is the packet type
		**/
		switch(tmp[0]) {
			case PACKET_TYPE_NCI:
				len = tmp[PACKET_HEADER_SIZE_NCI-1];
				break;

			case PACKET_TYPE_HCIEV:
				len = tmp[PACKET_HEADER_SIZE_HCI-1];
				if (len == 0)
					total--;				/*Since payload is 0, decrement total size (from 4 to 3) */
				else
					len--;					/*First byte of payload is in tmp[3] already */
				break;

			default:
				len = 0;					/*Unknown packet byte */
				break;
		} /* switch*/

		/** make sure full packet fits in the buffer
		**/
		if (len > 0 && (len + total) <= count) {
			/** read the remainder of the packet.
			**/
			ret = i2c_master_recv(bcm2079x_dev->client, tmp+total, len);
			if (ret == len)
				total += len;
		} /* if */
	} /* if */

	mutex_unlock(&bcm2079x_dev->read_mutex);

	if (total > count || copy_to_user(buf, tmp, total)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to copy to user space, total = %d\n", total);
		total = -EFAULT;
		bcm2079x_dev->error_read++;
	}

	return total;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct bcm2079x_dev *bcm2079x_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	NULL_CHECK(filp,"filp",return -EFAULT); 
	NULL_CHECK(filp->private_data,"filp->private_data",return -EFAULT); 
	NULL_CHECK(buf,"buf",return -EFAULT); 

	bcm2079x_dev = filp->private_data;
	NULL_CHECK(bcm2079x_dev->client,"filp->private_data->client",return -EFAULT);


	if (count > MAX_BUFFER_SIZE) {
		dev_err(&bcm2079x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&bcm2079x_dev->client->dev,
			"failed to copy from user space\n");
		return -EFAULT;
	}

	mutex_lock(&bcm2079x_dev->read_mutex);
	/* Write data */

	ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
	if (ret != count) {
        printk("bcm2079x_dev_write: count is:%d, ret is:%d\n", count, ret);
		if ((bcm2079x_dev->client->flags & I2C_CLIENT_TEN) != I2C_CLIENT_TEN && bcm2079x_dev->error_write == 0) {
			set_client_addr(bcm2079x_dev, 0x1FA);
			ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
            if (ret != count) {
				bcm2079x_dev->error_write++;
                set_client_addr(bcm2079x_dev, bcm2079x_dev->original_address);
            }
		} else {
			dev_err(&bcm2079x_dev->client->dev,
				"failed to write %d\n", ret);
			ret = -EIO;
			bcm2079x_dev->error_write++;
		}
	}
	mutex_unlock(&bcm2079x_dev->read_mutex);

	return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct bcm2079x_dev *bcm2079x_dev; 

	NULL_CHECK(filp,"filp",return -EFAULT); 
	NULL_CHECK(inode,"inode",return -EFAULT); 

	bcm2079x_dev = container_of(filp->private_data,
							   struct bcm2079x_dev,
							   bcm2079x_device);
	NULL_CHECK(bcm2079x_dev,"bcm2079x_dev",return -EFAULT);
	NULL_CHECK(bcm2079x_dev->client,"filp->private_data->client",return -EFAULT);
	
	filp->private_data = bcm2079x_dev;
	bcm2079x_init_stat(bcm2079x_dev);
	bcm2079x_enable_irq(bcm2079x_dev);
	dev_info(&bcm2079x_dev->client->dev,
		 "device node major=%d, minor=%d\n", imajor(inode), iminor(inode));

	return ret;
}

static long bcm2079x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct bcm2079x_dev *bcm2079x_dev;

	NULL_CHECK(filp,"filp",return -EFAULT); 
	NULL_CHECK(filp->private_data,"filp->private_data",return -EFAULT); 

	bcm2079x_dev = filp->private_data;
	NULL_CHECK(bcm2079x_dev->client,"filp->private_data->client",return -EFAULT);

	switch (cmd) {
	case BCMNFC_READ_FULL_PACKET:
		break;
	case BCMNFC_READ_MULTI_PACKETS:
		break;
	case BCMNFC_CHANGE_ADDR:
		dev_info(&bcm2079x_dev->client->dev,
			 "%s, BCMNFC_CHANGE_ADDR (%x, %lx):\n", __func__, cmd,
			 arg);
		change_client_addr(bcm2079x_dev, arg);
		break;
	case BCMNFC_POWER_CTL:
        printk("NFC------BCMNFC_POWER_CTL\n");
		dev_info(&bcm2079x_dev->client->dev,
			 "%s, BCMNFC_POWER_CTL (%x, %lx):\n", __func__, cmd,
			 arg);
        if (arg != 1)
            set_client_addr(bcm2079x_dev, bcm2079x_dev->original_address);
		gpio_set_value(bcm2079x_dev->en_gpio, arg);
        /* need delay 50 here */
        mdelay(50);
		break;
	case BCMNFC_WAKE_CTL:
		dev_info(&bcm2079x_dev->client->dev,
			 "%s, BCMNFC_WAKE_CTL (%x, %lx):\n", __func__, cmd,
			 arg);
		gpio_set_value(bcm2079x_dev->wake_gpio, arg);
		if(0 != arg && wake_lock_active(&bcm2079x_dev->wake_locker))
		{
		   printk("%s, release wake lock!!!\n", __func__);
		   wake_unlock(&bcm2079x_dev->wake_locker);
		}
		break;
	default:
		dev_err(&bcm2079x_dev->client->dev,
			"%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
		return 0;
	}

	return 0;
}

static const struct file_operations bcm2079x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = bcm2079x_dev_poll,
	.read = bcm2079x_dev_read,
	.write = bcm2079x_dev_write,
	.open = bcm2079x_dev_open,
	.unlocked_ioctl = bcm2079x_dev_unlocked_ioctl
};

/*
1. NFC_WAKE         MSM8x12(GPIO_91)
2. NFC_INT            MSM8x12(GPIO_77)
3. NFC_VEN           PM8110(GPIO_2)
4. CLK_REQ_NFC    MSM8x12(GPIO_75)
5. TCXO_OUT_NFC  MSM8x12(GPIO_78) 
6. I2C5_DATA        MSM8x12(GPIO_10)
7. I2C5_CLK          MSM8x12(GPIO_11)
*/
static int bcm_nfc_parse_dt(struct device *dev,
                struct bcm2079x_platform_data *platform_data)
{
    struct device_node *np;

    NULL_CHECK(platform_data,"platform_data",return -EFAULT); 
    NULL_CHECK(dev,"dev",return -EFAULT); 
    NULL_CHECK(dev->of_node,"dev->of_node",return -EFAULT); 
    np = dev->of_node;

    printk("bcm_nfc_parse_dt enter\n");

    //NFC_WAKE         MSM8x12(GPIO_91)
    platform_data->wake_gpio = of_get_named_gpio_flags(np,
           			"huawei,wake-gpio", 0, NULL);
    if (platform_data->wake_gpio < 0) {
             printk("bcm_nfc_parse_dt:Failed to get \" huawei,wake-gpio\"\n");
	      return -1;
    }
    printk("huawei,wake-gpio=%d\n",platform_data->wake_gpio);

    //NFC_INT            MSM8x12(GPIO_77)
    platform_data->irq_gpio = of_get_named_gpio_flags(np,
			        "huawei,irq-gpio", 0, NULL);
    if (platform_data->irq_gpio < 0) {
             printk("bcm_nfc_parse_dt:Failed to get \" huawei,irq-gpio\"\n");
	      return -1;
    }
    printk("huawei,irq-gpio=%d\n",platform_data->irq_gpio);
	
    //NFC_VEN           PM8110(GPIO_2)
    platform_data->en_gpio = of_get_named_gpio_flags(np,
			        "huawei,en-gpio", 0, NULL);
    if (platform_data->en_gpio < 0) {
             printk("bcm_nfc_parse_dt:Failed to get \" huawei,en-gpio\"\n");
	      return -1;
    }
    printk("huawei,en-gpio=%d\n",platform_data->en_gpio);
	
    //CLK_REQ_NFC    MSM8x12(GPIO_75)
    platform_data->clock_req_gpio = of_get_named_gpio_flags(np,
			        "huawei,clk-req-gpio", 0, NULL);
    if (platform_data->clock_req_gpio < 0) {
             printk("bcm_nfc_parse_dt:Failed to get \" huawei,clk-req-gpio\"\n");
	      return -1;
    }    
    printk("huawei,clk-req-gpio=%d\n",platform_data->clock_req_gpio);

    //TCXO_OUT_NFC  MSM8x12(GPIO_78) 
    platform_data->clock_out_gpio = of_get_named_gpio_flags(np,
			        "huawei,clk-out-gpio", 0, NULL);
    if (platform_data->clock_out_gpio < 0) {
             printk("bcm_nfc_parse_dt:Failed to get \" huawei,clk-out-gpio\"\n");
	      return -1;
    }    
    printk("huawei,clk-out-gpio=%d\n",platform_data->clock_out_gpio);

    printk("bcm_nfc_parse_dt exit\n");

    return 0;
}
static int bcm_nfc_gpio_conf(struct device *dev,
                struct bcm2079x_platform_data *platform_data)
{
      int ret = -1;
      int gpio_config=0;

      NULL_CHECK(platform_data,"platform_data",return -EFAULT); 

      printk("bcm_nfc_gpio_conf enter\n");

       //nfc_int
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret)
	{
		printk("gpio_request nfc_int error \n");
		goto err_exit4;
	}
	gpio_config = GPIO_CFG(platform_data->irq_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	gpio_direction_input(platform_data->irq_gpio);

	//nfc_ven:configure in msm-pm8110.dtsi->gpio@c100, no need configure here
	ret = gpio_request(platform_data->en_gpio, "nfc_ven");
	if (ret)
	{
		printk("gpio_request nfc_ven error \n");
		goto err_exit3;
	}
       
       //nfc_wake
	ret = gpio_request(platform_data->wake_gpio, "nfc_wake");
	if (ret)
	{
		printk("gpio_request nfc_wake error \n");
		goto err_exit2;
	}
	gpio_config = GPIO_CFG(platform_data->wake_gpio, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	gpio_direction_output(platform_data->wake_gpio,0);

       //nfc_clk_req
	ret = gpio_request(platform_data->clock_req_gpio, "nfc_clk_req");
	if (ret)
	{
		printk("gpio_request nfc_clk_req error \n");
		goto err_exit1;
	}
	gpio_config = GPIO_CFG(platform_data->clock_req_gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	gpio_direction_input(platform_data->clock_req_gpio);

	//nfc_clk_out
	ret = gpio_request(platform_data->clock_out_gpio, "nfc_clk_out");
	if (ret)
	{
		printk("gpio_request nfc_clk_out error \n");
		goto err_exit0;
	}
	gpio_config = GPIO_CFG(platform_data->clock_out_gpio, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA);
	gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);

	printk("bcm_nfc_gpio_conf exit\n");
	
	return 0;

 err_exit0:
	gpio_free(platform_data->clock_req_gpio);
 err_exit1:
	gpio_free(platform_data->wake_gpio);
 err_exit2:
	gpio_free(platform_data->en_gpio);
 err_exit3:
	gpio_free(platform_data->irq_gpio);
 err_exit4:

	printk("bcm_nfc_gpio_conf failed \n");

	return ret;

}

static int bcm2079x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bcm2079x_platform_data *platform_data;
	struct bcm2079x_dev *bcm2079x_dev;
	struct clk *nfc_clk = NULL;
	int nfc_clk_irq=-ENXIO;
	int retval = -1;

	NULL_CHECK(client,"client",return -1);  
	NULL_CHECK(client->adapter,"client->adapter",return -1); 
    
	printk("bcm2079x_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	/*get dts config if dts is enabled--of_node */
      if (client->dev.of_node) 
      {
          /*allocate memory for client->dev,return pointer to allocated memory*/
          platform_data = devm_kzalloc(&client->dev,
                                                       sizeof(*platform_data),
                                                       GFP_KERNEL);
          if (!platform_data)
          {
              dev_err(&client->dev, "Failed to allocate memory\n");
              return -ENOMEM;
          }
  
          retval = bcm_nfc_parse_dt(&client->dev, platform_data);
          if (retval)
          {
              printk("bcm_nfc_parse_dt failed, retval=%d!\n",retval);
              return retval;
          }
      } 
      else
      {
             dev_err(&client->dev, "%s:%d: Core device nfc: No OF data detected!Going to probing bcm2079x driver flags = %x\n", 
			 	                             __func__, __LINE__, client->flags);
		printk("No OF data detected!\n");	 
		platform_data = client->dev.platform_data;
        	if (platform_data == NULL) {
        		printk("nfc probe fail\n");
        		return -ENODEV;
        	}
      }

      //gpio configure
      retval = bcm_nfc_gpio_conf(&client->dev, platform_data);
      if (retval)
      {
           printk("bcm_nfc_gpio_conf failed, retval=%d!\n",retval);
           return retval;
      }
	  
	//configure nfc clk
	printk("bcm2079x dev kobj_name=%s,init_name=%s,id=%d\n",client->dev.kobj.name,client->dev.init_name, client->dev.id);
	nfc_clk  = clk_get(&client->dev, "nfc_clk");
	if (nfc_clk == NULL) {
		printk("failed to get clk \"nfc_clk\"\n");
		return retval;
	}
	retval = clk_set_rate(nfc_clk, 19200000); // set the gp1 clock to 19.2 MHz 
	if (retval) {
		printk("failed to set clk: %d\n", retval);
		return retval;
	}
	retval = clk_prepare(nfc_clk); 
	if (retval) {
		printk("failed to prepare clk: %d\n", retval);
		return retval;
	}     

       /*intialize*/
	gpio_set_value(platform_data->en_gpio, 0);
	gpio_set_value(platform_data->wake_gpio, 0);

	bcm2079x_dev = kzalloc(sizeof(*bcm2079x_dev), GFP_KERNEL);
	if (bcm2079x_dev == NULL) {
		dev_err(&client->dev,"failed to allocate memory for module data\n");
		retval = -ENOMEM;
		goto err_exit;
	}

	bcm2079x_dev->wake_gpio = platform_data->wake_gpio;
	bcm2079x_dev->irq_gpio = platform_data->irq_gpio;
	bcm2079x_dev->en_gpio = platform_data->en_gpio;
	bcm2079x_dev->clk_req_gpio = platform_data->clock_req_gpio;
	bcm2079x_dev->clk_out_gpio = platform_data->clock_out_gpio;
	bcm2079x_dev->client = client;
	bcm2079x_dev->clock_req_value = gpio_get_value(platform_data->clock_req_gpio);

	/* init mutex and queues */
	init_waitqueue_head(&bcm2079x_dev->read_wq);
	mutex_init(&bcm2079x_dev->read_mutex);
	spin_lock_init(&bcm2079x_dev->irq_enabled_lock);
	spin_lock_init(&bcm2079x_dev->clock_irq_lock);
	wake_lock_init(&bcm2079x_dev->wake_locker, WAKE_LOCK_SUSPEND, "nfc_wakelock");

	bcm2079x_dev->bcm2079x_device.minor = MISC_DYNAMIC_MINOR;
	bcm2079x_dev->bcm2079x_device.name = "bcm2079x";
	bcm2079x_dev->bcm2079x_device.fops = &bcm2079x_dev_fops;

	retval = misc_register(&bcm2079x_dev->bcm2079x_device);
	if (retval) {
		printk("misc_register failed\n");
		goto err_misc_register;
	}

	printk("%s, saving address %d\n", __func__, client->addr);
	bcm2079x_dev->original_address = client->addr;

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	client->irq = gpio_to_irq(platform_data->irq_gpio);
	printk("requesting IRQ %d with IRQF_NO_SUSPEND\n", client->irq);
	bcm2079x_dev->irq_enabled = true;
	retval = request_irq(client->irq, bcm2079x_dev_irq_handler,
			  IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND, client->name, bcm2079x_dev);
	if (retval) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}
	enable_irq_wake(client->irq);
	
	bcm2079x_disable_irq(bcm2079x_dev);

	//set clock req irq
	nfc_clk_irq = gpio_to_irq(platform_data->clock_req_gpio);
	retval = request_irq(nfc_clk_irq, bcm2079x_clkreq_irq_handler,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT, client->name, bcm2079x_dev);
	if (retval) {
		printk("set bcm2079x_clkreq_irq_failed\n");
		goto err_request_irq_failed;
	}
	enable_irq(nfc_clk_irq);
	enable_irq_wake(nfc_clk_irq);
	
	i2c_set_clientdata(client, bcm2079x_dev);
	printk(
		 "%s, probing bcm2079x driver exited successfully\n",
		 __func__);

	printk("bcm2079x_probe exit\n");

	return 0;

err_request_irq_failed:
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
err_misc_register:
	wake_lock_destroy(&bcm2079x_dev->wake_locker);
	mutex_destroy(&bcm2079x_dev->read_mutex);
	kfree(bcm2079x_dev);
err_exit:
	gpio_free(platform_data->wake_gpio);
	gpio_free(platform_data->en_gpio);
	gpio_free(platform_data->irq_gpio);
	gpio_free(platform_data->clock_req_gpio);
	gpio_free(platform_data->clock_out_gpio);
	return retval;
}

static int bcm2079x_remove(struct i2c_client *client)
{
	struct bcm2079x_dev *bcm2079x_dev;

    NULL_CHECK(client,"client",return 0);  
	
	bcm2079x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, bcm2079x_dev);
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
	wake_lock_destroy(&bcm2079x_dev->wake_locker);
	mutex_destroy(&bcm2079x_dev->read_mutex);
	gpio_free(bcm2079x_dev->irq_gpio);
	gpio_free(bcm2079x_dev->en_gpio);
	gpio_free(bcm2079x_dev->wake_gpio);
       gpio_free(bcm2079x_dev->clk_req_gpio);
	gpio_free(bcm2079x_dev->clk_out_gpio);
	kfree(bcm2079x_dev);

	return 0;
}

static const struct i2c_device_id bcm2079x_id[] = {
	{"bcm2079x-i2c", 0},
	{}
};

static const struct of_device_id nfc_match_table[] = {
	{.compatible = "bcm,bcm2079x-i2c", },
	{}
};

static struct i2c_driver bcm2079x_driver = {
	.id_table = bcm2079x_id,
	.probe = bcm2079x_probe,
	.remove = bcm2079x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bcm2079x-i2c",
	       .of_match_table = nfc_match_table,
	},
};


/*
 * module load/unload record keeping
 */

static int __init bcm2079x_dev_init(void)
{
	printk("bcm2079x_dev_init\n");
	return i2c_add_driver(&bcm2079x_driver);
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
	printk("bcm2079x_dev_exit\n");
	i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
