#include <linux/module.h>
#include <linux/input.h>
#include  <linux/timer.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#define POWER_KEY_LONG_PRESS_TIMEOUT	6000	/* Timeout (millisecond) */
#define POWER_KEY_INPUT "power_key_input"

static inline void power_key_long_press_cb(unsigned long arg)
{
	printk("power key has been pressed over 6s\n");
}

static int power_key_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = POWER_KEY_INPUT;

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void power_key_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static 	struct timer_list timer;

/*
 * power_key_input_event - monitor power key input and prompt long press on 
 * power key action
 *
 * when press down power key, setup a timer with a 6 senconds expire time
 * when the expire time run out but the power key is not released, print log
 * showing this issue.
 *
 * when release power key, delete the timer
 */
static void power_key_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	int timer_rc = 0;

	switch(code) {
	case KEY_POWER:
		if(value) {
			setup_timer(&timer, power_key_long_press_cb, 0);
			timer_rc = mod_timer(&timer, jiffies + msecs_to_jiffies(POWER_KEY_LONG_PRESS_TIMEOUT));
			if (timer_rc) {
				printk(KERN_ERR "Error adding power key long press monitor timer\n");
			}
		} else {
			del_timer(&timer);
		}
		break;
	default:
			break;
	}
}

static const struct input_device_id input_key_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler power_key_input_handler = {
	.event		= power_key_input_event,
	.connect	= power_key_input_connect,
	.disconnect	= power_key_input_disconnect,
	.name		= POWER_KEY_INPUT,
	.id_table	= input_key_ids,
};

/* registered or unregistered power_key_input_handler*/
static bool power_key_monitor_handler_registered = false;

static void power_key_monitor_register_handler(void)
{
	int error;

	error = input_register_handler(&power_key_input_handler);
	if (error) {
		printk(KERN_ERR "Failed to register input handler, error %d\n", error);
	} else {
		power_key_monitor_handler_registered = true;
	}
}

static void power_key_monitor_unregister_handler(void)
{
	if (power_key_monitor_handler_registered) {
		input_unregister_handler(&power_key_input_handler);
		power_key_monitor_handler_registered = false;
	}
}

static int __init power_key_monitor_init(void)
{
	power_key_monitor_register_handler();
	return 0;
}

static void __exit power_key_monitor_exit(void)
{
	power_key_monitor_unregister_handler();
}

module_init(power_key_monitor_init);
module_exit(power_key_monitor_exit);

MODULE_LICENSE("GPL");
