/*
 * Copyright (C) 2012 Exor International
 * Author: Stefano Galvan <stefanocv@gmail.com>
 * Author: Giovanni Pavoni <giovanni.pavoni@exorint.it>
 *
 * Beeper on touch driver - sends a SND_BELL event whenever BTN_TOUCH
 * event occurs. Based on evbug.c and keyboard.c
 */

/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/device.h>

static int enable = 0;

void bot_mksound(unsigned int);

static void bot_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if(enable) {
		if(type == EV_KEY && code == BTN_TOUCH) {
			if(value == 1)
				bot_mksound(1);
			else
				bot_mksound(0);
		}
	}
}

static int bot_connect(struct input_handler *handler, struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "beeper-on-touch";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	printk(KERN_DEBUG "beeper-on-touch.c: Connected device: %s (%s at %s)\n",
		dev_name(&dev->dev),
		dev->name ?: "unknown",
		dev->phys ?: "unknown");

	return 0;

 err_unregister_handle:
	input_unregister_handle(handle);
 err_free_handle:
	kfree(handle);
	return error;
}

static void bot_disconnect(struct input_handle *handle)
{
	printk(KERN_DEBUG "beeper-on-touch.c: Disconnected device: %s\n",
		dev_name(&handle->dev->dev));

	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id bot_ids[] = 	{
{
        .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
        .evbit = { BIT_MASK(EV_KEY) },
},
{
        .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
        .evbit = { BIT_MASK(EV_SND) },
},
{ },    /* Terminating entry */
};

MODULE_DEVICE_TABLE(input, bot_ids);

static struct input_handler bot_handler = {
	.event =	bot_event,
	.connect =	bot_connect,
	.disconnect =	bot_disconnect,
	.name =		"beeper-on-touch",
	.id_table =	bot_ids,
};

static int bot_sound_helper(struct input_handle *handle, void *data)
{
	unsigned int *hz = data;
	struct input_dev *dev = handle->dev;

	if (test_bit(EV_SND, dev->evbit)) {
		if (test_bit(SND_BELL, dev->sndbit))
			input_inject_event(handle, EV_SND, SND_BELL, *hz ? 1 : 0);
	}

	return 0;
}

void bot_mksound(unsigned int value)
{
	input_handler_for_each_handle(&bot_handler, &value, bot_sound_helper);
}

static int __init bot_init(void)
{
	return input_register_handler(&bot_handler);
}

static void __exit bot_exit(void)
{
	input_unregister_handler(&bot_handler);
}

module_init(bot_init);
module_exit(bot_exit);

module_param(enable, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Activates beeper on touch (default:1)");

MODULE_AUTHOR("Stefano Galvan/Giovanni Pavoni");
MODULE_DESCRIPTION("Beeper on touch");
MODULE_LICENSE("GPL");

