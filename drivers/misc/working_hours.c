/*
 *  working_hours.c - Linux kernel module for uSxx working hours counters
 *
 *  Written by: Giovanni Pavoni, Exor S.p.a.
 *  Copyright (c) 2014 Exor S.p.a.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

//#include <linux/gpio.h>
//#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
//#include <linux/fb.h>
//#include <linux/backlight.h>
#include <linux/err.h>
//#include <linux/pwm.h>
//#include <linux/pwm_backlight.h>
//#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
//#include <video/displayconfig.h>
//#include <linux/init.h> 

#define RESET_CMD		"Reset__counter"

#define WORKINGHOURS_DRV_NAME	"working_hours"
#define DRIVER_VERSION		"1.0"

struct hrs_data 
{
  struct mutex mutex;
};

/*
 * sysfs interface for backlight time counter
 */
static ssize_t show_blight_hours(struct device *dev, struct device_attribute *attr, char *buf)
{
  printk("TODO: show_blight_hours\n");
  return sprintf(buf, "0\n");
}

static ssize_t reset_blight_hours(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  if(strncmp(buf,RESET_CMD,strlen(RESET_CMD)))
  {
    dev_err(dev, "blight_hours: wrong cmd\n");
    return -1;
  }
  printk("TODO: reset_blight_hours\n");
  return count;
}
static DEVICE_ATTR(blight_hours, S_IRUGO | S_IWUSR, show_blight_hours, reset_blight_hours);

/*
 * sysfs interface for system time counter
 */
static ssize_t show_sys_hours(struct device *dev, struct device_attribute *attr, char *buf)
{
  printk("TODO: show_sys_hours\n");
  return sprintf(buf, "0\n");
}

static ssize_t reset_sys_hours(struct device *dev, struct device_attribute *attr, const  char *buf, size_t count)
{
  if(strncmp(buf,RESET_CMD,strlen(RESET_CMD)))
  {
    dev_err(dev, "sys_hours: wrong cmd\n");
    return -1;
  }
  printk("TODO: reset_sys_hours\n");
  return count;
}
static DEVICE_ATTR(sys_hours, S_IRUGO | S_IWUSR, show_sys_hours, reset_sys_hours);

static struct attribute *working_hours_attributes[] = {
  &dev_attr_blight_hours.attr,
  &dev_attr_sys_hours.attr,
  NULL
};

static const struct attribute_group working_hours_attr_group = {
  .name = WORKINGHOURS_DRV_NAME,
  .attrs = working_hours_attributes,
};

/*
 * Probe and remove functions
 */
static int workinghours_probe(struct platform_device *pdev)
{
  int res;
  struct hrs_data *data;
   
  data = kzalloc(sizeof(struct hrs_data), GFP_KERNEL);
  if (data == NULL) 
  {
    dev_err(&pdev->dev, "Memory allocation failed\n");
    return -ENOMEM;
  }
  dev_set_drvdata(&pdev->dev, data);
    
  res = sysfs_create_group(&pdev->dev.kobj, &working_hours_attr_group);
  if (res) 
  {
    dev_err(&pdev->dev, "device create file failed\n");
    goto hrs_error1;
  }
  
  mutex_init(&data->mutex); 
  printk("working hours driver installed !!!\n");
  return res;
hrs_error1:
  kfree(data);
  return res;
}
			     
static int workinghours_remove(struct platform_device *pdev)
{
  struct hrs_data *data = dev_get_drvdata(&pdev->dev);
  sysfs_remove_group(&pdev->dev.kobj, &working_hours_attr_group);
  kfree(data);
  return 0;
}

/*
 * Driver instantiation
 */
#ifdef CONFIG_OF
static struct of_device_id working_hours_of_match[] = {
  { .compatible = "working_hours" },
  { }
};

MODULE_DEVICE_TABLE(of, working_hours_of_match);
#endif

static struct platform_driver working_hours_driver = {
  .driver		= {
    .name		= WORKINGHOURS_DRV_NAME,
    .owner		= THIS_MODULE,
    .of_match_table	= of_match_ptr(working_hours_of_match),
  },
  .probe		= workinghours_probe,
  .remove		= workinghours_remove,
};

module_platform_driver(working_hours_driver);

MODULE_DESCRIPTION("working hours counter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:working_hours");
