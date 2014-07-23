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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/i2c/eeprom.h>
#include <linux/i2c/I2CSeeprom.h>
#include <linux/backlight.h>

#define RESET_CMD		"Reset__counter"
#define POLLING_INTERVAL	60000

#define WORKINGHOURS_DRV_NAME	"working_hours"
#define DRIVER_VERSION		"1.0"

/* Function prototype to retrieve the pwm_backlight status */
bool pwm_backlight_is_enabled(struct backlight_device* bl);

struct hrs_data 
{
  struct mutex             lock;
  // Update thread stuff
  struct task_struct*      auto_update;
  struct completion        auto_update_stop;
  unsigned int             auto_update_interval;
  // I2C SEEPROM accessor
  struct i2c_client*       seeprom_client;
  struct memory_accessor*  seeprom_macc;
  // Hours counters into I2C SEEPROM
  u32                      sys_hours;
  u32                      blight_hours;
  // Backlight related stuff
  struct backlight_device* backlight;
  bool                     bl_enabled;
};

/*
 * Static helper function to get the hours counters from I2C SEEPROM
 */
static int get_hours_from_seeprom(struct hrs_data* data)
{
  int i;
  int r1, r2;
  u8  chksum, tmp;
  u32 tmphours;
  struct memory_accessor* macc = data->seeprom_macc;
  
  data->sys_hours = 0;
  data->blight_hours = 0;

  mutex_lock(&data->lock);
  
  /* Read sys_hours counter from I2C SEEPROM, with retry */
  tmphours = 0;
  chksum = 0;
  for (i = 0; i < 5; i++) 
  {
    r1 = macc->read(macc, (u8*)&tmphours, SYSTEM_HOUR_COUNTER_OFFSET_I2C, sizeof(u32));
    r2 = macc->read(macc, &chksum, SYSTEM_HOUR_CHK_OFFSET_I2C, sizeof(u8));
    
    if((r1 == sizeof(u32)) && (r2 == sizeof(u8)))
    {
      break;
    }
    printk("Retrying to read the sys_hours counter n=%d\n",i);
    msleep(200);
  }

  tmp = 1 + (u8)((tmphours >> 24) & 0xff) + (u8)((tmphours >> 16) & 0xff) + (u8)((tmphours >> 8) & 0xff) + (u8)((tmphours >> 0) & 0xff);
  tmp = tmp - 0xaa;
  if(tmp == chksum) //The counter value is OK ... take it
    data->sys_hours = tmphours;
  
  /* Read blight_hours counter from I2C SEEPROM, with retry */
  tmphours = 0;
  chksum = 0;
  for (i = 0; i < 5; i++) 
  {
    r1 = macc->read(macc, (u8*)&tmphours, BACKLIGHT_HOUR_COUNTER_OFFSET_I2C, sizeof(u32));
    r2 = macc->read(macc, &chksum,   BACKLIGHT_HOUR_CHK_OFFSET_I2C,     sizeof(u8));
    
    if((r1 == sizeof(u32)) && (r2 == sizeof(u8)))
    {
      break;
    }
    printk("Retrying to read the blight_hours counter n=%d\n",i);
    msleep(200);
  }
  
  tmp = 1 + (u8)((tmphours >> 24) & 0xff) + (u8)((tmphours >> 16) & 0xff) + (u8)((tmphours >> 8) & 0xff) + (u8)((tmphours >> 0) & 0xff);
  tmp = tmp - 0xaa;
  if(tmp == chksum) //The counter value is OK ... take it
    data->blight_hours = tmphours;

  mutex_unlock(&data->lock);
  
  return 0;
}

/*
 * static helper function for parsing the DTB tree
 */
#ifdef CONFIG_OF
static int hrs_parse_dt(struct device *dev, struct hrs_data *data)
{
  struct device_node* node = dev->of_node;
  struct device_node* eeprom_node;
  struct device_node* backlight_node;
  u32                 eeprom_handle;
  int                 ret;
  /* Parse the DT to find the I2C SEEPROM bindings*/
  ret = of_property_read_u32(node, "eeprom", &eeprom_handle);
  if (ret != 0) 
  {
    dev_err(dev, "Failed to locate eeprom\n");
    return -ENODEV;
  }
  
  printk("eeprom_handle=0x%x\n",eeprom_handle); //!!!
  
  eeprom_node = of_find_node_by_phandle(eeprom_handle);
  if (eeprom_node == NULL) 
  {
    dev_err(dev, "Failed to find eeprom node\n");
    return -ENODEV;
  }

  printk("eeprom_node=%s\n",eeprom_node->name); //!!!

  data->seeprom_client = of_find_i2c_device_by_node(eeprom_node);
  if (data->seeprom_client == NULL) 
  {
    dev_err(dev, "Failed to find i2c client\n");
    of_node_put(eeprom_node);
    return -EPROBE_DEFER;
  }
  /* release ref to the node and inc reference to the I2C client used */
  of_node_put(eeprom_node);
  eeprom_node = NULL;
  i2c_use_client(data->seeprom_client);
  
  /* And now get the I2C SEEPROM memory accessor */
  data->seeprom_macc = i2c_eeprom_get_memory_accessor(data->seeprom_client);
  if (IS_ERR_OR_NULL(data->seeprom_macc)) 
  {
    dev_err(dev, "Failed to get memory accessor\n");
    return -ENODEV;
  }
  
  /* parse the DT to get the backlight references */
  backlight_node = of_parse_phandle(dev->of_node, "backlight", 0);
  if (backlight_node) 
  {
    data->backlight = of_find_backlight_by_node(backlight_node);
    of_node_put(backlight_node);
    if (!data->backlight)
      return -EPROBE_DEFER;
  }
  else
  {
    dev_err(dev, "Failed to get backlight node\n");
    return -ENODEV;
  }
    
  return 0;
}
#else
static int hrs_parse_dt(struct device *dev, struct hrs_data *data)
{
  return -ENODEV;
}
#endif

/* 
 * Polling thread for backlight and system working hours
 */
static int update_thread(void *p)
{
  struct hrs_data *data = dev_get_drvdata(p);
  
  while (!kthread_should_stop()) 
  {
    mutex_lock(&data->lock);
    data->bl_enabled = pwm_backlight_is_enabled(data->backlight);
    //TODO: Add polling and update logic here
    printk("Backlight=%d\n",data->bl_enabled);
    mutex_unlock(&data->lock);
    if (kthread_should_stop())
      break;
    msleep_interruptible(data->auto_update_interval);
  }
  
  complete_all(&data->auto_update_stop);
  return 0;
}

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

  res = hrs_parse_dt(&pdev->dev, data);
  if (res) {
    dev_err(&pdev->dev, "Could not find valid DT data.\n");
    goto hrs_error1;
  }
  
  res = sysfs_create_group(&pdev->dev.kobj, &working_hours_attr_group);
  if (res) 
  {
    dev_err(&pdev->dev, "device create file failed\n");
    goto hrs_error1;
  }
  
  mutex_init(&data->lock); 
  
  get_hours_from_seeprom(data);
  
  data->auto_update_interval = POLLING_INTERVAL;
  init_completion(&data->auto_update_stop);
  data->auto_update = kthread_run(update_thread, &pdev->dev, "%s", dev_name(&pdev->dev));
				  
  printk("working hours driver installed !!!\n");
  return res;
hrs_error1:
  kfree(data);
  return res;
}
			     
static int workinghours_remove(struct platform_device *pdev)
{
  struct hrs_data *data = dev_get_drvdata(&pdev->dev);
  
  kthread_stop(data->auto_update);
  wait_for_completion(&data->auto_update_stop);
  
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
