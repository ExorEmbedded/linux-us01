/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

#include <net/netlink.h>
#include <net/genetlink.h>

/* generic netlink attributes */
enum {
	BACKLIGHT_CONFIG_A_UNSPEC,
	BACKLIGHT_CONFIG_A_BRIGHTNESS_MIN,
	BACKLIGHT_CONFIG_A_BRIGHTNESS_MAX,
	BACKLIGHT_CONFIG_A_PWMFREQ,
	BACKLIGHT_CONFIG_A_INVERTER_NAME,
    __BACKLIGHT_CONFIG_A_MAX,
};
#define BACKLIGHT_CONFIG_A_MAX (__BACKLIGHT_CONFIG_A_MAX - 1)

/* generic netlink attribute policy */
static struct nla_policy backlight_config_genl_policy[BACKLIGHT_CONFIG_A_MAX + 1] = {
	[BACKLIGHT_CONFIG_A_BRIGHTNESS_MIN] = { .type = NLA_U32 },
	[BACKLIGHT_CONFIG_A_BRIGHTNESS_MAX] = { .type = NLA_U32 },
	[BACKLIGHT_CONFIG_A_PWMFREQ] = { .type = NLA_U64 },
	[BACKLIGHT_CONFIG_A_INVERTER_NAME] = { .type = NLA_NUL_STRING },
};

/* generic netlink family definition */
static struct genl_family backlight_config_gnl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = "BL_CONFIG",
	.version = 1,
	.maxattr = BACKLIGHT_CONFIG_A_MAX,
};

struct backlight_config {
	int min_brightness;
	int lth_brightness;
	int uth_brightness;
	int max_brightness;
	int pwm_period_ns;

	const char *name;		/* inverter name */
};

static struct backlight_config pwm_bl_config;

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

static int pwm_backlight_netlink_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (brightness == 0)
		brightness = 1;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
		brightness = pb->lth_brightness +
			(brightness * ((pb->period * pwm_bl_config.uth_brightness / 100) - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_netlink_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_netlink_update_status,
	.get_brightness	= pwm_backlight_netlink_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_netlink_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	data->max_brightness = pwm_bl_config.max_brightness;
	data->lth_brightness = pwm_bl_config.lth_brightness;
	data->pwm_period_ns = pwm_bl_config.pwm_period_ns;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_netlink_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_netlink_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_netlink_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspendnetlink_	NULL
#define pwm_backlight_resumenetlink_	NULL
#endif

static struct platform_driver pwm_backlight_netlink_driver = {
	.driver		= {
		.name	= "pwm-backlight-netlink",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_netlink_probe,
	.remove		= pwm_backlight_netlink_remove,
	.suspend	= pwm_backlight_netlink_suspend,
	.resume		= pwm_backlight_netlink_resume,
};

/* handler */
 static int backlight_config(struct sk_buff *skb, struct genl_info *info)
 {
       /* message handling code goes here; return 0 on success, negative
        * values on failure */
		if(skb == NULL) {
			printk(KERN_ERR "skb is NULL \n");
			return -1;
		}

		printk(KERN_DEBUG "Bl Received generic netlink message brightness_min: %d\n", nla_get_u32(info->attrs[BACKLIGHT_CONFIG_A_BRIGHTNESS_MIN]));
		printk(KERN_DEBUG "Bl Received generic netlink message brightness_max: %d\n", nla_get_u32(info->attrs[BACKLIGHT_CONFIG_A_BRIGHTNESS_MAX]));
		printk(KERN_DEBUG "Bl Received generic netlink message pwmfreq: %ld\n", (long)nla_get_u64(info->attrs[BACKLIGHT_CONFIG_A_PWMFREQ]));
		printk(KERN_DEBUG "Bl Received generic netlink message inverter_name: %s\n", (char *)nla_data(info->attrs[BACKLIGHT_CONFIG_A_INVERTER_NAME]));

		pwm_bl_config.min_brightness = 0;
		pwm_bl_config.lth_brightness = (long)(nla_get_u32(info->attrs[BACKLIGHT_CONFIG_A_BRIGHTNESS_MIN])*255/100);
		pwm_bl_config.uth_brightness = (long)(nla_get_u32(info->attrs[BACKLIGHT_CONFIG_A_BRIGHTNESS_MAX]));
		pwm_bl_config.max_brightness = 255;
		pwm_bl_config.pwm_period_ns = (long)((long)1000000000/(long)nla_get_u64(info->attrs[BACKLIGHT_CONFIG_A_PWMFREQ]));
		pwm_bl_config.name = (char *)nla_data(info->attrs[BACKLIGHT_CONFIG_A_INVERTER_NAME]);

		return platform_driver_register(&pwm_backlight_netlink_driver);
 }

 /* commands */
 enum {
	 BACKLIGHT_CONFIG_C_UNSPEC,
	 BACKLIGHT_CONFIG_C_SET,
	 __BACKLIGHT_CONFIG_C_MAX,
 };
 #define BACKLIGHT_CONFIG_C_MAX (__BACKLIGHT_CONFIG_C_MAX - 1)

 /* operation definition */
 static struct genl_ops backlight_config_gnl_ops_set = {
	.cmd = BACKLIGHT_CONFIG_C_SET,
	.flags = 0,
	.policy = backlight_config_genl_policy,
	.doit = backlight_config,
	.dumpit = NULL,
 };

static int __init pwm_backlight_netlink_init(void)
{
	int rf;
	int ro;


	printk(KERN_INFO "Bl Initializing generic netlink socket");

	rf = genl_register_family(&backlight_config_gnl_family);
	if (rf == 0)
		ro = genl_register_ops(&backlight_config_gnl_family, &backlight_config_gnl_ops_set);

	// use default values when the initialization of the generic netlink fails
	if (rf != 0 || ro != 0)
		return platform_driver_register(&pwm_backlight_netlink_driver);

	return 0;
//	return platform_driver_register(&pwm_backlight_netlink_driver);
}
module_init(pwm_backlight_netlink_init);

static void __exit pwm_backlight_netlink_exit(void)
{
	platform_driver_unregister(&pwm_backlight_netlink_driver);
}
module_exit(pwm_backlight_netlink_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver with netlink support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight-netlink");

