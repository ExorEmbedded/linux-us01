/*
 * LCD panel driver for Sharp LQ043T1DG01
 *
 * Copyright (C) 2012 Sitek
 * Author: Stefano Galvan <stefanocv@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>

#include <plat/display.h>

#include <linux/netlink.h>
#include <net/sock.h>
#include <net/netlink.h>
#include <net/genetlink.h>

/* generic netlink attributes */
enum {
	DISPLAY_CONFIG_A_UNSPEC,
	DISPLAY_CONFIG_A_BRIGHTNESS_MIN,
	DISPLAY_CONFIG_A_BRIGHTNESS_MAX,
	DISPLAY_CONFIG_A_PWMFREQ,
	DISPLAY_CONFIG_A_REZX,
	DISPLAY_CONFIG_A_REZY,
	DISPLAY_CONFIG_A_BPP,
	DISPLAY_CONFIG_A_HS_FP,
	DISPLAY_CONFIG_A_HS_BP,
	DISPLAY_CONFIG_A_HS_W,
	DISPLAY_CONFIG_A_HS_INV,
	DISPLAY_CONFIG_A_VS_FP,
	DISPLAY_CONFIG_A_VS_BP,
	DISPLAY_CONFIG_A_VS_W,
	DISPLAY_CONFIG_A_VS_INV,
	DISPLAY_CONFIG_A_BLANK_INV,
	DISPLAY_CONFIG_A_PCLK_FREQ,
	DISPLAY_CONFIG_A_PCLK_INV,
	DISPLAY_CONFIG_A_INVERTER_NAME,
    __DISPLAY_CONFIG_A_MAX,
};
#define DISPLAY_CONFIG_A_MAX (__DISPLAY_CONFIG_A_MAX - 1)

/* generic netlink attribute policy */
static struct nla_policy display_config_genl_policy[DISPLAY_CONFIG_A_MAX + 1] = {
	[DISPLAY_CONFIG_A_BRIGHTNESS_MIN] = { .type = NLA_U32 },
	[DISPLAY_CONFIG_A_BRIGHTNESS_MAX] = { .type = NLA_U32 },
	[DISPLAY_CONFIG_A_PWMFREQ] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_REZX] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_REZY] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_BPP] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_HS_FP] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_HS_BP] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_HS_W] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_HS_INV] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_VS_FP] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_VS_BP] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_VS_W] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_VS_INV] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_BLANK_INV] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_PCLK_FREQ] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_PCLK_INV] = { .type = NLA_U64 },
	[DISPLAY_CONFIG_A_INVERTER_NAME] = { .type = NLA_NUL_STRING },
};

/* generic netlink family definition */
static struct genl_family display_config_gnl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = "DISPLAY_CONFIG",
	.version = 1,
	.maxattr = DISPLAY_CONFIG_A_MAX,
};

struct panel_config {
	struct omap_video_timings timings;

	int acbi;       /* ac-bias pin transitions per interrupt */
	/* Unit: line clocks */
	int acb;        /* ac-bias pin frequency */

	enum omap_panel_config config;

	int power_on_delay;
	int power_off_delay;

	/*
	 * Used to match device to panel configuration
	 * when use generic panel driver
	 */
	const char *name;
};

/* Default panel configuration */
//TODO: use the most conservative configuration
static struct panel_config generic_netlink_panel = {
	{
		.x_res = 800,
		.y_res = 480,

		.pixel_clock	= 32000,

		.hsw		= 40,
		.hfp		= 48,
		.hbp		= 40,

		.vsw		= 16,
		.vfp		= 10,
		.vbp		= 26,
	},
	.acbi                   = 0x0,
	.acb                    = 0x0,
	.config                 = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
								OMAP_DSS_LCD_IHS,
	.power_on_delay         = 0,
	.power_off_delay        = 0,
	.name                   = "default",
};

static int generic_netlink_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
//	msleep(50); //STE Avoid delay for those displays that need a different enable sequence

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void generic_netlink_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int generic_netlink_panel_probe(struct omap_dss_device *dssdev)
{
    dssdev->panel.timings = generic_netlink_panel.timings;
	dssdev->panel.acb = generic_netlink_panel.acb;
	dssdev->panel.acbi = generic_netlink_panel.acbi;
	dssdev->panel.config = generic_netlink_panel.config;

	return 0;
}

static void generic_netlink_panel_remove(struct omap_dss_device *dssdev)
{
}

static int generic_netlink_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = generic_netlink_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void generic_netlink_panel_disable(struct omap_dss_device *dssdev)
{
	generic_netlink_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int generic_netlink_panel_suspend(struct omap_dss_device *dssdev)
{
	generic_netlink_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int generic_netlink_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = generic_netlink_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void generic_netlink_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void generic_netlink_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int generic_netlink_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver generic_netlink_panel_driver = {
	.probe		= generic_netlink_panel_probe,
	.remove		= generic_netlink_panel_remove,

	.enable		= generic_netlink_panel_enable,
	.disable	= generic_netlink_panel_disable,
	.suspend	= generic_netlink_panel_suspend,
	.resume		= generic_netlink_panel_resume,

	.set_timings	= generic_netlink_panel_set_timings,
	.get_timings	= generic_netlink_panel_get_timings,
	.check_timings	= generic_netlink_panel_check_timings,

	.driver         = {
		.name   = "generic_netlink_panel",
		.owner  = THIS_MODULE,
	},
};

 /* handler */
 static int display_config(struct sk_buff *skb, struct genl_info *info)
 {
       /* message handling code goes here; return 0 on success, negative
        * values on failure */
		if(skb == NULL) {
			printk(KERN_ERR "skb is NULL \n");
			return -1;
		}

		printk(KERN_DEBUG "Received generic netlink message brightness_min: %d\n", nla_get_u32(info->attrs[DISPLAY_CONFIG_A_BRIGHTNESS_MIN]));
		printk(KERN_DEBUG "Received generic netlink message brightness_max: %d\n", nla_get_u32(info->attrs[DISPLAY_CONFIG_A_BRIGHTNESS_MAX]));
		printk(KERN_DEBUG "Received generic netlink message pwmfreq: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_PWMFREQ]));
		printk(KERN_DEBUG "Received generic netlink message rezx: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_REZX]));
		printk(KERN_DEBUG "Received generic netlink message rezy: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_REZY]));
		printk(KERN_DEBUG "Received generic netlink message bpp: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_BPP]));
		printk(KERN_DEBUG "Received generic netlink message hs_fp: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_FP]));
		printk(KERN_DEBUG "Received generic netlink message hs_bp: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_BP]));
		printk(KERN_DEBUG "Received generic netlink message hs_w: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_W]));
		printk(KERN_DEBUG "Received generic netlink message hs_inv: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_INV])); //IHS
		printk(KERN_DEBUG "Received generic netlink message vs_fp: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_FP]));
		printk(KERN_DEBUG "Received generic netlink message vs_bp: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_BP]));
		printk(KERN_DEBUG "Received generic netlink message vs_w: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_W]));
		printk(KERN_DEBUG "Received generic netlink message vs_inv: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_INV])); //IVS
		printk(KERN_DEBUG "Received generic netlink message blank_inv: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_BLANK_INV])); //IEO
		printk(KERN_DEBUG "Received generic netlink message pclk_freq: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_PCLK_FREQ]));
		printk(KERN_DEBUG "Received generic netlink message pclk_inv: %ld\n", (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_PCLK_INV])); //IPC
		printk(KERN_DEBUG "Received generic netlink message inverter_name: %s\n", (char *)nla_data(info->attrs[DISPLAY_CONFIG_A_INVERTER_NAME]));


		//ONOFF sempre a 1
		generic_netlink_panel.timings.x_res = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_REZX]);
		generic_netlink_panel.timings.y_res = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_REZY]);

		generic_netlink_panel.timings.pixel_clock = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_PCLK_FREQ])*1000;

		generic_netlink_panel.timings.hsw = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_W]);
		generic_netlink_panel.timings.hfp = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_FP]);
		generic_netlink_panel.timings.hbp = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_HS_BP]);

		generic_netlink_panel.timings.vsw = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_W]);
		generic_netlink_panel.timings.vfp = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_FP]);
		generic_netlink_panel.timings.vbp = (long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_BP]);

		generic_netlink_panel.config = OMAP_DSS_LCD_TFT;
		if ((long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_VS_INV]))
			generic_netlink_panel.config |= OMAP_DSS_LCD_IVS;
		if ((long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_BLANK_INV]))
			generic_netlink_panel.config |= OMAP_DSS_LCD_IHS;
		if ((long)nla_get_u64(info->attrs[DISPLAY_CONFIG_A_PCLK_INV]))
			generic_netlink_panel.config |= OMAP_DSS_LCD_IPC;

		generic_netlink_panel.name = (char *)nla_data(info->attrs[DISPLAY_CONFIG_A_INVERTER_NAME]);

		return omap_dss_register_driver(&generic_netlink_panel_driver);
 }
 /* commands */
 enum {
	   DISPLAY_CONFIG_C_UNSPEC,
	   DISPLAY_CONFIG_C_SET,
       __DISPLAY_CONFIG_C_MAX,
 };
 #define DISPLAY_CONFIG_C_MAX (__DISPLAY_CONFIG_C_MAX - 1)
 /* operation definition */
 static struct genl_ops display_config_gnl_ops_set = {
       .cmd = DISPLAY_CONFIG_C_SET,
       .flags = 0,
       .policy = display_config_genl_policy,
       .doit = display_config,
       .dumpit = NULL,
 };
//End STE

static int __init generic_netlink_panel_drv_init(void)
{
	int rf;
	int ro;

#ifndef CONFIG_FB_OMAP2_MODULE
	printk(KERN_INFO "Built-in driver: not initializing generic netlink socket");
	return omap_dss_register_driver(&generic_netlink_panel_driver);
#else
	printk(KERN_INFO "Initializing generic netlink socket");

	rf = genl_register_family(&display_config_gnl_family);
	if (rf == 0)
		ro = genl_register_ops(&display_config_gnl_family, &display_config_gnl_ops_set);

	// use default values when the initialization of the generic netlink fails
	if (rf != 0 || ro != 0)
		return omap_dss_register_driver(&generic_netlink_panel_driver);
#endif
	//TODO: check if it is a correct return value
	return 0;
}

static void __exit generic_netlink_panel_drv_exit(void)
{
	printk(KERN_INFO "Goodbye");//STE
	omap_dss_unregister_driver(&generic_netlink_panel_driver);
}

module_init(generic_netlink_panel_drv_init);
module_exit(generic_netlink_panel_drv_exit);
MODULE_LICENSE("GPL");

