/*
 * linux/arch/arm/mach-omap2/board-am3517evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 * Author: Ranjith Lohithakshan <ranjithl@ti.com>
 *
 * Based on mach-omap2/board-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
//#include <linux/i2c/pca953x.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/davinci_emac.h>
#include <linux/irq.h>
#include <linux/i2c/tsc2004.h>
#include <linux/input.h>
//#include <linux/tca6416_keypad.h>
#include <linux/mmc/host.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
//#include <linux/mtd/nand.h>
//#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
//#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
//#include <plat/nand.h>

//#include <media/ti-media/vpfe_capture.h>
//#include <media/tvp514x.h>

#include <plat/omap-serial.h>
#include <plat/mcspi.h>//STE
#include <linux/spi/spi.h>//STE
#include <linux/spi/eeprom.h>//STE
//#include "mmc-am3517evm.h"

#include <linux/leds.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"

#define UN31_GPIO_LED_GREEN 158
#define UN31_GPIO_LED_RED 161
#define AM35XX_EVM_MDIO_FREQUENCY	(1000000)

//#define GPMC_CS0_BASE  0x60 //STE
//#define GPMC_CS_SIZE   0x30 //STE

static struct gpio_led gpio_leds[] = {
	{
		.name	= "led:green",
		.gpio	= UN31_GPIO_LED_GREEN,
	},
	{
		.name	= "led:red",
		.gpio	= UN31_GPIO_LED_RED,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct mdio_platform_data am3517_evm_mdio_pdata = {
	.bus_freq	= AM35XX_EVM_MDIO_FREQUENCY,
};

static struct resource am3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device am3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_mdio_resources),
	.resource	= am3517_mdio_resources,
	.dev.platform_data = &am3517_evm_mdio_pdata,
};

//#define AM35XX_EVM_PHY_MASK		(0xF)

static struct emac_platform_data am3517_evm_emac_pdata = {
//	.phy_mask       = AM35XX_EVM_PHY_MASK,
//	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
	.phy_id		= "ffffffff:03",
};

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
//		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR | AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR );
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_evm_ethernet_init(struct emac_platform_data *pdata)
{
	u32 regval, mac_lo, mac_hi;

	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
	pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
	pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
	pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= am3517_enable_ethernet_int;
	pdata->interrupt_disable	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&am3517_mdio_device);
	platform_device_register(&am3517_emac_device);
	platform_device_register(&leds_gpio);
	clk_add_alias(NULL, dev_name(&am3517_mdio_device.dev),
		      NULL, &am3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}



#define LCD_PANEL_PWR		6 //176 //STE
#define LCD_PANEL_BKLIGHT_PWR	7 //182 //STE
#define LCD_PANEL_PWM		55 //181 //STE

/*
 * RTC - S35390A
 */
//#define	GPIO_RTCS35390A_IRQ	55

/*
 * TSC 2004 Support
 */
//#define	GPIO_TSC2004_IRQ	65
#define	GPIO_TSC2004_IRQ	0 //STE

static int tsc2004_init_irq(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_TSC2004_IRQ, "tsc2004-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				GPIO_TSC2004_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC2004_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_TSC2004_IRQ);
		return -ENXIO;
	}

	gpio_set_debounce(GPIO_TSC2004_IRQ, 0xa);
	return ret;
}

static void tsc2004_exit_irq(void)
{
	gpio_free(GPIO_TSC2004_IRQ);
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}

struct tsc2004_platform_data am3517evm_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
};


static int lcd_enabled;

static void __init am3517_evm_display_init(void)
{
#if defined(CONFIG_PANEL_GENERIC_NETLINK) || \
		defined(CONFIG_PANEL_GENERIC_NETLINK_MODULE)
	int r;

	//omap_mux_init_gpio(LCD_PANEL_PWR, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(LCD_PANEL_PWR, OMAP_PIN_INPUT_PULLDOWN);//STE
	omap_mux_init_gpio(LCD_PANEL_BKLIGHT_PWR, OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_gpio(LCD_PANEL_PWM, OMAP_PIN_INPUT_PULLDOWN);
	/*
	 * Enable GPIO 182 = LCD Backlight Power
	 */
	r = gpio_request(LCD_PANEL_BKLIGHT_PWR, "lcd_backlight_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_backlight_pwr\n");
		return;
	}
	gpio_direction_output(LCD_PANEL_BKLIGHT_PWR, 1);
	/*
	 * Enable GPIO 181 = LCD Panel PWM
	 */
	r = gpio_request(LCD_PANEL_PWM, "lcd_pwm");
	if (r) {
		printk(KERN_ERR "failed to get lcd_pwm\n");
		goto err_1;
	}
	gpio_direction_output(LCD_PANEL_PWM, 1);
	/*
	 * Enable GPIO 176 = LCD Panel Power enable pin
	 */
	r = gpio_request(LCD_PANEL_PWR, "lcd_panel_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_pwr\n");
		goto err_2;
	}
	gpio_direction_output(LCD_PANEL_PWR, 0);

	//disable panel and backlight power
	gpio_set_value(LCD_PANEL_BKLIGHT_PWR, 0);//STE
	gpio_set_value(LCD_PANEL_PWR, 0);//STE

	printk(KERN_INFO "Display initialized successfully\n");
	return;

err_2:
	gpio_free(LCD_PANEL_PWM);
err_1:
	gpio_free(LCD_PANEL_BKLIGHT_PWR);
#else
	printk(KERN_INFO "Display option not selected\n");
#endif
}

static int am3517_evm_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	//Enable the panel
	gpio_set_value(LCD_PANEL_PWR, 1);//STE
	msleep(200);
	//Enable the backlight
	gpio_set_value(LCD_PANEL_BKLIGHT_PWR, 1);//STE
	lcd_enabled = 1;

	return 0;
}

static void am3517_evm_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	//Disable the backlight
	gpio_set_value(LCD_PANEL_BKLIGHT_PWR, 0);//STE
	//Disable the panel
	gpio_set_value(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device am3517_evm_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_netlink_panel",
	.phy.dpi.data_lines 	= 16,
	.platform_enable	= am3517_evm_panel_enable_lcd,
	.platform_disable	= am3517_evm_panel_disable_lcd,
};

static struct omap_dss_device *am3517_evm_dss_devices[] = {
	&am3517_evm_lcd_device,
};

static struct omap_dss_board_info am3517_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_evm_dss_devices),
	.devices	= am3517_evm_dss_devices,
	.default_device	= &am3517_evm_lcd_device,
};

static struct platform_device am3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_evm_dss_data,
	},
};

/* TPS65023 specific initialization */
/* VDCDC1 -> VDD_CORE */
/*static struct regulator_consumer_supply am3517_evm_vdcdc1_supplies[] = {*/
/*	{*/
/*		.supply = "vdd_core",*/
/*	},*/
/*};*/

/* VDCDC2 -> VDDSHV */
/*static struct regulator_consumer_supply am3517_evm_vdcdc2_supplies[] = {*/
/*	{*/
/*		.supply = "vddshv",*/
/*	},*/
/*};*/

/* VDCDC2 |-> VDDS
	   |-> VDDS_SRAM_CORE_BG
	   |-> VDDS_SRAM_MPU */
/*static struct regulator_consumer_supply am3517_evm_vdcdc3_supplies[] = {*/
/*	{*/
/*		.supply = "vdds",*/
/*	},*/
/*	{*/
/*		.supply = "vdds_sram_core_bg",*/
/*	},*/
/*	{*/
/*		.supply = "vdds_sram_mpu",*/
/*	},*/
/*};*/

/* LDO1 |-> VDDA1P8V_USBPHY
	 |-> VDDA_DAC */
/*static struct regulator_consumer_supply am3517_evm_ldo1_supplies[] = {*/
/*	{*/
/*		.supply = "vdda1p8v_usbphy",*/
/*	},*/
/*	{*/
/*		.supply = "vdda_dac",*/
/*	},*/
/*};*/

/* LDO2 -> VDDA3P3V_USBPHY */
/*static struct regulator_consumer_supply am3517_evm_ldo2_supplies[] = {*/
/*	{*/
/*		.supply = "vdda3p3v_usbphy",*/
/*	},*/
/*};*/

/*static struct regulator_init_data am3517_evm_regulator_data[] = {*/
	/* DCDC1 */
/*	{*/
/*		.constraints = {*/
/*			.min_uV = 1200000,*/
/*			.max_uV = 1200000,*/
/*			.valid_modes_mask = REGULATOR_MODE_NORMAL,*/
/*			.valid_ops_mask = REGULATOR_CHANGE_STATUS,*/
/*			.always_on = true,*/
/*			.apply_uV = false,*/
/*		},*/
/*		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc1_supplies),*/
/*		.consumer_supplies = am3517_evm_vdcdc1_supplies,*/
/*	},*/
	/* DCDC2 */
/*	{*/
/*		.constraints = {*/
/*			.min_uV = 3300000,*/
/*			.max_uV = 3300000,*/
/*			.valid_modes_mask = REGULATOR_MODE_NORMAL,*/
/*			.valid_ops_mask = REGULATOR_CHANGE_STATUS,*/
/*			.always_on = true,*/
/*			.apply_uV = false,*/
/*		},*/
/*		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc2_supplies),*/
/*		.consumer_supplies = am3517_evm_vdcdc2_supplies,*/
/*	},*/
	/* DCDC3 */
/*	{*/
/*		.constraints = {*/
/*			.min_uV = 1800000,*/
/*			.max_uV = 1800000,*/
/*			.valid_modes_mask = REGULATOR_MODE_NORMAL,*/
/*			.valid_ops_mask = REGULATOR_CHANGE_STATUS,*/
/*			.always_on = true,*/
/*			.apply_uV = false,*/
/*		},*/
/*		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc3_supplies),*/
/*		.consumer_supplies = am3517_evm_vdcdc3_supplies,*/
/*	},*/
	/* LDO1 */
/*	{*/
/*		.constraints = {*/
/*			.min_uV = 1800000,*/
/*			.max_uV = 1800000,*/
/*			.valid_modes_mask = REGULATOR_MODE_NORMAL,*/
/*			.valid_ops_mask = REGULATOR_CHANGE_STATUS,*/
/*			.always_on = false,*/
/*			.apply_uV = false,*/
/*		},*/
/*		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo1_supplies),*/
/*		.consumer_supplies = am3517_evm_ldo1_supplies,*/
/*	},*/
	/* LDO2 */
/*	{*/
/*		.constraints = {*/
/*			.min_uV = 3300000,*/
/*			.max_uV = 3300000,*/
/*			.valid_modes_mask = REGULATOR_MODE_NORMAL,*/
/*			.valid_ops_mask = REGULATOR_CHANGE_STATUS,*/
/*			.always_on = false,*/
/*			.apply_uV = false,*/
/*		},*/
/*		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo2_supplies),*/
/*		.consumer_supplies = am3517_evm_ldo2_supplies,*/
/*	},*/
/*};*/

//STE
static struct i2c_board_info __initdata am3517evm_i2c1_boardinfo[] = {
/*	{
		I2C_BOARD_INFO("tsc2004", 0x4B),
		.type		= "tsc2004",
		.platform_data	= &am3517evm_tsc2004data,
	},
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.type		= "s35390a",
	},*/
/*	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &am3517_evm_regulator_data[0],
	},*/
};

static struct i2c_board_info __initdata am3517evm_i2c2_boardinfo[] = {
	/*{
		I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},*/
	{//STE
		I2C_BOARD_INFO("tsc2004", 0x4B),
		.type		= "tsc2004",
		.platform_data	= &am3517evm_tsc2004data,
	},//STE
	{
		I2C_BOARD_INFO("m41t80", 0x68),
		.type		= "m41t80",
	},
};

static int __init am3517_evm_i2c_init(void)
{
//	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 100, NULL, 0);
//	omap_register_i2c_bus(3, 400, am3517evm_ui_tca6516_info,
//			ARRAY_SIZE(am3517evm_ui_tca6516_info)); //STE PROD

	return 0;
}

//STE
static struct omap2_mcspi_device_config am35x_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_eeprom fm25v05  = {
	.byte_len	= 65536,
	.name		= "fm25v05",
	.page_size	= 65535,
	.flags		= EE_ADDR2,
};

static struct spi_board_info spidev_board_info[] = {
    {
        .modalias    = "at25",
        .controller_data	= &am35x_mcspi_config,
		.platform_data	= &fm25v05,
        .max_speed_hz    = 12000000, //10 Mbps
        .bus_num    = 1,
        .chip_select    = 0,
        .mode = SPI_MODE_0,
    },
};
//STE

/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_evm_config[] __initdata = {
};

static struct platform_device *am3517_evm_devices[] __initdata = {
	&am3517_evm_dss_device,
};

static void __init am3517_evm_init_irq(void)
{
	omap_board_config = am3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(am3517_evm_config);
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
};

static __init void am3517_evm_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
#else
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
#endif
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 57,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};


/* NOR flash information */
//STE
/*La NOR è organizzata come segue:
- Base address 0x8000000, settori da 128KBytes, size = 128MBytes, mappatura lineare

NORFLASH memory map
===================
Sector0				offset 0x00000		Xloader
Sector1-2			offset 0x20000		U-boot image
Sector3				offset 0x60000		U-boot environment
Sector4				offset 0x80000		RFU (will be used for X-FIS support)
Sector5...7			offset 0xa0000		Splash image
Sector8...47		offset 0x100000		ConfigOs image
Sector48...359		offset 0x600000		WCE6 image (actually native WCE6 .bin image is handled; about 40MB size is assumed)
Sector360...1023	offset 0x2d00000    Filesystem (handled by WCE6)
End					offset 0x8000000

SPLASH 		  384KB 3 sectors
CONFIGOS 	 5120KB offset 0x100000 size 4786614 40 sectors
MAINOS		39936KB offset 0x600000 size 12598384 312 sectors
Filesystem 664 sectors
Tot 1024 sectors
*/

static struct mtd_partition am3517_evm_norflash_partitions[] = {
	/* Xloader in first sector */
	{
		.name           = "xloader",
		.offset         = 0,
		.size           = SZ_128K,
		.mask_flags     = MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader (U-Boot, etc) in the next 2 sectors */
	{
		.name           = "bootloader",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * SZ_128K,
		.mask_flags     = MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next 1 sectors */
	{
		.name           = "env",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_128K,
		.mask_flags     = 0,
	},
	/* X-FIS in the next 1 sectors */
	{
		.name           = "xfis",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_128K,
		.mask_flags     = 0
	},
	/* Splash image in the next 6 sectors */
	{
		.name           = "splashimage",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 3 * SZ_128K,
		.mask_flags     = 0
	},
	/* ConfigOs */
	{
		.name           = "configOs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * SZ_128K,
		.mask_flags     = 0
	},
	/* Main Is */
	{
		.name           = "mainOs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 312 * SZ_128K,
		.mask_flags     = 0
	},
	/* File System */
	{
			.name           = "filesystem",
			.offset         = MTDPART_OFS_APPEND,
			.size           = MTDPART_SIZ_FULL,
			.mask_flags     = 0
	}
};

static struct physmap_flash_data am3517_evm_norflash_data = {
	.width          = 2,
	.parts          = am3517_evm_norflash_partitions,
	.nr_parts       = ARRAY_SIZE(am3517_evm_norflash_partitions),
};

#define AM3517_EVM_NOR_BASE             0x08000000
static struct resource am3517_evm_norflash_resource = {
	.start          = AM3517_EVM_NOR_BASE,
	.end            = AM3517_EVM_NOR_BASE + SZ_128M - 1,
	.flags          = IORESOURCE_MEM,
};

static struct platform_device am3517_evm_norflash_device = {
	.name           = "physmap-flash",
	.id             = 0,
	.dev            = {
		.platform_data  = &am3517_evm_norflash_data,
	},
	.num_resources  = 1,
	.resource       = &am3517_evm_norflash_resource,
};

static void __init am3517_nor_init(void)
{
	int cs;
	int norcs;

	/* find out the chip-select on which NOR exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;

		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);
		if ((ret & 0xC00) == 0x0) {
			printk(KERN_INFO "Found NOR on CS%d\n", cs);
			norcs = cs;
			break;
		}
		cs++;
	}

	if (norcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NOR: Unable to find configuration in GPMC\n");
		return;
	}

	if (norcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NOR on CS%d\n", norcs);
		if (platform_device_register(&am3517_evm_norflash_device) < 0)
			printk(KERN_ERR "Unable to register NOR device\n");
	}
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	/*OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),*/
	OMAP3_MUX(MCBSP1_DX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	OMAP3_MUX(MCBSP1_FSX, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

/*
 * HECC information
 */

#define CAN_STB         214
static void hecc_phy_control(int on)
{
        int r;

        r = gpio_request(CAN_STB, "can_stb");
        if (r) {
                printk(KERN_ERR "failed to get can_stb \n");
                return;
        }

        gpio_direction_output(CAN_STB, (on==1)?0:1);
}

static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_hecc_resources),
	.resource	= am3517_hecc_resources,
};

static struct ti_hecc_platform_data am3517_evm_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
	.transceiver_switch     = hecc_phy_control,
};

static void am3517_evm_hecc_init(struct ti_hecc_platform_data *pdata)
{
	am3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&am3517_hecc_device);
}

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.caps          = MMC_CAP_4_BIT_DATA,
		/*TODO: Need to change*/
		.gpio_cd        = 128, //STE 127
		.gpio_wp        = 129, //STE 126	
	},
	{}      /* Terminator */
};

static struct omap_uart_port_info omap_uart[]= {
	{
		.gpio_dxen = -EINVAL,
		.gpio_rxen = -EINVAL,
		.gpio_mode = -EINVAL,
	},
	{
		.gpio_dxen = -EINVAL,
		.gpio_rxen = -EINVAL,
		.gpio_mode = -EINVAL,
	},
	{
		.gpio_dxen = 184, //149,
		.gpio_rxen = 170, //148,
		.gpio_mode = 185, //63,
	},
	{}	   /* Terminator */
};

static void __init am3517_evm_init(void)
{
	/* SPI FRAM init */ //STE
	omap_mux_init_signal("mcspi1_clk", OMAP_MUX_MODE0 | OMAP_PIN_INPUT);//STE
	omap_mux_init_signal("mcspi1_somi", OMAP_MUX_MODE0 | OMAP_PIN_INPUT);//STE
	omap_mux_init_signal("mcspi1_simo", OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT);//STE
	omap_mux_init_signal("mcspi1_cs0", OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT);//STE
	omap_mux_init_signal("mcspi1_cs1", OMAP_MUX_MODE0 |OMAP_PIN_OUTPUT);//STE

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	am3517_evm_i2c_init();

	platform_add_devices(am3517_evm_devices,
				ARRAY_SIZE(am3517_evm_devices));

	spi_register_board_info(spidev_board_info, ARRAY_SIZE(spidev_board_info));//STE

	//omap_serial_init();//STE
	omap_serial_board_init(omap_uart);

	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);

	/* RTC - S35390A */
	/*omap_mux_init_gpio(55, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(GPIO_RTCS35390A_IRQ, "rtcs35390a-irq") < 0)
		printk(KERN_WARNING "failed to request GPIO#%d\n",
				GPIO_RTCS35390A_IRQ);
	if (gpio_direction_input(GPIO_RTCS35390A_IRQ))
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_RTCS35390A_IRQ);*/
	//am3517evm_i2c1_boardinfo[1].irq = gpio_to_irq(GPIO_RTCS35390A_IRQ); 
	//am3517evm_i2c1_boardinfo[0].irq = gpio_to_irq(GPIO_RTCS35390A_IRQ); //STE

	/* TSC 2004 */
	//omap_mux_init_gpio(65, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(GPIO_TSC2004_IRQ, OMAP_PIN_INPUT_PULLUP); //STE
	//am3517evm_i2c1_boardinfo[0].irq = gpio_to_irq(GPIO_TSC2004_IRQ); 
	am3517evm_i2c2_boardinfo[0].irq = gpio_to_irq(GPIO_TSC2004_IRQ); //STE

	//i2c_register_board_info(1, am3517evm_i2c1_boardinfo,
	//			ARRAY_SIZE(am3517evm_i2c1_boardinfo));
	i2c_register_board_info(2, am3517evm_i2c2_boardinfo, //STE
				ARRAY_SIZE(am3517evm_i2c2_boardinfo));

	/* DSS */
	am3517_evm_display_init();

	/*Ethernet*/
	am3517_evm_ethernet_init(&am3517_evm_emac_pdata); 
	am3517_evm_hecc_init(&am3517_evm_hecc_pdata);	

	/* MUSB */
	am3517_evm_musb_init();

	/* MMC init function */
	omap2_hsmmc_init(mmc);

	/* NOR Flash on Application board */
	am3517_nor_init(); 
}

MACHINE_START(OMAP3517UN31, "OMAP3517/AM3517 UN31")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= am3517_evm_init_irq,
	.init_machine	= am3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END
