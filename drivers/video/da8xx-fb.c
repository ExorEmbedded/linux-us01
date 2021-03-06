/*
 * Copyright (C) 2008-2009 MontaVista Software Inc.
 * Copyright (C) 2008-2009 Texas Instruments Inc
 *
 * Based on the LCD driver for TI Avalanche processors written by
 * Ajay Singh and Shalom Hai.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/console.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <video/of_display_timing.h>
#include <video/da8xx-fb.h>
#include <video/displayconfig.h>
#include <linux/init.h> 

#ifdef CONFIG_FB_DA8XX_TDA998X
#include <video/da8xx-tda998x-hdmi.h>
#endif

#include <asm/div64.h>
#include <linux/io.h>
#include <linux/gpio.h>


#define DRIVER_NAME "da8xx_lcdc"

#define LCD_VERSION_1	1
#define LCD_VERSION_2	2

/* LCD Status Register */
#define LCD_END_OF_FRAME1		BIT(9)
#define LCD_END_OF_FRAME0		BIT(8)
#define LCD_PL_LOAD_DONE		BIT(6)
#define LCD_FIFO_UNDERFLOW		BIT(5)
#define LCD_SYNC_LOST			BIT(2)
#define LCD_FRAME_DONE			BIT(0)

/* LCD DMA Control Register */
#define LCD_DMA_BURST_SIZE(x)		((x) << 4)
#define LCD_DMA_BURST_1			0x0
#define LCD_DMA_BURST_2			0x1
#define LCD_DMA_BURST_4			0x2
#define LCD_DMA_BURST_8			0x3
#define LCD_DMA_BURST_16		0x4
#define LCD_V1_END_OF_FRAME_INT_ENA	BIT(2)
#define LCD_V2_END_OF_FRAME0_INT_ENA	BIT(8)
#define LCD_V2_END_OF_FRAME1_INT_ENA	BIT(9)
#define LCD_DUAL_FRAME_BUFFER_ENABLE	BIT(0)

/* LCD Control Register */
#define LCD_CLK_DIVISOR(x)		((x) << 8)
#define LCD_RASTER_MODE			0x01

/* LCD Raster Control Register */
#define LCD_PALETTE_LOAD_MODE(x)	((x) << 20)
#define PALETTE_AND_DATA		0x00
#define PALETTE_ONLY			0x01
#define DATA_ONLY			0x02

#define LCD_MONO_8BIT_MODE		BIT(9)
#define LCD_RASTER_ORDER		BIT(8)
#define LCD_TFT_MODE			BIT(7)
#define LCD_V1_UNDERFLOW_INT_ENA	BIT(6)
#define LCD_V2_UNDERFLOW_INT_ENA	BIT(5)
#define LCD_V1_PL_INT_ENA		BIT(4)
#define LCD_V2_PL_INT_ENA		BIT(6)
#define LCD_MONOCHROME_MODE		BIT(1)
#define LCD_RASTER_ENABLE		BIT(0)
#define LCD_TFT_ALT_ENABLE		BIT(23)
#define LCD_STN_565_ENABLE		BIT(24)
#define LCD_V2_DMA_CLK_EN		BIT(2)
#define LCD_V2_LIDD_CLK_EN		BIT(1)
#define LCD_V2_CORE_CLK_EN		BIT(0)
#define LCD_V2_LPP_B10			26
#define LCD_V2_TFT_24BPP_MODE		BIT(25)
#define LCD_V2_TFT_24BPP_UNPACK		BIT(26)

/* LCD Raster Timing 2 Register */
#define LCD_AC_BIAS_TRANSITIONS_PER_INT(x)	((x) << 16)
#define LCD_AC_BIAS_FREQUENCY(x)		((x) << 8)
#define LCD_SYNC_CTRL				BIT(25)
#define LCD_SYNC_EDGE				BIT(24)
#define LCD_INVERT_PIXEL_CLOCK			BIT(22)
#define LCD_INVERT_LINE_CLOCK			BIT(21)
#define LCD_INVERT_FRAME_CLOCK			BIT(20)

/* LCD Block */
#define  LCD_PID_REG				0x0
#define  LCD_CTRL_REG				0x4
#define  LCD_STAT_REG				0x8
#define  LCD_RASTER_CTRL_REG			0x28
#define  LCD_RASTER_TIMING_0_REG		0x2C
#define  LCD_RASTER_TIMING_1_REG		0x30
#define  LCD_RASTER_TIMING_2_REG		0x34
#define  LCD_DMA_CTRL_REG			0x40
#define  LCD_DMA_FRM_BUF_BASE_ADDR_0_REG	0x44
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG	0x48
#define  LCD_DMA_FRM_BUF_BASE_ADDR_1_REG	0x4C
#define  LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG	0x50

/* Interrupt Registers available only in Version 2 */
#define  LCD_RAW_STAT_REG			0x58
#define  LCD_MASKED_STAT_REG			0x5c
#define  LCD_INT_ENABLE_SET_REG			0x60
#define  LCD_INT_ENABLE_CLR_REG			0x64
#define  LCD_END_OF_INT_IND_REG			0x68

/* Clock registers available only on Version 2 */
#define  LCD_CLK_ENABLE_REG			0x6c
#define  LCD_CLK_RESET_REG			0x70
#define  LCD_CLK_MAIN_RESET			BIT(3)

#define LCD_NUM_BUFFERS	2

#define WSI_TIMEOUT	50
#define PALETTE_SIZE	256

#define	CLK_MIN_DIV	2
#define	CLK_MAX_DIV	255

static void __iomem *da8xx_fb_reg_base;
static unsigned int lcd_revision;
static irq_handler_t lcdc_irq_handler;
static wait_queue_head_t frame_done_wq;
static int frame_done_flag;

/*----------------------------------------------------------------------------------------------------------------*
   Helper functions to retrieve the display id value, when passed from the cmdline, and use it to set the
   display parameters/timings (the contents of the DTB file are overridden, if a valid dispaly id is passed from
   cmdline.
 *----------------------------------------------------------------------------------------------------------------*/
extern int hw_dispid; //This is an exported variable holding the display id value, if passed from cmdline

/*
 * Writes the fb_videomode structure according with the contents of the displayconfig.h file and the passed dispid parameter.
 * Returns 0 if success, -1 if failure (ie: no match found)
 */
int dispid_get_fb_videomode(struct fb_videomode *fbmode, int dispid)
{
  int i=0;
  unsigned int htotal, vtotal;
  
  // Scan the display array to search for the required dispid
  if(dispid == NODISPLAY)
    return -1;
  
  while((displayconfig[i].dispid != NODISPLAY) && (displayconfig[i].dispid != dispid))
    i++;
  
  if(displayconfig[i].dispid == NODISPLAY)
    return -1;
  
  // If we are here, we have a valid array index pointing to the desired display
  fbmode->xres         = displayconfig[i].rezx;
  fbmode->left_margin  = displayconfig[i].hs_bp;     
  fbmode->right_margin = displayconfig[i].hs_fp;
  fbmode->hsync_len    = displayconfig[i].hs_w;
  
  fbmode->yres         = displayconfig[i].rezy;
  fbmode->upper_margin = displayconfig[i].vs_bp;
  fbmode->lower_margin = displayconfig[i].vs_fp;
  fbmode->vsync_len    = displayconfig[i].vs_w;

  /* prevent division by zero in KHZ2PICOS macro */
  fbmode->pixclock = displayconfig[i].pclk_freq ? KHZ2PICOS(displayconfig[i].pclk_freq) : 0;
  
  fbmode->sync = 0;
  fbmode->vmode = 0;
  
  if(displayconfig[i].hs_inv == 0)
    fbmode->sync |= FB_SYNC_HOR_HIGH_ACT;

  if(displayconfig[i].vs_inv == 0)
    fbmode->sync |= FB_SYNC_VERT_HIGH_ACT;
  
  if(displayconfig[i].pclk_inv == 1)
    fbmode->sync |= FB_SYNC_CLK_INVERT;
  
  fbmode->flag = 0;
  
  htotal = displayconfig[i].rezx + displayconfig[i].hs_bp + displayconfig[i].hs_fp + displayconfig[i].hs_w;
  vtotal = displayconfig[i].rezy + displayconfig[i].vs_bp + displayconfig[i].vs_fp + displayconfig[i].vs_w;

  /* prevent division by zero */
  if (htotal && vtotal) 
  {
    fbmode->refresh = displayconfig[i].pclk_freq / (htotal * vtotal);
    /* a mode must have htotal and vtotal != 0 or it is invalid */
  } 
  else 
  {
    fbmode->refresh = 0;
    return -EINVAL;
  }
  
  return 0;
}

/*----------------------------------------------------------------------------------------------------------------*
 *----------------------------------------------------------------------------------------------------------------*/

static LIST_HEAD(encoder_modules);

void da8xx_register_encoder(struct da8xx_encoder *encoder)
{
	INIT_LIST_HEAD(&encoder->list);
	list_add(&encoder->list, &encoder_modules);
}
EXPORT_SYMBOL(da8xx_register_encoder);

void da8xx_unregister_encoder(struct da8xx_encoder *encoder)
{
	list_del(&encoder->list);
}
EXPORT_SYMBOL(da8xx_unregister_encoder);


static struct da8xx_encoder
	*da8xx_get_encoder_from_phandle(struct device_node *node)
{
	struct da8xx_encoder *entry;
	list_for_each_entry(entry, &encoder_modules, list)
		if (entry->node == node)
			return entry;

	return NULL;
}

static unsigned int lcdc_read(unsigned int addr)
{
	return (unsigned int)__raw_readl(da8xx_fb_reg_base + (addr));
}

static void lcdc_write(unsigned int val, unsigned int addr)
{
	__raw_writel(val, da8xx_fb_reg_base + (addr));
}

struct da8xx_fb_par {
	struct device		*dev;
	resource_size_t p_palette_base;
	unsigned char *v_palette_base;
	dma_addr_t		vram_phys;
	unsigned long		vram_size;
	void			*vram_virt;
	unsigned int		dma_start;
	unsigned int		dma_end;
	struct clk *lcdc_clk;
	int irq;
	unsigned int palette_sz;
	int blank;
	wait_queue_head_t	vsync_wait;
	int			vsync_flag;
	int			vsync_timeout;
	spinlock_t		lock_for_chan_update;

	wait_queue_head_t	palette_wait;
	int			palette_loaded_flag;

	/*
	 * LCDC has 2 ping pong DMA channels, channel 0
	 * and channel 1.
	 */
	unsigned int		which_dma_channel_done;
#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif
	unsigned int		lcdc_clk_rate;
	void (*panel_power_ctrl)(int);
	u32 pseudo_palette[16];
	struct fb_videomode	mode;
	struct lcd_ctrl_config	cfg;
	struct device_node *hdmi_node;
};

static struct fb_var_screeninfo da8xx_fb_var;

static struct fb_fix_screeninfo da8xx_fb_fix = {
	.id = "DA8xx FB Drv",
	.type = FB_TYPE_PACKED_PIXELS,
	.type_aux = 0,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.xpanstep = 0,
	.ypanstep = 1,
	.ywrapstep = 0,
	.accel = FB_ACCEL_NONE
};

static vsync_callback_t vsync_cb_handler;
static void *vsync_cb_arg;

static struct fb_videomode known_lcd_panels[] = {
	/* Sharp LCD035Q3DG01 */
	[0] = {
		.name           = "Sharp_LCD035Q3DG01",
		.xres           = 320,
		.yres           = 240,
		.pixclock       = KHZ2PICOS(4607),
		.left_margin    = 6,
		.right_margin   = 8,
		.upper_margin   = 2,
		.lower_margin   = 2,
		.hsync_len      = 0,
		.vsync_len      = 0,
		.sync           = FB_SYNC_CLK_INVERT |
			FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	},
	/* Sharp LK043T1DG01 */
	[1] = {
		.name           = "Sharp_LK043T1DG01",
		.xres           = 480,
		.yres           = 272,
		.pixclock       = KHZ2PICOS(7833),
		.left_margin    = 2,
		.right_margin   = 2,
		.upper_margin   = 2,
		.lower_margin   = 2,
		.hsync_len      = 41,
		.vsync_len      = 10,
		.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.flag           = 0,
	},
	[2] = {
		/* Hitachi SP10Q010 */
		.name           = "SP10Q010",
		.xres           = 320,
		.yres           = 240,
		.pixclock       = KHZ2PICOS(7833),
		.left_margin    = 10,
		.right_margin   = 10,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.flag           = 0,
	},
};

static bool da8xx_fb_is_raster_enabled(void)
{
	return !!(lcdc_read(LCD_RASTER_CTRL_REG) & LCD_RASTER_ENABLE);
}

/* Enable the Raster Engine of the LCD Controller */
static void lcd_enable_raster(void)
{
	u32 reg;

	/* Put LCDC in reset for several cycles */
	if (lcd_revision == LCD_VERSION_2)
		/* Write 1 to reset LCDC */
		lcdc_write(LCD_CLK_MAIN_RESET, LCD_CLK_RESET_REG);
	mdelay(1);

	/* Bring LCDC out of reset */
	if (lcd_revision == LCD_VERSION_2)
		lcdc_write(0, LCD_CLK_RESET_REG);
	mdelay(1);

	/* Above reset sequence doesnot reset register context */
	reg = lcdc_read(LCD_RASTER_CTRL_REG);
	if (!(reg & LCD_RASTER_ENABLE))
		lcdc_write(reg | LCD_RASTER_ENABLE, LCD_RASTER_CTRL_REG);
}

/* Disable the Raster Engine of the LCD Controller */
static void lcd_disable_raster(enum da8xx_frame_complete wait_for_frame_done)
{
	u32 reg;
	int ret;

	reg = lcdc_read(LCD_RASTER_CTRL_REG);
	if (reg & LCD_RASTER_ENABLE)
		lcdc_write(reg & ~LCD_RASTER_ENABLE, LCD_RASTER_CTRL_REG);
	else
		/* return if already disabled */
		return;

	if ((wait_for_frame_done == DA8XX_FRAME_WAIT) &&
			(lcd_revision == LCD_VERSION_2)) {
		frame_done_flag = 0;
		ret = wait_event_interruptible_timeout(frame_done_wq,
				frame_done_flag != 0,
				msecs_to_jiffies(50));
		if (ret == 0)
			pr_err("LCD Controller timed out\n");
	}
}

static void lcd_blit(int load_mode, struct da8xx_fb_par *par)
{
	u32 start;
	u32 end;
	u32 reg_ras;
	u32 reg_dma;
	u32 reg_int;

	/* init reg to clear PLM (loading mode) fields */
	reg_ras = lcdc_read(LCD_RASTER_CTRL_REG);
	reg_ras &= ~(3 << 20);

	reg_dma  = lcdc_read(LCD_DMA_CTRL_REG);

	if (load_mode == LOAD_DATA) {
		start    = par->dma_start;
		end      = par->dma_end;

		reg_ras |= LCD_PALETTE_LOAD_MODE(DATA_ONLY);
		if (lcd_revision == LCD_VERSION_1) {
			reg_dma |= LCD_V1_END_OF_FRAME_INT_ENA;
		} else {
			reg_int = lcdc_read(LCD_INT_ENABLE_SET_REG) |
				LCD_V2_END_OF_FRAME0_INT_ENA |
				LCD_V2_END_OF_FRAME1_INT_ENA |
				LCD_FRAME_DONE | LCD_SYNC_LOST;
			lcdc_write(reg_int, LCD_INT_ENABLE_SET_REG);
		}
		reg_dma |= LCD_DUAL_FRAME_BUFFER_ENABLE;

		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	} else if (load_mode == LOAD_PALETTE) {
		start    = par->p_palette_base;
		end      = start + par->palette_sz - 1;

		reg_ras |= LCD_PALETTE_LOAD_MODE(PALETTE_ONLY);

		if (lcd_revision == LCD_VERSION_1) {
			reg_ras |= LCD_V1_PL_INT_ENA;
		} else {
			reg_int = lcdc_read(LCD_INT_ENABLE_SET_REG) |
				LCD_V2_PL_INT_ENA;
			lcdc_write(reg_int, LCD_INT_ENABLE_SET_REG);
		}

		lcdc_write(start, LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
		lcdc_write(end, LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	}

	lcdc_write(reg_dma, LCD_DMA_CTRL_REG);
	lcdc_write(reg_ras, LCD_RASTER_CTRL_REG);

	/*
	 * The Raster enable bit must be set after all other control fields are
	 * set.
	 */
	lcd_enable_raster();
}

static void lcd_load_palette(struct da8xx_fb_par *par)
{
	int r;

	par->palette_loaded_flag = 0;

	lcd_blit(LOAD_PALETTE, par);

	r = wait_event_interruptible_timeout(par->palette_wait,
					       par->palette_loaded_flag != 0,
					       msecs_to_jiffies(50));

	if (r == 0)
		pr_err("LCDC timeout when loading palette\n");

	lcd_blit(LOAD_DATA, par);
}

/* Configure the Burst Size and fifo threhold of DMA */
static int lcd_cfg_dma(int burst_size, int fifo_th)
{
	u32 reg;

	reg = lcdc_read(LCD_DMA_CTRL_REG) & 0x00000001;
	switch (burst_size) {
	case 1:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_1);
		break;
	case 2:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_2);
		break;
	case 4:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_4);
		break;
	case 8:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_8);
		break;
	case 16:
	default:
		reg |= LCD_DMA_BURST_SIZE(LCD_DMA_BURST_16);
		break;
	}

	reg |= (fifo_th << 8);

	lcdc_write(reg, LCD_DMA_CTRL_REG);

	return 0;
}

static void lcd_cfg_ac_bias(int period, int transitions_per_int)
{
	u32 reg;

	/* Set the AC Bias Period and Number of Transisitons per Interrupt */
	reg = lcdc_read(LCD_RASTER_TIMING_2_REG) & 0xFFF00000;
	reg |= LCD_AC_BIAS_FREQUENCY(period) |
		LCD_AC_BIAS_TRANSITIONS_PER_INT(transitions_per_int);
	lcdc_write(reg, LCD_RASTER_TIMING_2_REG);
}

static void lcd_cfg_horizontal_sync(int back_porch, int pulse_width,
		int front_porch)
{
	u32 reg;

	reg = lcdc_read(LCD_RASTER_TIMING_0_REG) & 0xf;
	reg |= (((back_porch-1) & 0xff) << 24)
	    | (((front_porch-1) & 0xff) << 16)
	    | (((pulse_width-1) & 0x3f) << 10);
	lcdc_write(reg, LCD_RASTER_TIMING_0_REG);

	/*
	 * LCDC Version 2 adds some extra bits that increase the allowable
	 * size of the horizontal timing registers.
	 * remember that the registers use 0 to represent 1 so all values
	 * that get set into register need to be decremented by 1
	 */
	if (lcd_revision == LCD_VERSION_2) {
		/* Mask off the bits we want to change */
		reg = lcdc_read(LCD_RASTER_TIMING_2_REG) & ~0x780000ff;
		reg |= ((front_porch-1) & 0x300) >> 8;
		reg |= ((back_porch-1) & 0x300) >> 4;
		reg |= ((pulse_width-1) & 0x3c0) << 21;
		lcdc_write(reg, LCD_RASTER_TIMING_2_REG);
	}
}

static void lcd_cfg_vertical_sync(int back_porch, int pulse_width,
		int front_porch)
{
	u32 reg;

	reg = lcdc_read(LCD_RASTER_TIMING_1_REG) & 0x3ff;
	reg |= ((back_porch & 0xff) << 24)
	    | ((front_porch & 0xff) << 16)
	    | (((pulse_width-1) & 0x3f) << 10);
	lcdc_write(reg, LCD_RASTER_TIMING_1_REG);
}

static int lcd_cfg_display(const struct lcd_ctrl_config *cfg,
		struct fb_videomode *panel)
{
	u32 reg;
	u32 reg_int;

	reg = lcdc_read(LCD_RASTER_CTRL_REG) & ~(LCD_TFT_MODE |
						LCD_MONO_8BIT_MODE |
						LCD_MONOCHROME_MODE);

	switch (cfg->panel_shade) {
	case MONOCHROME:
		reg |= LCD_MONOCHROME_MODE;
		if (cfg->mono_8bit_mode)
			reg |= LCD_MONO_8BIT_MODE;
		break;
	case COLOR_ACTIVE:
		reg |= LCD_TFT_MODE;
		if (cfg->tft_alt_mode)
			reg |= LCD_TFT_ALT_ENABLE;
		break;

	case COLOR_PASSIVE:
		/* AC bias applicable only for Pasive panels */
		lcd_cfg_ac_bias(cfg->ac_bias, cfg->ac_bias_intrpt);
		if (cfg->bpp == 12 && cfg->stn_565_mode)
			reg |= LCD_STN_565_ENABLE;
		break;

	default:
		return -EINVAL;
	}

	/* enable additional interrupts here */
	if (lcd_revision == LCD_VERSION_1) {
		reg |= LCD_V1_UNDERFLOW_INT_ENA;
	} else {
		reg_int = lcdc_read(LCD_INT_ENABLE_SET_REG) |
			LCD_V2_UNDERFLOW_INT_ENA;
		lcdc_write(reg_int, LCD_INT_ENABLE_SET_REG);
	}

	lcdc_write(reg, LCD_RASTER_CTRL_REG);

	reg = lcdc_read(LCD_RASTER_TIMING_2_REG);

	reg |= LCD_SYNC_CTRL;

	if (cfg->sync_edge)
		reg |= LCD_SYNC_EDGE;
	else
		reg &= ~LCD_SYNC_EDGE;

	if ((panel->sync & FB_SYNC_HOR_HIGH_ACT) == 0)
		reg |= LCD_INVERT_LINE_CLOCK;
	else
		reg &= ~LCD_INVERT_LINE_CLOCK;

	if ((panel->sync & FB_SYNC_VERT_HIGH_ACT) == 0)
		reg |= LCD_INVERT_FRAME_CLOCK;
	else
		reg &= ~LCD_INVERT_FRAME_CLOCK;

	lcdc_write(reg, LCD_RASTER_TIMING_2_REG);

	return 0;
}

static int lcd_cfg_frame_buffer(struct da8xx_fb_par *par, u32 width, u32 height,
		u32 bpp, u32 raster_order)
{
	u32 reg;

	if (bpp > 16 && lcd_revision == LCD_VERSION_1)
		return -EINVAL;

	/* Set the Panel Width */
	/* Pixels per line = (PPL + 1)*16 */
	if (lcd_revision == LCD_VERSION_1) {
		/*
		 * 0x3F in bits 4..9 gives max horizontal resolution = 1024
		 * pixels.
		 */
		width &= 0x3f0;
	} else {
		/*
		 * 0x7F in bits 4..10 gives max horizontal resolution = 2048
		 * pixels.
		 */
		width &= 0x7f0;
	}

	reg = lcdc_read(LCD_RASTER_TIMING_0_REG);
	reg &= 0xfffffc00;
	if (lcd_revision == LCD_VERSION_1) {
		reg |= ((width >> 4) - 1) << 4;
	} else {
		width = (width >> 4) - 1;
		reg |= ((width & 0x3f) << 4) | ((width & 0x40) >> 3);
	}
	lcdc_write(reg, LCD_RASTER_TIMING_0_REG);

	/* Set the Panel Height */
	/* Set bits 9:0 of Lines Per Pixel */
	reg = lcdc_read(LCD_RASTER_TIMING_1_REG);
	reg = ((height - 1) & 0x3ff) | (reg & 0xfffffc00);
	lcdc_write(reg, LCD_RASTER_TIMING_1_REG);

	/* Set bit 10 of Lines Per Pixel */
	if (lcd_revision == LCD_VERSION_2) {
		reg = lcdc_read(LCD_RASTER_TIMING_2_REG);
		reg |= ((height - 1) & 0x400) << 16;
		lcdc_write(reg, LCD_RASTER_TIMING_2_REG);
	}

	/* Set the Raster Order of the Frame Buffer */
	reg = lcdc_read(LCD_RASTER_CTRL_REG) & ~(1 << 8);
	if (raster_order)
		reg |= LCD_RASTER_ORDER;

	par->palette_sz = 16 * 2;

	if (lcd_revision == LCD_VERSION_2) {
		reg &= ~LCD_V2_TFT_24BPP_MODE;
		reg &= ~LCD_V2_TFT_24BPP_UNPACK;
	}

	switch (bpp) {
	case 1:
	case 2:
	case 4:
	case 16:
		break;
	case 24:
		reg |= LCD_V2_TFT_24BPP_MODE;
		break;
	case 32:
		reg |= LCD_V2_TFT_24BPP_MODE;
		reg |= LCD_V2_TFT_24BPP_UNPACK;
		break;
	case 8:
		par->palette_sz = 256 * 2;
		break;

	default:
		return -EINVAL;
	}

	lcdc_write(reg, LCD_RASTER_CTRL_REG);

	return 0;
}

#define CNVT_TOHW(val, width) ((((val) << (width)) + 0x7FFF - (val)) >> 16)
static int fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			      unsigned blue, unsigned transp,
			      struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	unsigned short *palette = (unsigned short *) par->v_palette_base;
	u_short pal;
	int update_hw = 0;

	if (regno > 255)
		return 1;

	if (info->fix.visual == FB_VISUAL_DIRECTCOLOR)
		return 1;

	if (info->var.bits_per_pixel > 16 && lcd_revision == LCD_VERSION_1)
		return -EINVAL;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		break;
	case FB_VISUAL_PSEUDOCOLOR:
		switch (info->var.bits_per_pixel) {
		case 4:
			if (regno > 15)
				return -EINVAL;

			if (info->var.grayscale) {
				pal = regno;
			} else {
				red >>= 4;
				green >>= 8;
				blue >>= 12;

				pal = red & 0x0f00;
				pal |= green & 0x00f0;
				pal |= blue & 0x000f;
			}
			if (regno == 0)
				pal |= 0x2000;
			palette[regno] = pal;
			break;

		case 8:
			red >>= 4;
			green >>= 8;
			blue >>= 12;

			pal = (red & 0x0f00);
			pal |= (green & 0x00f0);
			pal |= (blue & 0x000f);

			if (palette[regno] != pal) {
				update_hw = 1;
				palette[regno] = pal;
			}
			break;
		}
		break;
	}

	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno > 15)
			return -EINVAL;

		v = (red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset);

		switch (info->var.bits_per_pixel) {
		case 16:
			((u16 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		if (palette[0] != 0x4000) {
			update_hw = 1;
			palette[0] = 0x4000;
		}
	}

	/* Update the palette in the h/w as needed. */
	if (update_hw)
		lcd_load_palette(par);

	return 0;
}
#undef CNVT_TOHW

static void da8xx_fb_lcd_reset(void)
{
	/* DMA has to be disabled */
	lcdc_write(0, LCD_DMA_CTRL_REG);
	lcdc_write(0, LCD_RASTER_CTRL_REG);

	if (lcd_revision == LCD_VERSION_2) {
		lcdc_write(0, LCD_INT_ENABLE_SET_REG);
		/* Write 1 to reset */
		lcdc_write(LCD_CLK_MAIN_RESET, LCD_CLK_RESET_REG);
		lcdc_write(0, LCD_CLK_RESET_REG);
	}
}

static int da8xx_fb_config_clk_divider(struct da8xx_fb_par *par,
					      unsigned lcdc_clk_div,
					      unsigned lcdc_clk_rate)
{
	int ret;

	if (par->lcdc_clk_rate != lcdc_clk_rate) {
		ret = clk_set_rate(par->lcdc_clk, lcdc_clk_rate);
		if (IS_ERR_VALUE(ret)) {
			dev_err(par->dev,
				"unable to set clock rate at %u\n",
				lcdc_clk_rate);
			return ret;
		}
		par->lcdc_clk_rate = clk_get_rate(par->lcdc_clk);
	}

	/* Configure the LCD clock divisor. */
	lcdc_write(LCD_CLK_DIVISOR(lcdc_clk_div) |
			(LCD_RASTER_MODE & 0x1), LCD_CTRL_REG);

	if (lcd_revision == LCD_VERSION_2)
		lcdc_write(LCD_V2_DMA_CLK_EN | LCD_V2_LIDD_CLK_EN |
				LCD_V2_CORE_CLK_EN, LCD_CLK_ENABLE_REG);

	return 0;
}

static unsigned int da8xx_fb_calc_clk_divider(struct da8xx_fb_par *par,
			      unsigned pixclock,
			      unsigned *lcdc_clk_rate)
{
	unsigned lcdc_clk_div;

	pixclock = PICOS2KHZ(pixclock) * 1000;

	*lcdc_clk_rate = par->lcdc_clk_rate;

	if (pixclock < (*lcdc_clk_rate / CLK_MAX_DIV)) {
		*lcdc_clk_rate = clk_round_rate(par->lcdc_clk,
						pixclock * CLK_MAX_DIV);
		lcdc_clk_div = CLK_MAX_DIV;
	} else if (pixclock > (*lcdc_clk_rate / CLK_MIN_DIV)) {
		*lcdc_clk_rate = clk_round_rate(par->lcdc_clk,
						pixclock * CLK_MIN_DIV);
		lcdc_clk_div = CLK_MIN_DIV;
	} else {
		lcdc_clk_div = *lcdc_clk_rate / pixclock;
	}

	return lcdc_clk_div;
}

static int da8xx_fb_calc_config_clk_divider(struct da8xx_fb_par *par,
					    struct fb_videomode *mode)
{
	unsigned lcdc_clk_rate;
	unsigned lcdc_clk_div = da8xx_fb_calc_clk_divider(par, mode->pixclock,
							  &lcdc_clk_rate);

	return da8xx_fb_config_clk_divider(par, lcdc_clk_div, lcdc_clk_rate);
}

static unsigned da8xx_fb_round_clk(struct da8xx_fb_par *par,
				  unsigned pixclock)
{
	unsigned lcdc_clk_div, lcdc_clk_rate;

	lcdc_clk_div = da8xx_fb_calc_clk_divider(par, pixclock, &lcdc_clk_rate);
	return KHZ2PICOS(lcdc_clk_rate / (1000 * lcdc_clk_div));
}

static int lcd_init(struct da8xx_fb_par *par, const struct lcd_ctrl_config *cfg,
		struct fb_videomode *panel)
{
	u32 bpp;
	int ret = 0;
	struct da8xx_encoder *enc = NULL;

	if (IS_ENABLED(CONFIG_FB_DA8XX_TDA998X) && par->hdmi_node) {
		const unsigned clkdiv = 2; /* we use fixed div 2 */
		unsigned long pixclock;

		pixclock = PICOS2KHZ(panel->pixclock) * 1000;

		pr_debug("target pixclock %d ps, %lu Hz\n",
			panel->pixclock, pixclock);

		ret = clk_set_rate(par->lcdc_clk, pixclock * clkdiv);
		if (ret < 0) {
			dev_err(par->dev,
				"failed to set display clock rate to %lu: %d\n",
				pixclock, ret);
			return ret;
		}

		par->lcdc_clk_rate = clk_get_rate(par->lcdc_clk);
		pixclock = par->lcdc_clk_rate / clkdiv;

		pr_debug("lcd_clk=%u, pixclock=%lu, div=%u\n",
			par->lcdc_clk_rate, pixclock, clkdiv);

		/* Configure the LCD clock divisor. */
		lcdc_write(LCD_CLK_DIVISOR(clkdiv) |
			(LCD_RASTER_MODE & 0x1), LCD_CTRL_REG);

		if (lcd_revision == LCD_VERSION_2)
			lcdc_write(LCD_V2_DMA_CLK_EN | LCD_V2_LIDD_CLK_EN |
				LCD_V2_CORE_CLK_EN, LCD_CLK_ENABLE_REG);
	} else {
		/*
		 * Not using external encoder, using old and more inaccurate
		 * method of setting the clocks.
		 */
		ret = da8xx_fb_calc_config_clk_divider(par, panel);
		if (IS_ERR_VALUE(ret)) {
			dev_err(par->dev, "unable to configure clock\n");
			return ret;
		}
	}

	if (panel->sync & FB_SYNC_CLK_INVERT)
		lcdc_write((lcdc_read(LCD_RASTER_TIMING_2_REG) |
			LCD_INVERT_PIXEL_CLOCK), LCD_RASTER_TIMING_2_REG);
	else
		lcdc_write((lcdc_read(LCD_RASTER_TIMING_2_REG) &
			~LCD_INVERT_PIXEL_CLOCK), LCD_RASTER_TIMING_2_REG);

	/* Configure the DMA burst size and fifo threshold. */
	ret = lcd_cfg_dma(cfg->dma_burst_sz, cfg->fifo_th);
	if (ret < 0)
		return ret;

	/* Configure the vertical and horizontal sync properties. */
	lcd_cfg_vertical_sync(panel->upper_margin, panel->vsync_len,
			panel->lower_margin);
	lcd_cfg_horizontal_sync(panel->left_margin, panel->hsync_len,
			panel->right_margin);

	/* Configure for disply */
	ret = lcd_cfg_display(cfg, panel);
	if (ret < 0)
		return ret;

	bpp = cfg->bpp;

	if (bpp == 12)
		bpp = 16;
	ret = lcd_cfg_frame_buffer(par, (unsigned int)panel->xres,
				(unsigned int)panel->yres, bpp,
				cfg->raster_order);
	if (ret < 0)
		return ret;

	lcd_load_palette(par);

	/* Configure FDD */
	lcdc_write((lcdc_read(LCD_RASTER_CTRL_REG) & 0xfff00fff) |
		       (cfg->fdd << 12), LCD_RASTER_CTRL_REG);

	if (IS_ENABLED(CONFIG_FB_DA8XX_TDA998X) && par->hdmi_node) {
		/*
		 * keep doing this lookup, because there is a posibility that
		 * somebody went and unloaded the encoder driver from out
		 * beneath us
		 */
		enc = da8xx_get_encoder_from_phandle(par->hdmi_node);
		if (enc)
			enc->set_mode(enc, panel);
	}
	return 0;
}

int register_vsync_cb(vsync_callback_t handler, void *arg, int idx)
{
	if ((vsync_cb_handler == NULL) && (vsync_cb_arg == NULL)) {
		vsync_cb_arg = arg;
		vsync_cb_handler = handler;
	} else {
		return -EEXIST;
	}

	return 0;
}
EXPORT_SYMBOL(register_vsync_cb);

int unregister_vsync_cb(vsync_callback_t handler, void *arg, int idx)
{
	if ((vsync_cb_handler == handler) && (vsync_cb_arg == arg)) {
		vsync_cb_handler = NULL;
		vsync_cb_arg = NULL;
	} else {
		return -ENXIO;
	}

	return 0;
}
EXPORT_SYMBOL(unregister_vsync_cb);

/* IRQ handler for version 2 of LCDC */
static irqreturn_t lcdc_irq_handler_rev02(int irq, void *arg)
{
	struct da8xx_fb_par *par = arg;
	u32 stat = lcdc_read(LCD_MASKED_STAT_REG);

	if ((stat & LCD_SYNC_LOST) && (stat & LCD_FIFO_UNDERFLOW)) {
		lcd_disable_raster(DA8XX_FRAME_NOWAIT);
		lcdc_write(stat, LCD_MASKED_STAT_REG);
		lcd_enable_raster();
	} else if (stat & LCD_PL_LOAD_DONE) {
		/*
		 * Must disable raster before changing state of any control bit.
		 * And also must be disabled before clearing the PL loading
		 * interrupt via the following write to the status register. If
		 * this is done after then one gets multiple PL done interrupts.
		 */
		lcd_disable_raster(DA8XX_FRAME_NOWAIT);

		lcdc_write(stat, LCD_MASKED_STAT_REG);

		/* Disable PL completion interrupt */
		lcdc_write(LCD_V2_PL_INT_ENA, LCD_INT_ENABLE_CLR_REG);

		par->palette_loaded_flag = 1;
		wake_up_interruptible(&par->palette_wait);
	} else {
		lcdc_write(stat, LCD_MASKED_STAT_REG);

		if (stat & LCD_END_OF_FRAME0) {
			par->which_dma_channel_done = 0;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
			if (vsync_cb_handler)
				vsync_cb_handler(vsync_cb_arg);
		}

		if (stat & LCD_END_OF_FRAME1) {
			par->which_dma_channel_done = 1;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
			if (vsync_cb_handler)
				vsync_cb_handler(vsync_cb_arg);
		}

		/* Set only when controller is disabled and at the end of
		 * active frame
		 */
		if (stat & BIT(0)) {
			frame_done_flag = 1;
			wake_up_interruptible(&frame_done_wq);
		}
	}

	lcdc_write(0, LCD_END_OF_INT_IND_REG);
	return IRQ_HANDLED;
}

/* IRQ handler for version 1 LCDC */
static irqreturn_t lcdc_irq_handler_rev01(int irq, void *arg)
{
	struct da8xx_fb_par *par = arg;
	u32 stat = lcdc_read(LCD_STAT_REG);
	u32 reg_ras;

	if ((stat & LCD_SYNC_LOST) && (stat & LCD_FIFO_UNDERFLOW)) {
		lcd_disable_raster(DA8XX_FRAME_NOWAIT);
		lcdc_write(stat, LCD_STAT_REG);
		lcd_enable_raster();
	} else if (stat & LCD_PL_LOAD_DONE) {
		/*
		 * Must disable raster before changing state of any control bit.
		 * And also must be disabled before clearing the PL loading
		 * interrupt via the following write to the status register. If
		 * this is done after then one gets multiple PL done interrupts.
		 */
		lcd_disable_raster(DA8XX_FRAME_NOWAIT);

		lcdc_write(stat, LCD_STAT_REG);

		/* Disable PL completion inerrupt */
		reg_ras  = lcdc_read(LCD_RASTER_CTRL_REG);
		reg_ras &= ~LCD_V1_PL_INT_ENA;
		lcdc_write(reg_ras, LCD_RASTER_CTRL_REG);

		par->palette_loaded_flag = 1;
		wake_up_interruptible(&par->palette_wait);
	} else {
		lcdc_write(stat, LCD_STAT_REG);

		if (stat & LCD_END_OF_FRAME0) {
			par->which_dma_channel_done = 0;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
		}

		if (stat & LCD_END_OF_FRAME1) {
			par->which_dma_channel_done = 1;
			lcdc_write(par->dma_start,
				   LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
			lcdc_write(par->dma_end,
				   LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
			par->vsync_flag = 1;
			wake_up_interruptible(&par->vsync_wait);
		}
	}

	return IRQ_HANDLED;
}

static int fb_check_var(struct fb_var_screeninfo *var,
			struct fb_info *info)
{
	int err = 0;
	struct da8xx_fb_par *par = info->par;
	int bpp = var->bits_per_pixel >> 3;
	unsigned long line_size = var->xres_virtual * bpp;

	if (var->bits_per_pixel > 16 && lcd_revision == LCD_VERSION_1)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = 0;
		break;
	case 4:
		var->red.offset = 0;
		var->red.length = 4;
		var->green.offset = 0;
		var->green.length = 4;
		var->blue.offset = 0;
		var->blue.length = 4;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = FB_NONSTD_REV_PIX_IN_B;
		break;
	case 16:		/* RGB 565 */
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		var->nonstd = 0;
		break;
	case 24:
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->nonstd = 0;
		break;
	case 32:
		var->transp.offset = 24;
		var->transp.length = 8;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->nonstd = 0;
		break;
	default:
		err = -EINVAL;
	}

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	if (line_size * var->yres_virtual > par->vram_size)
		var->yres_virtual = par->vram_size / line_size;

	if (var->yres > var->yres_virtual)
		var->yres = var->yres_virtual;

	if (var->xres > var->xres_virtual)
		var->xres = var->xres_virtual;

	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;

	/*
	 * if we don't have an encoder attached, use the legacy
	 * clock setting code that works on da8xx but is a bit
	 * inaccurate for the encoders on AM335x
	 */
	if (!IS_ENABLED(CONFIG_FB_DA8XX_TDA998X) || (par->hdmi_node == NULL))
		var->pixclock = da8xx_fb_round_clk(par, var->pixclock);

	return err;
}

#ifdef CONFIG_CPU_FREQ
static int lcd_da8xx_cpufreq_transition(struct notifier_block *nb,
				     unsigned long val, void *data)
{
	struct da8xx_fb_par *par;

	par = container_of(nb, struct da8xx_fb_par, freq_transition);
	if (val == CPUFREQ_POSTCHANGE) {
		if (par->lcdc_clk_rate != clk_get_rate(par->lcdc_clk)) {
			par->lcdc_clk_rate = clk_get_rate(par->lcdc_clk);
			lcd_disable_raster(DA8XX_FRAME_WAIT);
			da8xx_fb_calc_config_clk_divider(par, &par->mode);
			if (par->blank == FB_BLANK_UNBLANK)
				lcd_enable_raster();
		}
	}

	return 0;
}

static int lcd_da8xx_cpufreq_register(struct da8xx_fb_par *par)
{
	par->freq_transition.notifier_call = lcd_da8xx_cpufreq_transition;

	return cpufreq_register_notifier(&par->freq_transition,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static void lcd_da8xx_cpufreq_deregister(struct da8xx_fb_par *par)
{
	cpufreq_unregister_notifier(&par->freq_transition,
				    CPUFREQ_TRANSITION_NOTIFIER);
}
#endif

static int fb_remove(struct platform_device *dev)
{
	struct fb_info *info = dev_get_drvdata(&dev->dev);

	if (info) {
		struct da8xx_fb_par *par = info->par;

#ifdef CONFIG_CPU_FREQ
		lcd_da8xx_cpufreq_deregister(par);
#endif
		if (par->panel_power_ctrl)
			par->panel_power_ctrl(0);

		lcd_disable_raster(DA8XX_FRAME_WAIT);
		lcdc_write(0, LCD_RASTER_CTRL_REG);

		/* disable DMA  */
		lcdc_write(0, LCD_DMA_CTRL_REG);

		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base,
				  par->p_palette_base);
		dma_free_coherent(NULL, par->vram_size, par->vram_virt,
				  par->vram_phys);
		pm_runtime_put_sync(&dev->dev);
		pm_runtime_disable(&dev->dev);
		framebuffer_release(info);

	}
	return 0;
}

/*
 * Function to wait for vertical sync which for this LCD peripheral
 * translates into waiting for the current raster frame to complete.
 */
static int fb_wait_for_vsync(struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	int ret;

	/*
	 * Set flag to 0 and wait for isr to set to 1. It would seem there is a
	 * race condition here where the ISR could have occurred just before or
	 * just after this set. But since we are just coarsely waiting for
	 * a frame to complete then that's OK. i.e. if the frame completed
	 * just before this code executed then we have to wait another full
	 * frame time but there is no way to avoid such a situation. On the
	 * other hand if the frame completed just after then we don't need
	 * to wait long at all. Either way we are guaranteed to return to the
	 * user immediately after a frame completion which is all that is
	 * required.
	 */
	par->vsync_flag = 0;
	ret = wait_event_interruptible_timeout(par->vsync_wait,
					       par->vsync_flag != 0,
					       par->vsync_timeout);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static int fb_ioctl(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{
	struct lcd_sync_arg sync_arg;

	switch (cmd) {
	case FBIOGET_CONTRAST:
	case FBIOPUT_CONTRAST:
	case FBIGET_BRIGHTNESS:
	case FBIPUT_BRIGHTNESS:
	case FBIGET_COLOR:
	case FBIPUT_COLOR:
		return -ENOTTY;
	case FBIPUT_HSYNC:
		if (copy_from_user(&sync_arg, (void __user *)arg,
				sizeof(struct lcd_sync_arg)))
			return -EFAULT;
		lcd_cfg_horizontal_sync(sync_arg.back_porch,
					sync_arg.pulse_width,
					sync_arg.front_porch);
		break;
	case FBIPUT_VSYNC:
		if (copy_from_user(&sync_arg, (void __user *)arg,
				sizeof(struct lcd_sync_arg)))
			return -EFAULT;
		lcd_cfg_vertical_sync(sync_arg.back_porch,
					sync_arg.pulse_width,
					sync_arg.front_porch);
		break;
	case FBIO_WAITFORVSYNC:
		return fb_wait_for_vsync(info);
	default:
		return -EINVAL;
	}
	return 0;
}

static int cfb_blank(int blank, struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	int ret = 0;

	if (par->blank == blank)
		return 0;

	par->blank = blank;
	switch (blank) {
	case FB_BLANK_UNBLANK:
		lcd_enable_raster();

		if (par->panel_power_ctrl)
			par->panel_power_ctrl(1);
		break;
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		if (par->panel_power_ctrl)
			par->panel_power_ctrl(0);

		lcd_disable_raster(DA8XX_FRAME_WAIT);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Set new x,y offsets in the virtual display for the visible area and switch
 * to the new mode.
 */
static int da8xx_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *fbi)
{
	int ret = 0;
	struct fb_var_screeninfo new_var;
	struct da8xx_fb_par         *par = fbi->par;
	struct fb_fix_screeninfo    *fix = &fbi->fix;
	unsigned int end;
	unsigned int start;
	unsigned long irq_flags;

	if (var->xoffset != fbi->var.xoffset ||
			var->yoffset != fbi->var.yoffset) {
		memcpy(&new_var, &fbi->var, sizeof(new_var));
		new_var.xoffset = var->xoffset;
		new_var.yoffset = var->yoffset;
		if (fb_check_var(&new_var, fbi))
			ret = -EINVAL;
		else {
			memcpy(&fbi->var, &new_var, sizeof(new_var));

			start	= fix->smem_start +
				new_var.yoffset * fix->line_length +
				new_var.xoffset * fbi->var.bits_per_pixel / 8;
			end	= start + fbi->var.yres * fix->line_length - 1;
			par->dma_start	= start;
			par->dma_end	= end;
			spin_lock_irqsave(&par->lock_for_chan_update,
					irq_flags);
			if (par->which_dma_channel_done == 0) {
				lcdc_write(par->dma_start,
					   LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
				lcdc_write(par->dma_end,
					   LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
			} else if (par->which_dma_channel_done == 1) {
				lcdc_write(par->dma_start,
					   LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
				lcdc_write(par->dma_end,
					   LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
			}
			spin_unlock_irqrestore(&par->lock_for_chan_update,
					irq_flags);
		}
	}

	return ret;
}

static int da8xxfb_set_par(struct fb_info *info)
{
	struct da8xx_fb_par *par = info->par;
	int ret;
	bool raster = da8xx_fb_is_raster_enabled();

	if (raster)
		lcd_disable_raster(DA8XX_FRAME_WAIT);

	fb_var_to_videomode(&par->mode, &info->var);

	par->cfg.bpp = info->var.bits_per_pixel;

	info->fix.visual = (par->cfg.bpp <= 8) ?
				FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;
	info->fix.line_length = (par->mode.xres * par->cfg.bpp) / 8;

	ret = lcd_init(par, &par->cfg, &par->mode);
	if (ret < 0) {
		dev_err(par->dev, "lcd init failed\n");
		return ret;
	}

	par->dma_start = info->fix.smem_start +
			 info->var.yoffset * info->fix.line_length +
			 info->var.xoffset * info->var.bits_per_pixel / 8;
	par->dma_end   = par->dma_start +
			 info->var.yres * info->fix.line_length - 1;

	lcdc_write(par->dma_start, LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
	lcdc_write(par->dma_end, LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	lcdc_write(par->dma_start, LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
	lcdc_write(par->dma_end, LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);

	if (raster)
		lcd_enable_raster();

	return 0;
}

static struct fb_ops da8xx_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = fb_check_var,
	.fb_set_par = da8xxfb_set_par,
	.fb_setcolreg = fb_setcolreg,
	.fb_pan_display = da8xx_pan_display,
	.fb_ioctl = fb_ioctl,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_blank = cfb_blank,
};

static struct lcd_ctrl_config *da8xx_fb_create_cfg(struct platform_device *dev)
{
	struct lcd_ctrl_config *cfg;

	cfg = devm_kzalloc(&dev->dev, sizeof(struct fb_videomode), GFP_KERNEL);
	if (!cfg)
		return NULL;

	/* default values */
	cfg->bpp = 16;

	/*
	 * For panels so far used with this LCDC, below statement is sufficient.
	 * For new panels, if required, struct lcd_ctrl_cfg fields to be updated
	 * with additional/modified values. Those values would have to be then
	 * obtained from dt(requiring new dt bindings).
	 */

	cfg->panel_shade = COLOR_ACTIVE;

	return cfg;
}

static struct fb_videomode *da8xx_fb_get_videomode(struct platform_device *dev)
{
	struct da8xx_lcdc_platform_data *fb_pdata = dev->dev.platform_data;
	struct fb_videomode *lcdc_info;
	struct device_node *np = dev->dev.of_node;
	int i;

	if (np) {
		lcdc_info = devm_kzalloc(&dev->dev,
					 sizeof(struct fb_videomode),
					 GFP_KERNEL);
		if (!lcdc_info)
			return NULL;
		
		if(!dispid_get_fb_videomode(lcdc_info, hw_dispid))
		{
		  return lcdc_info;
		}

		if (of_get_fb_videomode(np, lcdc_info, OF_USE_NATIVE_MODE)) {
			dev_err(&dev->dev, "timings not available in DT\n");
			return NULL;
		}
		return lcdc_info;
	}

	for (i = 0, lcdc_info = known_lcd_panels;
		i < ARRAY_SIZE(known_lcd_panels); i++, lcdc_info++) {
		if (strcmp(fb_pdata->type, lcdc_info->name) == 0)
			break;
	}

	if (i == ARRAY_SIZE(known_lcd_panels)) {
		dev_err(&dev->dev, "no panel found\n");
		return NULL;
	}
	dev_info(&dev->dev, "found %s panel\n", lcdc_info->name);

	return lcdc_info;
}

static int fb_probe(struct platform_device *device)
{
	struct da8xx_lcdc_platform_data *fb_pdata =
						device->dev.platform_data;
	static struct resource *lcdc_regs;
	struct lcd_ctrl_config *lcd_cfg;
	struct fb_videomode *lcdc_info;
	struct fb_info *da8xx_fb_info;
	struct da8xx_fb_par *par;
	struct clk *tmp_lcdc_clk;
	int ret;
	unsigned long ulcm;
	struct device_node *hdmi_node = NULL;

#ifdef CONFIG_SOC_AM33XX	
	/* HACK: Statically enable the display on HSE03 and ECO carriers
	*/
	int r;
	r=gpio_request(61,"en_vdd");
	if (r)
	  printk(KERN_ERR "!!!! failed to get en_vdd pin\n");
	gpio_direction_output(61,1);
#endif


	if (fb_pdata == NULL && !device->dev.of_node) {
		dev_err(&device->dev, "Can not get platform data\n");
		return -ENOENT;
	}

	if (device->dev.of_node) {
		hdmi_node = of_parse_phandle(device->dev.of_node,
					"hdmi", 0);
		if (hdmi_node &&
			(da8xx_get_encoder_from_phandle(hdmi_node) == NULL)) {
			/* i2c encoder has not initialized yet, defer */
			of_node_put(hdmi_node);
			return -EPROBE_DEFER;
		}
	}

	lcdc_info = da8xx_fb_get_videomode(device);
	if (lcdc_info == NULL)
		return -ENODEV;

	lcdc_regs = platform_get_resource(device, IORESOURCE_MEM, 0);
	da8xx_fb_reg_base = devm_ioremap_resource(&device->dev, lcdc_regs);
	if (IS_ERR(da8xx_fb_reg_base))
		return PTR_ERR(da8xx_fb_reg_base);

	tmp_lcdc_clk = devm_clk_get(&device->dev, "fck");
	if (IS_ERR(tmp_lcdc_clk)) {
		dev_err(&device->dev, "Can not get device clock\n");
		return PTR_ERR(tmp_lcdc_clk);
	}

	pm_runtime_enable(&device->dev);
	pm_runtime_get_sync(&device->dev);

	/*
	 * disable creation of new console during suspend.
	 * this works around a problem where a ctrl-c is needed
	 * to be entered on the VT to actually get the device
	 * to continue into the suspend state.
	 */
	pm_set_vt_switch(0);

	/* Determine LCD IP Version */
	switch (lcdc_read(LCD_PID_REG)) {
	case 0x4C100102:
		lcd_revision = LCD_VERSION_1;
		break;
	case 0x4F200800:
	case 0x4F201000:
		lcd_revision = LCD_VERSION_2;
		break;
	default:
		dev_warn(&device->dev, "Unknown PID Reg value 0x%x, "
				"defaulting to LCD revision 1\n",
				lcdc_read(LCD_PID_REG));
		lcd_revision = LCD_VERSION_1;
		break;
	}

	if (device->dev.of_node)
		lcd_cfg = da8xx_fb_create_cfg(device);
	else
		lcd_cfg = fb_pdata->controller_data;

	if (!lcd_cfg) {
		ret = -EINVAL;
		goto err_pm_runtime_disable;
	}

	da8xx_fb_info = framebuffer_alloc(sizeof(struct da8xx_fb_par),
					&device->dev);
	if (!da8xx_fb_info) {
		dev_dbg(&device->dev, "Memory allocation failed for fb_info\n");
		ret = -ENOMEM;
		goto err_pm_runtime_disable;
	}

	par = da8xx_fb_info->par;
	par->dev = &device->dev;
	par->lcdc_clk = tmp_lcdc_clk;
	par->lcdc_clk_rate = clk_get_rate(par->lcdc_clk);

	if (fb_pdata && fb_pdata->panel_power_ctrl) {
		par->panel_power_ctrl = fb_pdata->panel_power_ctrl;
		par->panel_power_ctrl(1);
	}

	if (device->dev.of_node)
		par->hdmi_node = hdmi_node;

	fb_videomode_to_var(&da8xx_fb_var, lcdc_info);
	par->cfg = *lcd_cfg;

	da8xx_fb_lcd_reset();

	/* allocate frame buffer */
	par->vram_size = lcdc_info->xres * lcdc_info->yres * lcd_cfg->bpp;
	ulcm = lcm((lcdc_info->xres * lcd_cfg->bpp)/8, PAGE_SIZE);
	par->vram_size = roundup(par->vram_size/8, ulcm);
	par->vram_size = par->vram_size * LCD_NUM_BUFFERS;

	par->vram_virt = dma_alloc_coherent(NULL,
					    par->vram_size,
					    (resource_size_t *) &par->vram_phys,
					    GFP_KERNEL | GFP_DMA);
	if (!par->vram_virt) {
		dev_err(&device->dev,
			"GLCD: kmalloc for frame buffer failed\n");
		ret = -EINVAL;
		goto err_release_fb;
	}

	da8xx_fb_info->screen_base = (char __iomem *) par->vram_virt;
	da8xx_fb_fix.smem_start    = par->vram_phys;
	da8xx_fb_fix.smem_len      = par->vram_size;
	da8xx_fb_fix.line_length   = (lcdc_info->xres * lcd_cfg->bpp) / 8;

	par->dma_start = par->vram_phys;
	par->dma_end   = par->dma_start + lcdc_info->yres *
		da8xx_fb_fix.line_length - 1;

	/* allocate palette buffer */
	par->v_palette_base = dma_alloc_coherent(NULL,
					       PALETTE_SIZE,
					       (resource_size_t *)
					       &par->p_palette_base,
					       GFP_KERNEL | GFP_DMA);
	if (!par->v_palette_base) {
		dev_err(&device->dev,
			"GLCD: kmalloc for palette buffer failed\n");
		ret = -EINVAL;
		goto err_release_fb_mem;
	}
	memset(par->v_palette_base, 0, PALETTE_SIZE);

	par->irq = platform_get_irq(device, 0);
	if (par->irq < 0) {
		ret = -ENOENT;
		goto err_release_pl_mem;
	}

	if (lcd_revision == LCD_VERSION_1)
		lcdc_irq_handler = lcdc_irq_handler_rev01;
	else {
		init_waitqueue_head(&frame_done_wq);
		lcdc_irq_handler = lcdc_irq_handler_rev02;
	}

	ret = devm_request_irq(&device->dev, par->irq, lcdc_irq_handler, 0,
			       DRIVER_NAME, par);
	if (ret)
		goto err_release_pl_mem;

	da8xx_fb_var.grayscale =
	    lcd_cfg->panel_shade == MONOCHROME ? 1 : 0;
	da8xx_fb_var.bits_per_pixel = lcd_cfg->bpp;

	/* Initialize fbinfo */
	da8xx_fb_info->flags = FBINFO_FLAG_DEFAULT;
	da8xx_fb_info->fix = da8xx_fb_fix;
	da8xx_fb_info->var = da8xx_fb_var;
	da8xx_fb_info->fbops = &da8xx_fb_ops;
	da8xx_fb_info->pseudo_palette = par->pseudo_palette;
	da8xx_fb_info->fix.visual = (da8xx_fb_info->var.bits_per_pixel <= 8) ?
				FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;

	ret = fb_alloc_cmap(&da8xx_fb_info->cmap, PALETTE_SIZE, 0);
	if (ret)
		goto err_release_pl_mem;
	da8xx_fb_info->cmap.len = par->palette_sz;

	dev_set_drvdata(&device->dev, da8xx_fb_info);

	/* initialize the vsync wait queue */
	init_waitqueue_head(&par->vsync_wait);
	par->vsync_timeout = HZ / 5;
	par->which_dma_channel_done = -1;
	spin_lock_init(&par->lock_for_chan_update);

	init_waitqueue_head(&par->palette_wait);

	/* set var and par */
	da8xx_fb_var.activate = FB_ACTIVATE_FORCE;
	fb_set_var(da8xx_fb_info, &da8xx_fb_var);

	/* Register the Frame Buffer  */
	if (register_framebuffer(da8xx_fb_info) < 0) {
		dev_err(&device->dev,
			"GLCD: Frame Buffer Registration Failed!\n");
		ret = -EINVAL;
		goto err_dealloc_cmap;
	}

#ifdef CONFIG_CPU_FREQ
	ret = lcd_da8xx_cpufreq_register(par);
	if (ret) {
		dev_err(&device->dev, "failed to register cpufreq\n");
		goto err_cpu_freq;
	}
#endif

	return 0;

#ifdef CONFIG_CPU_FREQ
err_cpu_freq:
#endif
	unregister_framebuffer(da8xx_fb_info);

err_dealloc_cmap:
	fb_dealloc_cmap(&da8xx_fb_info->cmap);

err_release_pl_mem:
	dma_free_coherent(NULL, PALETTE_SIZE, par->v_palette_base,
			  par->p_palette_base);

err_release_fb_mem:
	dma_free_coherent(NULL, par->vram_size, par->vram_virt, par->vram_phys);

err_release_fb:
	framebuffer_release(da8xx_fb_info);

err_pm_runtime_disable:
	pm_runtime_put_sync(&device->dev);
	pm_runtime_disable(&device->dev);

	return ret;
}

#ifdef CONFIG_PM
static struct lcdc_context {
	u32 clk_enable;
	u32 ctrl;
	u32 dma_ctrl;
	u32 raster_timing_0;
	u32 raster_timing_1;
	u32 raster_timing_2;
	u32 int_enable_set;
	u32 dma_frm_buf_base_addr_0;
	u32 dma_frm_buf_ceiling_addr_0;
	u32 dma_frm_buf_base_addr_1;
	u32 dma_frm_buf_ceiling_addr_1;
	u32 raster_ctrl;
} reg_context;

static void lcd_context_save(void)
{
	if (lcd_revision == LCD_VERSION_2) {
		reg_context.clk_enable = lcdc_read(LCD_CLK_ENABLE_REG);
		reg_context.int_enable_set = lcdc_read(LCD_INT_ENABLE_SET_REG);
	}

	reg_context.ctrl = lcdc_read(LCD_CTRL_REG);
	reg_context.dma_ctrl = lcdc_read(LCD_DMA_CTRL_REG);
	reg_context.raster_timing_0 = lcdc_read(LCD_RASTER_TIMING_0_REG);
	reg_context.raster_timing_1 = lcdc_read(LCD_RASTER_TIMING_1_REG);
	reg_context.raster_timing_2 = lcdc_read(LCD_RASTER_TIMING_2_REG);
	reg_context.dma_frm_buf_base_addr_0 =
		lcdc_read(LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
	reg_context.dma_frm_buf_ceiling_addr_0 =
		lcdc_read(LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	reg_context.dma_frm_buf_base_addr_1 =
		lcdc_read(LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
	reg_context.dma_frm_buf_ceiling_addr_1 =
		lcdc_read(LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	reg_context.raster_ctrl = lcdc_read(LCD_RASTER_CTRL_REG);
	return;
}

static void lcd_context_restore(void)
{
	if (lcd_revision == LCD_VERSION_2) {
		lcdc_write(reg_context.clk_enable, LCD_CLK_ENABLE_REG);
		lcdc_write(reg_context.int_enable_set, LCD_INT_ENABLE_SET_REG);
	}

	lcdc_write(reg_context.ctrl, LCD_CTRL_REG);
	lcdc_write(reg_context.dma_ctrl, LCD_DMA_CTRL_REG);
	lcdc_write(reg_context.raster_timing_0, LCD_RASTER_TIMING_0_REG);
	lcdc_write(reg_context.raster_timing_1, LCD_RASTER_TIMING_1_REG);
	lcdc_write(reg_context.raster_timing_2, LCD_RASTER_TIMING_2_REG);
	lcdc_write(reg_context.dma_frm_buf_base_addr_0,
			LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
	lcdc_write(reg_context.dma_frm_buf_ceiling_addr_0,
			LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
	lcdc_write(reg_context.dma_frm_buf_base_addr_1,
			LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
	lcdc_write(reg_context.dma_frm_buf_ceiling_addr_1,
			LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	lcdc_write(reg_context.raster_ctrl, LCD_RASTER_CTRL_REG);
	return;
}

static int fb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct da8xx_fb_par *par = info->par;

	console_lock();
	if (par->panel_power_ctrl)
		par->panel_power_ctrl(0);

	fb_set_suspend(info, 1);
	lcd_disable_raster(DA8XX_FRAME_WAIT);
	lcd_context_save();
	pm_runtime_put_sync(&dev->dev);
	console_unlock();

	/* Select sleep pin state */
	pinctrl_pm_select_sleep_state(&dev->dev);

	return 0;
}
static int fb_resume(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct da8xx_fb_par *par = info->par;

	/* Select default pin state */
	pinctrl_pm_select_default_state(&dev->dev);

	console_lock();
	pm_runtime_get_sync(&dev->dev);
	lcd_context_restore();
	if (par->blank == FB_BLANK_UNBLANK) {
		lcd_enable_raster();

		if (par->panel_power_ctrl)
			par->panel_power_ctrl(1);
	}

	fb_set_suspend(info, 0);
	console_unlock();

	return 0;
}
#else
#define fb_suspend NULL
#define fb_resume NULL
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id da8xx_fb_of_match[] = {
	/*
	 * this driver supports version 1 and version 2 of the
	 * Texas Instruments lcd controller (lcdc) hardware block
	 */
	{.compatible = "ti,da8xx-tilcdc", },
	{.compatible = "ti,am33xx-tilcdc", },
	{},
};
MODULE_DEVICE_TABLE(of, da8xx_fb_of_match);
#endif

static struct platform_driver da8xx_fb_driver = {
	.probe = fb_probe,
	.remove = fb_remove,
	.suspend = fb_suspend,
	.resume = fb_resume,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(da8xx_fb_of_match),
		   },
};

static int __init da8xx_fb_init(void)
{
	return platform_driver_register(&da8xx_fb_driver);
}

static void __exit da8xx_fb_cleanup(void)
{
	platform_driver_unregister(&da8xx_fb_driver);
}

module_init(da8xx_fb_init);
module_exit(da8xx_fb_cleanup);

MODULE_DESCRIPTION("Framebuffer driver for TI da8xx/omap-l1xx");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
