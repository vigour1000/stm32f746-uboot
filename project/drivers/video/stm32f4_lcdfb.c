/*
 * (C) Copyright 2011, 2012, 2013, 2015
 *
 * Pavel Boldin, Emcraft Systems, paboldin@emcraft.com
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <lcd.h>
#include <asm/io.h>
#include <asm/arch/stm32.h>
#include <../../include/asm-arm/arch-stm32/stm32f2_gpio.h>

vidinfo_t panel_info = {
	vl_col:		CONFIG_STM32F4_LTDC_XRES,
	vl_row:		CONFIG_STM32F4_LTDC_YRES,
	vl_bpix:	CONFIG_STM32F4_LTDC_BPP
};

/* Start of framebuffer memory */
void *lcd_base;
/* Start of console buffer */
void *lcd_console_address;

int lcd_line_length;
int lcd_color_fg;
int lcd_color_bg;

short console_col;
short console_row;

/* STM32F4 LTDC registers */
#define LTDC_SSCR	0x08
#define LTDC_BPCR	0x0c
#define LTDC_AWCR	0x10
#define LTDC_TWCR	0x14
#define LTDC_GCR	0x18
#define LTDC_SRCR	0x24
#define LTDC_BCCR	0x2c

/* STM32F4 LTDC per-layer registers */
#define LTDC_LAYER_CR(i)	(0x84 + 0x80 * (i))
#define LTDC_LAYER_WHPCR(i)	(0x88 + 0x80 * (i))
#define LTDC_LAYER_WVPCR(i)	(0x8c + 0x80 * (i))
#define LTDC_LAYER_PFCR(i)	(0x94 + 0x80 * (i))
#define LTDC_LAYER_CFBAR(i)	(0xac + 0x80 * (i))
#define LTDC_LAYER_CFBLR(i)	(0xb0 + 0x80 * (i))
#define LTDC_LAYER_CFBLNR(i)	(0xb4 + 0x80 * (i))

/* LTDC GCR Mask */
#define GCR_MASK	((u32)0x0FFE888F)

/* Enable flag for LTDC */
#define STM32_RCC_ENR_LTDCEN	(1 << 26)

static inline u32 ltdc_writel(u32 val, u32 offset)
{
	return writel(val, STM32F4_LTDC_BASE + offset);
}

static inline u32 ltdc_readl(u32 offset)
{
	return readl(STM32F4_LTDC_BASE + offset);
}

void lcd_enable(void)
{
}

void lcd_disable(void)
{
}

ulong calc_fbsize(void)
{
	return ((panel_info.vl_col * panel_info.vl_row *
		NBITS(panel_info.vl_bpix)) / 8);
}

int overwrite_console(void)
{
	/* Keep stdout / stderr on serial, our LCD is for splashscreen only */
	return 1;
}

static void ltdc_reload_config(void)
{
	/* Reload configutation immediately */
	ltdc_writel(1, LTDC_SRCR);
}

static void fb_enable_panel(void *lcdbase, int bpp)
{
	int i;
	u32 acc_h_bporch;
	u32 acc_v_bporch;

	i = 0;

//	acc_h_bporch = CONFIG_STM32F4_LTDC_HSYNC_LEN +
//		CONFIG_STM32F4_LTDC_LEFT_MARGIN;
//	acc_v_bporch = CONFIG_STM32F4_LTDC_VSYNC_LEN +
//		CONFIG_STM32F4_LTDC_UPPER_MARGIN;
//	ltdc_writel(
//		(acc_h_bporch << 0) |
//		((acc_h_bporch + CONFIG_STM32F4_LTDC_XRES) << 16),
//		LTDC_LAYER_WHPCR(i));
//	ltdc_writel(
//		(acc_v_bporch << 0) |
//		((acc_v_bporch + CONFIG_STM32F4_LTDC_YRES) << 16),
//		LTDC_LAYER_WVPCR(i));
//
//	switch (bpp) {
//		case 24:
//			/* Set pixel format to RGB888 */
//			ltdc_writel(1, LTDC_LAYER_PFCR(i));
//			break;
//		case 32:
//			/* Set pixel format to ARGB8888 */
//			ltdc_writel(0, LTDC_LAYER_PFCR(i));
//			break;
//		default:
//			puts("ERROR: Unknown BPP for LCD\n");
//			return;
//	}

	ltdc_writel((u32)lcdbase, LTDC_LAYER_CFBAR(i));

//	ltdc_writel(((CONFIG_STM32F4_LTDC_XRES * (bpp >> 3)) << 16) |
//		(CONFIG_STM32F4_LTDC_XRES * (bpp >> 3) + 7),
//		LTDC_LAYER_CFBLR(i));
//	ltdc_writel(CONFIG_STM32F4_LTDC_YRES, LTDC_LAYER_CFBLNR(i));
//
//	/* Enable layer */
//	ltdc_writel(ltdc_readl(LTDC_LAYER_CR(i)) | 1,
//		LTDC_LAYER_CR(i));
}

static void enable_lcdc(void)
{
	/* Enable LTDC */
	ltdc_writel(ltdc_readl(LTDC_GCR) | (1 << 0), LTDC_GCR);
}

static void disable_lcdc(void)
{
	/* Disable LTDC */
	ltdc_writel(ltdc_readl(LTDC_GCR) & ~(1 << 0), LTDC_GCR);
}

extern void sai_r_clk_enable(void);

void lcd_ctrl_init(void *lcdbase)
{
	u32 acc_h_cycles;
	u32 acc_v_cycles;

	if (!lcdbase)
		return;

	/* Enable LCDC module */
//	STM32_RCC->apb2enr |= STM32_RCC_ENR_LTDCEN;
	u32* APB2ENR;
	APB2ENR = (u32*)(0x40023844);
	*APB2ENR = 0x04000800;


	/* Enable pixel clock */
//	sai_r_clk_enable();
	u32* PLLI2SCFGR;
	PLLI2SCFGR = (u32*)(0x40023888);
	*PLLI2SCFGR = 0x54003000;


	disable_lcdc();

//	/*
//	 * Accumulated cycles starting with back porch:
//	 *   sync_len - 1 + back_porch + resolution + front_porch
//	 * We substract one to simplify writing to registers.
//	 */
//	acc_h_cycles = CONFIG_STM32F4_LTDC_HSYNC_LEN - 1;
//	acc_v_cycles = CONFIG_STM32F4_LTDC_VSYNC_LEN - 1;
//
//	/* Sets Synchronization size */
//	ltdc_writel((acc_h_cycles << 16) | acc_v_cycles, LTDC_SSCR);
//
//	/* Sets Accumulated Back porch */
//	acc_h_cycles += 13u;//CONFIG_STM32F4_LTDC_LEFT_MARGIN;
//	acc_v_cycles += CONFIG_STM32F4_LTDC_UPPER_MARGIN;
//	ltdc_writel((acc_h_cycles << 16) | acc_v_cycles, LTDC_BPCR);
//
//	/* Sets Accumulated Active Width */
//	acc_h_cycles += 480u;//CONFIG_STM32F4_LTDC_XRES;
//	acc_v_cycles += 272u;//CONFIG_STM32F4_LTDC_YRES;
//	ltdc_writel((acc_h_cycles << 16) | acc_v_cycles, LTDC_AWCR);
//
//	/* Sets Total Width */
//	acc_h_cycles += 32;//CONFIG_STM32F4_LTDC_RIGHT_MARGIN;
//	acc_v_cycles += 2;//CONFIG_STM32F4_LTDC_LOWER_MARGIN;
//	ltdc_writel((acc_h_cycles << 16) | acc_v_cycles, LTDC_TWCR);
//
//	/* Disable uncommon features of LTDC, and invert input pixclock */
//	ltdc_writel((ltdc_readl(LTDC_GCR) & GCR_MASK) | (1 << 28), LTDC_GCR);
//
//	/* Set background color to black */
//	ltdc_writel(0, LTDC_BCCR);

	u32* base_u32p;

	/* SSCR */
	base_u32p = (u32*) (0x40016808);
	*base_u32p = 0x00280009;
	/* BPCR */
	base_u32p = (u32*) (0x4001680C);
	*base_u32p = 0x0035000B;
	/* AWCR */
	base_u32p = (u32*) (0x40016810);
	*base_u32p = 0x0215011B;
	/* TWCR */
	base_u32p = (u32*) (0x40016814);
	*base_u32p = 0x0235011D;
	/* GCR */
	base_u32p = (u32*) (0x40016818);
	*base_u32p = 0x00002221;
	/* SRCR */
	base_u32p = (u32*) (0x40016824);
	*base_u32p = 0;
	/* BCCR */
	base_u32p = (u32*) (0x4001682C);
	*base_u32p = 0;
	/* IER */
	base_u32p = (u32*) (0x40016834);
	*base_u32p = 0x00000006;
	/* ISR */
	base_u32p = (u32*) (0x40016838);
	*base_u32p = 0x00000000;
	/* LIPCR */
	base_u32p = (u32*) (0x40016840);
	*base_u32p = 0;
	/* L1CR */
//	base_u32p = (u32*) (0x40016884);
//	*base_u32p = 0x00000001;
	/* L1WHPCR */
//	base_u32p = (u32*) (0x40016888);
//	*base_u32p = 0x02150036;
	/* L1WVPCR */
//	base_u32p = (u32*) (0x4001688C);
//	*base_u32p = 0x011B000C;
	/* L1CACR */
	base_u32p = (u32*) (0x40016898);
	*base_u32p = 0x000000FF;
	/* L1BFCR */
	base_u32p = (u32*) (0x400168A0);
	*base_u32p = 0x00000607;
//	/* L1CFBAR */
//	base_u32p = (u32*) (0x400168AC);
//	*base_u32p = &lcd_base;//0xC0000000;
//	/* L1CFBLR */
//	base_u32p = (u32*) (0x400168B0);
//	*base_u32p = 0x07800783;
//	/* L1CFBLNR */
//	base_u32p = (u32*) (0x400168B4);
//	*base_u32p = 0x00000110;
	/* L2CR */
	base_u32p = (u32*) (0x40016904);
	*base_u32p = 0x00000001;
	/* L2WHPCR */
	base_u32p = (u32*) (0x40016908);
	*base_u32p = 0x02150036;
	/* L2WVPCR */
	base_u32p = (u32*) (0x4001690C);
	*base_u32p = 0x011B000C;
	/* L2CACR */
	base_u32p = (u32*) (0x40016918);
	*base_u32p = 0x000000FF;
	/* L2BFCR */
	base_u32p = (u32*) (0x40016920);
	*base_u32p = 0x00000607;
	/* L2CFBAR */
	base_u32p = (u32*) (0x4001692C);
	*base_u32p = &lcd_base;//0xC007F800;
	/* L2CFBLR */
	base_u32p = (u32*) (0x40016930);
	*base_u32p = 0x07800783;
	/* L2CFBLNR */
	base_u32p = (u32*) (0x40016934);
	*base_u32p = 0x00000110;


//	/* Set background color to black */
//	ltdc_writel(0, LTDC_BCCR);

	/* Enable the LCD controller */
	enable_lcdc();


	const struct stm32f2_gpio_dsc port_bl = {STM32F2_GPIO_PORT_K, STM32F2_GPIO_PIN_3};
	stm32f2_gpout_set(&port_bl,1);

	const struct stm32f2_gpio_dsc port_dis = {STM32F2_GPIO_PORT_I, STM32F2_GPIO_PIN_12};
	stm32f2_gpout_set(&port_dis,1);

	/* Enable layer 1 */
//	fb_enable_panel(lcdbase, NBITS(panel_info.vl_bpix));

	ltdc_reload_config();


}

void lcd_setcolreg (ushort regno, ushort red, ushort green, ushort blue)
{
}
