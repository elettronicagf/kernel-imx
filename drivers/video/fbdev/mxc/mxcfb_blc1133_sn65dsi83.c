/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static int blc1133_brightness;

static struct fb_videomode egf_lcd_modedb[] = {
	{
	 "EGF-BLC1133", 60, 1024, 600, 19531,
	 160, 160,
	 23, 12,
	 60, 10,
	 FB_SYNC_OE_LOW_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = (0x2),
	.max_phy_clk    = 500,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_blc1133_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &egf_lcd_modedb[0];
	*size = ARRAY_SIZE(egf_lcd_modedb);
	*data = &lcd_config;
}

int mipid_blc1133_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD EGF BLC1133 setup.\n");
	return 0;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return blc1133_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};
