/*
 * Copyright (C) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <linux/bitfield.h>
#include <errno.h>
#include <i2c.h>
#include <edid.h>
#include <clk.h>
#include <video_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/drm_dp_helper.h>
#include "../drm/rockchip_bridge.h"
#include <power/regulator.h>
#include "it6151.h"

#include <asm/io.h>
#include <reset.h>
DECLARE_GLOBAL_DATA_PTR;
#define EDID_BUF_SIZE 256

enum {
	PORT_DIR_IN,
	PORT_DIR_OUT,
};

enum it6151_type {
	ITE_6151_A1 = 0,
	ITE_6151_XX,
};

struct it6151_priv {
	u8			chipid;
	u8 			edid[EDID_BUF_SIZE];
	struct mipi_dsi_device	dsi;
	struct drm_display_mode	mode;
	uint 			mipi_addr;
	u32			rev;
	struct udevice *vcc1v8_supply;
	struct udevice *vcc1v2_supply;
	struct gpio_desc enable;
	u8 num_dsi_lanes;
	u8 channel_id;						/* dsi channel 0/1 */

	// u8 phy_channel;

	// it6151 setting from dtb
	u8 n_hs_settle;
	u8 n_skip_stg;

	// dp tx related params
	u8 num_dp_lanes;

	enum it6151_type ite_type;
};


static int it6151_write(struct udevice *dev, unsigned int addr_off,
			 unsigned char reg_addr, unsigned char value)
{
	uint8_t buf[2];
	struct i2c_msg msg;
	int ret;

	msg.addr = addr_off;
	msg.flags = 0;
	buf[0] = reg_addr;
	buf[1] = value;
	msg.buf = buf;
	msg.len = 2;
	ret = dm_i2c_xfer(dev, &msg, 1);
	if (ret) {
		debug("%s: write failed, reg=%#x, value=%#x, ret=%d\n",
		      __func__, reg_addr, value, ret);
		return ret;
	}

	return 0;
}

static int it6151_read(struct udevice *dev, unsigned int addr_off,
			unsigned char reg_addr, unsigned char *value)
{
	uint8_t addr, val;
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = addr_off;
	msg[0].flags = 0;
	addr = reg_addr;
	msg[0].buf = &addr;
	msg[0].len = 1;
	msg[1].addr = addr_off;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = 1;
	ret = dm_i2c_xfer(dev, msg, 2);
	if (ret) {
		debug("%s: read failed, reg=%.2x, value=%p, ret=%d\n",
		      __func__, (int)reg_addr, value, ret);
		return ret;
	}
	*value = val;

	return 0;
}

static int it6151_write_dp(struct udevice *dev, unsigned char reg_addr,
			    unsigned char value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);

	return it6151_write(dev, chip->chip_addr, reg_addr, value);
}

static int it6151_read_dp(struct udevice *dev, unsigned char reg_addr,
			   unsigned char *value)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);

	return it6151_read(dev, chip->chip_addr, reg_addr, value);
}

static int it6151_write_mipi(struct udevice *dev, unsigned char reg_addr,
			    unsigned char value)
{
	struct it6151_priv *priv = dev_get_priv(dev);

	return it6151_write(dev, priv->mipi_addr, reg_addr, value);
}

static int it6151_read_mipi(struct udevice *dev, unsigned char reg_addr,
			   unsigned char *value)
{
	struct it6151_priv *priv = dev_get_priv(dev);

	return it6151_read(dev, priv->mipi_addr, reg_addr, value);
}

static void it6151_hw_reset(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);

	if (priv->enable.dev) {

		dm_gpio_set_value(&priv->enable, 0);   
		usleep_range(50000, 60000);
		dm_gpio_set_value(&priv->enable, 1);   
		usleep_range(50000, 60000);
	}
}


static void it6151_dp_sw_reset(struct udevice *dev,  u32 reset_bits)
{
	int ret;

	ret = it6151_write_dp(dev, DP_REG_SW_RESET, reset_bits);
	WARN_ON(ret != 0);	
}

static void it6151_mipi_sw_reset(struct udevice *dev,  u32 reset_bits)
{
	int ret;

	ret = it6151_write_mipi(dev, MIPI_REG_SW_RESET, reset_bits);
	WARN_ON(ret != 0);
}

static int get_it6151_pixel_format(int mipi_dsi_pfmt)
{
	switch (mipi_dsi_pfmt)
	{
	case MIPI_DSI_FMT_RGB888:
		return PKT_RGB_24b;

	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return PKT_RGB_18b_P;
	case MIPI_DSI_FMT_RGB565:
	default:
		return -1;
	}
}

/* config mipi dsi dpi at enable */
static void it6151_config_dsi(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *dsi = &priv->dsi;
	struct drm_display_mode *mode = &priv->mode;
	int left_margin = mode->htotal - mode->hsync_end;
	int right_margin = mode->hsync_start - mode->hdisplay;
	int upper_margin = mode->vtotal - mode->vsync_end;
	int lower_margin = mode->vsync_start - mode->vdisplay;
	int ret = 0;
	u32 val;

#ifdef	ENABLE_MULTI_LANE_DESKEW
	/* default is Force continuous clock mode */
	it6151_write_mipi(dev, MIPI_REGn_PHY_PROTOCOL(1) /*0x19*/,(MP_CONTINUOUS_CLK << 1) | MP_LANE_DESKEW);
#else
	it6151_write_mipi(dev, MIPI_REGn_PHY_PROTOCOL(1) /*0x19*/,(MP_CONTINUOUS_CLK << 1));
#endif

	/* Color Formats, VESA + RGB888 */
	if ((ret = get_it6151_pixel_format(dsi->format)) < 0)
		val = PKT_RGB_24b;
	else
		val = ret;

	dev_info(dev, "DSI format is: %d\n", dsi->format);
	ret = it6151_write_mipi(dev, MIPI_REG_PKT_DTYPE_VCNUM/*0x27*/, val);
	
	dev_info(dev, "DSI Display Size: %d x %d %d %d %d %d\n", mode->hdisplay, mode->vdisplay, left_margin, right_margin, upper_margin, lower_margin);
#ifndef	ENABLE_MULTI_LANE_DESKEW  // user define timing
	/* hfp low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HFP_LOW, left_margin & MIPI_HFP_LOW_MASK);
	/* hfp high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HFP_HIGH, (left_margin & MIPI_HFP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);
	
	/* hbp low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HBP_LOW, right_margin & MIPI_HBP_LOW_MASK);
	/* hbp high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HBP_HIGH, (right_margin & MIPI_HBP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* htotal low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HTOTAL_LOW, mode->hdisplay & MIPI_HTOTAL_LOW_MASK);
	/* htotal high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_HTOTAL_HIGH, (mode->hdisplay & MIPI_HTOTAL_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vfp low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VFP_LOW, upper_margin & MIPI_VFP_LOW_MASK);
	/* vfp high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VFP_HIGH, (upper_margin & MIPI_VFP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vbp low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VBP_LOW, lower_margin & MIPI_VBP_LOW_MASK);
	/* vbp high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VBP_HIGH, (lower_margin & MIPI_VBP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vtotal low */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VTOTAL_LOW, mode->vdisplay & MIPI_VTOTAL_LOW_MASK);
	/* vtotal high */
	it6151_write_mipi(dev, MIPI_REG_MIPI_VTOTAL_HIGH, (mode->vdisplay & MIPI_VTOTAL_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	// todo check events mode
	/* hsw low/high vm.hsync_len */
	//it6151_write_mipi(dev, 0x33, MIPI_TIMING_USER_DEF | vm.hsync_len >> 8);
	//it6151_write_mipi(dev, 0x32, vm.hsync_len & 0xff);

	/* vsw low/high  vm.vsync_len */
	//it6151_write_mipi(dev, MIPI_TIMING_USER_DEF | vm.vsync_len >> 8);
	//it6151_write_mipi(dev, 0x3C, vm.vsync_len & 0xff);
#endif
	it6151_write_mipi(dev, MIPI_REGn_UFO(0)/*0x28*/, ((mode->hdisplay / 4 - 1) >> 2) & 0xC0);
	// it6151_write_mipi(dev, MIPI_REGn_UFO(0)/*0x28*/, ((mode->hdisplay / 4 - 1)  & MIPI_UFO_BLK_NUM_HIGH_MASK) >> 2) ;
	it6151_write_mipi(dev, MIPI_REGn_UFO(1)/*0x29*/, (mode->hdisplay / 4 - 1) & MIPI_UFO_BLK_NUM_LOW_MASK);


	it6151_write_mipi(dev, MIPI_REG_HDE_DELAY/*0x2e*/, 0x34);
	it6151_write_mipi(dev, MIPI_REGn_UFO(7)/*0x2f*/, 0x01);   // Enable UFO re-sync

	val = 0x0;
	
	if (mode->flags & DRM_MODE_FLAG_PHSYNC) {
		val |= HSYNC_POL_MASK;
	}
	if (mode->flags & DRM_MODE_FLAG_PVSYNC) {
		val |= VSYNC_POL_MASK;
	}

	val |= H_ReSync << 2;
	val |= V_ReSync << 3;
	it6151_write_mipi(dev, MIPI_REG_HSVSPOL_HVRESYNC /* 0x4e */, val);


	/* pclk/hsclk not multi 2, but div 2 */
	it6151_write_mipi(dev, MIPI_REGn_AFE(0)/*0x80*/, (0 << 5) | 0x2); 
//	it6151_write_mipi(dev, MIPI_REGn_AFE(4)/*0x84*/, 0xcf);   //UTI Add 
	/* Enable PPSM video stable change intr */
	it6151_write_mipi(dev, MIPI_REGn_INTR_MASK(0)/*0x09*/, 0x1);
	/* Timer interrupt count number */
	it6151_write_mipi(dev, MIPI_REGn_MISC(2) /*0x92 */, TIMER_CNT); //UTI Sub

	/* todo , eotp set */
}

/* config mipi rx */
static void it6151_config_rx(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *dsi = &priv->dsi;
//	int ret;
//	u32 val;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return;

	// mipirx reset
	it6151_mipi_sw_reset(dev, 0x00);

	/* rx lanes */
	// it6151_write_mipi(dev, MIPI_REG_SYS_CONFIG, dsi->lanes - 1 /* no swap*/);
	it6151_write_mipi(dev, MIPI_REG_SYS_CONFIG, (MP_LANE_SWAP << 7) | (MP_PN_SWAP << 6)
			| ((dsi->lanes - 1) << 4) /*| UFO = disable */);

	it6151_write_mipi(dev, MIPI_REG_CLKBUF_CTRL_MCLK_SET/*0x11*/, MP_MCLK_INV);

}

/* config dptx */
static void it6151_config_dp(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	// dptx reset
	it6151_dp_sw_reset(dev, 0x29);
	it6151_dp_sw_reset(dev, 0x00);
	
	// Enable HPD_IRQ,HPD_CHG,VIDSTABLE
	it6151_write_dp(dev,  DP_REGn_INTR_MASK(0)/*0x09*/,INT_MASK);
	it6151_write_dp(dev, DP_REGn_INTR_MASK(1)/*0x0A*/,0x00);
	it6151_write_dp(dev, DP_REGn_INTR_MASK(2)/*0x0B*/,0x00);

	it6151_write_dp(dev, DP_REGn_PSR_CTRL(1)/*0xC5*/, 0xC1);
	it6151_write_dp(dev, DP_REGn_INPUT_VID_TIMING(21)/*0xB5*/, 0x00);
	it6151_write_dp(dev, DP_REGn_INPUT_VID_TIMING(23)/*0xB7*/, 0x80);
	
	// Panel Self Refresh
	it6151_write_dp(dev, DP_REGn_PSR_CTRL(0)/*0xC4*/, 0xF0);

	// Clear all interrupt
	it6151_write_dp(dev, DP_REGn_INTR_STATUS_CLEAR(0)/*0x06*/,0xFF);
	it6151_write_dp(dev, DP_REGn_INTR_STATUS_CLEAR(1)/*0x07*/,0xFF);
	it6151_write_dp(dev, DP_REGn_INTR_STATUS_CLEAR(2)/*0x08*/,0xFF);

	it6151_dp_sw_reset(dev, 0x00);

	// i2c driving strength
	it6151_write_dp(dev, DP_REG_SYS_CONFIG/*0x0c*/, 0x08);
	// LS2V 
	it6151_write_dp(dev, DP_REGn_AUX_CH(1)/*0x21*/, 0x05);
	// HW HPD IRQ response enable/video blank gating disable
	it6151_write_dp(dev, DP_REG_HPD_VBG/*0x3a*/, 0x04);
	// CR value in EQ phase
	it6151_write_dp(dev, DP_REGn_PHY(7)/*0x5f*/, 0x06);
	// HDP IRQ Event timer
	it6151_write_dp(dev, DP_REGn_MISC_CTRL(1)/*0xc9*/, 0xf5);
	// AUX debug sel
	it6151_write_dp(dev, DP_REGn_MISC_CTRL(2)/*0xca*/, 0x4c);
	// I2C read 
	it6151_write_dp(dev, DP_REGn_MISC_CTRL(3)/*0xcb*/, 0x37);
	// 
	it6151_write_dp(dev, DP_REGn_MISC_CTRL(6)/*0xce*/, 0x80);
	// Enhanced Framing Mode / video fifo reset enable
	it6151_write_dp(dev, DP_REGn_DATA_LINK(3)/*0xd3*/, 0x03);
	// 
	it6151_write_dp(dev, DP_REGn_DATA_LINK(4)/*0xd4*/, 0x60);

	// AVI info frame enable / General info frame enable
	it6151_write_dp(dev, DP_REG_PACKET/*0xe8*/, 0x11);

	// non-Zero value for CEA setting, check the given input format.
	it6151_write_dp(dev, DP_REGn_AVI_INF_FRAME_PKT(3)/*0xec*/, 16);
	usleep_range(5000, 5000);
	
	// #define  DP_REGn_AUX_CH(n)				(0x20 + (n))    // 0x20-0x2f
	// PC master / EDID no Segment write 
	it6151_write_dp(dev, DP_REGn_AUX_CH(3)/*0x23*/, 0x42);
	// PC request cmd offset
	it6151_write_dp(dev, DP_REGn_AUX_CH(4)/*0x24*/, 0x07);
	it6151_write_dp(dev, DP_REGn_AUX_CH(5)/*0x25*/, 0x01);
	it6151_write_dp(dev, DP_REGn_AUX_CH(6)/*0x26*/, 0x00);
	// PC request command write data byte1
	it6151_write_dp(dev, DP_REGn_AUX_CH(7)/*0x27*/, 0x10);
	// Native Aux Write
	it6151_write_dp(dev, DP_REGn_AUX_CH(11)/*0x2B*/, 0x05);
	// FIFO clear to Normal operation
	it6151_write_dp(dev, DP_REGn_AUX_CH(3)/*0x23*/,0x40);
	// no dp lane swap/no aux pn swap / Auto AXRCLK average / main link off to gate AXCLK
	it6151_write_dp(dev, DP_REGn_AUX_CH(2)/*0x22*/,(DP_AUX_PN_SWAP << 3)|(DP_PN_SWAP << 2) | 0x03);
	// DP lanes
	it6151_write_dp(dev, DP_REGn_LINK_TRAINING(0)/*0x16*/, (DPTX_SSC_SETTING << 4) | 
		 (DP_LANE_SWAP << 3) | ((priv->num_dp_lanes -1) << 1) | TRAINING_BITRATE);
	
	// select bank1 
	it6151_write_dp(dev, DP_REG_SYS_DEBUG/*0x0f*/, 0x01);

	it6151_write_dp(dev, 0x76,0xa7);
	it6151_write_dp(dev, 0x77,0xaf);
	it6151_write_dp(dev, 0x7e,0x8f);
	it6151_write_dp(dev, 0x7f,0x07);

	it6151_write_dp(dev, 0x80,0xef);
	it6151_write_dp(dev, 0x81,0x5f);
	it6151_write_dp(dev, 0x82,0xef);
	it6151_write_dp(dev, 0x83,0x07);
	it6151_write_dp(dev, 0x88,0x38);
	it6151_write_dp(dev, 0x89,0x1f);
	it6151_write_dp(dev, 0x8a,0x48);

	it6151_write_dp(dev, 0x0f,0x00);
	it6151_write_dp(dev, 0x5c,0xf3);
	it6151_write_dp(dev, 0x17,0x04);
	it6151_write_dp(dev, 0x17,0x01);

	usleep_range(50000, 50000);
}

/* start it6151 */
static void it6151_start_display(struct udevice *dev)
{
	unsigned char  value_6c, value_5c;
	it6151_read_mipi(dev, 0x0d, &value_6c);
	it6151_read_dp(dev, 0x0d, &value_5c);

	printf("Chip Status %x %x\n", value_5c, value_6c);
}

static void it6151_enable(struct udevice *dev)
{
	usleep_range(30000, 60000);
	it6151_config_rx(dev);

	it6151_config_dsi(dev);

	it6151_config_dp(dev);

	it6151_start_display(dev);
}	


static void it6151_disable(struct udevice *dev)
{

}

static struct udevice *rockchip_of_find_dsi(struct udevice *bridge_dev)
{
	ofnode node, ports, port, ep;
	struct udevice *dev;
	int ret;

	ports = dev_read_subnode(bridge_dev, "ports");
	if (!ofnode_valid(ports))
		return NULL;

	ofnode_for_each_subnode(port, ports) {
		u32 reg;

		if (ofnode_read_u32(port, "reg", &reg))
			continue;

		if (reg != PORT_DIR_IN)
			continue;

		ofnode_for_each_subnode(ep, port) {
			ofnode _ep, _port, _ports;
			uint phandle;

			if (ofnode_read_u32(ep, "remote-endpoint", &phandle))
				continue;

			_ep = ofnode_get_by_phandle(phandle);
			if (!ofnode_valid(_ep))
				continue;

			_port = ofnode_get_parent(_ep);
			if (!ofnode_valid(_port))
				continue;

			_ports = ofnode_get_parent(_port);
			if (!ofnode_valid(_ports))
				continue;

			node = ofnode_get_parent(_ports);
			if (!ofnode_valid(node))
				continue;

			ret = uclass_get_device_by_ofnode(UCLASS_DISPLAY,
							  node, &dev);
			if (!ret)
				goto found;
		}
	}
	return NULL;
found:
	return dev;
}

static int it6151_attach(struct udevice *dev)
{
	struct udevice *mipi_dev;
	struct mipi_dsi_host *host;
	struct it6151_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *dsi = &priv->dsi;
	int ret;

	usleep_range(30000, 60000);
	mipi_dev = rockchip_of_find_dsi(dev);
	host = dev_get_platdata(mipi_dev);

	dsi->lanes = priv->num_dsi_lanes;
	dsi->channel = 0;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
                          /* MIPI_DSI_MODE_EOT_PACKET |*/ MIPI_DSI_MODE_VIDEO_HSE;

	dsi->host = host;

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		printf("failed to attach dsi host: %d\n", ret);
		return ret;
	}
	return video_bridge_set_active(dev, true);
}

static int it6151_parse_dt(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	u32 num_lanes = 0, channel_id = 0;

	channel_id = dev_read_u32_default(dev, "ite,dsi-channel", 0);
	num_lanes = dev_read_u32_default(dev, "ite,dsi-lanes", 4);

	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}

	if (channel_id > 3) {
		dev_err(dev, "Invalid dsi-channel: %d\n", channel_id);
		return -EINVAL;
	}

	priv->num_dsi_lanes = num_lanes;
	priv->channel_id = channel_id;

	// dp lanes
	num_lanes = dev_read_u32_default(dev, "ite,dp-lanes", 4);
	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}
	priv->num_dp_lanes = num_lanes;
	printf("it6151->num_dp_lanes %d\n", priv->num_dp_lanes);


	priv->n_skip_stg = dev_read_u32_default(dev, "ite,skip-stage", 0);
	priv->n_hs_settle = dev_read_u32_default(dev, "ite,hs-settle;", 0);

	return 0;
}

/* check 6121/22 */
static int it6151_check_chipid(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	u8 devid_low, devid_high;
	u8 venid_low, venid_high;
	u8 rev_id;
	int ret = 0;
	
	ret = it6151_read_dp(dev, REG_DEV_ID_LOW, &devid_low);
	if (ret) {
		dev_err(dev, "regmap_read REG_DEV_ID_LOW failed %d\n", ret);
		return ret;
	}

	ret = it6151_read_dp(dev, REG_DEV_ID_HIGH, &devid_high);
	if (ret) {
		dev_err(dev, "regmap_read REG_DEV_ID_HIGH failed %d\n", ret);
		return ret;
	}

	if (devid_high != DEV_ID_HIGH &&
			devid_low != DEV_ID_LOW51) {
			dev_err(dev,
				"Invalid ite6151 device id 0x%2x%2x (expect 0x6151)\n",
				devid_high, devid_low);
			return -EINVAL;
	}

	// 0x04 = 0xA1
	priv->ite_type = ITE_6151_A1;
	ret = it6151_read_dp(dev, REG_VEND_ID_LOW, &venid_low);
	ret = it6151_read_dp(dev, REG_VEND_ID_HIGH, &venid_high);
	ret = it6151_read_dp(dev, REG_REV_ID, &rev_id);

	printf("ite6151 device id 0x%2x%2x\n", devid_high, devid_low);
	printf("ite6151 vendor id 0x%2x%2x\n", venid_high, venid_low);
	printf("ite6151 REV id 0x%2x\n", rev_id);
	return ret;
}

static int it6151_init(struct udevice *dev)
{
//	struct mipi_dsi_device *dsi = it6151->dsi;

	// u8 test =0;

	/* 0x4 */
	it6151_dp_sw_reset(dev, 0x04);
	it6151_write_dp(dev, DP_REG_MIPI_PORT, (MIPI_I2C_ADDR << 1) | DP_MIPI_MIPIRX_EN);
	// it6151_mipi_sw_reset(it6151, 0x0);

	//it6151_read_dp(dev, DP_REG_MIPI_PORT, &test);
	
	// set hs /stg
	/* 0x18
	it6151_write_dp(dev, MIPI_REGn_PHY_PROTOCOL(0),
			it6151->n_skip_stg << 4 | it6151->n_hs_settle);
	*/

	return 0;
}

static int it6151_probe(struct udevice *dev)
{
	struct it6151_priv *priv = dev_get_priv(dev);
	struct rockchip_bridge *bridge =
		(struct rockchip_bridge *)dev_get_driver_data(dev);
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;


	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "1vcc1v8-supply", &priv->vcc1v8_supply);
	if (ret && ret != -ENOENT) {
		printf("%s: Cannot get power supply: %d\n", __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "1vcc1v2-supply", &priv->vcc1v2_supply);
	if (ret && ret != -ENOENT) {
		printf("%s: Cannot get power supply: %d\n", __func__, ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->enable,
				   GPIOD_IS_OUT );
	if (ret) {
		debug("%s: Could not decode reset-gpios (%d)\n", __func__, ret);
	}
	
	bridge->dev = dev;

	if (priv->vcc1v8_supply){
		regulator_set_enable(priv->vcc1v8_supply, 1);
	}	
	if (priv->vcc1v2_supply){
		regulator_set_enable(priv->vcc1v2_supply, 1);
	}	

	priv->mipi_addr = MIPI_I2C_ADDR;

	it6151_hw_reset(dev);
	ret = it6151_parse_dt(dev);
	if (ret) {
		printf("Parse dts failure!\n");
		return ret;
	}

	ret = it6151_check_chipid(dev);		// check ite6151 dev id
	if (ret) {
		printf("Check ID failure!\n");
		return ret;
	}

	it6151_init(dev);
	return 0;
}

struct video_bridge_ops it6151_ops = {
	.attach = it6151_attach,
/*	.read_edid = it6151_read_edid, */
};

static void it6151_bridge_pre_enable(struct rockchip_bridge *bridge)
{
	usleep_range(30000, 60000);
}

static void it6151_bridge_enable(struct rockchip_bridge *bridge)
{
	it6151_enable(bridge->dev);
}

static void it6151_bridge_disable(struct rockchip_bridge *bridge)
{
	it6151_disable(bridge->dev);
}

static void it6151_bridge_mode_set(struct rockchip_bridge *bridge,
			 const struct drm_display_mode *mode)
{
	struct it6151_priv *priv = dev_get_priv(bridge->dev);
	priv->mode = *mode;
	return;
}

static const struct rockchip_bridge_funcs it6151_bridge_funcs = {
	.pre_enable = it6151_bridge_pre_enable,
	.enable = it6151_bridge_enable,
	.disable = it6151_bridge_disable,
	.mode_set = it6151_bridge_mode_set,
};

static struct rockchip_bridge it6151_driver_data = {
	.funcs = &it6151_bridge_funcs,
};

static const struct udevice_id it6151_ids[] = {
	{ .compatible = "ite,it6151",
       	  .data = (ulong)&it6151_driver_data,	},
	{ }
};

U_BOOT_DRIVER(parade_it6151) = {
	.name	= "it6151",
	.id	= UCLASS_VIDEO_BRIDGE,
	.probe	= it6151_probe,
	.of_match = it6151_ids,
	.ops	= &it6151_ops,
	.priv_auto_alloc_size = sizeof(struct it6151_priv),
};
