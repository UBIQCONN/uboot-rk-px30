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
#include <video_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/drm_dp_helper.h>
#include "../drm/rockchip_bridge.h"
#include <power/regulator.h>
#include "tc358770.h"

DECLARE_GLOBAL_DATA_PTR;
#define EDID_BUF_SIZE 256

enum {
	PORT_DIR_IN,
	PORT_DIR_OUT,
};

struct tc_edp_link {
	struct drm_dp_link	base;
	u8			assr;
	bool			scrambler_dis;
	bool			spread;
};

struct tc358770_priv {
	u8			chipid;
	u8 			edid[EDID_BUF_SIZE];
	struct mipi_dsi_device	dsi;
	struct drm_display_mode	mode;
	struct tc_edp_link	link;
	u8			assr;
	u32			rev;
	struct udevice *vcc1v8_supply;
	struct udevice *vcc1v2_supply;
};


int tc358770_write(struct udevice *dev, u16 reg, u32 val)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct i2c_msg msg;

	u8 buf[] = {
		(reg >> 8) & 0xff, (reg >> 0) & 0xff,
		(val >> 0) & 0xff, (val >> 8) & 0xff,
		(val >> 16) & 0xff, (val >> 24) & 0xff
	};
	int ret;

	msg.addr = chip->chip_addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	ret = dm_i2c_xfer(dev, &msg, 1);
	if (ret) {
		printf("Could not execute transfer: %d\n", ret);
		return ret;
	}

	return 0;
}

int tc358770_read(struct udevice *dev, u16 reg, u32 *val)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	u32 data;
	u16 swap = ((reg & 0xff) << 8) | ((reg & 0xff00) >> 8); 
	struct i2c_msg msg[] = {
		{
			.addr = chip->chip_addr,
			.flags = 0,
			.buf = (u8 *)&swap,
			.len = 2,
		}, {
			.addr = chip->chip_addr,
			.flags = I2C_M_RD,
			.buf = (u8 *)&data,
			.len = 4,
		}
	};
	int ret;

	ret = dm_i2c_xfer(dev, msg, 2);
	if (ret) {
		printf("Could not execute transfer: %d\n", ret);
		return ret;
	}
	*val = data;

	return 0;
}

int tc358770_update_bits(struct udevice *dev, unsigned int reg,
                       unsigned int mask, unsigned int val)
{
	unsigned int tmp_val;

	tc358770_read(dev, reg, &tmp_val);
	tmp_val &= ~(mask);
	tmp_val |= val;
	return tc358770_write(dev, reg, tmp_val); 
}	


static inline int tc_poll_timeout(struct udevice *dev, unsigned int addr,
				  unsigned int cond_mask,
				  unsigned int cond_value,
				  unsigned long sleep_us, u64 timeout_us)
{
	u32 val;
        unsigned long timeout = timer_get_us() + timeout_us; 
	bool cond;

        for (;;) {
                tc358770_read(dev, addr, &val); 
		cond = ((val & cond_mask) == cond_value);
                if (cond) 
                        break; 
                if (timeout_us && time_after(timer_get_us(), timeout)) { 
                        tc358770_read(dev, addr, &val);
			cond = ((val & cond_mask) == cond_value);
                        break; 
                } 
        }  
 	if (cond)
		return 0;
	return -ETIMEDOUT;
}

static int tc_aux_wait_busy(struct udevice *dev)
{
	return tc_poll_timeout(dev, DP0_AUXSTATUS, AUX_BUSY, 0, 100, 100000);
}

static int tc_aux_write_data(struct udevice *dev, const void *data,
			     size_t size)
{
	u32 auxwdata[DP_AUX_MAX_PAYLOAD_BYTES / sizeof(u32)] = { 0 };
	int ret, count = (size - 1) / sizeof(u32) + 1;
	int i;
	
	memcpy(auxwdata, data, size);
	for (i=0; i<count; i++){
		ret = tc358770_write(dev, DP0_AUXWDATA(i), auxwdata[i]);
		if (ret)
			return ret;
	}	

	return size;
}

static int tc_aux_read_data(struct udevice *dev, void *data, size_t size)
{
	u32 auxrdata[DP_AUX_MAX_PAYLOAD_BYTES / sizeof(u32)];
	int ret, count = (size - 1) / sizeof(u32) + 1;
	int i;

	for (i=0; i<count; i++){
		ret = tc358770_read(dev, DP0_AUXRDATA(i), auxrdata+i);
		if (ret)
			return ret;
	}	

	memcpy(data, auxrdata, size);

	return size;
}

static u32 tc_auxcfg0(u8 req, size_t size)
{
	u32 auxcfg0 = req;

	if (size)
		auxcfg0 |= FIELD_PREP(DP0_AUXCFG0_BSIZE, size - 1);
	else
		auxcfg0 |= DP0_AUXCFG0_ADDR_ONLY;

	return auxcfg0;
}

static int tc_aux_transfer(struct udevice *dev, u8 req,
				u32 addr, u8 *buf, size_t len)
{

	size_t size = min_t(size_t, DP_AUX_MAX_PAYLOAD_BYTES - 1, len);
	u8 request = req & ~DP_AUX_I2C_MOT;
	u32 auxstatus;
	int ret;

	ret = tc358770_read(dev, DP0_AUXSTATUS, &auxstatus);
	ret = tc_aux_wait_busy(dev);
	if (ret)
		return ret;

	switch (request) {
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		break;
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
		if (size) {
			ret = tc_aux_write_data(dev, buf, size);
			if (ret < 0)
				return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	/* Store address */
	ret = tc358770_write(dev, DP0_AUXADDR, addr);
	if (ret)
		return ret;
	/* Start transfer */
	ret = tc358770_write(dev, DP0_AUXCFG0, tc_auxcfg0(req, size));
	if (ret)
		return ret;

	ret = tc_aux_wait_busy(dev);
	if (ret)
		return ret;

	ret = tc358770_read(dev, DP0_AUXSTATUS, &auxstatus);
	if (ret)
		return ret;

	if (auxstatus & AUX_TIMEOUT)
		return -ETIMEDOUT;
	/*
	 * For some reason address-only DP_AUX_I2C_WRITE (MOT), still
	 * reports 1 byte transferred in its status. To deal we that
	 * we ignore aux_bytes field if we know that this was an
	 * address-only transfer
	 */
	if (size)
		size = FIELD_GET(AUX_BYTES, auxstatus);

	switch (request) {
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		if (size)
			return tc_aux_read_data(dev, buf, size);
		break;
	}

	return size;

}

static int tc_read_aux_i2c(struct udevice *dev, u8 chip_addr,
				u8 offset, size_t count, u8 *buf)
{
	int i, ret;
	u8 cur_offset;

	for (i = 0; i < count; i += 8) {
		cur_offset = (u8)i;
		ret = tc_aux_transfer(dev, DP_AUX_I2C_WRITE,
					   chip_addr, &cur_offset, 1);
		if (ret<=0) {
			printf("%s: failed to set i2c offset: %d\n",
			      __func__, ret);
			return ret;
		}
		ret = tc_aux_transfer(dev, DP_AUX_I2C_READ,
					   chip_addr, buf + i, 8);
		if (ret<=0) {
			printf("%s: failed to read from i2c device: %d\n",
			      __func__, ret);
			return ret;
		}
	}

	return 0;
}

/*
static int tc_dpcd_readb(struct udevice *dev, u32 reg, u8 *val)
{
	int ret;

	ret = tc_aux_transfer(dev, DP_AUX_NATIVE_READ,
				   reg, val, 1);
	if (ret) {
		debug("Failed to read DPCD\n");
		return ret;
	}

	return 0;
}
*/
static int tc_dpcd_writeb(struct udevice *dev, u32 reg, u8 val)
{
	int ret;

	ret = tc_aux_transfer(dev, DP_AUX_NATIVE_WRITE,
				   reg, &val, 1);
	if (ret) {
		debug("Failed to read DPCD\n");
		return ret;
	}

	return 0;
}

static int tc_dpcd_write(struct udevice *dev, u32 reg, u8 *val, size_t len)
{
	int ret;
	
	ret = tc_aux_transfer(dev, DP_AUX_NATIVE_WRITE,
				   reg, val, len);
	if (ret) {
		debug("Failed to write DPCD\n");
		return ret;
	}

	return 0;
}

static int tc_dpcd_read(struct udevice *dev, u32 reg, u8 *val, size_t len)
{
	int ret;

	ret = tc_aux_transfer(dev, DP_AUX_NATIVE_READ,
				   reg, val, len);
	if (ret) {
		debug("Failed to read DPCD\n");
		return ret;
	}

	return 0;
}

static const char * const training_pattern1_errors[] = {
	"No errors",
	"Aux write error",
	"Aux read error",
	"Max voltage reached error",
	"Loop counter expired error",
	"res", "res", "res"
};

static const char * const training_pattern2_errors[] = {
	"No errors",
	"Aux write error",
	"Aux read error",
	"Clock recovery failed error",
	"Loop counter expired error",
	"res", "res", "res"
};

static int tc_pllupdate(struct udevice *dev, unsigned int pllctrl)
{
	int ret;

	ret = tc358770_write(dev, pllctrl, PLLUPDATE | PLLEN);
	if (ret)
		return ret;

	/* Wait for PLL to lock: up to 2.09 ms, depending on refclk */
	usleep_range(3000, 6000);

	return 0;
}

static int tc_pxl_pll_dis(struct udevice *dev)
{
	/* Enable PLL bypass, power down PLL */
	return tc358770_write(dev, PXL_PLLCTRL, PLLBYP);
}

static int tc_stream_clock_calc(struct udevice *dev)
{
	/*
	 * If the Stream clock and Link Symbol clock are
	 * asynchronous with each other, the value of M changes over
	 * time. This way of generating link clock and stream
	 * clock is called Asynchronous Clock mode. The value M
	 * must change while the value N stays constant. The
	 * value of N in this Asynchronous Clock mode must be set
	 * to 2^15 or 32,768.
	 *
	 * LSCLK = 1/10 of high speed link clock
	 *
	 * f_STRMCLK = M/N * f_LSCLK
	 * M/N = f_STRMCLK / f_LSCLK
	 *
	 */
	return tc358770_write(dev, DP0_VIDMNGEN1, 0xA8C0);
}

static int tc_set_dsi_configuration(struct udevice *dev)
{

	 tc358770_write(dev, DSI0_PPI_TX_RX_TA, 0x0008000B);
	 tc358770_write(dev, DSI0_PPI_LPTXTIMECNT, 0x00000007);
	 tc358770_write(dev, DSI0_PPI_D0S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(dev, DSI0_PPI_D1S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(dev, DSI0_PPI_D2S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(dev, DSI0_PPI_D3S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(dev, DSI0_PPI_LANEENABLE, 0x0000001F);
	 tc358770_write(dev, DSI0_DSI_LANEENABLE, 0x0000001F);
	 tc358770_write(dev, DSI0_PPI_STARTPPI, 0x00000001);
	 tc358770_write(dev, DSI0_DSI_STARTDSI, 0x00000001);
	 
	 tc358770_write(dev, DSI1_PPI_TX_RX_TA, 0x00070009);
	 tc358770_write(dev, DSI1_PPI_LPTXTIMECNT, 0x00000006);
	 tc358770_write(dev, DSI1_PPI_D0S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(dev, DSI1_PPI_D1S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(dev, DSI1_PPI_D2S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(dev, DSI1_PPI_D3S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(dev, DSI1_PPI_LANEENABLE, 0x0000001F);
	 tc358770_write(dev, DSI1_DSI_LANEENABLE, 0x0000001F);
	 tc358770_write(dev, DSI1_PPI_STARTPPI, 0x00000001);
	 tc358770_write(dev, DSI1_DSI_STARTDSI, 0x00000001);

	 return 1;
}

static int tc_setup_main_link(struct udevice *dev)
{
	tc358770_write(dev, DP0_SRCCTRL, 0x0000C095);
	tc358770_write(dev, SYS_PLLPARAM, 0x00000106);

	return 1;
}

static int tc_dp_phy_plls(struct udevice *dev)
{
	int ret;

	tc358770_write(dev, DP_PHY_CTRL, 0x0000000F);

	/* PLL setup */
	ret = tc_pllupdate(dev, DP0_PLLCTRL);
	if (ret)
		return ret;

	tc358770_write(dev, PXL_PLLPARAM, 0x00228236);

	/* Force PLL parameter update and disable bypass */
	return tc_pllupdate(dev, PXL_PLLCTRL);
}	

static int tc_reset_enable_main_link(struct udevice *dev)
{

	u32 dp_phy_ctrl;
	int ret;

	/* Reset/Enable Main Links */
	dp_phy_ctrl = PHY_M0_EN | PHY_A0_EN | PHY_2LANE | BGREN | PHY_M0_RST;
	ret = tc358770_write(dev, DP_PHY_CTRL, dp_phy_ctrl);
	usleep_range(100, 200);
	dp_phy_ctrl = PHY_M0_EN | PHY_A0_EN | PHY_2LANE | BGREN ;
	ret = tc358770_write(dev, DP_PHY_CTRL, dp_phy_ctrl);

	ret = tc_poll_timeout(dev, DP_PHY_CTRL, PHY_RDY, PHY_RDY, 500, 100000);
	if (ret) {
		printf("timeout waiting for phy become ready");
		return ret;
	}
	return 0;
}	

static int tc_bw_code_to_link_rate(u8 link_bw)
{
	switch (link_bw) {
	case DP_LINK_BW_1_62:
	default:
		return 162000;
	case DP_LINK_BW_2_7:
		return 270000;
	case DP_LINK_BW_5_4:
		return 540000;
	}
}

static u8 tc_dp_link_rate_to_bw_code(int link_rate)
{
        switch (link_rate) {
        case 162000:
        default:
                return DP_LINK_BW_1_62;
        case 270000:
                return DP_LINK_BW_2_7;
        case 540000:
                return DP_LINK_BW_5_4;
        }
}

static int tc_dp_link_probe(struct udevice *dev, struct drm_dp_link *link)
{
	u8 values[3];
	int err;

	memset(link, 0, sizeof(*link));

	err = tc_dpcd_read(dev, DP_DPCD_REV, values, sizeof(values));
	if (err < 0)
		return err;

	link->revision = values[0];
	link->rate = tc_bw_code_to_link_rate(values[1]);
	link->num_lanes = values[2] & DP_MAX_LANE_COUNT_MASK;

	if (values[2] & DP_ENHANCED_FRAME_CAP)
		link->capabilities |= DP_LINK_CAP_ENHANCED_FRAMING;

	return 0;
}

int tc_dp_link_configure(struct udevice *dev, struct drm_dp_link *link)
{
        u8 values[2];
        int err;

        values[0] = tc_dp_link_rate_to_bw_code(link->rate);
        values[1] = link->num_lanes;

        if (link->capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
                values[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;

        err = tc_dpcd_write(dev, DP_LINK_BW_SET, values, sizeof(values));
        if (err < 0)
                return err;

        return 0;
}


static int tc_set_display_props(struct udevice *dev)
{	

	int ret;
	struct tc358770_priv *priv = dev_get_priv(dev);

	ret = tc358770_write(dev, DP0_AUXCFG1, 0x0001063F);
	
	/* Read DP Rx Link Capability */
	ret = tc_dp_link_probe(dev, &priv->link.base);
	if (ret < 0)
		goto err_dpcd_read;
	if (priv->link.base.rate != 162000 && priv->link.base.rate != 270000) {
		printf("Falling to 2.7 Gbps rate\n");
		priv->link.base.rate = 270000;
	}

	if (priv->link.base.num_lanes > 2) {
		printf("Falling to 2 lanes\n");
		priv->link.base.num_lanes = 2;
	}

	printf("DPCD rev: %d.%d, rate: %s, lanes: %d, framing: %s\n",
		priv->link.base.revision >> 4, priv->link.base.revision & 0x0f,
		(priv->link.base.rate == 162000) ? "1.62Gbps" : "2.7Gbps",
		priv->link.base.num_lanes,
		(priv->link.base.capabilities & DP_LINK_CAP_ENHANCED_FRAMING) ?
		"enhanced" : "non-enhanced");
	printf("Downspread: %s, scrambler: %s\n",
		priv->link.spread ? "0.5%" : "0.0%",
		priv->link.scrambler_dis ? "disabled" : "enabled");
	printf("Display ASSR: %d, TC358770 ASSR: %d\n",
		priv->link.assr, priv->assr);

	/* Setup Link & DPRx Config for Training */
	ret = tc_dp_link_configure(dev, &priv->link.base);
	if (ret < 0)
		goto err_dpcd_write;

        ret = tc_dpcd_writeb(dev, DP_MAIN_LINK_CHANNEL_CODING_SET,
                                 DP_SET_ANSI_8B10B);
        if (ret > 0)
                return ret;

err_dpcd_write:
err_dpcd_read:
	printf("failed to read or write DPCD: %d\n", ret);
	return ret;

}

static int tc_wait_link_training(struct udevice *dev)
{
	u32 value;
	int ret;

	ret = tc_poll_timeout(dev, DP0_LTSTAT, LT_LOOPDONE,
			      LT_LOOPDONE, 5000, 100000);
	if (ret) {
		printf("Link training timeout waiting for LT_LOOPDONE!\n");
		return ret;
	}

	ret = tc358770_read(dev, DP0_LTSTAT, &value);
	if (ret)
		return ret;

	return (value >> 8) & 0x7;
}

static int tc_training_pattern(struct udevice *dev)
{

	int ret;
	u32 value;
	u8 tmp[DP_LINK_STATUS_SIZE];
	struct tc358770_priv *priv = dev_get_priv(dev);

	/* Reset voltage-swing & pre-emphasis */
	tmp[0] = tmp[1] = DP_TRAIN_VOLTAGE_SWING_LEVEL_0 |
			  DP_TRAIN_PRE_EMPH_LEVEL_0;
	ret = tc_dpcd_write(dev, DP_TRAINING_LANE0_SET, tmp, 2);
	if (ret < 0)
		goto err_dpcd_write;

	/* Set DPCD 0x102 for Training Pattern 1 */
	ret = tc358770_write(dev, DP0_SNKLTCTRL,  0x00000021);
	ret = tc358770_write(dev, DP0_LTLOOPCTRL, 0x7600000D);
	ret = tc358770_write(dev, DP0_SRCCTRL, 0x0000C195);


	ret = tc358770_write(dev, DP0CTL, VID_MN_GEN | DP_EN);

	/* wait */
	ret = tc_wait_link_training(dev);
	if (ret < 0)
		return ret;

	if (ret) {
		printf("Link training phase 1 failed: %s\n",
			training_pattern1_errors[ret]);
		return -ENODEV;
	}

	/* Channel Equalization */

	ret = tc358770_write(dev, DP0_SNKLTCTRL,  0x00000022);
	ret = tc358770_write(dev, DP0_SRCCTRL, 0x0000C295);


	/* wait */
	ret = tc_wait_link_training(dev);
	if (ret < 0)
		return ret;

	if (ret) {
		printf("Link training phase 2 failed: %s\n",
			training_pattern2_errors[ret]);
		return -ENODEV;
	}

        /* Clear DPCD 0x102 */
        /* Note: Can Not use DP0_SNKLTCTRL (0x06E4) short cut */
        tmp[0] = priv->link.scrambler_dis ? DP_LINK_SCRAMBLING_DISABLE : 0x00;
        ret = tc_dpcd_writeb(dev, DP_TRAINING_PATTERN_SET, tmp[0]);
        if (ret < 0)
                goto err_dpcd_write;

	ret = tc358770_write(dev, DP0_SRCCTRL, 0x00004095);
        /* Check link status */
	
	ret = tc_dpcd_read(dev, DPCD_LANE0_1_STATUS, tmp,
				DP_LINK_STATUS_SIZE);
        if (ret < 0)
                goto err_dpcd_read;

        ret = 0;

        value = tmp[0] & DP_CHANNEL_EQ_BITS;

        if (value != DP_CHANNEL_EQ_BITS) {
                printf("Lane 0 failed: %x\n", value);
                ret = -ENODEV;
        }

        if (priv->link.base.num_lanes == 2) {
                value = (tmp[0] >> 4) & DP_CHANNEL_EQ_BITS;

                if (value != DP_CHANNEL_EQ_BITS) {
                        printf("Lane 1 failed: %x\n", value);
                        ret = -ENODEV;
                }

                if (!(tmp[2] & DP_INTERLANE_ALIGN_DONE)) {
                        printf("Interlane align failed\n");
                        ret = -ENODEV;
                }
        }

        if (ret) {
                printf("0x0202 LANE0_1_STATUS:            0x%02x\n", tmp[0]);
                printf("0x0203 LANE2_3_STATUS             0x%02x\n", tmp[1]);
                printf("0x0204 LANE_ALIGN_STATUS_UPDATED: 0x%02x\n", tmp[2]);
                printf("0x0205 SINK_STATUS:               0x%02x\n", tmp[3]);
                printf("0x0206 ADJUST_REQUEST_LANE0_1:    0x%02x\n", tmp[4]);
                printf("0x0207 ADJUST_REQUEST_LANE2_3:    0x%02x\n", tmp[5]);
                return ret;
        }
        
	tmp[0] = priv->assr;
        ret = tc_dpcd_writeb(dev, DP_EDP_CONFIGURATION_SET, tmp[0]);
        if (ret < 0)
            goto err_dpcd_read;

	/* DOWNSPREAD_CTRL */
	tmp[0] = priv->link.spread ? DP_SPREAD_AMP_0_5 : 0x00;
	ret = tc_dpcd_write(dev, DP_DOWNSPREAD_CTRL, tmp, 1);
	if (ret < 0)
		goto err_dpcd_write;

        return 0;

err_dpcd_write:
err_dpcd_read:
        printf("Failed to read/write DPCD: %d\n", ret);
        return ret;
}	


static int tc_set_video_mode(struct udevice *dev,
			     const struct drm_display_mode *mode)
{
	int ret;
	int vid_sync_dly;
	int max_tu_symbol;

	int left_margin = mode->htotal - mode->hsync_end;
	int right_margin = mode->hsync_start - mode->hdisplay;
	int hsync_len = mode->hsync_end - mode->hsync_start;
	int upper_margin = mode->vtotal - mode->vsync_end;
	int lower_margin = mode->vsync_start - mode->vdisplay;
	int vsync_len = mode->vsync_end - mode->vsync_start;
	u32 dp0_syncval;
//	u32 bits_per_pixel = 24;
//	u32 in_bw, out_bw;

	/*
	 * Recommended maximum number of symbols transferred in a transfer unit:
	 * DIV_ROUND_UP((input active video bandwidth in bytes) * tu_size,
	 *              (output active video bandwidth in bytes))
	 * Must be less than tu_size.
	 */

//	in_bw = mode->clock * bits_per_pixel / 8;
//	out_bw = tc->link.base.num_lanes * tc->link.base.rate;
//	max_tu_symbol = DIV_ROUND_UP(in_bw * TU_SIZE_RECOMMENDED, out_bw);
	max_tu_symbol = 0x1e;

	printf("set mode %dx%d\n",
		mode->hdisplay, mode->vdisplay);
	printf("H margin %d,%d sync %d\n",
		left_margin, right_margin, hsync_len);
	printf("V margin %d,%d sync %d\n",
		upper_margin, lower_margin, vsync_len);
	printf("total: %dx%d\n", mode->htotal, mode->vtotal);

	/* Combiner Layer */
	tc358770_write(dev, CMBCTRL, 0);
	tc358770_write(dev, LRSIZE, (mode->hdisplay)<<16);
	tc358770_write(dev, RMPXL, 0);

	/*
	 * LCD Ctl Frame Size
	 * datasheet is not clear of vsdelay in case of DPI
	 * assume we do not need any delay when DPI is a source of
	 * sync signals
	 */
	ret = tc358770_write(dev, VPCTRL0,
			   FIELD_PREP(VSDELAY, 0x108) |
			   OPXLFMT_RGB666 | FRMSYNC_DISABLED | MSF_DISABLED);
	if (ret)
		return ret;

	ret = tc358770_write(dev, HTIM01,
			   FIELD_PREP(HBPR, ALIGN(left_margin, 2)) |
			   FIELD_PREP(HPW, ALIGN(hsync_len, 2)));
	if (ret)
		return ret;

	ret = tc358770_write(dev, HTIM02,
			   FIELD_PREP(HDISPR, ALIGN(mode->hdisplay, 2)) |
			   FIELD_PREP(HFPR, ALIGN(right_margin, 2)));
	if (ret)
		return ret;

	ret = tc358770_write(dev, VTIM01,
			   FIELD_PREP(VBPR, upper_margin) |
			   FIELD_PREP(VSPR, vsync_len));
	if (ret)
		return ret;

	ret = tc358770_write(dev, VTIM02,
			   FIELD_PREP(VFPR, lower_margin) |
			   FIELD_PREP(VDISPR, mode->vdisplay));
	if (ret)
		return ret;

	ret = tc358770_write(dev, VFUEN0, VFUEN); /* update settings */
	if (ret)
		return ret;

	/* Test pattern settings */
	ret = tc358770_write(dev, TSTCTL,
			   FIELD_PREP(COLOR_R, 120) |
			   FIELD_PREP(COLOR_G, 20) |
			   FIELD_PREP(COLOR_B, 99) |
			   ENI2CFILTER |
			   FIELD_PREP(COLOR_BAR_MODE, COLOR_BAR_MODE_BARS));
	if (ret)
		return ret;

	/* DP Main Stream Attributes */
	vid_sync_dly = hsync_len + left_margin + mode->hdisplay;
	ret = tc358770_write(dev, DP0_VIDSYNCDELAY,
		 FIELD_PREP(THRESH_DLY, max_tu_symbol) |
		 FIELD_PREP(VID_SYNC_DLY, vid_sync_dly));
	ret = tc358770_write(dev, DP0_VIDSYNCDELAY, 0x1f0804);

	ret = tc358770_write(dev, DP0_TOTALVAL,
			   FIELD_PREP(H_TOTAL, mode->htotal) |
			   FIELD_PREP(V_TOTAL, mode->vtotal));
	if (ret)
		return ret;

	ret = tc358770_write(dev, DP0_STARTVAL,
			   FIELD_PREP(H_START, left_margin + hsync_len) |
			   FIELD_PREP(V_START, upper_margin + vsync_len));
	if (ret)
		return ret;

	ret = tc358770_write(dev, DP0_ACTIVEVAL,
			   FIELD_PREP(V_ACT, mode->vdisplay) |
			   FIELD_PREP(H_ACT, mode->hdisplay));
	if (ret)
		return ret;

	dp0_syncval = FIELD_PREP(VS_WIDTH, vsync_len) |
		      FIELD_PREP(HS_WIDTH, hsync_len);

//	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		dp0_syncval |= SYNCVAL_VS_POL_ACTIVE_LOW;

//	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		dp0_syncval |= SYNCVAL_HS_POL_ACTIVE_LOW;

	ret = tc358770_write(dev, DP0_SYNCVAL, dp0_syncval);
	if (ret)
		return ret;

	ret = tc358770_write(dev, DPIPXLFMT,
			   VS_POL_ACTIVE_LOW | HS_POL_ACTIVE_LOW |
			   DE_POL_ACTIVE_HIGH | SUB_CFG_TYPE_CONFIG1 |
			   DPI_BPP_RGB888);
	if (ret)
		return ret;

	ret = tc358770_write(dev, DP0_MISC,0x1EBF0000);
/*	ret = tc358770_write(dev, DP0_MISC,
			   FIELD_PREP(MAX_TU_SYMBOL, max_tu_symbol) |
			   FIELD_PREP(TU_SIZE, TU_SIZE_RECOMMENDED) |
			   BPC_6);*/
	if (ret)
		return ret;

	return 0;
}

static int tc_main_link_disable(struct udevice *dev)
{
	int ret;

	printf("link disable\n");

	ret = tc358770_write(dev, DP0_SRCCTRL, 0);
	if (ret)
		return ret;

	return tc358770_write(dev, DP0CTL, 0);
}

static int tc_stream_enable(struct udevice *dev)
{
	int ret;
	u32 value;
	struct tc358770_priv *priv = dev_get_priv(dev);

	printf("enable video stream\n");

	ret = tc_set_video_mode(dev, &priv->mode);
	if (ret)
		return ret;

	/* Set M/N */
	ret = tc_stream_clock_calc(dev);
	if (ret)
		return ret;

	value = VID_MN_GEN | DP_EN;
	if (priv->link.base.capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		value |= EF_EN;
	ret = tc358770_write(dev, DP0CTL, value);
	if (ret)
		return ret;
	/*
	 * VID_EN assertion should be delayed by at least N * LSCLK
	 * cycles from the time VID_MN_GEN is enabled in order to
	 * generate stable values for VID_M. LSCLK is 270 MHz or
	 * 162 MHz, VID_N is set to 32768 in  tc_stream_clock_calc(),
	 * so a delay of at least 203 us should suffice.
	 */
	usleep_range(500, 1000);
	value |= VID_EN;
	ret = tc358770_write(dev, DP0CTL, value);
	if (ret)
		return ret;
	/* Set input interface */
	value = DP0_AUDSRC_NO_INPUT;
	value |= DP0_VIDSRC_DSI_RX;
	ret = tc358770_write(dev, SYSCTRL, value);
	if (ret)
		return ret;

	return 0;
}

static int tc_stream_disable(struct udevice *dev)
{
	int ret;

	printf("disable video stream\n");

	ret = tc358770_update_bits(dev, DP0CTL, VID_EN, 0);
	if (ret)
		return ret;

	tc_pxl_pll_dis(dev);

	return 0;
}

static void tc358770_enable(struct udevice *dev)
{
	int ret;

	ret = tc_setup_main_link(dev);
	if (ret < 0) {
		printf("failed to setup main link: %d\n", ret);
		return;
	}

	ret = tc_dp_phy_plls(dev);
	if (ret < 0) {
		printf("failed to DP Phy and PLLs: %d\n", ret);
		return;
	}

	ret = tc_reset_enable_main_link(dev);
	if (ret < 0) {
		printf("failed to reset and enable main link: %d\n", ret);
		return;
	}

	ret = tc_set_display_props(dev);
	if (ret < 0) {
		printf("failed to set display property: %d\n", ret);
		return;
	}

	ret = tc_training_pattern(dev);
	if (ret < 0) {
		printf("failed to training pattern: %d\n", ret);
		return;
	}

	ret = tc_stream_enable(dev);
	if (ret < 0) {
		printf("main link stream start error: %d\n", ret);
		tc_main_link_disable(dev);
		return;
	}
}	

static int tc358770_read_edid(struct udevice *dev, u8 *buf, int size)
{

	struct tc358770_priv *priv = dev_get_priv(dev);
	int ret;


	ret = tc_read_aux_i2c(dev, 0x50, 0x0, EDID_BUF_SIZE, priv->edid);
	if (ret < 0) {
		printf("failed to get edid\n");
		return ret;
	}

	if (size > EDID_BUF_SIZE)
		size = EDID_BUF_SIZE;
		
	memcpy(buf, priv->edid, EDID_BUF_SIZE);
	return size;

}

static void tc358770_disable(struct udevice *dev)
{
	int ret;

	ret = tc_stream_disable(dev);
	if (ret < 0) {
		printf("main link stream start error: %d\n", ret);
	}
	tc_main_link_disable(dev);
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

static int tc358770_attach(struct udevice *dev)
{
	struct udevice *mipi_dev;
	struct mipi_dsi_host *host;
	struct tc358770_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_device *dsi = &priv->dsi;
	int ret;

	mipi_dev = rockchip_of_find_dsi(dev);
	host = dev_get_platdata(mipi_dev);

	dsi->lanes = 4;
	dsi->channel = 0;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET;
	dsi->host = host;

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		printf("failed to attach dsi host: %d\n", ret);
		return ret;
	}

	if (priv->vcc1v8_supply){
		regulator_set_enable(priv->vcc1v8_supply, 1);
	}	
	if (priv->vcc1v2_supply){
		regulator_set_enable(priv->vcc1v2_supply, 1);
	}	

	return video_bridge_set_active(dev, true);
}

static int tc358770_probe(struct udevice *dev)
{
	struct tc358770_priv *priv = dev_get_priv(dev);
	struct rockchip_bridge *bridge =
		(struct rockchip_bridge *)dev_get_driver_data(dev);
	int ret;

	if (device_get_uclass_id(dev->parent) != UCLASS_I2C)
		return -EPROTONOSUPPORT;

	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "vcc1v8-supply", &priv->vcc1v8_supply);
	if (ret && ret != -ENOENT) {
		printf("%s: Cannot get power supply: %d\n", __func__, ret);
		return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
					   "vcc1v2-supply", &priv->vcc1v2_supply);
	if (ret && ret != -ENOENT) {
		printf("%s: Cannot get power supply: %d\n", __func__, ret);
		return ret;
	}

	bridge->dev = dev;

	tc_set_dsi_configuration(dev);

	return 0;
}

struct video_bridge_ops tc358770_ops = {
	.attach = tc358770_attach,
	.read_edid = tc358770_read_edid,
};

static void tc358770_bridge_pre_enable(struct rockchip_bridge *bridge)
{
	struct udevice* dev = bridge->dev;
	struct tc358770_priv *priv = dev_get_priv(dev);
	int ret;
       
	ret = tc358770_read(dev, TC_IDREG, &priv->rev);
        if (ret) {
                printf("can not read device ID: %d\n", ret);
                return;
        }

        if ((priv->rev != 0x7001) && (priv->rev != 0x7003)) {
                printf("invalid device ID: 0x%08x\n", priv->rev);
                return;
        }

        priv->assr = (priv->rev == 0x7001); /* Enable ASSR for eDP panels */
}

static void tc358770_bridge_enable(struct rockchip_bridge *bridge)
{
	tc358770_enable(bridge->dev);
}

static void tc358770_bridge_disable(struct rockchip_bridge *bridge)
{
	tc358770_disable(bridge->dev);
}

static void tc358770_bridge_mode_set(struct rockchip_bridge *bridge,
			 const struct drm_display_mode *mode)
{
	struct tc358770_priv *priv = dev_get_priv(bridge->dev);
	priv->mode = *mode;
	return;
}

static const struct rockchip_bridge_funcs tc358770_bridge_funcs = {
	.pre_enable = tc358770_bridge_pre_enable,
	.enable = tc358770_bridge_enable,
	.disable = tc358770_bridge_disable,
	.mode_set = tc358770_bridge_mode_set,
};

static struct rockchip_bridge tc358770_driver_data = {
	.funcs = &tc358770_bridge_funcs,
};

static const struct udevice_id tc358770_ids[] = {
	{ .compatible = "toshiba,tc358770",
       	  .data = (ulong)&tc358770_driver_data,	},
	{ }
};

U_BOOT_DRIVER(parade_tc358770) = {
	.name	= "toshiba_tc358770",
	.id	= UCLASS_VIDEO_BRIDGE,
	.probe	= tc358770_probe,
	.of_match = tc358770_ids,
	.ops	= &tc358770_ops,
	.priv_auto_alloc_size = sizeof(struct tc358770_priv),
};
