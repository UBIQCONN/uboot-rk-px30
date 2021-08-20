
#define ENABLE_MULTI_LANE_DESKEW


#define usleep_range(a, b) udelay((b))
#define DP_I2C_ADDR 					(0xB8 >> 1)   // 0x5c
#define MIPI_I2C_ADDR 					(0xD8 >> 1)   // 0x6c

// MIPI Packed Pixel Stream
#define  PKT_RGB_24b					(0x3E)
#define  PKT_RGB_30b					(0x0D)
#define  PKT_RGB_36b					(0x1D)
#define  PKT_RGB_18b_P					(0x1E)
#define  PKT_RGB_18b_L					(0x2E)
#define  PKT_YCbCr_16b					(0x2C)
#define  PKT_YCbCr_20b					(0x0C)
#define  PKT_YCbCr_24b					(0x1C)

// DPTX reg62[3:0]
#define B_DPTXIN_6Bpp					(0)
#define B_DPTXIN_8Bpp					(1)
#define B_DPTXIN_10Bpp					(2)
#define B_DPTXIN_12Bpp					(3)

#define B_LBR    						(1)
#define B_HBR    						(0)

#define DP_4_LANE 						(3)
#define DP_2_LANE 						(1)
#define DP_1_LANE 						(0)

#define B_SSC_ENABLE   					(1)
#define B_SSC_DISABLE					(0)

#define TRAINING_BITRATE				(B_HBR)
#define DPTX_SSC_SETTING				(B_SSC_ENABLE)	//(B_SSC_DISABLE)
#define HIGH_PCLK						(1)
#define MP_MCLK_INV						(1)
#define MP_CONTINUOUS_CLK				(1)
#define MP_LANE_DESKEW					(1)
#define MP_LANE_SWAP					(0)
#define MP_PN_SWAP						(0)

#define DP_PN_SWAP						(0)
#define DP_AUX_PN_SWAP					(0)
#define DP_LANE_SWAP					(0)	//(0) our convert board need to LANE SWAP for data lane
#define FRAME_RESYNC					(1)
#define VESA_MAP						(1) // '0' for JEIDA , '1' for VESA MAP

#define INT_MASK						(3)
#define MIPI_RECOVER					(1)

#define En_UFO							(1)
#define H_ReSync						(1)  
#define V_ReSync						(1)
#define TIMER_CNT						(0x1A)

// DPTX/MIPIRX 
#define  REG_VEND_ID_LOW				0x00
#define  REG_VEND_ID_HIGH				0x01
#define  REG_DEV_ID_LOW					0x02
#define  REG_DEV_ID_HIGH				0x03
#define  REG_REV_ID						0x04
#define  DEV_ID_HIGH					0x61
#define  DEV_ID_LOW51					0x51

// DPTX Register Bank0
#define  DP_REG_SW_RESET				0x05
#define  DP_SW_RESET_VEDIO_CLK_MASK		(0x1 << 0)
#define  DP_SW_RESET_REF_CLK_MASK		(0x1 << 2)
#define  DP_SW_RESET_AUX_CLK_MASK		(0x1 << 3)
#define  DP_SW_RESET_SDM_CLK_MASK		(0x1 << 5)

#define  DP_REGn_INTR_STATUS_CLEAR(n)	(0x06 + (n))	// 0x6/0x7/0x8
#define  DP_REGn_INTR_MASK(n)			(0x09 + (n))	// 0x9/0xa/0xb

#define  DP_REG_SYS_CONFIG				0x0C			
#define  DP_REGn_SYS_STATUS(n)			(0x0D + (n))	// 0xd/0xe

#define  DP_REG_SYS_DEBUG				0x0F

#define  DP_REGn_CLKBUF_CTRL(n)			(0x10 + (n))    // 0x11-0x14

#define  DP_REGn_LINK_TRAINING(n)		(0x16 + (n))	// 0x16-0x1f

#define  DP_REGn_AUX_CH(n)				(0x20 + (n))    // 0x20-0x2f

#define  DP_REG_HPD_VBG					0x3A

#define	 DP_REGn_PHY(n)					(0x58 + (n))    // 0x58-0x5f

#define  DP_REGn_VIDEO(n)				(0x60 + (n))    // 0x60/0x62

#define  DP_REGn_SYNC_DE_GEN(n)			(0x79 + (n))	// 0x79-0x94

#define  DP_REGn_PATTERN_GEN(n)			(0x95 + (n))    // 0x95-0x9f

#define  DP_REGn_INPUT_VID_TIMING(n)	(0xA0 + (n))    // 0xa0-0xb7

#define	 DP_REGn_PSR_CTRL(n)			(0xC4 + (n))	// 0xc4-0xc7

#define	 DP_REGn_MISC_CTRL(n)			(0xC8 + (n))    // 0xc8-0xcf

#define  DP_REGn_DATA_LINK(n)			(0xD0 + (n))    // 0xd0-0xe7

#define  DP_REG_PACKET					0xE8

#define  DP_REGn_AVI_INF_FRAME_PKT(n)	(0xE9 + (n))	// 0xe9-0xfc

#define  DP_REG_MIPI_PORT				0xFD
#define  DP_MIPI_MIPIRX_EN				(0x1 << 0)

// DPTX Register Bank1

// MIPIRX Register
#define  MIPI_REG_SW_RESET				0x05
#define  MIPI_SW_RESET_OCLK_MASK		(0x1 << 0)				
#define  MIPI_SW_RESET_MCLK_MASK		(0x1 << 1)			
#define  MIPI_SW_RESET_BCLK_MASK		(0x1 << 2)				
#define  MIPI_SW_RESET_NCLK_MASK		(0x1 << 3)
#define  MIPI_SW_RESET_MP_PCLK_MASK		(0x1 << 4)
#define  MIPI_SW_RESET_DP_PCLK_MASK		(0x1 << 5)

#define  MIPI_REGn_INTR_STATUS_CLEAR(n)	(0x06 + (n))	// 0x6/0x7/0x8
#define  MIPI_REGn_INTR_MASK(n)			(0x09 + (n))	// 0x9/0xa/0xb

#define  MIPI_REG_SYS_CONFIG			0x0C   

#define  MIPI_REG_SYS_STATUS			0x0D

#define  MIPI_REGn_CLKBUF_CTRL(n)		(0x10 + (n))	// 0x10-0x17
#define  MIPI_REG_CLKBUF_CTRL_GATE		MIPI_REGn_CLKBUF_CTRL(0)
#define  MIPI_REG_CLKBUF_CTRL_MCLK_SET	MIPI_REGn_CLKBUF_CTRL(1)
#define  MIPI_REG_CLKBUF_CTRL_PSR_SET	MIPI_REGn_CLKBUF_CTRL(2)

#define  MIPI_REGn_PHY_PROTOCOL(n)		(0x18 + (n))	// 0x18/0x19

#define  MIPI_REGn_LANE_PKT_DECODER(n)	(0x20 + (n))	// 0x20/0x27
#define  MIPI_REGn_PKT_DEC_SET			0x20
#define  MIPI_REG_PKT_DTYPE_VCNUM		0x27

#define  MIPI_REGn_UFO(n)				(0x28 + (n))	// 0x28-0x2f
#define  MIPI_VLC_EN_MASK				(0x1)
#define  MIPI_UFO_BLK_NUM_HIGH_MASK		(0x300)
#define  MIPI_UFO_BLK_NUM_LOW_MASK		(0x0FF)
#define  MIPI_REG_HDE_DELAY				0x2E

#define  MIPI_TIMING_USER_DEF			(0x1 << 7)

/* HFP */
#define  MIPI_REG_MIPI_HFP_LOW			0x30
#define  MIPI_HFP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HFP_HIGH			0x31
#define  MIPI_HFP_HIGH_MASK				0x0f00

/* HBP */
#define  MIPI_REG_MIPI_HBP_LOW			0x32
#define  MIPI_HBP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HBP_HIGH			0x33
#define  MIPI_HBP_HIGH_MASK				0x0f00

/* HDES */
#define  MIPI_REG_MIPI_HDES_LOW			0x34
#define  MIPI_HDES_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HDES_HIGH		0x35
#define  MIPI_HDES_HIGH_MASK			0x0f00

/* HDEE */
#define  MIPI_REG_MIPI_HDEE_LOW			0x36
#define  MIPI_HDEE_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HDEE_HIGH		0x37
#define  MIPI_HDEE_HIGH_MASK			0x0f00

/* HTotal*/
#define  MIPI_REG_MIPI_HTOTAL_LOW		0x38
#define  MIPI_HTOTAL_LOW_MASK			0xFF
#define  MIPI_REG_MIPI_HTOTAL_HIGH		0x39
#define  MIPI_HTOTAL_HIGH_MASK			0x0f00

/* VFP */
#define  MIPI_REG_MIPI_VFP_LOW			0x3A
#define  MIPI_VFP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VFP_HIGH			0x3B
#define  MIPI_VFP_HIGH_MASK				0x0f00

/* VBP */
#define  MIPI_REG_MIPI_VBP_LOW			0x3C
#define  MIPI_VBP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VBP_HIGH			0x3D
#define  MIPI_VBP_HIGH_MASK				0x0f00

/* VDES */
#define  MIPI_REG_MIPI_VDES_LOW			0x3E
#define  MIPI_VDES_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VDES_HIGH		0x3F
#define  MIPI_VDES_HIGH_MASK			0x0f00

/* VDEE */
#define  MIPI_REG_MIPI_VDEE_LOW			0x40
#define  MIPI_VDEE_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VDEE_HIGH		0x41
#define  MIPI_VDEE_HIGH_MASK			0x0f00

/* VTotal */
#define  MIPI_REG_MIPI_VTOTAL_LOW		0x42
#define  MIPI_VTOTAL_LOW_MASK			0xFF
#define  MIPI_REG_MIPI_VTOTAL_HIGH		0x43
#define  MIPI_VTOTAL_HIGH_MASK			0x0f00

#define  MIPI_REG_HSVSPOL_HVRESYNC		0x4E
#define  HSYNC_POL_MASK					(0x1 << 0)
#define  VSYNC_POL_MASK					(0x1 << 1)
#define  HRESYNC_EN_MASK				(0x1 << 2)
#define  VRESYNC_EN_MASK				(0x1 << 3)
//#define  FIFO_RESET_EN_MASK			(0x1 << 4)
#define  FORCE_MCLK_ON					(0x1 << 4)
#define  FORCE_PPS_STABLE				(0x1 << 7)

#define  MIPI_REG_FIFOAUTO_RESET_DEBUG_PPS	0x4F

/* 0x50-0x63 readonly registers*/

#define  MIPI_REG_MCLK_HSHIFT_HAVG_PCLK_HSHIFT	0x70

#define  MIPI_REGn_AFE(n)				(0x80 + (n))   // 0x80-0x8c

#define	 MIPI_REGn_MISC(n)				(0x90 + (n))   // 0x90-0x9b
#define	 MIPI_REG_TM_STAMP_EN			MIPI_REGn_MISC(0x0)
#define  MIPI_REG_10US_TIME_INTPART		MIPI_REGn_MISC(0x1)
