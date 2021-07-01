
struct drm_dp_link {
	unsigned char revision;
	unsigned int rate;
	unsigned int num_lanes;
	unsigned long capabilities;
};

#define usleep_range(a, b) udelay((b))
/* Registers */
#define DSI0_PPI_STARTPPI 		0x0104
#define DSI0_PPI_LPTXTIMECNT		0x0114
#define DSI0_PPI_LANEENABLE		0x0134
#define DSI0_PPI_TX_RX_TA		0x013C
#define DSI0_PPI_D0S_CLRSIPOCOUNT	0x0164
#define DSI0_PPI_D1S_CLRSIPOCOUNT	0x0168
#define DSI0_PPI_D2S_CLRSIPOCOUNT	0x016c
#define DSI0_PPI_D3S_CLRSIPOCOUNT	0x0170
#define DSI0_DSI_STARTDSI		0x0204
#define DSI0_DSI_LANEENABLE		0x0210
#define DSI1_PPI_STARTPPI 		0x1104
#define DSI1_PPI_LPTXTIMECNT		0x1114
#define DSI1_PPI_LANEENABLE		0x1134
#define DSI1_PPI_TX_RX_TA		0x113C
#define DSI1_PPI_D0S_CLRSIPOCOUNT	0x1164
#define DSI1_PPI_D1S_CLRSIPOCOUNT	0x1168
#define DSI1_PPI_D2S_CLRSIPOCOUNT	0x116c
#define DSI1_PPI_D3S_CLRSIPOCOUNT	0x1170
#define DSI1_DSI_STARTDSI		0x1204
#define DSI1_DSI_LANEENABLE		0x1210

/* Display Parallel Interface */
#define DPIPXLFMT		0x0440
#define VS_POL_ACTIVE_LOW		(1 << 10)
#define HS_POL_ACTIVE_LOW		(1 << 9)
#define DE_POL_ACTIVE_HIGH		(0 << 8)
#define SUB_CFG_TYPE_CONFIG1		(0 << 2) /* LSB aligned */
#define SUB_CFG_TYPE_CONFIG2		(1 << 2) /* Loosely Packed */
#define SUB_CFG_TYPE_CONFIG3		(2 << 2) /* LSB aligned 8-bit */
#define DPI_BPP_RGB888			(0 << 0)
#define DPI_BPP_RGB666			(1 << 0)
#define DPI_BPP_RGB565			(2 << 0)

/* Video Path */
#define VPCTRL0			0x0450
#define VSDELAY			GENMASK(31, 20)
#define OPXLFMT_RGB666			(0 << 8)
#define OPXLFMT_RGB888			(1 << 8)
#define FRMSYNC_DISABLED		(0 << 4) /* Video Timing Gen Disabled */
#define FRMSYNC_ENABLED			(1 << 4) /* Video Timing Gen Enabled */
#define MSF_DISABLED			(0 << 0) /* Magic Square FRC disabled */
#define MSF_ENABLED			(1 << 0) /* Magic Square FRC enabled */
#define HTIM01			0x0454
#define HPW			GENMASK(8, 0)
#define HBPR			GENMASK(24, 16)
#define HTIM02			0x0458
#define HDISPR			GENMASK(10, 0)
#define HFPR			GENMASK(24, 16)
#define VTIM01			0x045c
#define VSPR			GENMASK(7, 0)
#define VBPR			GENMASK(23, 16)
#define VTIM02			0x0460
#define VFPR			GENMASK(23, 16)
#define VDISPR			GENMASK(10, 0)
#define VFUEN0			0x0464
#define VFUEN				BIT(0)   /* Video Frame Timing Upload */


/* Combiner Layer Registers */
#define CMBCTRL			0x0480
#define LRSIZE			0x0484
#define RMPXL			0x048C


/* System */
#define TC_IDREG		0x0500
#define SYSSTAT			0x0508
#define SYSCTRL			0x0510
#define DP0_AUDSRC_NO_INPUT		(0 << 3)
#define DP0_AUDSRC_I2S_RX		(1 << 3)
#define DP0_VIDSRC_NO_INPUT		(0 << 0)
#define DP0_VIDSRC_DSI_RX		(1 << 0)
#define DP0_VIDSRC_DPI_RX		(2 << 0)
#define DP0_VIDSRC_COLOR_BAR		(3 << 0)
#define SYSRSTENB		0x050c
#define ENBI2C				(1 << 0)
#define ENBLCD0				(1 << 2)
#define ENBBM				(1 << 3)
#define ENBDSIRX			(1 << 4)
#define ENBREG				(1 << 5)
#define ENBHDCP				(1 << 8)
#define GPIOM			0x0540
#define GPIOC			0x0544
#define GPIOO			0x0548
#define GPIOI			0x054c
#define INTCTL_G		0x0560
#define INTSTS_G		0x0564

#define INT_SYSERR		BIT(16)
#define INT_GPIO_H(x)		(1 << (x == 0 ? 2 : 10))
#define INT_GPIO_LC(x)		(1 << (x == 0 ? 3 : 11))

#define INT_GP0_LCNT		0x0584
#define INT_GP1_LCNT		0x0588

/* Control */
#define DP0CTL			0x0600
#define VID_MN_GEN			BIT(6)   /* Auto-generate M/N values */
#define EF_EN				BIT(5)   /* Enable Enhanced Framing */
#define VID_EN				BIT(1)   /* Video transmission enable */
#define DP_EN				BIT(0)   /* Enable DPTX function */

/* Clocks */
#define DP0_VIDMNGEN0		0x0610
#define DP0_VIDMNGEN1		0x0614
#define DP0_VMNGENSTATUS	0x0618

/* Main Channel */
#define DP0_SECSAMPLE		0x0640
#define DP0_VIDSYNCDELAY	0x0644
#define VID_SYNC_DLY		GENMASK(15, 0)
#define THRESH_DLY		GENMASK(31, 16)

#define DP0_TOTALVAL		0x0648
#define H_TOTAL			GENMASK(15, 0)
#define V_TOTAL			GENMASK(31, 16)
#define DP0_STARTVAL		0x064c
#define H_START			GENMASK(15, 0)
#define V_START			GENMASK(31, 16)
#define DP0_ACTIVEVAL		0x0650
#define H_ACT			GENMASK(15, 0)
#define V_ACT			GENMASK(31, 16)

#define DP0_SYNCVAL		0x0654
#define VS_WIDTH		GENMASK(30, 16)
#define HS_WIDTH		GENMASK(14, 0)
#define SYNCVAL_HS_POL_ACTIVE_LOW	(1 << 15)
#define SYNCVAL_VS_POL_ACTIVE_LOW	(1 << 31)
#define DP0_MISC		0x0658
#define TU_SIZE_RECOMMENDED		(63) /* LSCLK cycles per TU */
#define MAX_TU_SYMBOL		GENMASK(28, 23)
#define TU_SIZE			GENMASK(21, 16)
#define BPC_6				(0 << 5)
#define BPC_8				(1 << 5)

/* AUX channel */
#define DP0_AUXCFG0		0x0660
#define DP0_AUXCFG0_BSIZE	GENMASK(11, 8)
#define DP0_AUXCFG0_ADDR_ONLY	BIT(4)
#define DP0_AUXCFG1		0x0664
#define AUX_RX_FILTER_EN		BIT(16)

#define DP0_AUXADDR		0x0668
#define DP0_AUXWDATA(i)		(0x066c + (i) * 4)
#define DP0_AUXRDATA(i)		(0x067c + (i) * 4)
#define DP0_AUXSTATUS		0x068c
#define AUX_BYTES		GENMASK(15, 8)
#define AUX_STATUS		GENMASK(7, 4)
#define AUX_TIMEOUT		BIT(1)
#define AUX_BUSY		BIT(0)
#define DP0_AUXI2CADR		0x0698

/* Link Training */
#define DP0_SRCCTRL		0x06a0
#define DP0_SRCCTRL_SCRMBLDIS		BIT(13)
#define DP0_SRCCTRL_EN810B		BIT(12)
#define DP0_SRCCTRL_NOTP		(0 << 8)
#define DP0_SRCCTRL_TP1			(1 << 8)
#define DP0_SRCCTRL_TP2			(2 << 8)
#define DP0_SRCCTRL_LANESKEW		BIT(7)
#define DP0_SRCCTRL_SSCG		BIT(3)
#define DP0_SRCCTRL_LANES_1		(0 << 2)
#define DP0_SRCCTRL_LANES_2		(1 << 2)
#define DP0_SRCCTRL_BW27		(1 << 1)
#define DP0_SRCCTRL_BW162		(0 << 1)
#define DP0_SRCCTRL_AUTOCORRECT		BIT(0)
#define DP0_LTSTAT		0x06d0
#define LT_LOOPDONE			BIT(22)
#define LT_STATUS_MASK			(0x1f << 8)
#define LT_CHANNEL1_EQ_BITS		(DP_CHANNEL_EQ_BITS << 4)
#define LT_INTERLANE_ALIGN_DONE		BIT(3)
#define LT_CHANNEL0_EQ_BITS		(DP_CHANNEL_EQ_BITS)
#define DP0_SNKLTCHGREQ		0x06d4
#define DP0_LTLOOPCTRL		0x06d8
#define DP0_SNKLTCTRL		0x06e4

#define DP1_SRCCTRL		0x07a0

/* PHY */
#define DP_PHY_CTRL		0x0800
#define PHY_RDY				BIT(16)  /* PHY Main Channels Ready */
#define PHY_M0_RST			BIT(4)   /* Reset PHY0 Main Channel */
#define BGREN				BIT(3)   /* AUX PHY BGR Enable */
#define PHY_2LANE			BIT(2)   /* PHY Enable 2 lanes */
#define PHY_A0_EN			BIT(1)   /* PHY Aux Channel0 Enable */
#define PHY_M0_EN			BIT(0)   /* PHY Main Channel0 Enable */

/* PLL */
#define DP0_PLLCTRL		0x0900
#define DP1_PLLCTRL		0x0904	/* not defined in DS */
#define PXL_PLLCTRL		0x0908
#define PLLUPDATE			BIT(2)
#define PLLBYP				BIT(1)
#define PLLEN				BIT(0)
#define PXL_PLLPARAM		0x0914
#define IN_SEL_REFCLK			(2 << 14)
#define SYS_PLLPARAM		0x0918
#define REF_FREQ_38M4			(0 << 8) /* 38.4 MHz */
#define REF_FREQ_19M2			(1 << 8) /* 19.2 MHz */
#define REF_FREQ_26M			(2 << 8) /* 26 MHz */
#define REF_FREQ_13M			(3 << 8) /* 13 MHz */
#define SYSCLK_SEL_LSCLK		(1 << 2)
#define LSCLK_DIV_1			(1 << 0)
#define LSCLK_DIV_2			(2 << 0)

/* Test & Debug */
#define TSTCTL			0x0a00
#define COLOR_R			GENMASK(31, 24)
#define COLOR_G			GENMASK(23, 16)
#define COLOR_B			GENMASK(15, 8)
#define ENI2CFILTER		BIT(4)
#define COLOR_BAR_MODE		GENMASK(1, 0)
#define COLOR_BAR_MODE_BARS	2
#define PLL_DBG			0x0a04


#define DP_AUX_MAX_PAYLOAD_BYTES	16
#define DP_LINK_CAP_ENHANCED_FRAMING (1 << 0)
#define DP_LINK_STATUS_SIZE	   6
#define DPCD_LANE0_1_STATUS			(0x0202)
#define DPCD_LINK_BW_SET			(0x0100)
