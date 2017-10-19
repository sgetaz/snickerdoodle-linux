/*
 * Copyright (C) 2017 Gateworks Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/tda1997x.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <dt-bindings/media/tda1997x.h>

/* debug level */
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

/* Page 0x00 - General Control */
#define REG_VERSION		0x0000
#define REG_INPUT_SEL		0x0001
#define REG_SVC_MODE		0x0002
#define REG_HPD_MAN_CTRL	0x0003
#define REG_RT_MAN_CTRL		0x0004
#define REG_STANDBY_SOFT_RST	0x000A
#define REG_HDMI_SOFT_RST	0x000B
#define REG_HDMI_INFO_RST	0x000C
#define REG_INT_FLG_CLR_TOP	0x000E
#define REG_INT_FLG_CLR_SUS	0x000F
#define REG_INT_FLG_CLR_DDC	0x0010
#define REG_INT_FLG_CLR_RATE	0x0011
#define REG_INT_FLG_CLR_MODE	0x0012
#define REG_INT_FLG_CLR_INFO	0x0013
#define REG_INT_FLG_CLR_AUDIO	0x0014
#define REG_INT_FLG_CLR_HDCP	0x0015
#define REG_INT_FLG_CLR_AFE	0x0016
#define REG_INT_MASK_TOP	0x0017
#define REG_INT_MASK_SUS	0x0018
#define REG_INT_MASK_DDC	0x0019
#define REG_INT_MASK_RATE	0x001A
#define REG_INT_MASK_MODE	0x001B
#define REG_INT_MASK_INFO	0x001C
#define REG_INT_MASK_AUDIO	0x001D
#define REG_INT_MASK_HDCP	0x001E
#define REG_INT_MASK_AFE	0x001F
#define REG_DETECT_5V		0x0020
#define REG_SUS_STATUS		0x0021
#define REG_V_PER		0x0022
#define REG_H_PER		0x0025
#define REG_HS_WIDTH		0x0027
#define REG_FMT_H_TOT		0x0029
#define REG_FMT_H_ACT		0x002b
#define REG_FMT_H_FRONT		0x002d
#define REG_FMT_H_SYNC		0x002f
#define REG_FMT_H_BACK		0x0031
#define REG_FMT_V_TOT		0x0033
#define REG_FMT_V_ACT		0x0035
#define REG_FMT_V_FRONT_F1	0x0037
#define REG_FMT_V_FRONT_F2	0x0038
#define REG_FMT_V_SYNC		0x0039
#define REG_FMT_V_BACK_F1	0x003a
#define REG_FMT_V_BACK_F2	0x003b
#define REG_FMT_DE_ACT		0x003c
#define REG_RATE_CTRL		0x0040
#define REG_CLK_MIN_RATE	0x0043
#define REG_CLK_MAX_RATE	0x0046
#define REG_CLK_A_STATUS	0x0049
#define REG_CLK_A_RATE		0x004A
#define REG_DRIFT_CLK_A_REG	0x004D
#define REG_CLK_B_STATUS	0x004E
#define REG_CLK_B_RATE		0x004F
#define REG_DRIFT_CLK_B_REG	0x0052
#define REG_HDCP_CTRL		0x0060
#define REG_HDCP_KDS		0x0061
#define REG_HDCP_BCAPS		0x0063
#define REG_HDCP_KEY_CTRL	0x0064
#define REG_INFO_CTRL		0x0076
#define REG_INFO_EXCEED		0x0077
#define REG_PIX_REPEAT		0x007B
#define REG_AUDIO_PATH		0x007C
#define REG_AUDCFG		0x007D
#define REG_AUDIO_OUT_ENABLE	0x007E
#define REG_AUDIO_OUT_HIZ	0x007F
#define REG_VDP_CTRL		0x0080
#define REG_VDP_MATRIX		0x0081
#define REG_VHREF_CTRL		0x00A0
#define REG_PXCNT_PR		0x00A2
#define REG_PXCNT_NPIX		0x00A4
#define REG_LCNT_PR		0x00A6
#define REG_LCNT_NLIN		0x00A8
#define REG_HREF_S		0x00AA
#define REG_HREF_E		0x00AC
#define REG_HS_S		0x00AE
#define REG_HS_E		0x00B0
#define REG_VREF_F1_S		0x00B2
#define REG_VREF_F1_WIDTH	0x00B4
#define REG_VREF_F2_S		0x00B5
#define REG_VREF_F2_WIDTH	0x00B7
#define REG_VS_F1_LINE_S	0x00B8
#define REG_VS_F1_LINE_WIDTH	0x00BA
#define REG_VS_F2_LINE_S	0x00BB
#define REG_VS_F2_LINE_WIDTH	0x00BD
#define REG_VS_F1_PIX_S		0x00BE
#define REG_VS_F1_PIX_E		0x00C0
#define REG_VS_F2_PIX_S		0x00C2
#define REG_VS_F2_PIX_E		0x00C4
#define REG_FREF_F1_S		0x00C6
#define REG_FREF_F2_S		0x00C8
#define REG_FDW_S		0x00ca
#define REG_FDW_E		0x00cc
#define REG_BLK_GY		0x00da
#define REG_BLK_BU		0x00dc
#define REG_BLK_RV		0x00de
#define REG_FILTERS_CTRL	0x00e0
#define REG_DITHERING_CTRL	0x00E9
#define REG_OF			0x00EA
#define REG_PCLK		0x00EB
#define REG_HS_HREF		0x00EC
#define REG_VS_VREF		0x00ED
#define REG_DE_FREF		0x00EE
#define REG_VP35_32_CTRL	0x00EF
#define REG_VP31_28_CTRL	0x00F0
#define REG_VP27_24_CTRL	0x00F1
#define REG_VP23_20_CTRL	0x00F2
#define REG_VP19_16_CTRL	0x00F3
#define REG_VP15_12_CTRL	0x00F4
#define REG_VP11_08_CTRL	0x00F5
#define REG_VP07_04_CTRL	0x00F6
#define REG_VP03_00_CTRL	0x00F7
#define REG_CURPAGE_00H		0xFF

#define MASK_VPER		0x3fffff
#define MASK_VHREF		0x3fff
#define MASK_HPER		0x0fff
#define MASK_HSWIDTH		0x03ff

/* Input Select */
#define INPUT_SEL_RST_FMT	BIT(7)	/* 1=reset format measurement */
#define INPUT_SEL_RST_VDP	BIT(2)	/* 1=reset video data path */
#define INPUT_SEL_OUT_MODE	BIT(1)	/* 0=loop 1=bypass */
#define INPUT_SEL_B		BIT(0)	/* 0=inputA 1=inputB */

/* Service Mode */
#define SVC_MODE_CLK2_MASK	0xc0
#define SVC_MODE_CLK2_SHIFT	6
#define SVC_MODE_CLK2_XTL	0L
#define SVC_MODE_CLK2_XTLDIV2	1L
#define SVC_MODE_CLK2_HDMIX2	3L
#define SVC_MODE_CLK1_MASK	0x30
#define SVC_MODE_CLK1_SHIFT	4
#define SVC_MODE_CLK1_XTAL	0L
#define SVC_MODE_CLK1_XTLDIV2	1L
#define SVC_MODE_CLK1_HDMI	3L
#define SVC_MODE_RAMP		BIT(3)	/* 0=colorbar 1=ramp */
#define SVC_MODE_PAL		BIT(2)	/* 0=NTSC(480i/p) 1=PAL(576i/p) */
#define SVC_MODE_INT_PROG	BIT(1)	/* 0=interlaced 1=progressive */
#define SVC_MODE_SM_ON		BIT(0)	/* Enable color bars and tone gen */

/* HDP Manual Control */
#define HPD_MAN_CTRL_HPD_PULSE	BIT(7)	/* HPD Pulse low 110ms */
#define HPD_MAN_CTRL_5VEN	BIT(2)	/* Output 5V */
#define HPD_MAN_CTRL_HPD_B	BIT(1)	/* Assert HPD High for Input A */
#define HPD_MAN_CTRL_HPD_A	BIT(0)	/* Assert HPD High for Input A */

/* RT_MAN_CTRL */
#define RT_MAN_CTRL_RT_AUTO	BIT(7)
#define RT_MAN_CTRL_RT		BIT(6)
#define RT_MAN_CTRL_RT_B	BIT(1)	/* enable TMDS pull-up on Input B */
#define RT_MAN_CTRL_RT_A	BIT(0)	/* enable TMDS pull-up on Input A */

/* VDP_CTRL */
#define VDP_CTRL_COMPDEL_BP	BIT(5)	/* bypass compdel */
#define VDP_CTRL_FORMATTER_BP	BIT(4)	/* bypass formatter */
#define VDP_CTRL_PREFILTER_BP	BIT(1)	/* bypass prefilter */
#define VDP_CTRL_MATRIX_BP	BIT(0)	/* bypass matrix conversion */

/* REG_VHREF_CTRL */
#define VHREF_INT_DET		BIT(7)	/* interlace detect: 1=alt 0=frame */
#define VHREF_VSYNC_MASK	0x60
#define VHREF_VSYNC_SHIFT	6
#define VHREF_VSYNC_AUTO	0L
#define VHREF_VSYNC_FDW		1L
#define VHREF_VSYNC_EVEN	2L
#define VHREF_VSYNC_ODD		3L
#define VHREF_STD_DET_MASK	0x18
#define VHREF_STD_DET_SHIFT	3
#define VHREF_STD_DET_PAL	0L
#define VHREF_STD_DET_NTSC	1L
#define VHREF_STD_DET_AUTO	2L
#define VHREF_STD_DET_OFF	3L
#define VHREF_VREF_SRC_STD	BIT(2)	/* 1=from standard 0=manual */
#define VHREF_HREF_SRC_STD	BIT(1)	/* 1=from standard 0=manual */
#define VHREF_HSYNC_SEL_HS	BIT(0)	/* 1=HS 0=VS */

/* AUDIO_OUT_ENABLE */
#define AUDIO_OUT_ENABLE_ACLK	BIT(5)
#define AUDIO_OUT_ENABLE_WS	BIT(4)
#define AUDIO_OUT_ENABLE_AP3	BIT(3)
#define AUDIO_OUT_ENABLE_AP2	BIT(2)
#define AUDIO_OUT_ENABLE_AP1	BIT(1)
#define AUDIO_OUT_ENABLE_AP0	BIT(0)

/* Prefilter Control */
#define FILTERS_CTRL_BU_MASK	0x0c
#define FILTERS_CTRL_BU_SHIFT	2
#define FILTERS_CTRL_RV_MASK	0x03
#define FILTERS_CTRL_RV_SHIFT	0
#define FILTERS_CTRL_OFF	0L	/* off */
#define FILTERS_CTRL_2TAP	1L	/* 2 Taps */
#define FILTERS_CTRL_7TAP	2L	/* 7 Taps */
#define FILTERS_CTRL_2_7TAP	3L	/* 2/7 Taps */

/* PCLK Configuration */
#define PCLK_DELAY_MASK		0x70
#define PCLK_DELAY_SHIFT	4	/* Pixel delay (-8..+7) */
#define PCLK_INV_SHIFT		2
#define PCLK_SEL_MASK		0x03	/* clock scaler */
#define PCLK_SEL_SHIFT		0
#define PCLK_SEL_X1		0L
#define PCLK_SEL_X2		1L
#define PCLK_SEL_DIV2		2L
#define PCLK_SEL_DIV4		3L

/* Pixel Repeater */
#define PIX_REPEAT_MASK_UP_SEL	0x30
#define PIX_REPEAT_MASK_REP	0x0f
#define PIX_REPEAT_SHIFT	4
#define PIX_REPEAT_CHROMA	1

/* Page 0x01 - HDMI info and packets */
#define REG_HDMI_FLAGS		0x0100
#define REG_DEEP_COLOR_MODE	0x0101
#define REG_AUDIO_FLAGS		0x0108
#define REG_AUDIO_FREQ		0x0109
#define REG_ACP_PACKET_TYPE	0x0141
#define REG_ISRC1_PACKET_TYPE	0x0161
#define REG_ISRC2_PACKET_TYPE	0x0181
#define REG_GBD_PACKET_TYPE	0x01a1

/* HDMI_FLAGS */
#define HDMI_FLAGS_AUDIO	BIT(7)	/* Audio packet in last videoframe */
#define HDMI_FLAGS_HDMI		BIT(6)	/* HDMI detected */
#define HDMI_FLAGS_EESS		BIT(5)	/* EESS detected */
#define HDMI_FLAGS_HDCP		BIT(4)	/* HDCP detected */
#define HDMI_FLAGS_AVMUTE	BIT(3)	/* AVMUTE */
#define HDMI_FLAGS_AUD_LAYOUT	BIT(2)	/* Layout status Audio sample packet */
#define HDMI_FLAGS_AUD_FIFO_OF	BIT(1)	/* FIFO read/write pointers crossed */
#define HDMI_FLAGS_AUD_FIFO_LOW	BIT(0)	/* FIFO read ptr within 2 of write */

/* Page 0x12 - HDMI Extra control and debug */
#define REG_CLK_CFG		0x1200
#define REG_CLK_OUT_CFG		0x1201
#define REG_CFG1		0x1202
#define REG_CFG2		0x1203
#define REG_WDL_CFG		0x1210
#define REG_DELOCK_DELAY	0x1212
#define REG_PON_OVR_EN		0x12A0
#define REG_PON_CBIAS		0x12A1
#define REG_PON_RESCAL		0x12A2
#define REG_PON_RES		0x12A3
#define REG_PON_CLK		0x12A4
#define REG_PON_PLL		0x12A5
#define REG_PON_EQ		0x12A6
#define REG_PON_DES		0x12A7
#define REG_PON_OUT		0x12A8
#define REG_PON_MUX		0x12A9
#define REG_MODE_REC_CFG1	0x12F8
#define REG_MODE_REC_CFG2	0x12F9
#define REG_MODE_REC_STS	0x12FA
#define REG_AUDIO_LAYOUT	0x12D0

#define PON_EN			1
#define PON_DIS			0

/* CLK CFG */
#define CLK_CFG_INV_OUT_CLK	BIT(7)
#define CLK_CFG_INV_BUS_CLK	BIT(6)
#define CLK_CFG_SEL_ACLK_EN	BIT(1)
#define CLK_CFG_SEL_ACLK	BIT(0)
#define CLK_CFG_DIS		0

/* Page 0x13 - HDMI Extra control and debug */
#define REG_DEEP_COLOR_CTRL	0x1300
#define REG_CGU_DBG_SEL		0x1305
#define REG_HDCP_DDC_ADDR	0x1310
#define REG_HDCP_KIDX		0x1316
#define REG_DEEP_PLL7_BYP	0x1347
#define REG_HDCP_DE_CTRL	0x1370
#define REG_HDCP_EP_FILT_CTRL	0x1371
#define REG_HDMI_CTRL		0x1377
#define REG_HMTP_CTRL		0x137a
#define REG_TIMER_D		0x13CF
#define REG_SUS_SET_RGB0	0x13E1
#define REG_SUS_SET_RGB1	0x13E2
#define REG_SUS_SET_RGB2	0x13E3
#define REG_SUS_SET_RGB3	0x13E4
#define REG_SUS_SET_RGB4	0x13E5
#define REG_MAN_SUS_HDMI_SEL	0x13E8
#define REG_MAN_HDMI_SET	0x13E9
#define REG_SUS_CLOCK_GOOD	0x13EF

/* HDCP DE Control */
#define HDCP_DE_MODE_MASK	0xc0	/* DE Measurement mode */
#define HDCP_DE_MODE_SHIFT	6
#define HDCP_DE_REGEN_EN	BIT(5)	/* enable regen mode */
#define HDCP_DE_FILTER_MASK	0x18	/* DE filter sensitivity */
#define HDCP_DE_FILTER_SHIFT	3
#define HDCP_DE_COMP_MASK	0x07	/* DE Composition mode */
#define HDCP_DE_COMP_MIXED	6L
#define HDCP_DE_COMP_OR		5L
#define HDCP_DE_COMP_AND	4L
#define HDCP_DE_COMP_CH3	3L
#define HDCP_DE_COMP_CH2	2L
#define HDCP_DE_COMP_CH1	1L
#define HDCP_DE_COMP_CH0	0L

/* HDCP EP Filter Control */
#define HDCP_EP_FIL_CTL_MASK	0x30
#define HDCP_EP_FIL_CTL_SHIFT	4
#define HDCP_EP_FIL_VS_MASK	0x0c
#define HDCP_EP_FIL_VS_SHIFT	2
#define HDCP_EP_FIL_HS_MASK	0x03
#define HDCP_EP_FIL_HS_SHIFT	0

/* HDMI_CTRL */
#define HDMI_CTRL_MUTE_MASK	0x0c
#define HDMI_CTRL_MUTE_SHIFT	2
#define HDMI_CTRL_MUTE_AUTO	0L
#define HDMI_CTRL_MUTE_OFF	1L
#define HDMI_CTRL_MUTE_ON	2L
#define HDMI_CTRL_HDCP_MASK	0x03
#define HDMI_CTRL_HDCP_SHIFT	0
#define HDMI_CTRL_HDCP_EESS	2L
#define HDMI_CTRL_HDCP_OESS	1L
#define HDMI_CTRL_HDCP_AUTO	0L

/* CGU_DBG_SEL bits */
#define CGU_DBG_CLK_SEL_MASK	0x18
#define CGU_DBG_CLK_SEL_SHIFT	3
#define CGU_DBG_XO_FRO_SEL	BIT(2)
#define CGU_DBG_VDP_CLK_SEL	BIT(1)
#define CGU_DBG_PIX_CLK_SEL	BIT(0)

/* REG_MAN_SUS_HDMI_SEL / REG_MAN_HDMI_SET bits */
#define MAN_DIS_OUT_BUF		BIT(7)
#define MAN_DIS_ANA_PATH	BIT(6)
#define MAN_DIS_HDCP		BIT(5)
#define MAN_DIS_TMDS_ENC	BIT(4)
#define MAN_DIS_TMDS_FLOW	BIT(3)
#define MAN_RST_HDCP		BIT(2)
#define MAN_RST_TMDS_ENC	BIT(1)
#define MAN_RST_TMDS_FLOW	BIT(0)

/* Page 0x14 - Audio Extra control and debug */
#define REG_FIFO_LATENCY_VAL	0x1403
#define REG_AUDIO_CLOCK		0x1411
#define REG_TEST_NCTS_CTRL	0x1415
#define REG_TEST_AUDIO_FREQ	0x1426
#define REG_TEST_MODE		0x1437

/* Audio Clock Configuration */
#define AUDIO_CLOCK_PLL_PD	BIT(7)	/* powerdown PLL */
#define AUDIO_CLOCK_SEL_MASK	0x7f
#define AUDIO_CLOCK_SEL_16FS	0L	/* 16*fs */
#define AUDIO_CLOCK_SEL_32FS	1L	/* 32*fs */
#define AUDIO_CLOCK_SEL_64FS	2L	/* 64*fs */
#define AUDIO_CLOCK_SEL_128FS	3L	/* 128*fs */
#define AUDIO_CLOCK_SEL_256FS	4L	/* 256*fs */
#define AUDIO_CLOCK_SEL_512FS	5L	/* 512*fs */

/* Page 0x20: EDID and Hotplug Detect */
#define REG_EDID_IN_BYTE0	0x2000 /* EDID base */
#define REG_EDID_IN_VERSION	0x2080
#define REG_EDID_ENABLE		0x2081
#define REG_HPD_POWER		0x2084
#define REG_HPD_AUTO_CTRL	0x2085
#define REG_HPD_DURATION	0x2086
#define REG_RX_HPD_HEAC		0x2087

/* EDID_ENABLE */
#define EDID_ENABLE_NACK_OFF	BIT(7)
#define EDID_ENABLE_EDID_ONLY	BIT(6)
#define EDID_ENABLE_B_EN	BIT(1)
#define EDID_ENABLE_A_EN	BIT(0)

/* HPD Power */
#define HPD_POWER_BP_MASK	0x0c
#define HPD_POWER_BP_SHIFT	2
#define HPD_POWER_BP_LOW	0L
#define HPD_POWER_BP_HIGH	1L
#define HPD_POWER_EDID_ONLY	BIT(1)

/* HPD Auto control */
#define HPD_AUTO_READ_EDID	BIT(7)
#define HPD_AUTO_HPD_F3TECH	BIT(5)
#define HPD_AUTO_HP_OTHER	BIT(4)
#define HPD_AUTO_HPD_UNSEL	BIT(3)
#define HPD_AUTO_HPD_ALL_CH	BIT(2)
#define HPD_AUTO_HPD_PRV_CH	BIT(1)
#define HPD_AUTO_HPD_NEW_CH	BIT(0)

/* Page 0x21 - EDID content */
#define REG_EDID_IN_BYTE128	0x2100 /* CEA Extension block */
#define REG_EDID_IN_SPA_SUB	0x2180
#define REG_EDID_IN_SPA_AB_A	0x2181
#define REG_EDID_IN_SPA_CD_A	0x2182
#define REG_EDID_IN_CKSUM_A	0x2183
#define REG_EDID_IN_SPA_AB_B	0x2184
#define REG_EDID_IN_SPA_CD_B	0x2185
#define REG_EDID_IN_CKSUM_B	0x2186

/* Page 0x30 - NV Configuration */
#define REG_RT_AUTO_CTRL	0x3000
#define REG_EQ_MAN_CTRL0	0x3001
#define REG_EQ_MAN_CTRL1	0x3002
#define REG_OUTPUT_CFG		0x3003
#define REG_MUTE_CTRL		0x3004
#define REG_SLAVE_ADDR		0x3005
#define REG_CMTP_REG6		0x3006
#define REG_CMTP_REG7		0x3007
#define REG_CMTP_REG8		0x3008
#define REG_CMTP_REG9		0x3009
#define REG_CMTP_REGA		0x300A
#define REG_CMTP_REGB		0x300B
#define REG_CMTP_REGC		0x300C
#define REG_CMTP_REGD		0x300D
#define REG_CMTP_REGE		0x300E
#define REG_CMTP_REGF		0x300F
#define REG_CMTP_REG10		0x3010
#define REG_CMTP_REG11		0x3011

/* Page 0x80 - CEC */
#define REG_PWR_CONTROL		0x80F4
#define REG_OSC_DIVIDER		0x80F5
#define REG_EN_OSC_PERIOD_LSB	0x80F8
#define REG_CONTROL		0x80FF

/* global interrupt flags (INT_FLG_CRL_TOP) */
#define INTERRUPT_AFE		BIT(7) /* AFE module */
#define INTERRUPT_HDCP		BIT(6) /* HDCP module */
#define INTERRUPT_AUDIO		BIT(5) /* Audio module */
#define INTERRUPT_INFO		BIT(4) /* Infoframe module */
#define INTERRUPT_MODE		BIT(3) /* HDMI mode module */
#define INTERRUPT_RATE		BIT(2) /* rate module */
#define INTERRUPT_DDC		BIT(1) /* DDC module */
#define INTERRUPT_SUS		BIT(0) /* SUS module */

/* INT_FLG_CLR_HDCP bits */
#define MASK_HDCP_MTP		BIT(7) /* HDCP MTP busy */
#define MASK_HDCP_DLMTP		BIT(4) /* HDCP end download MTP to SRAM */
#define MASK_HDCP_DLRAM		BIT(3) /* HDCP end download keys from SRAM */
#define MASK_HDCP_ENC		BIT(2) /* HDCP ENC */
#define MASK_STATE_C5		BIT(1) /* HDCP State C5 reached */
#define MASK_AKSV		BIT(0) /* AKSV received (start of auth) */

/* INT_FLG_CLR_RATE bits */
#define MASK_RATE_B_DRIFT	BIT(7) /* Rate measurement drifted */
#define MASK_RATE_B_ST		BIT(6) /* Rate measurement stability change */
#define MASK_RATE_B_ACT		BIT(5) /* Rate measurement activity change */
#define MASK_RATE_B_PST		BIT(4) /* Rate measreument presence change */
#define MASK_RATE_A_DRIFT	BIT(3) /* Rate measurement drifted */
#define MASK_RATE_A_ST		BIT(2) /* Rate measurement stability change */
#define MASK_RATE_A_ACT		BIT(1) /* Rate measurement presence change */
#define MASK_RATE_A_PST		BIT(0) /* Rate measreument presence change */

/* INT_FLG_CLR_SUS (Start Up Sequencer) bits */
#define MASK_MPT		BIT(7) /* Config MTP end of process */
#define MASK_FMT		BIT(5) /* Video format changed */
#define MASK_RT_PULSE		BIT(4) /* End of termination resistance pulse */
#define MASK_SUS_END		BIT(3) /* SUS last state reached */
#define MASK_SUS_ACT		BIT(2) /* Activity of selected input changed */
#define MASK_SUS_CH		BIT(1) /* Selected input changed */
#define MASK_SUS_ST		BIT(0) /* SUS state changed */

/* INT_FLG_CLR_DDC bits */
#define MASK_EDID_MTP		BIT(7) /* EDID MTP end of process */
#define MASK_DDC_ERR		BIT(6) /* master DDC error */
#define MASK_DDC_CMD_DONE	BIT(5) /* master DDC cmd send correct */
#define MASK_READ_DONE		BIT(4) /* End of down EDID read */
#define MASK_RX_DDC_SW		BIT(3) /* Output DDC switching finished */
#define MASK_HDCP_DDC_SW	BIT(2) /* HDCP DDC switching finished */
#define MASK_HDP_PULSE_END	BIT(1) /* End of Hot Plug Detect pulse */
#define MASK_DET_5V		BIT(0) /* Detection of +5V */

/* INT_FLG_CLR_MODE bits */
#define MASK_HDMI_FLG		BIT(7) /* HDMI mode/avmute/encrypt/FIFO fail */
#define MASK_GAMUT		BIT(6) /* Gamut packet */
#define MASK_ISRC2		BIT(5) /* ISRC2 packet */
#define MASK_ISRC1		BIT(4) /* ISRC1 packet */
#define MASK_ACP		BIT(3) /* Audio Content Protection packet */
#define MASK_DC_NO_GCP		BIT(2) /* GCP not received in 5 frames */
#define MASK_DC_PHASE		BIT(1) /* deepcolor pixel phase needs update */
#define MASK_DC_MODE		BIT(0) /* deepcolor color depth changed */

/* INT_FLG_CLR_INFO bits (Infoframe Change Status) */
#define MASK_MPS_IF		BIT(6) /* MPEG Source Product */
#define MASK_AUD_IF		BIT(5) /* Audio */
#define MASK_SPD_IF		BIT(4) /* Source Product Descriptor */
#define MASK_AVI_IF		BIT(3) /* Auxiliary Video IF */
#define MASK_VS_IF_OTHER_BK2	BIT(2) /* Vendor Specific (bank2) */
#define MASK_VS_IF_OTHER_BK1	BIT(1) /* Vendor Specific (bank1) */
#define MASK_VS_IF_HDMI		BIT(0) /* Vendor Specific (w/ HDMI LLC code) */

/* INT_FLG_CLR_AUDIO bits */
#define MASK_AUDIO_FREQ_FLG	BIT(5) /* Audio freq change */
#define MASK_AUDIO_FLG		BIT(4) /* DST, OBA, HBR, ASP change */
#define MASK_MUTE_FLG		BIT(3) /* Audio Mute */
#define MASK_CH_STATE		BIT(2) /* Channel status */
#define MASK_UNMUTE_FIFO	BIT(1) /* Audio Unmute */
#define MASK_ERROR_FIFO_PT	BIT(0) /* Audio FIFO pointer error */

/* INT_FLG_CLR_AFE bits */
#define MASK_AFE_WDL_UNLOCKED	BIT(7) /* Wordlocker was unlocked */
#define MASK_AFE_GAIN_DONE	BIT(6) /* Gain calibration done */
#define MASK_AFE_OFFSET_DONE	BIT(5) /* Offset calibration done */
#define MASK_AFE_ACTIVITY_DET	BIT(4) /* Activity detected on data */
#define MASK_AFE_PLL_LOCK	BIT(3) /* TMDS PLL is locked */
#define MASK_AFE_TRMCAL_DONE	BIT(2) /* Termination calibration done */
#define MASK_AFE_ASU_STATE	BIT(1) /* ASU state is reached */
#define MASK_AFE_ASU_READY	BIT(0) /* AFE calibration done: TMDS ready */

/* Audio Output */
#define AUDCFG_CLK_INVERT	BIT(7)	/* invert A_CLK polarity */
#define AUDCFG_TEST_TONE	BIT(6)	/* enable test tone generator */
#define AUDCFG_BUS_SHIFT	5
#define AUDCFG_BUS_I2S		0L
#define AUDCFG_BUS_SPDIF	1L
#define AUDCFG_I2SW_SHIFT	4
#define AUDCFG_I2SW_16		0L
#define AUDCFG_I2SW_32		1L
#define AUDCFG_AUTO_MUTE_EN	BIT(3)	/* Enable Automatic audio mute */
#define AUDCFG_HBR_SHIFT	2
#define AUDCFG_HBR_STRAIGHT	0L	/* straight via AP0 */
#define AUDCFG_HBR_DEMUX	1L	/* demuxed via AP0:AP3 */
#define AUDCFG_TYPE_MASK	0x03
#define AUDCFG_TYPE_SHIFT	0
#define AUDCFG_TYPE_DST		3L	/* Direct Stream Transfer (DST) */
#define AUDCFG_TYPE_OBA		2L	/* One Bit Audio (OBA) */
#define AUDCFG_TYPE_HBR		1L	/* High Bit Rate (HBR) */
#define AUDCFG_TYPE_SAMPLES	0L	/* Audio samples */

/* Video Formatter */
#define OF_VP_ENABLE		BIT(7)	/* VP[35:0]/HS/VS/DE/CLK */
#define OF_BLK			BIT(4)	/* blanking codes */
#define OF_TRC			BIT(3)	/* timing codes (SAV/EAV) */
#define OF_FMT_MASK		0x3
#define OF_FMT_444		0L	/* RGB444/YUV444 */
#define OF_FMT_422_SMPT		1L	/* YUV422 semi-planar */
#define OF_FMT_422_CCIR		2L	/* YUV422 CCIR656 */

/* HS/HREF output control */
#define HS_HREF_DELAY_MASK	0xf0
#define HS_HREF_DELAY_SHIFT	4	/* Pixel delay (-8..+7) */
#define HS_HREF_PXQ_SHIFT	3	/* Timing codes from HREF */
#define HS_HREF_INV_SHIFT	2	/* polarity (1=invert) */
#define HS_HREF_SEL_MASK	0x03
#define HS_HREF_SEL_SHIFT	0
#define HS_HREF_SEL_HS_VHREF	0L	/* HS from VHREF */
#define HS_HREF_SEL_HREF_VHREF	1L	/* HREF from VHREF */
#define HS_HREF_SEL_HREF_HDMI	2L	/* HREF from HDMI */
#define HS_HREF_SEL_NONE	3L	/* not generated */

/* VS output control */
#define VS_VREF_DELAY_MASK	0xf0
#define VS_VREF_DELAY_SHIFT	4	/* Pixel delay (-8..+7) */
#define VS_VREF_INV_SHIFT	2	/* polarity (1=invert) */
#define VS_VREF_SEL_MASK	0x03
#define VS_VREF_SEL_SHIFT	0
#define VS_VREF_SEL_VS_VHREF	0L	/* VS from VHREF */
#define VS_VREF_SEL_VREF_VHREF	1L	/* VREF from VHREF */
#define VS_VREF_SEL_VREF_HDMI	2L	/* VREF from HDMI */
#define VS_VREF_SEL_NONE	3L	/* not generated */

/* DE/FREF output control */
#define DE_FREF_DELAY_MASK	0xf0
#define DE_FREF_DELAY_SHIFT	4	/* Pixel delay (-8..+7) */
#define DE_FREF_DE_PXQ_SHIFT	3	/* Timing codes from DE */
#define DE_FREF_INV_SHIFT	2	/* polarity (1=invert) */
#define DE_FREF_SEL_MASK	0x03
#define DE_FREF_SEL_SHIFT	0
#define DE_FREF_SEL_DE_VHREF	0L	/* DE from VHREF (HREF and not(VREF) */
#define DE_FREF_SEL_FREF_VHREF	1L	/* FREF from VHREF */
#define DE_FREF_SEL_FREF_HDMI	2L	/* FREF from HDMI */
#define DE_FREF_SEL_NONE	3L	/* not generated */

/* HDMI_SOFT_RST bits */
#define RESET_DC		BIT(7)	/* Reset deep color module */
#define RESET_HDCP		BIT(6)	/* Reset HDCP module */
#define RESET_KSV		BIT(5)	/* Reset KSV-FIFO */
#define RESET_SCFG		BIT(4)	/* Reset HDCP and repeater function */
#define RESET_HCFG		BIT(3)	/* Reset HDCP DDC part */
#define RESET_PA		BIT(2)	/* Reset polarity adjust */
#define RESET_EP		BIT(1)	/* Reset Error protection */
#define RESET_TMDS		BIT(0)	/* Reset TMDS (calib, encoding, flow) */

/* HDMI_INFO_RST bits */
#define NACK_HDCP		BIT(7)	/* No ACK on HDCP request */
#define RESET_FIFO		BIT(4)	/* Reset Audio FIFO control */
#define RESET_GAMUT		BIT(3)	/* Clear Gamut packet */
#define RESET_AI		BIT(2)	/* Clear ACP and ISRC packets */
#define RESET_IF		BIT(1)	/* Clear all Audio infoframe packets */
#define RESET_AUDIO		BIT(0)	/* Reset Audio FIFO control */

/* HDCP_BCAPS bits */
#define HDCP_HDMI		BIT(7)	/* HDCP suports HDMI (vs DVI only) */
#define HDCP_REPEATER		BIT(6)	/* HDCP supports repeater function */
#define HDCP_READY		BIT(5)	/* set by repeater function */
#define HDCP_FAST		BIT(4)	/* Up to 400kHz */
#define HDCP_11			BIT(1)	/* HDCP 1.1 supported */
#define HDCP_FAST_REAUTH	BIT(0)	/* fast reauthentication supported */

/* Audio output formatter */
#define AUDIO_LAYOUT_SP_FLAG	BIT(2)	/* sp flag used by FIFO */
#define AUDIO_LAYOUT_MANUAL	BIT(1)	/* manual layout (vs per pkt) */

/* masks for interrupt status registers */
#define MASK_SUS_STATUS		0x1F
#define LAST_STATE_REACHED	0x1B
#define MASK_CLK_STABLE		0x04
#define MASK_CLK_ACTIVE		0x02
#define MASK_SUS_STATE		0x10
#define MASK_SR_FIFO_FIFO_CTRL	0x30
#define MASK_AUDIO_FLAG		0x10

/* Rate measurement */
#define RATE_REFTIM_ENABLE	0x01
#define CLK_MIN_RATE		0x0057e4
#define CLK_MAX_RATE		0x0395f8
#define WDL_CFG_VAL		0x82
#define DC_FILTER_VAL		0x31

/* Infoframe */
#define VS_HDMI_IF_UPDATE	0x0200
#define VS_HDMI_IF		0x0201
#define VS_BK1_IF_UPDATE	0x0220
#define VS_BK1_IF		0x0221
#define VS_BK2_IF_UPDATE	0x0240
#define VS_BK2_IF		0x0241
#define AVI_IF_UPDATE		0x0260
#define AVI_IF			0x0261
#define SPD_IF_UPDATE		0x0280
#define SPD_IF			0x0281
#define AUD_IF_UPDATE		0x02a0
#define AUD_IF			0x02a1
#define MPS_IF_UPDATE		0x02c0
#define MPS_IF			0x02c1

/* Video output port formats */
static const char * const vidfmt_names[] = {
	"RGB444/YUV444",	/* RGB/YUV444 16bit data bus, 8bpp */
	"YUV422 semi-planar",	/* YUV422 16bit data base, 8bpp */
	"YUV422 CCIR656",	/* BT656 (YUV 8bpp 2 clock per pixel) */
	"invalid",
};

/* Audio output port formats */
static const char * const audfmt_names[] = {
	"disabled",
	"I2S16",
	"I2S32",
	"SPDIF",
	"OBA",
	"DST",
	"I2S16_HBR",
	"I2S16_HBR_DEMUX",
	"I2S32_HBR_DEMUX",
	"SPDIF_HBR_DEMUX",
};

/*
 * Video Output formats
 * There are 24 video output pins on TDA19971 and 36 on TDA19973 supporting
 * the following output formats:
 * - RGB444
 * - YUV444
 * - YUV422 semi-planar based on ITU-R BT.601
 * - YUV422 ITU-R BT.656
 *
 * TDA19971 can output 3x8bits per pixel
 * TDA19973 can output 3x8, 3x10, or 3x12bit per pixel
 *
 * Deep color modes (3x10 or 3x12 bits) are possible in any case.
 *
 * Reference: NXP AN1206 - TDA19971_TDA19973 receiver HW recommendation: 3.3.4
 */
static u32 tda19971_video_formats[] = {
	/* 24bit RGB444: 1 pixel in 1x24bit sample: VP[23:0] */
	MEDIA_BUS_FMT_RGB888_1X24,
	/* 24bit YUV444: 1 pixel in 1x24bit sample: VP[23:0] */
	MEDIA_BUS_FMT_YUV8_1X24,
	/* 24bit YUV422: 1 pixel in 1x24bit sample: VP[23:12]/VP[11:0] */
	MEDIA_BUS_FMT_UYVY12_1X24,
	/* 20bit YUV422: 1 pixel in 1x20bit sample: VP[23:14]/VP[11:2] */
	MEDIA_BUS_FMT_UYVY10_1X20,
	/* 16bit YUV422: 1 pixel in 1x16bit sample: VP[23:16]/VP[15:8] */
	MEDIA_BUS_FMT_UYVY8_1X16,
	/* 12bit CCIR656: 1 pixel in 2x12bit samples: VP[23:12] */
	MEDIA_BUS_FMT_UYVY12_2X12,
	/* 10bit CCIR656: 1 pixel in 2x10bit samples: VP[23:14] */
	MEDIA_BUS_FMT_UYVY10_2X10,
	/* 8bit CCIR656: 1 pixel in 2x8bit samples: VP[23:16] */
	MEDIA_BUS_FMT_UYVY8_2X8,
};
static u32 tda19973_video_formats[] = {
	/* 36bit RGB444: 1 pixel in 1x36bit sample on VP[35:0] */
	MEDIA_BUS_FMT_RGB121212_1X36,
	/* 36bit YUV444 1 pixel in 1x36bit sample on VP[35:0] */
	MEDIA_BUS_FMT_YUV12_1X36,
	/* 24bit YUV422: 1 pixel in 1x24bit sample on VP[35:24]/VP[11:0]  */
	MEDIA_BUS_FMT_UYVY12_1X24,
	/* 12bit CCIR656: 1 pixel in 2x12bit samples on VP[11:0] */
	MEDIA_BUS_FMT_UYVY12_2X12,
};

/*
 * Video Input formats
 */
struct vhref_values {
	u16 href_start;
	u16 href_end;
	u16 vref_f1_start;
	u8  vref_f1_width;
	u16 vref_f2_start;
	u8  vref_f2_width;
	u16 fieldref_f1_start;
	u8  fieldPolarity;
	u16 fieldref_f2_start;
};

struct tda1997x_video_std {
	const struct v4l2_dv_timings timings;
	const struct vhref_values vhref_values;
};

static const struct tda1997x_video_std tda1997x_hdmi_modes[] = {
	/* Low TV */
	{ V4L2_DV_BT_CEA_1280X720P24,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1280X720P25,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1280X720P30,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1920X1080P24,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1920X1080P25,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1920X1080P30,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},

	/* 60 Hz TV */
	{ V4L2_DV_BT_CEA_720X480P59_94,
		{123, 843, 516, 45, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1280X720P60,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1920X1080I60,
		{193, 2113, 1123, 22, 560, 23, 1, 0, 563}
	},
	{ V4L2_DV_BT_CEA_720X480I59_94,
		{120, 840, 521, 22, 258, 23, 1, 0, 263}
	},
	{ V4L2_DV_BT_CEA_1920X1080P60,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},

	/* 50 Hz TV */
	{ V4L2_DV_BT_CEA_720X576P50,
		{133, 853, 620, 49, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1280X720P50,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_CEA_1920X1080I50,
		{193, 2113, 1123, 22, 560, 23, 1, 0, 563}
	},
	{ V4L2_DV_BT_CEA_720X576I50,
		{133, 853, 623, 24, 310, 25, 1, 0, 313 }
	},
	{ V4L2_DV_BT_CEA_1920X1080P50,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},

	/* 60 Hz PC */
	{ V4L2_DV_BT_DMT_640X480P60,
		{145, 785, 515, 45, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_DMT_800X600P60,
		{217, 1017, 627, 28, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1024X768P60,
		{297, 1321, 803, 38, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X768P60,
		{321, 1601, 795, 30, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X960P60,
		{425, 1705, 999, 40, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X1024P60,
		{361, 1641, 1065, 42, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1440X900P60,
		{385, 1825, 931, 34, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1600X1200P60,
		{497, 2097, 1249, 50, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1680X1050P60_RB,
		{113, 1793, 1077, 30, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1920X1200P60_RB,
		{113, 2033, 1232, 35, 0, 0, 0, 0, 0}
	},

	/* 75 HZ PC */
	{ V4L2_DV_BT_DMT_640X480P75,
		{185, 825, 499, 20, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_DMT_800X600P75,
		{241, 1041, 624, 25, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1024X768P75,
		{273, 1297, 799, 32, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X768P75,
		{337, 1617, 802, 37, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X1024P75,
		{393, 1673, 1065, 42, 0, 0, 0, 0, 0}
	},

	/* 85 HZ PC */
	{ V4L2_DV_BT_DMT_640X480P85,
		{137, 777, 508, 29, 0, 0, 1, 0, 0}
	},
	{ V4L2_DV_BT_DMT_800X600P85,
		{217, 1017, 630, 31, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1024X768P85,
		{305, 1329, 807, 40, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X768P85,
		{353, 1633, 905, 140, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1280X1024P85,
		{385, 1665, 1071, 48, 0, 0, 0, 0, 0}
	},

	{ V4L2_DV_BT_DMT_1360X768P60,
		{369, 1729, 792, 27, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1400X1050P60,
		{377, 1777, 1086, 39, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1400X1050P60_RB,
		{113, 1513, 1077, 30, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_1024X768P70,
		{281, 1305, 803, 38, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_640X480P72,
		{169, 809, 511, 40, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_800X600P72,
		{185, 985, 629, 66, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_640X350P85,
		{161, 801, 413, 95, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_640X400P85,
		{161, 801, 444, 45, 0, 0, 0, 0, 0}
	},
	{ V4L2_DV_BT_DMT_720X400P85,
		{181, 901, 445, 46, 0, 0, 0, 0, 0}
	},
};

/* regulator supplies */
static const char * const tda1997x_supply_name[] = {
	"DOVDD", /* Digital I/O supply */
	"DVDD",  /* Digital Core supply */
	"AVDD",  /* Analog supply */
};

#define TDA1997X_NUM_SUPPLIES ARRAY_SIZE(tda1997x_supply_name)

enum tda1997x_type {
	TDA19971,
	TDA19973,
};

enum tda1997x_hdmi_pads {
	TDA1997X_PAD_SOURCE,
	TDA1997X_NUM_PADS,
};

struct tda1997x_chip_info {
	enum tda1997x_type type;
	const char *name;
	const u32 *formats;
	unsigned int nformats;
};

struct tda1997x_state {
	const struct tda1997x_chip_info *info;
	struct tda1997x_platform_data pdata;
	struct i2c_client *client;
	struct i2c_client *client_cec;
	struct v4l2_subdev sd;
	struct regulator_bulk_data supplies[TDA1997X_NUM_SUPPLIES];
	struct media_pad pads[TDA1997X_NUM_PADS];
	struct mutex lock;
	struct mutex page_lock;
	char page;

	/* detected info from chip */
	int chip_revision;
	char port_30bit;
	char output_2p5;
	char tmdsb_clk;
	char tmdsb_soc;

	/* status info */
	char hdmi_status;
	char mptrw_in_progress;
	char state_c5_reached;
	char activity_status;
	char input_detect[2];
	char vendor[12];
	char product[18];

	/* video */
	enum hdmi_colorspace colorspace;
	enum hdmi_colorimetry colorimetry;
	const struct v4l2_dv_timings *timings;
	int fps;
	u32 code;
	enum v4l2_mbus_type bus_type;
	char vid_fmt;

	/* audio source */
	u8  audio_ch_alloc;
	int audio_samplerate;
	int audio_channels;
};

static const struct v4l2_event tda1997x_ev_fmt = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static const struct tda1997x_chip_info tda1997x_chip_info[] = {
	[TDA19971] = {
		.type = TDA19971,
		.name = "tda19971",
		.formats = tda19971_video_formats,
		.nformats = ARRAY_SIZE(tda19971_video_formats),
	},
	[TDA19973] = {
		.type = TDA19973,
		.name = "tda19973",
		.formats = tda19973_video_formats,
		.nformats = ARRAY_SIZE(tda19973_video_formats),
	},
};

static inline struct tda1997x_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tda1997x_state, sd);
}

static int tda1997x_cec_read(struct v4l2_subdev *sd, u8 reg)
{
	struct tda1997x_state *state = to_state(sd);
	int val;

	val = i2c_smbus_read_byte_data(state->client_cec, reg);
	if (val < 0) {
		v4l_err(state->client, "read reg error: reg=%2x\n", reg);
		val = -1;
	}

	return val;
}

static int tda1997x_cec_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct tda1997x_state *state = to_state(sd);
	int ret = 0;

	ret = i2c_smbus_write_byte_data(state->client_cec, reg, val);
	if (ret < 0) {
		v4l_err(state->client, "write reg error:reg=%2x,val=%2x\n",
			reg, val);
		ret = -1;
	}

	return ret;
}

/* -----------------------------------------------------------------------------
 * I2C transfer
 */

static int tda1997x_setpage(struct v4l2_subdev *sd, u8 page)
{
	struct tda1997x_state *state = to_state(sd);
	int ret;

	if (state->page != page) {
		ret = i2c_smbus_write_byte_data(state->client,
			REG_CURPAGE_00H, page);
		if (ret < 0) {
			v4l_err(state->client,
				"write reg error:reg=%2x,val=%2x\n",
				REG_CURPAGE_00H, page);
			return ret;
		}
		state->page = page;
	}
	return 0;
}

static inline int io_read(struct v4l2_subdev *sd, u16 reg)
{
	struct tda1997x_state *state = to_state(sd);
	int val;

	mutex_lock(&state->page_lock);
	if (tda1997x_setpage(sd, reg >> 8)) {
		val = -1;
		goto out;
	}

	val = i2c_smbus_read_byte_data(state->client, reg&0xff);
	if (val < 0) {
		v4l_err(state->client, "read reg error: reg=%2x\n", reg & 0xff);
		val = -1;
		goto out;
	}

out:
	mutex_unlock(&state->page_lock);
	return val;
}

static inline long io_read16(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;
	long lval = 0;

	val = io_read(sd, reg);
	if (val < 0)
		return val;
	lval |= (val << 8);
	val = io_read(sd, reg + 1);
	if (val < 0)
		return val;
	lval |= val;

	return lval;
}

static inline long io_read24(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;
	long lval = 0;

	val = io_read(sd, reg);
	if (val < 0)
		return val;
	lval |= (val << 16);
	val = io_read(sd, reg + 1);
	if (val < 0)
		return val;
	lval |= (val << 8);
	val = io_read(sd, reg + 2);
	if (val < 0)
		return val;
	lval |= val;

	return lval;
}

static unsigned int io_readn(struct v4l2_subdev *sd, u16 reg, u8 len, u8 *data)
{
	int i;
	int sz = 0;
	u8 val;

	for (i = 0; i < len; i++) {
		val = io_read(sd, reg + i);
		if (val < 0)
			break;
		data[i] = val;
		sz++;
	}

	return sz;
}

static int io_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	struct tda1997x_state *state = to_state(sd);
	s32 ret = 0;

	mutex_lock(&state->page_lock);
	if (tda1997x_setpage(sd, reg >> 8)) {
		ret = -1;
		goto out;
	}

	ret = i2c_smbus_write_byte_data(state->client, reg & 0xff, val);
	if (ret < 0) {
		v4l_err(state->client, "write reg error:reg=%2x,val=%2x\n",
			reg&0xff, val);
		ret = -1;
		goto out;
	}

out:
	mutex_unlock(&state->page_lock);
	return ret;
}

static int io_write16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	int ret;

	ret = io_write(sd, reg, (val >> 8) & 0xff);
	if (ret < 0)
		return ret;
	ret = io_write(sd, reg + 1, val & 0xff);
	if (ret < 0)
		return ret;
	return 0;
}

static int io_write24(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	int ret;

	ret = io_write(sd, reg, (val >> 16) & 0xff);
	if (ret < 0)
		return ret;
	ret = io_write(sd, reg + 1, (val >> 8) & 0xff);
	if (ret < 0)
		return ret;
	ret = io_write(sd, reg + 2, val & 0xff);
	if (ret < 0)
		return ret;
	return 0;
}

/* -----------------------------------------------------------------------------
 * Hotplug
 */

enum hpd_mode {
	HPD_LOW_BP,	/* HPD low and pulse of at least 100ms */
	HPD_LOW_OTHER,	/* HPD low and pulse of at least 100ms */
	HPD_HIGH_BP,	/* HIGH */
	HPD_HIGH_OTHER,
	HPD_PULSE,	/* HPD low pulse */
};

/* manual HPD (Hot Plug Detect) control */
static int tda1997x_manual_hpd(struct v4l2_subdev *sd, enum hpd_mode mode)
{
	u8 hpd_auto, hpd_pwr, hpd_man;

	hpd_auto = io_read(sd, REG_HPD_AUTO_CTRL);
	hpd_pwr = io_read(sd, REG_HPD_POWER);
	hpd_man = io_read(sd, REG_HPD_MAN_CTRL);

	/* mask out unused bits */
	hpd_man &= (HPD_MAN_CTRL_HPD_PULSE |
		    HPD_MAN_CTRL_5VEN |
		    HPD_MAN_CTRL_HPD_B |
		    HPD_MAN_CTRL_HPD_A);

	switch (mode) {
	/* HPD low and pulse of at least 100ms */
	case HPD_LOW_BP:
		/* hpd_bp=0 */
		hpd_pwr &= ~HPD_POWER_BP_MASK;
		/* disable HPD_A and HPD_B */
		hpd_man &= ~(HPD_MAN_CTRL_HPD_A | HPD_MAN_CTRL_HPD_B);
		io_write(sd, REG_HPD_POWER, hpd_pwr);
		io_write(sd, REG_HPD_MAN_CTRL, hpd_man);
		break;
	/* HPD high */
	case HPD_HIGH_BP:
		/* hpd_bp=1 */
		hpd_pwr &= ~HPD_POWER_BP_MASK;
		hpd_pwr |= 1 << HPD_POWER_BP_SHIFT;
		io_write(sd, REG_HPD_POWER, hpd_pwr);
		break;
	/* HPD low and pulse of at least 100ms */
	case HPD_LOW_OTHER:
		/* disable HPD_A and HPD_B */
		hpd_man &= ~(HPD_MAN_CTRL_HPD_A | HPD_MAN_CTRL_HPD_B);
		/* hp_other=0 */
		hpd_auto &= ~HPD_AUTO_HP_OTHER;
		io_write(sd, REG_HPD_AUTO_CTRL, hpd_auto);
		io_write(sd, REG_HPD_MAN_CTRL, hpd_man);
		break;
	/* HPD high */
	case HPD_HIGH_OTHER:
		hpd_auto |= HPD_AUTO_HP_OTHER;
		io_write(sd, REG_HPD_AUTO_CTRL, hpd_auto);
		break;
	/* HPD low pulse */
	case HPD_PULSE:
		/* disable HPD_A and HPD_B */
		hpd_man &= ~(HPD_MAN_CTRL_HPD_A | HPD_MAN_CTRL_HPD_B);
		io_write(sd, REG_HPD_MAN_CTRL, hpd_man);
		break;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Signal Control
 */

/*
 * The color conversion matrix will convert between the colorimetry of the
 * HDMI input to the desired output format RGB|YUV
 */
static int
tda1997x_configure_conv(struct v4l2_subdev *sd,
			enum hdmi_colorspace colorspace,
			enum hdmi_colorimetry colorimetry)
{
	struct tda1997x_state *state = to_state(sd);
	/* Colorspace conversion matrix coefficients and offsets */
	struct color_matrix_coefs {
		/* Input offsets */
		s16 offint1;
		s16 offint2;
		s16 offint3;
		/* Coeficients */
		s16 p11coef;
		s16 p12coef;
		s16 p13coef;
		s16 p21coef;
		s16 p22coef;
		s16 p23coef;
		s16 p31coef;
		s16 p32coef;
		s16 p33coef;
		/* Output offsets */
		s16 offout1;
		s16 offout2;
		s16 offout3;
	};
	/* Conversion matrixes */
	enum {
		ITU709_RGBLIMITED,
		RGBLIMITED_ITU601,
		ITU601_RGBLIMITED,
	};
	static const struct color_matrix_coefs conv_matrix[] = {
		/* ITU709 -> RGBLimited */
		{
			-256, -2048,  -2048,
			4096, -1875,   -750,
			4096,  6307,      0,
			4096,     0,   7431,
			 256,   256,    256,
		},
		/* RGBLimited -> ITU601 */
		{
			-256,  -256,   -256,
			2404,  1225,    467,
			-1754, 2095,   -341,
			-1388, -707,   2095,
			256,   2048,   2048,
		},
		/* YUV601 -> RGBLimited */
		{
			-256, -2048,  -2048,
			4096, -2860,  -1378,
			4096,  5615,      0,
			4096,     0,   7097,
			256,    256,    256,
		},
	};
	/* Blanking code values depend on output colorspace (RGB or YUV) */
	struct blanking_codes {
		s16 code_gy;
		s16 code_bu;
		s16 code_rv;
	};
	static const struct blanking_codes rgb_blanking = {64, 64, 64};
	static const struct blanking_codes yuv_blanking = {64, 512, 512};
	const struct color_matrix_coefs *coefficients = NULL;
	const struct blanking_codes *blanking_codes = NULL;
	u8 reg;

	switch (state->vid_fmt) {
	/* RGB444 */
	case OF_FMT_444:
		blanking_codes = &rgb_blanking;
		if (colorspace != HDMI_COLORSPACE_RGB) {
			if (colorimetry == HDMI_COLORIMETRY_ITU_709)
				coefficients = &conv_matrix[ITU709_RGBLIMITED];
			else
				coefficients = &conv_matrix[ITU601_RGBLIMITED];
		}
		break;

	/* YUV422 */
	case OF_FMT_422_SMPT: /* semi-planar */
	case OF_FMT_422_CCIR: /* CCIR656 */
		blanking_codes = &yuv_blanking;
		if (colorspace == HDMI_COLORSPACE_RGB)
			coefficients = &conv_matrix[RGBLIMITED_ITU601];
		break;
	}

	if (coefficients) {
		/* enable matrix conversion */
		reg = io_read(sd, REG_VDP_CTRL);
		reg &= ~VDP_CTRL_MATRIX_BP;
		io_write(sd, REG_VDP_CTRL, reg);
		/* offset inputs */
		io_write16(sd, REG_VDP_MATRIX + 0, coefficients->offint1);
		io_write16(sd, REG_VDP_MATRIX + 2, coefficients->offint2);
		io_write16(sd, REG_VDP_MATRIX + 4, coefficients->offint3);
		/* coefficients */
		io_write16(sd, REG_VDP_MATRIX + 6, coefficients->p11coef);
		io_write16(sd, REG_VDP_MATRIX + 8, coefficients->p12coef);
		io_write16(sd, REG_VDP_MATRIX + 10, coefficients->p13coef);
		io_write16(sd, REG_VDP_MATRIX + 12, coefficients->p21coef);
		io_write16(sd, REG_VDP_MATRIX + 14, coefficients->p22coef);
		io_write16(sd, REG_VDP_MATRIX + 16, coefficients->p23coef);
		io_write16(sd, REG_VDP_MATRIX + 18, coefficients->p31coef);
		io_write16(sd, REG_VDP_MATRIX + 20, coefficients->p32coef);
		io_write16(sd, REG_VDP_MATRIX + 22, coefficients->p33coef);
		/* offset outputs */
		io_write16(sd, REG_VDP_MATRIX + 24, coefficients->offout1);
		io_write16(sd, REG_VDP_MATRIX + 26, coefficients->offout2);
		io_write16(sd, REG_VDP_MATRIX + 28, coefficients->offout3);
	} else {
		/* disable matrix conversion */
		reg = io_read(sd, REG_VDP_CTRL);
		reg |= VDP_CTRL_MATRIX_BP;
		io_write(sd, REG_VDP_CTRL, reg);
	}

	/* SetBlankingCodes */
	if (blanking_codes) {
		io_write16(sd, REG_BLK_GY, blanking_codes->code_gy);
		io_write16(sd, REG_BLK_BU, blanking_codes->code_bu);
		io_write16(sd, REG_BLK_RV, blanking_codes->code_rv);
	}

	return 0;
}

/* Configure frame detection window and VHREF timing generator */
static int
tda1997x_configure_input_resolution(struct v4l2_subdev *sd,
				    const struct tda1997x_video_std *std)
{
	struct tda1997x_state *state = to_state(sd);
	const struct v4l2_bt_timings *bt = &std->timings.bt;
	const struct vhref_values *vh = &std->vhref_values;
	int width, lines;
	u8 reg;

	width = bt->width + bt->hfrontporch + bt->hsync +
		bt->hbackporch;
	lines = bt->height + bt->vfrontporch + bt->vsync +
		bt->vbackporch;

	/*
	 * Configure Frame Detection Window:
	 *  horiz area where the VHREF module consider a VSYNC a new frame
	 */
	io_write16(sd, REG_FDW_S, 0x2ef); /* start position */
	io_write16(sd, REG_FDW_E, 0x141); /* end position */

	/* Set Pixel And Line Counters */
	if (state->chip_revision == 0)
		io_write16(sd, REG_PXCNT_PR, 4);
	else
		io_write16(sd, REG_PXCNT_PR, 1);
	io_write16(sd, REG_PXCNT_NPIX, width & MASK_VHREF);
	io_write16(sd, REG_LCNT_PR, 1);
	io_write16(sd, REG_LCNT_NLIN, lines & MASK_VHREF);

	/*
	 * Configure the VHRef timing generator responsible for rebuilding all
	 * horiz and vert synch and ref signals from its input allowing auto
	 * detection algorithms and forcing predefined modes (480i & 576i)
	 */
	reg = VHREF_STD_DET_OFF << VHREF_STD_DET_SHIFT;
	io_write(sd, REG_VHREF_CTRL, reg);

	/*
	 * Configure the VHRef timing values. In case the VHREF generator has
	 * been configured in manual mode, this will allow to manually set all
	 * horiz and vert ref values (non-active pixel areas) of the generator
	 * and allows setting the frame reference params.
	 */
	/* horizontal reference start/end */
	io_write16(sd, REG_HREF_S, vh->href_start & MASK_VHREF);
	io_write16(sd, REG_HREF_E, vh->href_end & MASK_VHREF);
	/* vertical reference f1 start/end */
	io_write16(sd, REG_VREF_F1_S, vh->vref_f1_start & MASK_VHREF);
	io_write(sd, REG_VREF_F1_WIDTH, vh->vref_f1_width);
	/* vertical reference f2 start/end */
	io_write16(sd, REG_VREF_F2_S, vh->vref_f2_start & MASK_VHREF);
	io_write(sd, REG_VREF_F2_WIDTH, vh->vref_f2_width);
	/* F1/F2 FREF, field polarity */
	io_write16(sd, REG_FREF_F1_S, (vh->fieldref_f1_start & MASK_VHREF)
		|| (vh->fieldPolarity << 8));
	io_write16(sd, REG_FREF_F2_S, vh->fieldref_f2_start & MASK_VHREF);

	return 0;
}

/* Configure Video Output port signals */
static int
tda1997x_configure_vidout(struct tda1997x_state *state)
{
	struct v4l2_subdev *sd = &state->sd;
	struct tda1997x_platform_data *pdata = &state->pdata;
	u8 prefilter;
	u8 reg;

	/* Configure pixel clock generator: delay, polarity, rate */
	reg = (state->vid_fmt == OF_FMT_422_CCIR) ?
	       PCLK_SEL_X2 : PCLK_SEL_X1;
	reg |= pdata->vidout_delay_pclk << PCLK_DELAY_SHIFT;
	reg |= pdata->vidout_inv_pclk << PCLK_INV_SHIFT;
	io_write(sd, REG_PCLK, reg);

	/* Configure pre-filter */
	prefilter = 0; /* filters off */
	/* YUV422 mode requires conversion */
	if ((state->vid_fmt == OF_FMT_422_SMPT)
	 || (state->vid_fmt == OF_FMT_422_CCIR)) {
		/* 2/7taps for Rv and Bu */
		prefilter = FILTERS_CTRL_2_7TAP << FILTERS_CTRL_BU_SHIFT |
			    FILTERS_CTRL_2_7TAP << FILTERS_CTRL_RV_SHIFT;
	}
	io_write(sd, REG_FILTERS_CTRL, prefilter);

	/* Configure video port */
	reg = state->vid_fmt & OF_FMT_MASK;
	if (state->vid_fmt == OF_FMT_422_CCIR)
		reg |= (OF_BLK | OF_TRC);
	reg |= OF_VP_ENABLE;
	io_write(sd, REG_OF, reg);

	/* Configure formatter and conversions */
	reg = io_read(sd, REG_VDP_CTRL);
	/* pre-filter is needed unless (REG_FILTERS_CTRL == 0) */
	if (!prefilter)
		reg |= VDP_CTRL_PREFILTER_BP;
	else
		reg &= ~VDP_CTRL_PREFILTER_BP;
	/* formatter is needed for YUV422 and for trc/blc codes */
	if (state->vid_fmt == OF_FMT_444)
		reg |= VDP_CTRL_FORMATTER_BP;
	/* formatter and compdel needed for timing/blanking codes */
	else
		reg &= ~(VDP_CTRL_FORMATTER_BP | VDP_CTRL_COMPDEL_BP);
	/* activate compdel for small sync delays */
	if ((pdata->vidout_delay_vs < 4) || (pdata->vidout_delay_hs < 4))
		reg &= ~VDP_CTRL_COMPDEL_BP;
	io_write(sd, REG_VDP_CTRL, reg);

	/* Configure DE output signal: delay, polarity, and source */
	reg = pdata->vidout_delay_de << DE_FREF_DELAY_SHIFT |
	      pdata->vidout_inv_de << DE_FREF_INV_SHIFT |
	      pdata->vidout_sel_de << DE_FREF_SEL_SHIFT;
	io_write(sd, REG_DE_FREF, reg);

	/* Configure HS/HREF output signal: delay, polarity, and source */
	if (state->vid_fmt != OF_FMT_422_CCIR) {
		reg = pdata->vidout_delay_hs << HS_HREF_DELAY_SHIFT |
		      pdata->vidout_inv_hs << HS_HREF_INV_SHIFT |
		      pdata->vidout_sel_hs << HS_HREF_SEL_SHIFT;
	} else
		reg = HS_HREF_SEL_NONE << HS_HREF_SEL_SHIFT;
	io_write(sd, REG_HS_HREF, reg);

	/* Configure VS/VREF output signal: delay, polarity, and source */
	if (state->vid_fmt != OF_FMT_422_CCIR) {
		reg = pdata->vidout_delay_vs << VS_VREF_DELAY_SHIFT |
		      pdata->vidout_inv_vs << VS_VREF_INV_SHIFT |
		      pdata->vidout_sel_vs << VS_VREF_SEL_SHIFT;
	} else
		reg = VS_VREF_SEL_NONE << VS_VREF_SEL_SHIFT;
	io_write(sd, REG_VS_VREF, reg);

	return 0;
}

/* Configure Audio output port signals */
static int
tda1997x_configure_audout(struct v4l2_subdev *sd, u8 channel_assignment)
{
	struct tda1997x_state *state = to_state(sd);
	struct tda1997x_platform_data *pdata = &state->pdata;
	bool sp_used_by_fifo = 0;
	bool ws_active = 0;
	u8 reg;

	if (!pdata->audout_fmt)
		return 0;

	/* channel assignment (CEA-861-D Table 20) */
	io_write(sd, REG_AUDIO_PATH, channel_assignment);

	/* Audio output configuration */
	reg = 0;
	switch (pdata->audout_fmt) {
	case TDA1997X_I2S16:
		/* 16bit I2S mode, SP flag used by FIFO */
		reg = AUDCFG_BUS_I2S << AUDCFG_BUS_SHIFT |
		      AUDCFG_I2SW_16 << AUDCFG_I2SW_SHIFT |
		      AUDCFG_TYPE_SAMPLES << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		ws_active = 1;
		break;
	case TDA1997X_I2S32:
		/* 32bit I2S mode, SP flag used by FIFO */
		reg = AUDCFG_BUS_I2S << AUDCFG_BUS_SHIFT |
		      AUDCFG_I2SW_32 << AUDCFG_I2SW_SHIFT |
		      AUDCFG_TYPE_SAMPLES << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		ws_active = 1;
		break;
	case TDA1997X_OBA:
		/* One Bit Audio */
		reg = AUDCFG_TYPE_OBA << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		ws_active = 1;
		break;
	case TDA1997X_SPDIF:
		/* SPDIF mode, SP flag used by FIFO */
		reg = AUDCFG_BUS_SPDIF << AUDCFG_BUS_SHIFT |
		      AUDCFG_TYPE_SAMPLES << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		break;
	case TDA1997X_I2S16_HBR_DEMUX:
		/* 16bit I2S High Bit Rate demux */
		reg = AUDCFG_BUS_I2S << AUDCFG_BUS_SHIFT |
		      AUDCFG_I2SW_16 << AUDCFG_I2SW_SHIFT |
		      AUDCFG_HBR_DEMUX << AUDCFG_HBR_SHIFT |
		      AUDCFG_TYPE_HBR << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		ws_active = 1;
		break;
	case TDA1997X_I2S32_HBR_DEMUX:
		/* 32bit I2S High Bit Rate demux */
		reg = AUDCFG_BUS_I2S << AUDCFG_BUS_SHIFT |
		      AUDCFG_I2SW_32 << AUDCFG_I2SW_SHIFT |
		      AUDCFG_HBR_DEMUX << AUDCFG_HBR_SHIFT |
		      AUDCFG_TYPE_HBR << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 1;
		ws_active = 1;
		break;
	case TDA1997X_DST:
		/* Direct Stream Transfer */
		reg = AUDCFG_TYPE_DST << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 0;
		break;
	case TDA1997X_SPDIF_HBR_DEMUX:
		/* SPDIF High Bit Rate demux */
		reg = AUDCFG_BUS_SPDIF << AUDCFG_BUS_SHIFT |
		      AUDCFG_HBR_DEMUX << AUDCFG_HBR_SHIFT |
		      AUDCFG_TYPE_HBR << AUDCFG_TYPE_SHIFT;
		sp_used_by_fifo = 0;
		break;
	}
	/* automatic hardware mute */
	if (pdata->audio_auto_mute)
		reg |= AUDCFG_AUTO_MUTE_EN;
	/* clock polarity */
	if (pdata->audout_invert_clk)
		reg |= AUDCFG_CLK_INVERT;
	io_write(sd, REG_AUDCFG, reg);

	/* audio layout */
	reg = pdata->audout_layout;
	if (!pdata->audout_layoutauto)
		reg |= AUDIO_LAYOUT_MANUAL;
	if (sp_used_by_fifo)
		reg |= AUDIO_LAYOUT_SP_FLAG;
	io_write(sd, REG_AUDIO_LAYOUT, reg);

	/* FIFO Latency value */
	io_write(sd, REG_FIFO_LATENCY_VAL, 0x80);

	/* Audio output port config */
	if (sp_used_by_fifo) {
		reg = AUDIO_OUT_ENABLE_AP0;
		if (channel_assignment >= 0x01)
			reg |= AUDIO_OUT_ENABLE_AP1;
		if (channel_assignment >= 0x04)
			reg |= AUDIO_OUT_ENABLE_AP2;
		if (channel_assignment >= 0x0c)
			reg |= AUDIO_OUT_ENABLE_AP3;
		/* specific cases where AP1 is not used */
		if ((channel_assignment == 0x04)
		 || (channel_assignment == 0x08)
		 || (channel_assignment == 0x0c)
		 || (channel_assignment == 0x10)
		 || (channel_assignment == 0x14)
		 || (channel_assignment == 0x18)
		 || (channel_assignment == 0x1c))
			reg &= ~AUDIO_OUT_ENABLE_AP1;
		/* specific cases where AP2 is not used */
		if ((channel_assignment >= 0x14)
		 && (channel_assignment <= 0x17))
			reg &= ~AUDIO_OUT_ENABLE_AP2;
	} else {
		reg = AUDIO_OUT_ENABLE_AP3 |
		      AUDIO_OUT_ENABLE_AP2 |
		      AUDIO_OUT_ENABLE_AP1 |
		      AUDIO_OUT_ENABLE_AP0;
	}
	if (ws_active)
		reg |= (AUDIO_OUT_ENABLE_ACLK | AUDIO_OUT_ENABLE_WS);
	io_write(sd, REG_AUDIO_OUT_ENABLE, reg);

	/* reset test mode to normal audio freq auto selection */
	io_write(sd, REG_TEST_MODE, 0x00);

	return 0;
}

/* Soft Reset of specific hdmi info */
static int
tda1997x_hdmi_info_reset(struct v4l2_subdev *sd, u8 info_rst, bool reset_sus)
{
	u8 reg;

	/* reset infoframe engine packets */
	reg = io_read(sd, REG_HDMI_INFO_RST);
	io_write(sd, REG_HDMI_INFO_RST, info_rst);

	/* if infoframe engine has been reset clear INT_FLG_MODE */
	if (reg & RESET_IF) {
		reg = io_read(sd, REG_INT_FLG_CLR_MODE);
		io_write(sd, REG_INT_FLG_CLR_MODE, reg);
	}

	/* Disable REFTIM to restart start-up-sequencer (SUS) */
	reg = io_read(sd, REG_RATE_CTRL);
	reg &= ~RATE_REFTIM_ENABLE;
	if (!reset_sus)
		reg |= RATE_REFTIM_ENABLE;
	reg = io_write(sd, REG_RATE_CTRL, reg);

	return 0;
}

static void
tda1997x_power_mode(struct tda1997x_state *state, bool enable)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg;

	if (enable) {
		/* Automatic control of TMDS */
		io_write(sd, REG_PON_OVR_EN, PON_DIS);
		/* Enable current bias unit */
		io_write(sd, REG_CFG1, PON_EN);
		/* Enable deep color PLL */
		io_write(sd, REG_DEEP_PLL7_BYP, PON_DIS);
		/* Output buffers active */
		reg = io_read(sd, REG_OF);
		reg &= ~OF_VP_ENABLE;
		io_write(sd, REG_OF, reg);
	} else {
		/* Power down EDID mode sequence */
		/* Output buffers in HiZ */
		reg = io_read(sd, REG_OF);
		reg |= OF_VP_ENABLE;
		io_write(sd, REG_OF, reg);
		/* Disable deep color PLL */
		io_write(sd, REG_DEEP_PLL7_BYP, PON_EN);
		/* Disable current bias unit */
		io_write(sd, REG_CFG1, PON_DIS);
		/* Manual control of TMDS */
		io_write(sd, REG_PON_OVR_EN, PON_EN);
	}
}

const struct tda1997x_video_std *
tda1997x_detect_resolution(struct tda1997x_state *state)
{
	struct v4l2_subdev *sd = &state->sd;
	struct tda1997x_platform_data *pdata = &state->pdata;
	u32 vper;
	u16 hper;
	u16 hsper;
	int i;

	/*
	 * Read the FMT registers
	 *   REG_V_PER: Period of a frame (or two fields) in MCLK(27MHz) cycles
	 *   REG_H_PER: Period of a line in MCLK(27MHz) cycles
	 *   REG_HS_WIDTH: Period of horiz sync pulse in MCLK(27MHz) cycles
	 */
	vper = io_read24(sd, REG_V_PER) & MASK_VPER;
	hper = io_read16(sd, REG_H_PER) & MASK_HPER;
	hsper = io_read16(sd, REG_HS_WIDTH) & MASK_HSWIDTH;
	if (!vper || !hper || !hsper)
		return NULL;

	/* look for matching timings */
	for (i = 0; i < ARRAY_SIZE(tda1997x_hdmi_modes); i++) {
		const struct tda1997x_video_std *std = &tda1997x_hdmi_modes[i];
		const struct v4l2_bt_timings *bt = &std->timings.bt;
		int lines, width, _hper, _vper, _hsper;
		int vmin, vmax, hmin, hmax, hsmin, hsmax;
		int hmatch, vmatch, hsmatch;

		width = bt->width + bt->hfrontporch + bt->hsync +
			bt->hbackporch;
		lines = bt->height + bt->vfrontporch + bt->vsync +
			bt->vbackporch;

		_hper = (int)bt->pixelclock / (int)width;
		_vper = _hper / lines;
		_hsper = (int)bt->pixelclock / (int)bt->hsync;
		if (bt->interlaced)
			_vper *= 2;
		/* vper +/- 0.7% */
		vmin = 993 * (27000000 / _vper) / 1000;
		vmax = 1007 * (27000000 / _vper) / 1000;
		/* hper +/- 0.7% */
		hmin = 993 * (27000000 / _hper) / 1000;
		hmax = 1007 * (27000000 / _hper) / 1000;
		/* hsper +/- 0.7% */
		hsmin = 993 * (27000000 / _hsper) / 1000;
		hsmax = 1007 * (27000000 / _hsper) / 1000;

		vmatch = ((vper <= vmax) && (vper >= vmin)) ? 1 : 0;
		hmatch = ((hper <= hmax) && (hper >= hmin)) ? 1 : 0;
		hsmatch = ((hsper <= hsmax) && (hsper >= hsmin)) ? 1 : 0;
		if (hmatch && vmatch && hsmatch) {
			int pixrate;

			v4l_info(state->client,
				 "resolution: %dx%d%c@%d (%d/%d/%d)\n",
				 bt->width, bt->height, bt->interlaced?'i':'p',
				 _vper, vper, hper, hsper);

			/* validate input mode */
			pixrate = (bt->width * bt->height * _vper) / 1000000;
			if (bt->interlaced)
				pixrate /= 2;

			/* CCIR requires 2-clocks per pixel */
			if (state->vid_fmt == OF_FMT_422_CCIR)
				pixrate *= 2;
			if (pdata->max_pixel_rate &&
			    (pixrate > pdata->max_pixel_rate)) {
				v4l_err(state->client,
					"Error: %dMP/s exceeds max of %dMP/s\n",
					pixrate, pdata->max_pixel_rate);
				return NULL;
			}

			state->timings = &std->timings;
			state->fps = (int)bt->pixelclock / (width * lines);

			return std;
		}
	}

	v4l_err(state->client, "no resolution match for timings: %d/%d/%d\n",
		vper, hper, hsper);
	return NULL;
}

/* some sort of errata workaround for chip revision 0 (N1) */
static void tda1997x_reset_n1(struct tda1997x_state *state)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg;

	/* clear HDMI mode flag in BCAPS */
	io_write(sd, REG_CLK_CFG, CLK_CFG_SEL_ACLK_EN | CLK_CFG_SEL_ACLK);
	io_write(sd, REG_PON_OVR_EN, PON_EN);
	io_write(sd, REG_PON_CBIAS, PON_EN);
	io_write(sd, REG_PON_PLL, PON_EN);

	reg = io_read(sd, REG_MODE_REC_CFG1);
	reg &= ~0x06;
	reg |= 0x02;
	io_write(sd, REG_MODE_REC_CFG1, reg);
	io_write(sd, REG_CLK_CFG, CLK_CFG_DIS);
	io_write(sd, REG_PON_OVR_EN, PON_DIS);
	reg = io_read(sd, REG_MODE_REC_CFG1);
	reg &= ~0x06;
	io_write(sd, REG_MODE_REC_CFG1, reg);
}

/*
 * Activity detection must only be notified when stable_clk_x AND active_x
 * bits are set to 1. If only stable_clk_x bit is set to 1 but not
 * active_x, it means that the TMDS clock is not in the defined range
 * and activity detection must not be notified.
 */
static u8
tda1997x_read_activity_status_regs(struct v4l2_subdev *sd)
{
	u8 reg, status = 0;

	/* Read CLK_A_STATUS register */
	reg = io_read(sd, REG_CLK_A_STATUS);
	/* when stable_clk_x is set to 1, check active_x bit */
	if ((reg & MASK_CLK_STABLE) && !(reg & MASK_CLK_ACTIVE))
		reg &= ~MASK_CLK_STABLE;
	status |= ((reg & MASK_CLK_STABLE) >> 2);

	/* Read CLK_B_STATUS register */
	reg = io_read(sd, REG_CLK_B_STATUS);
	/* when stable_clk_x is set to 1, check active_x bit */
	if ((reg & MASK_CLK_STABLE) && !(reg & MASK_CLK_ACTIVE))
		reg &= ~MASK_CLK_STABLE;
	status |= ((reg & MASK_CLK_STABLE) >> 1);

	/* Read the SUS_STATUS register */
	reg = io_read(sd, REG_SUS_STATUS);

	/* If state = 5 => TMDS is locked */
	if ((reg & MASK_SUS_STATUS) == LAST_STATE_REACHED)
		status |= MASK_SUS_STATE;
	else
		status &= ~MASK_SUS_STATE;

	return status;
}

/* parse an infoframe and do some sanity checks on it */
static unsigned int
tda1997x_parse_infoframe(struct tda1997x_state *state, u16 addr)
{
	struct v4l2_subdev *sd = &state->sd;
	union hdmi_infoframe frame;
	u8 buffer[40];
	u8 reg;
	int len, err;

	/* read data */
	len = io_readn(sd, addr, sizeof(buffer), buffer);
	err = hdmi_infoframe_unpack(&frame, buffer);
	if (err) {
		v4l_err(state->client,
			"failed parsing %d byte infoframe: 0x%04x\n",
			len, addr);
		return err;
	}
	if (debug > 1)
		hdmi_infoframe_log(KERN_INFO, &state->client->dev, &frame);
	switch (frame.any.type) {
	/* Audio InfoFrame: see HDMI spec 8.2.2 */
	case HDMI_INFOFRAME_TYPE_AUDIO:
		/* Channel Count */
		state->audio_channels = frame.audio.channels;
		if (frame.audio.channel_allocation &&
		    frame.audio.channel_allocation != state->audio_ch_alloc) {
			/* use the channel assignment from the infoframe */
			state->audio_ch_alloc = frame.audio.channel_allocation;
			tda1997x_configure_audout(sd, state->audio_ch_alloc);
			/* reset the audio FIFO */
			tda1997x_hdmi_info_reset(sd, RESET_AUDIO, false);
		}
		break;

	/* Source Product Descriptor information (SPD) */
	case HDMI_INFOFRAME_TYPE_SPD:
		strncpy(frame.spd.vendor, state->vendor,
			sizeof(frame.spd.vendor));
		strncpy(frame.spd.product, state->product,
			sizeof(frame.spd.product));
		v4l_info(state->client, "Source Product Descriptor: %s %s\n",
			 state->vendor, state->product);
		break;

	/* Auxiliary Video information (AVI) InfoFrame: see HDMI spec 8.2.1 */
	case HDMI_INFOFRAME_TYPE_AVI:
		state->colorspace = frame.avi.colorspace;
		state->colorimetry = frame.avi.colorimetry;
		/*
		 * If colorimetry not specified, conversion depends on res type:
		 *  - SDTV: ITU601 for SD (480/576/240/288 line resolution)
		 *  - HDTV: ITU709 for HD (720/1080 line resolution)
		 *  -   PC: sRGB
		 * see HDMI specification section 6.7
		 */
		if ((state->colorspace == HDMI_COLORSPACE_YUV422 ||
		     state->colorspace == HDMI_COLORSPACE_YUV444) &&
		    (state->colorimetry == HDMI_COLORIMETRY_EXTENDED ||
		     state->colorimetry == HDMI_COLORIMETRY_NONE)) {
			switch (state->timings->bt.height) {
			case 480:
			case 576:
			case 240:
			case 288:
				state->colorimetry = HDMI_COLORIMETRY_ITU_601;
				break;
			case 720:
			case 1080:
				state->colorimetry = HDMI_COLORIMETRY_ITU_709;
				break;
			default:
				state->colorimetry = HDMI_COLORIMETRY_NONE;
			}
		}
		v4l_dbg(1, debug, state->client,
			"Colorspace=%d Colorimetry=%d\n",
			state->colorspace, state->colorimetry);

		/* configure upsampler: 0=bypass 1=repeatchroma 2=interpolate */
		reg = io_read(sd, REG_PIX_REPEAT);
		reg &= ~PIX_REPEAT_MASK_UP_SEL;
		if (state->colorspace == HDMI_COLORSPACE_YUV422)
			reg |= (PIX_REPEAT_CHROMA << PIX_REPEAT_SHIFT);
		io_write(sd, REG_PIX_REPEAT, reg);

		/* ConfigurePixelRepeater: repeat n-times each pixel */
		reg = io_read(sd, REG_PIX_REPEAT);
		reg &= ~PIX_REPEAT_MASK_REP;
		reg |= frame.avi.pixel_repeat;
		io_write(sd, REG_PIX_REPEAT, reg);

		/* configure the receiver with the new colorspace */
		tda1997x_configure_conv(sd, state->colorspace,
					state->colorimetry);
		break;
	default:
		break;
	}
	return 0;
}

static void tda1997x_irq_sus(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg, source;

	source = io_read(sd, REG_INT_FLG_CLR_SUS);
	io_write(sd, REG_INT_FLG_CLR_SUS, source);

	if (source & MASK_MPT) {
		/* reset MTP in use flag if set */
		if (state->mptrw_in_progress)
			state->mptrw_in_progress = 0;
	}

	if (source & MASK_SUS_END) {
		/* reset audio FIFO */
		reg = io_read(sd, REG_HDMI_INFO_RST);
		reg |= MASK_SR_FIFO_FIFO_CTRL;
		io_write(sd, REG_HDMI_INFO_RST, reg);
		reg &= ~MASK_SR_FIFO_FIFO_CTRL;
		io_write(sd, REG_HDMI_INFO_RST, reg);

		/* reset HDMI flags */
		state->hdmi_status = 0;
	}

	/* filter FMT interrupt based on SUS state */
	reg = io_read(sd, REG_SUS_STATUS);
	if (((reg & MASK_SUS_STATUS) != LAST_STATE_REACHED)
	   || (source & MASK_MPT)) {
		source &= ~MASK_FMT;
	}

	if (source & (MASK_FMT | MASK_SUS_END)) {
		const struct tda1997x_video_std *std;

		reg = io_read(sd, REG_SUS_STATUS);
		if ((reg & MASK_SUS_STATUS) != LAST_STATE_REACHED) {
			v4l_err(state->client, "BAD SUS STATUS\n");
			return;
		}

		/* There is new activity, the status for HDCP repeater state */
		state->state_c5_reached = 0;

		/* Detect the new resolution */
		std = tda1997x_detect_resolution(state);
		if (std)
			tda1997x_configure_input_resolution(sd, std);
		state->colorspace = HDMI_COLORSPACE_RGB;
		state->colorimetry = HDMI_COLORIMETRY_NONE;
		tda1997x_configure_conv(sd, state->colorspace,
					state->colorimetry);
		v4l2_subdev_notify_event(&state->sd, &tda1997x_ev_fmt);
	}
}

static void tda1997x_irq_ddc(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 source;

	source = io_read(sd, REG_INT_FLG_CLR_DDC);
	io_write(sd, REG_INT_FLG_CLR_DDC, source);
	if (source & MASK_EDID_MTP) {
		/* reset MTP in use flag if set */
		if (state->mptrw_in_progress)
			state->mptrw_in_progress = 0;
	}
}

static void tda1997x_irq_rate(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg, source;

	u8 irq_status, last_irq_status;

	source = io_read(sd, REG_INT_FLG_CLR_RATE);
	io_write(sd, REG_INT_FLG_CLR_RATE, source);

	/* read status regs */
	last_irq_status = irq_status = tda1997x_read_activity_status_regs(sd);

	/*
	 * read clock status reg until INT_FLG_CLR_RATE is still 0
	 * after the read to make sure its the last one
	 */
	reg = source;
	while (reg != 0) {
		irq_status = tda1997x_read_activity_status_regs(sd);
		reg = io_read(sd, REG_INT_FLG_CLR_RATE);
		io_write(sd, REG_INT_FLG_CLR_RATE, reg);
		source |= reg;
	}

	/* we only pay attention to stability change events */
	if (source & (MASK_RATE_A_ST | MASK_RATE_B_ST)) {
		int input = (source & MASK_RATE_A_ST)?0:1;
		u8 mask = 1<<input;

		/* state change */
		if ((irq_status & mask) != (state->activity_status & mask)) {
			/* activity lost */
			if ((irq_status & mask) == 0) {
				v4l_info(state->client,
					 "HDMI-%c: Digital Activity Lost\n",
					 input+'A');
				state->timings = NULL;
				state->fps = 0;
				state->input_detect[input] = 0;

				/* bypass up/down sampler and pixel repeater */
				reg = io_read(sd, REG_PIX_REPEAT);
				reg &= ~PIX_REPEAT_MASK_UP_SEL;
				reg &= ~PIX_REPEAT_MASK_REP;
				io_write(sd, REG_PIX_REPEAT, reg);

				if (state->chip_revision == 0)
					tda1997x_reset_n1(state);
			}

			/* activity detected */
			else {
				v4l_info(state->client,
					 "HDMI-%c: Digital Activity Detected\n",
					 input+'A');
				state->input_detect[input] = 1;
			}

			/* hold onto current state */
			state->activity_status = (irq_status & mask);
		}
	}
}

static void tda1997x_irq_info(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 source;

	source = io_read(sd, REG_INT_FLG_CLR_INFO);
	io_write(sd, REG_INT_FLG_CLR_INFO, source);

	/* Audio infoframe */
	if (source & MASK_AUD_IF) {
		tda1997x_parse_infoframe(state, AUD_IF);
		source &= ~MASK_AUD_IF;
	}

	/* Source Product Descriptor infoframe change */
	if (source & MASK_SPD_IF) {
		tda1997x_parse_infoframe(state, SPD_IF);
		source &= ~MASK_SPD_IF;
	}

	/* Auxiliary Video Information infoframe */
	if (source & MASK_AVI_IF) {
		tda1997x_parse_infoframe(state, AVI_IF);
		source &= ~MASK_AVI_IF;
	}
}

static void tda1997x_irq_audio(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg, source;

	source = io_read(sd, REG_INT_FLG_CLR_AUDIO);
	io_write(sd, REG_INT_FLG_CLR_AUDIO, source);

	/* reset audio FIFO on FIFO pointer error or audio mute */
	if (source & MASK_ERROR_FIFO_PT ||
	    source & MASK_MUTE_FLG) {
		/* audio reset audio FIFO */
		reg = io_read(sd, REG_SUS_STATUS);
		if ((reg & MASK_SUS_STATUS) == LAST_STATE_REACHED) {
			reg = io_read(sd, REG_HDMI_INFO_RST);
			reg |= MASK_SR_FIFO_FIFO_CTRL;
			io_write(sd, REG_HDMI_INFO_RST, reg);
			reg &= ~MASK_SR_FIFO_FIFO_CTRL;
			io_write(sd, REG_HDMI_INFO_RST, reg);
			/* reset channel status IT if present */
			source &= ~(MASK_CH_STATE);
		}
	}
	if (source & MASK_AUDIO_FREQ_FLG) {
		static const int freq[] = {
			0, 32000, 44100, 48000, 88200, 96000, 176400, 192000
		};

		reg = io_read(sd, REG_AUDIO_FREQ);
		state->audio_samplerate = freq[reg & 7];
		v4l_info(state->client, "Audio Frequency Change: %dHz\n",
			 state->audio_samplerate);
	}
}

static void tda1997x_irq_hdcp(struct tda1997x_state *state, u8 *flags)
{
	struct v4l2_subdev *sd = &state->sd;
	u8 reg, source;

	source = io_read(sd, REG_INT_FLG_CLR_HDCP);
	io_write(sd, REG_INT_FLG_CLR_HDCP, source);

	/* reset MTP in use flag if set */
	if (source & MASK_HDCP_MTP)
		state->mptrw_in_progress = 0;
	if (source & MASK_STATE_C5) {
		/* REPEATER: mask AUDIO and IF irqs to avoid IF during auth */
		reg = io_read(sd, REG_INT_MASK_TOP);
		reg &= ~(INTERRUPT_AUDIO | INTERRUPT_INFO);
		io_write(sd, REG_INT_MASK_TOP, reg);
		*flags &= (INTERRUPT_AUDIO | INTERRUPT_INFO);
	}
}

static irqreturn_t tda1997x_isr_thread(int irq, void *d)
{
	struct tda1997x_state *state = d;
	struct v4l2_subdev *sd = &state->sd;
	u8 flags;

	mutex_lock(&state->lock);
	do {
		/* read interrupt flags */
		flags = io_read(sd, REG_INT_FLG_CLR_TOP);
		if (flags == 0)
			break;

		/* SUS interrupt source (Input activity events) */
		if (flags & INTERRUPT_SUS)
			tda1997x_irq_sus(state, &flags);
		/* DDC interrupt source (Display Data Channel) */
		else if (flags & INTERRUPT_DDC)
			tda1997x_irq_ddc(state, &flags);
		/* RATE interrupt source (Digital Input activity) */
		else if (flags & INTERRUPT_RATE)
			tda1997x_irq_rate(state, &flags);
		/* Infoframe change interrupt */
		else if (flags & INTERRUPT_INFO)
			tda1997x_irq_info(state, &flags);
		/* Audio interrupt source:
		 *   freq change, DST,OBA,HBR,ASP flags, mute, FIFO err
		 */
		else if (flags & INTERRUPT_AUDIO)
			tda1997x_irq_audio(state, &flags);
		/* HDCP interrupt source (content protection) */
		if (flags & INTERRUPT_HDCP)
			tda1997x_irq_hdcp(state, &flags);
	} while (flags != 0);
	mutex_unlock(&state->lock);

	return IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * v4l2_subdev_video_ops
 */

static int
tda1997x_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tda1997x_state *state = to_state(sd);

	mutex_lock(&state->lock);
	if (state->timings)
		*status = 0;
	else
		*status |= V4L2_IN_ST_NO_SIGNAL;
	mutex_unlock(&state->lock);

	return 0;
};

static int tda1997x_s_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings)
{
	struct tda1997x_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;

	if (v4l2_match_dv_timings(state->timings, timings, 0, false))
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);

	return 0;
}

static int tda1997x_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tda1997x_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;
	*timings = *state->timings;
	return 0;
}

static int tda1997x_query_dv_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	struct tda1997x_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (!state->timings) {
		v4l2_dbg(1, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	*timings = *state->timings;
	return 0;
}

static int tda1997x_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tda1997x_state *state = to_state(sd);

	mutex_lock(&state->lock);
	if (!state->timings)
		v4l_dbg(1, debug, state->client, "Invalid HDMI signal\n");
	mutex_unlock(&state->lock);

	return 0;
}

static int tda1997x_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *cfg)
{
	struct tda1997x_state *state = to_state(sd);

	cfg->type = state->bus_type;

	return 0;
}

static const struct v4l2_subdev_video_ops tda1997x_video_ops = {
	.g_input_status = tda1997x_g_input_status,
	.s_dv_timings = tda1997x_s_dv_timings,
	.g_dv_timings = tda1997x_g_dv_timings,
	.query_dv_timings = tda1997x_query_dv_timings,
	.s_stream = tda1997x_s_stream,
	.g_mbus_config = tda1997x_g_mbus_config,
};


/* -----------------------------------------------------------------------------
 * v4l2_subdev_pad_ops
 */

static int tda1997x_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct tda1997x_state *state = to_state(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = state->code;

	return 0;
}

static int tda1997x_fill_format(struct tda1997x_state *state,
				struct v4l2_mbus_framefmt *format)
{
	if (!state->timings)
		return -EINVAL;
	memset(format, 0, sizeof(*format));

	format->width = state->timings->bt.width;
	format->height = state->timings->bt.height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;

	if (state->timings->bt.flags & V4L2_DV_FL_IS_CE_VIDEO)
		format->colorspace = (state->timings->bt.height <= 576) ?
		V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;

	return 0;
}

static int tda1997x_get_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *format)
{
	struct tda1997x_state *state = to_state(sd);

	if (format->pad != TDA1997X_PAD_SOURCE)
		return -EINVAL;

	tda1997x_fill_format(state, &format->format);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;

		fmt = v4l2_subdev_get_try_format(sd, cfg, format->pad);
		format->format.code = format->format.code;
	} else
		format->format.code = state->code;

	return 0;
}

static int tda1997x_set_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != TDA1997X_PAD_SOURCE)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return tda1997x_get_pad_format(sd, cfg, format);

	fmt = v4l2_subdev_get_try_format(sd, cfg, format->pad);
	*fmt = format->format;

	return 0;
}

static const struct v4l2_subdev_pad_ops tda1997x_pad_ops = {
	.enum_mbus_code = tda1997x_enum_mbus_code,
	.get_fmt = tda1997x_get_pad_format,
	.set_fmt = tda1997x_set_pad_format,
};

/* -----------------------------------------------------------------------------
 * v4l2_subdev_core_ops
 */

static int tda1997x_log_status(struct v4l2_subdev *sd)
{
	struct tda1997x_state *state = to_state(sd);
	const struct v4l2_dv_timings *timings = state->timings;

	v4l2_info(sd, "-----Signal status-----\n");
	if (!timings) {
		v4l2_info(sd, "no signal\n");
		return 0;
	}
	v4l2_info(sd, "resolution: %dx%d%c@%dHz\n",
		  timings->bt.width, timings->bt.height,
		  timings->bt.interlaced ? 'i' : 'p',
		  state->fps);
	v4l2_print_dv_timings(sd->name, "Detected format: ",
			      timings, true);
	v4l2_info(sd, "colorspace: %d\n", state->colorspace);
	v4l2_info(sd, "colorimetry: %d\n", state->colorimetry);
	if (state->audio_channels)
		v4l2_info(sd, "audio: %dch %dHz\n", state->audio_channels,
			  state->audio_samplerate);
		else
			v4l2_info(sd, "audio: none\n");
	v4l2_info(sd, "vendor: %s\n", state->vendor);
	v4l2_info(sd, "product: %s\n", state->product);

	return 0;
}

static int tda1997x_subscribe_event(struct v4l2_subdev *sd,
				    struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops tda1997x_core_ops = {
	.log_status = tda1997x_log_status,
	.subscribe_event = tda1997x_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/* -----------------------------------------------------------------------------
 * v4l2_subdev_ops
 */

static const struct v4l2_subdev_ops tda1997x_subdev_ops = {
	.core = &tda1997x_core_ops,
	.video = &tda1997x_video_ops,
	.pad = &tda1997x_pad_ops,
};

static int tda1997x_core_init(struct v4l2_subdev *sd)
{
	struct tda1997x_state *state = to_state(sd);
	struct tda1997x_platform_data *pdata = &state->pdata;
	u8 reg;
	int i;

	/* disable HPD */
	io_write(sd, REG_HPD_AUTO_CTRL, HPD_AUTO_HPD_UNSEL);
	if (state->chip_revision == 0) {
		io_write(sd, REG_MAN_SUS_HDMI_SEL, MAN_DIS_HDCP | MAN_RST_HDCP);
		io_write(sd, REG_CGU_DBG_SEL, 1 << CGU_DBG_CLK_SEL_SHIFT);
	}

	/* reset infoframe at end of start-up-sequencer */
	io_write(sd, REG_SUS_SET_RGB2, 0x06);
	io_write(sd, REG_SUS_SET_RGB3, 0x06);

	/* Enable TMDS pull-ups */
	io_write(sd, REG_RT_MAN_CTRL, RT_MAN_CTRL_RT |
		 RT_MAN_CTRL_RT_B | RT_MAN_CTRL_RT_A);

	/* enable sync measurement timing */
	tda1997x_cec_write(sd, REG_PWR_CONTROL & 0xff, 0x04);
	/* adjust CEC clock divider */
	tda1997x_cec_write(sd, REG_OSC_DIVIDER & 0xff, 0x03);
	tda1997x_cec_write(sd, REG_EN_OSC_PERIOD_LSB & 0xff, 0xa0);
	io_write(sd, REG_TIMER_D, 0x54);
	/* enable power switch */
	reg = tda1997x_cec_read(sd, REG_CONTROL & 0xff);
	reg |= 0x20;
	tda1997x_cec_write(sd, REG_CONTROL & 0xff, reg);
	mdelay(50);

	/* read the chip version */
	reg = io_read(sd, REG_VERSION);
	/* get the chip configuration */
	reg = io_read(sd, REG_CMTP_REG10);

	/* enable interrupts we care about */
	io_write(sd, REG_INT_MASK_TOP,
		 INTERRUPT_HDCP | INTERRUPT_AUDIO | INTERRUPT_INFO |
		 INTERRUPT_RATE | INTERRUPT_SUS);
	/* config_mtp,fmt,sus_end,sus_st */
	io_write(sd, REG_INT_MASK_SUS, MASK_MPT | MASK_FMT | MASK_SUS_END);
	/* rate stability change for inputs A/B */
	io_write(sd, REG_INT_MASK_RATE, MASK_RATE_B_ST | MASK_RATE_A_ST);
	/* aud,spd,avi*/
	io_write(sd, REG_INT_MASK_INFO,
		 MASK_AUD_IF | MASK_SPD_IF | MASK_AVI_IF);
	/* audio_freq,audio_flg,mute_flg,fifo_err */
	io_write(sd, REG_INT_MASK_AUDIO,
		 MASK_AUDIO_FREQ_FLG | MASK_AUDIO_FLG | MASK_MUTE_FLG |
		 MASK_ERROR_FIFO_PT);
	/* HDCP C5 state reached */
	io_write(sd, REG_INT_MASK_HDCP, MASK_STATE_C5);
	/* don't care about AFE/DDC/MODE */
	io_write(sd, REG_INT_MASK_AFE, 0);
	io_write(sd, REG_INT_MASK_DDC, 0);
	io_write(sd, REG_INT_MASK_MODE, 0);

	/* clear all interrupts */
	io_write(sd, REG_INT_FLG_CLR_TOP, 0xff);
	io_write(sd, REG_INT_FLG_CLR_SUS, 0xff);
	io_write(sd, REG_INT_FLG_CLR_DDC, 0xff);
	io_write(sd, REG_INT_FLG_CLR_RATE, 0xff);
	io_write(sd, REG_INT_FLG_CLR_MODE, 0xff);
	io_write(sd, REG_INT_FLG_CLR_INFO, 0xff);
	io_write(sd, REG_INT_FLG_CLR_AUDIO, 0xff);
	io_write(sd, REG_INT_FLG_CLR_HDCP, 0xff);
	io_write(sd, REG_INT_FLG_CLR_AFE, 0xff);

	/* init TMDS equalizer */
	if (state->chip_revision == 0)
		io_write(sd, REG_CGU_DBG_SEL, 1 << CGU_DBG_CLK_SEL_SHIFT);
	io_write24(sd, REG_CLK_MIN_RATE, CLK_MIN_RATE);
	io_write24(sd, REG_CLK_MAX_RATE, CLK_MAX_RATE);
	if (state->chip_revision == 0)
		io_write(sd, REG_WDL_CFG, WDL_CFG_VAL);
	/* DC filter */
	io_write(sd, REG_DEEP_COLOR_CTRL, DC_FILTER_VAL);
	/* disable test pattern */
	io_write(sd, REG_SVC_MODE, 0x00);
	/* update HDMI INFO CTRL */
	io_write(sd, REG_INFO_CTRL, 0xff);
	/* write HDMI INFO EXCEED value */
	io_write(sd, REG_INFO_EXCEED, 3);

	if (state->chip_revision == 0)
		tda1997x_reset_n1(state);

	/*
	 * No HDCP acknowledge when HDCP is disabled
	 * and reset SUS to force format detection
	 */
	tda1997x_hdmi_info_reset(sd, NACK_HDCP, true);

	/* Set HPD low */
	tda1997x_manual_hpd(sd, HPD_LOW_BP);

	/* Configure receiver capabilities */
	io_write(sd, REG_HDCP_BCAPS, HDCP_HDMI | HDCP_FAST_REAUTH);

	/* Configure HDMI: Auto HDCP mode, packet controlled mute */
	reg = HDMI_CTRL_MUTE_AUTO << HDMI_CTRL_MUTE_SHIFT;
	reg |= HDMI_CTRL_HDCP_AUTO << HDMI_CTRL_HDCP_SHIFT;
	io_write(sd, REG_HDMI_CTRL, reg);

	/* reset start-up-sequencer to force format detection */
	tda1997x_hdmi_info_reset(sd, 0, true);

	/* Set HPD high */
	tda1997x_manual_hpd(sd, HPD_HIGH_OTHER);
	tda1997x_manual_hpd(sd, HPD_HIGH_BP);

	/* disable matrix conversion */
	reg = io_read(sd, REG_VDP_CTRL);
	reg |= VDP_CTRL_MATRIX_BP;
	io_write(sd, REG_VDP_CTRL, reg);

	/* set video output mode */
	tda1997x_configure_vidout(state);

	/* configure video output port */
	for (i = 0; i < 9; i++) {
		v4l_dbg(1, debug, state->client, "vidout_cfg[%d]=0x%02x\n", i,
			pdata->vidout_port_cfg[i]);
		io_write(sd, REG_VP35_32_CTRL + i, pdata->vidout_port_cfg[i]);
	}

	/* configure audio output port */
	tda1997x_configure_audout(sd, 0);

	/* configure audio clock freq */
	io_write(sd, REG_AUDIO_CLOCK, pdata->audout_sysclk);

	/* reset advanced infoframes (ISRC1/ISRC2/ACP) */
	tda1997x_hdmi_info_reset(sd, RESET_AI, false);
	/* reset infoframe */
	tda1997x_hdmi_info_reset(sd, RESET_IF, false);
	/* reset audio infoframes */
	tda1997x_hdmi_info_reset(sd, RESET_AUDIO, false);
	/* reset gamut */
	tda1997x_hdmi_info_reset(sd, RESET_GAMUT, false);

	/* get initial HDMI status */
	state->hdmi_status = io_read(sd, REG_HDMI_FLAGS);

	return 0;
}

static int tda1997x_set_power(struct tda1997x_state *state, bool on)
{
	int ret = 0;

	if (on) {
		ret = regulator_bulk_enable(TDA1997X_NUM_SUPPLIES,
					     state->supplies);
		msleep(300);
	}

	ret = regulator_bulk_disable(TDA1997X_NUM_SUPPLIES,
			       state->supplies);
	return ret;
}

static const struct i2c_device_id tda1997x_i2c_id[] = {
	{"tda19971", (kernel_ulong_t)&tda1997x_chip_info[TDA19971]},
	{"tda19973", (kernel_ulong_t)&tda1997x_chip_info[TDA19973]},
	{ },
};
MODULE_DEVICE_TABLE(i2c, tda1997x_i2c_id);

static const struct of_device_id tda1997x_of_id[] __maybe_unused = {
	{ .compatible = "nxp,tda19971", .data = &tda1997x_chip_info[TDA19971] },
	{ .compatible = "nxp,tda19973", .data = &tda1997x_chip_info[TDA19973] },
	{ },
};
MODULE_DEVICE_TABLE(of, tda1997x_of_id);

static int tda1997x_parse_dt_audio(struct tda1997x_state *state,
				   struct device_node *np)
{
	struct tda1997x_platform_data *pdata = &state->pdata;
	const u32 *port_data;
	u32 v;

	port_data = of_get_property(np, "audio-port", &v);
	if (!port_data)
		return 0; /* no audio output */
	v /= sizeof(u32);
	if (v != 3) {
		dev_err(&state->client->dev, "Invalid audio-port property\n");
		return -EINVAL;
	}

	/* audio output format */
	v = be32_to_cpup(&port_data[0]);
	if (v > TDA1997X_SPDIF_HBR_DEMUX) {
		dev_err(&state->client->dev, "Invalid audio format %u\n", v);
		return -EINVAL;
	}
	pdata->audout_fmt = v;

	/* audio output layout */
	v = be32_to_cpup(&port_data[1]);
	if (v > TDA1997X_LAYOUT1) {
		dev_err(&state->client->dev, "Invalid audio layout %u\n", v);
		return -EINVAL;
	}
	pdata->audout_layout = v;

	/* audio output clock freq */
	v = be32_to_cpup(&port_data[2]);
	if (v > TDA1997X_ACLK_512FS) {
		dev_err(&state->client->dev, "Invalid audio clock %u\n", v);
		return -EINVAL;
	}
	pdata->audout_sysclk = v;

	return 0;
}

static int tda1997x_parse_dt(struct tda1997x_state *state)
{
	struct tda1997x_platform_data *pdata = &state->pdata;
	struct v4l2_fwnode_endpoint bus_cfg;
	struct device_node *ep;
	struct device_node *np;
	unsigned int flags;
	int ret;
	u32 v;

	/*
	 * setup default values:
	 * - HREF: active high from start to end of row
	 * - VS: Vertical Sync active high at beginning of frame
	 * - DE: Active high when data valid
	 * - A_CLK: 128*Fs
	 */
	pdata->vidout_sel_hs = HS_HREF_SEL_HREF_VHREF;
	pdata->vidout_sel_vs = VS_VREF_SEL_VREF_HDMI;
	pdata->vidout_sel_de = DE_FREF_SEL_DE_VHREF;

	np = state->client->dev.of_node;
	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);
	if (ret) {
		of_node_put(ep);
		return ret;
	}
	of_node_put(ep);

	/* polarity of HS/VS/DE */
	flags = bus_cfg.bus.parallel.flags;
	if (flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		pdata->vidout_inv_hs = 1;
	if (flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		pdata->vidout_inv_vs = 1;
	if (flags & V4L2_MBUS_DATA_ACTIVE_LOW)
		pdata->vidout_inv_de = 1;
	pdata->vidout_bus_width = bus_cfg.bus.parallel.bus_width;

	/* video output port config */
	ret = of_property_count_u32_elems(np, "vidout_portcfg");
	if (ret) {
		u32 reg, val, i;

		for (i = 0; i < ret / 2 && i < 9; i++) {
			of_property_read_u32_index(np, "vidout_portcfg",
						   i * 2, &reg);
			of_property_read_u32_index(np, "vidout_portcfg",
						   i * 2 + 1, &val);
			if (reg < 9)
				pdata->vidout_port_cfg[reg] = val;
		}
	}

	/* max pixrate */
	if (!of_property_read_u32(np, "max-pixel-rate", &v))
		pdata->max_pixel_rate = v;
	else {
		v4l_err(state->client, "get of property max-pixel-rate fail\n");
		return -EINVAL;
	}

	return tda1997x_parse_dt_audio(state, np);
}

static int tda1997x_get_regulators(struct tda1997x_state *state)
{
	int i;

	for (i = 0; i < TDA1997X_NUM_SUPPLIES; i++)
		state->supplies[i].supply = tda1997x_supply_name[i];

	return devm_regulator_bulk_get(&state->client->dev,
				       TDA1997X_NUM_SUPPLIES,
				       state->supplies);
}

static int tda1997x_identify_module(struct tda1997x_state *state)
{
	struct v4l2_subdev *sd = &state->sd;
	enum tda1997x_type type;
	u8 reg;

	/* Read chip configuration*/
	reg = io_read(sd, REG_CMTP_REG10);
	state->tmdsb_clk = (reg >> 6) & 0x01; /* use tmds clock B_inv for B */
	state->tmdsb_soc = (reg >> 5) & 0x01; /* tmds of input B */
	state->port_30bit = (reg >> 2) & 0x03; /* 30bit vs 24bit */
	state->output_2p5 = (reg >> 1) & 0x01; /* output supply 2.5v */
	switch ((reg >> 4) & 0x03) {
	case 0x00:
		type = TDA19971;
		break;
	case 0x02:
	case 0x03:
		type = TDA19973;
		break;
	default:
		dev_err(&state->client->dev, "unsupported chip ID\n");
		return -EIO;
	}
	if (state->info->type != type) {
		dev_err(&state->client->dev, "chip id mismatch\n");
		return -EIO;
	}

	/* read chip revision */
	state->chip_revision = io_read(sd, REG_CMTP_REG11);

	return 0;
}

static const struct media_entity_operations tda1997x_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int tda1997x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tda1997x_state *state;
	struct tda1997x_platform_data *pdata;
	struct v4l2_subdev *sd;
	int ret;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	state = kzalloc(sizeof(struct tda1997x_state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->client = client;
	pdata = &state->pdata;
	if (IS_ENABLED(CONFIG_OF) && client->dev.of_node) {
		const struct of_device_id *oid;

		oid = of_match_node(tda1997x_of_id, client->dev.of_node);
		state->info = oid->data;

		ret = tda1997x_parse_dt(state);
		if (ret < 0) {
			v4l_err(client, "DT parsing error\n");
			goto err_free_state;
		}
	} else if (client->dev.platform_data) {
		struct tda1997x_platform_data *pdata =
			client->dev.platform_data;
		state->info =
			(const struct tda1997x_chip_info *)id->driver_data;
		state->pdata = *pdata;
	} else {
		v4l_err(client, "No platform data\n");
		ret = -ENODEV;
		goto err_free_state;
	}

	ret = tda1997x_get_regulators(state);
	if (ret)
		goto err_free_state;

	ret = tda1997x_set_power(state, 1);
	if (ret)
		goto err_free_state;

	mutex_init(&state->page_lock);
	mutex_init(&state->lock);
	state->page = 0xff;

	/* set video format based on chip and bus width */
	ret = tda1997x_identify_module(state);
	if (ret)
		goto err_free_mutex;

	/* initialize subdev */
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tda1997x_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		 id->name, i2c_adapter_id(client->adapter),
		 client->addr);
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.ops = &tda1997x_media_ops;

	switch (state->info->type) {
	case TDA19973:
		switch (pdata->vidout_bus_width) {
		case 36: /* 36bit RGB (could also be 36bit YUV) */
			state->code = MEDIA_BUS_FMT_YUV12_1X36;
			break;
		case 24: /* 24bit BT656 (YUV422 semi-planar: 1-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY12_1X24;
			break;
		case 12: /* 12bit BT656 (2-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY12_2X12;
			break;
		}
		break;
	case TDA19971:
		switch (pdata->vidout_bus_width) {
		case 24: /* 24bit YUV (could also be RGB or YUV422) */
			state->code = MEDIA_BUS_FMT_YUV8_1X24;
			break;
		case 20: /* 20bit YUV422 */
			state->code = MEDIA_BUS_FMT_UYVY10_1X20;
			break;
		case 16: /* 16bit BT656 (YUV422 semi-planar: 1-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY8_1X16;
			break;
		case 12: /* 12bit BT656 (2-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY12_2X12;
			break;
		case 10: /* 10bit BT656 (2-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY10_2X10;
			break;
		case 8: /* 8bit BT656 (2-cycle) */
			state->code = MEDIA_BUS_FMT_UYVY8_2X8;
			break;
		}
	}
	switch (state->code) {
	case MEDIA_BUS_FMT_RGB121212_1X36:
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_YUV12_1X36:
	case MEDIA_BUS_FMT_YUV8_1X24:
		state->bus_type = V4L2_MBUS_PARALLEL;
		state->vid_fmt = OF_FMT_444;
		break;
	case MEDIA_BUS_FMT_UYVY12_1X24:
	case MEDIA_BUS_FMT_UYVY8_1X16:
		state->bus_type = V4L2_MBUS_BT656;
		state->vid_fmt = OF_FMT_422_SMPT;
		break;
	case MEDIA_BUS_FMT_UYVY12_2X12:
	case MEDIA_BUS_FMT_UYVY10_2X10:
	case MEDIA_BUS_FMT_UYVY8_2X8:
		state->bus_type = V4L2_MBUS_BT656;
		state->vid_fmt = OF_FMT_422_CCIR;
		break;
	default:
		v4l_err(state->client, "incompatible code (%x)\n", state->code);
		ret = -EINVAL;
		goto err_free_mutex;
	}
	state->colorspace = HDMI_COLORSPACE_RGB;
	state->colorimetry = HDMI_COLORIMETRY_NONE;

	/* disable/reset HDCP to get correct I2C access to Rx HDMI */
	io_write(sd, REG_MAN_SUS_HDMI_SEL, MAN_RST_HDCP | MAN_DIS_HDCP);

	/*
	 * if N2 version, reset compdel_bp as it may generate some small pixel
	 * shifts in case of embedded sync/or delay lower than 4
	 */
	if (state->chip_revision != 0) {
		io_write(sd, REG_MAN_SUS_HDMI_SEL, 0x00);
		io_write(sd, REG_VDP_CTRL, 0x1f);
	}

	v4l_info(client, "NXP %s N%d detected\n", state->info->name,
		 state->chip_revision + 1);
	v4l_info(client, "video: %dbit %s %s MBUS_FMT=0x%x\n",
		pdata->vidout_bus_width,
		(state->bus_type == V4L2_MBUS_PARALLEL) ? "parallel" : "BT656",
		vidfmt_names[state->vid_fmt & OF_FMT_MASK],
		state->code);
	if (pdata->audout_fmt) {
		v4l_info(client, "audio: %dch %s sysclk=%d*fs\n",
			 pdata->audout_layout ? 2 : 8,
			 audfmt_names[pdata->audout_fmt],
			 1 << (pdata->audout_sysclk + 4));
	}

	ret = 0x34 + ((io_read(sd, REG_SLAVE_ADDR)>>4) & 0x03);
	state->client_cec = i2c_new_dummy(client->adapter, ret);
	v4l_info(client, "CEC slave address 0x%02x\n", ret);

	if (pdata->max_pixel_rate) {
		v4l_info(client, "max pixel rate: %d MP/sec\n",
			 pdata->max_pixel_rate);
	}

	ret = tda1997x_core_init(sd);
	if (ret)
		goto err_free_mutex;

	/* initialize source pads */
	state->pads[TDA1997X_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, TDA1997X_NUM_PADS,
		state->pads);
	if (ret) {
		v4l_err(client, "failed entity_init: %d", ret);
		goto err_free_mutex;
	}

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_free_media;

	/* TODO: create/register audio codec */

	/* request irq */
	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, tda1997x_isr_thread,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					KBUILD_MODNAME, state);
	if (ret) {
		v4l_err(client, "irq%d reg failed: %d\n", client->irq, ret);
		goto err_free_media;
	}

	return 0;

err_free_media:
	media_entity_cleanup(&sd->entity);
err_free_mutex:
	mutex_destroy(&state->page_lock);
	mutex_destroy(&state->lock);
err_free_state:
	kfree(state);
	dev_err(&client->dev, "%s failed: %d\n", __func__, ret);

	return ret;
}

static int tda1997x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tda1997x_state *state = to_state(sd);

	disable_irq(state->client->irq);
	tda1997x_power_mode(state, 0);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	regulator_bulk_disable(TDA1997X_NUM_SUPPLIES, state->supplies);
	i2c_unregister_device(state->client_cec);
	mutex_destroy(&state->page_lock);
	mutex_destroy(&state->lock);

	kfree(state);

	return 0;
}

static struct i2c_driver tda1997x_i2c_driver = {
	.driver = {
		.name = "tda1997x",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tda1997x_of_id),
	},
	.probe = tda1997x_probe,
	.remove = tda1997x_remove,
	.id_table = tda1997x_i2c_id,
};

module_i2c_driver(tda1997x_i2c_driver);

MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_DESCRIPTION("TDA1997X HDMI Receiver driver");
MODULE_LICENSE("GPL v2");
