/******************************************************************************
 *
 * Copyright(c) 2009-2013  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Christian Lamparter <chunkeey@googlemail.com>
 * Joshua Roys <Joshua.Roys@gtri.gatech.edu>
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#ifndef __R92SU_REG_H__
#define __R92SU_REG_H__

/* Hardware / Firmware Memory Layout */
#define RTL8712_IOBASE_TXPKT		(0x10200000)
#define RTL8712_IOBASE_RXPKT		(0x10210000)
#define RTL8712_IOBASE_RXCMD		(0x10220000)
#define RTL8712_IOBASE_TXSTATUS		(0x10230000)
#define RTL8712_IOBASE_RXSTATUS		(0x10240000)
#define RTL8712_IOBASE_IOREG		(0x10250000)
#define RTL8712_IOBASE_SCHEDULER	(0x10260000)

#define RTL8712_IOBASE_TRXDMA		(0x10270000)
#define RTL8712_IOBASE_TXLLT		(0x10280000)
#define RTL8712_IOBASE_WMAC		(0x10290000)
#define RTL8712_IOBASE_FW2HW		(0x102A0000)
#define RTL8712_IOBASE_ACCESS_PHYREG	(0x102B0000)

#define RTL8712_IOBASE_FF		(0x10300000)

/* IOREG Offsets for 8712 and friends */
#define RTL8712_SYSCFG_			(RTL8712_IOBASE_IOREG)
#define RTL8712_CMDCTRL_		(RTL8712_IOBASE_IOREG + 0x40)
#define RTL8712_MACIDSETTING_		(RTL8712_IOBASE_IOREG + 0x50)
#define RTL8712_TIMECTRL_		(RTL8712_IOBASE_IOREG + 0x80)
#define RTL8712_FIFOCTRL_		(RTL8712_IOBASE_IOREG + 0xA0)
#define RTL8712_RATECTRL_		(RTL8712_IOBASE_IOREG + 0x160)
#define RTL8712_EDCASETTING_		(RTL8712_IOBASE_IOREG + 0x1D0)
#define RTL8712_WMAC_			(RTL8712_IOBASE_IOREG + 0x200)
#define RTL8712_SECURITY_		(RTL8712_IOBASE_IOREG + 0x240)
#define RTL8712_POWERSAVE_		(RTL8712_IOBASE_IOREG + 0x260)
#define RTL8712_BB_			(RTL8712_IOBASE_IOREG + 0x2C0)
#define RTL8712_OFFLOAD_		(RTL8712_IOBASE_IOREG + 0x2D0)
#define RTL8712_GP_			(RTL8712_IOBASE_IOREG + 0x2E0)
#define RTL8712_INTERRUPT_		(RTL8712_IOBASE_IOREG + 0x300)
#define RTL8712_DEBUGCTRL_		(RTL8712_IOBASE_IOREG + 0x310)
#define RTL8712_IOCMD_			(RTL8712_IOBASE_IOREG + 0x370)
#define RTL8712_PHY_			(RTL8712_IOBASE_IOREG + 0x800)
#define RTL8712_PHY_P1_			(RTL8712_IOBASE_IOREG + 0x900)
#define RTL8712_PHY_CCK_		(RTL8712_IOBASE_IOREG + 0xA00)
#define RTL8712_PHY_OFDM_		(RTL8712_IOBASE_IOREG + 0xC00)
#define RTL8712_PHY_RXAGC_		(RTL8712_IOBASE_IOREG + 0xE00)
#define RTL8712_USB_			(RTL8712_IOBASE_IOREG + 0xFE00)

/* ----------------------------------------------------- */
/* 0x10250000h ~ 0x1025003Fh System Configuration */
/* ----------------------------------------------------- */
#define REG_SYS_ISO_CTRL		(RTL8712_SYSCFG_ + 0x0000)
#define		ISO_MD2PP			BIT(0)
#define		ISO_PA2PCIE			BIT(3)
#define		ISO_PLL2MD			BIT(4)
#define		ISO_PWC_DV2RP			BIT(11)
#define		ISO_PWC_RV2RP			BIT(12)

#define REG_SYS_FUNC_EN			(RTL8712_SYSCFG_ + 0x0002)
#define		FEN_CPUEN			BIT(10)
#define		FEN_DCORE			BIT(11)
#define		FEN_MREGEN			BIT(15)

#define REG_PMC_FSM			(RTL8712_SYSCFG_ + 0x0004)
#define		PCM_FSM_VER			0x0000f8000
#define		PCM_FSM_VER_S			15

#define REG_SYS_CLKR			(RTL8712_SYSCFG_ + 0x0008)
#define		SYS_CLKSEL_80M			BIT(0)
#define		SYS_PS_CLKSEL			BIT(1)
#define		SYS_CPU_CLKSEL			BIT(2)
#define		SYS_MAC_CLK_EN			BIT(11)
#define		SYS_SWHW_SEL			BIT(14)
#define		SYS_FWHW_SEL			BIT(15)

#define REG_EE_9346CR			(RTL8712_SYSCFG_ + 0x000A)
#define REG_EEPROM_CMD			(RTL8712_SYSCFG_ + 0x000A)/* DUP */
#define		EEPROM_CMD_93C46		BIT(4)
#define		EEPROM_CMD_AUTOLOAD_OK		BIT(5)

#define REG_EE_VPD			(RTL8712_SYSCFG_ + 0x000C)
#define REG_AFE_MISC			(RTL8712_SYSCFG_ + 0x0010)
#define		AFE_BGEN			BIT(0)
#define		AFE_MBEN			BIT(1)
#define		AFE_MISC_I32_EN			BIT(3)

#define REG_SPS0_CTRL			(RTL8712_SYSCFG_ + 0x0011)
#define REG_SPS1_CTRL			(RTL8712_SYSCFG_ + 0x0018)
#define REG_RF_CTRL			(RTL8712_SYSCFG_ + 0x001F)
#define REG_LDOA15_CTRL			(RTL8712_SYSCFG_ + 0x0020)
#define		LDA15_EN			BIT(0)

#define REG_LDOV12D_CTRL		(RTL8712_SYSCFG_ + 0x0021)
#define		LDV12_EN			BIT(0)
#define		LDV12_SDBY			BIT(1)

#define REG_LDOHCI12_CTRL		(RTL8712_SYSCFG_ + 0x0022)
#define REG_LDO_USB_CTRL		(RTL8712_SYSCFG_ + 0x0023)
#define REG_LPLDO_CTRL			(RTL8712_SYSCFG_ + 0x0024)
#define REG_AFE_XTAL_CTRL		(RTL8712_SYSCFG_ + 0x0026)
#define REG_AFE_PLL_CTRL		(RTL8712_SYSCFG_ + 0x0028)
#define REG_EFUSE_CTRL			(RTL8712_SYSCFG_ + 0x0030)
#define REG_EFUSE_TEST			(RTL8712_SYSCFG_ + 0x0034)
#define REG_PWR_DATA			(RTL8712_SYSCFG_ + 0x0038)
#define REG_DPS_TIMER			(RTL8712_SYSCFG_ + 0x003C)
#define REG_RCLK_MON			(RTL8712_SYSCFG_ + 0x003E)
#define REG_EFUSE_CLK_CTRL		(RTL8712_SYSCFG_ + 0x02F8)

/* ----------------------------------------------------- */
/* 0x10250040h ~ 0x1025004Fh Command Control */
/* ----------------------------------------------------- */
#define REG_CR				(RTL8712_CMDCTRL_ + 0x0000)
#define		HCI_TXDMA_EN			BIT(2)
#define		HCI_RXDMA_EN			BIT(3)
#define		TXDMA_EN			BIT(4)
#define		RXDMA_EN			BIT(5)
#define		FW2HW_EN			BIT(6)
#define		DDMA_EN				BIT(7)
#define		MACTXEN				BIT(8)
#define		MACRXEN				BIT(9)
#define		SCHEDULE_EN			BIT(10)
#define		BB_GLB_RSTN			BIT(12)
#define		BBRSTN				BIT(13)
#define		APSDOFF				BIT(14)
#define		APSDOFF_STATUS			BIT(15)

#define REG_TXPAUSE			(RTL8712_CMDCTRL_ + 0x0002)
#define		STOPBK				BIT(0)
#define		STOPBE				BIT(1)
#define		STOPVI				BIT(2)
#define		STOPVO				BIT(3)
#define		STOPMGT				BIT(4)
#define		STOPHIGH			BIT(5)
#define		STOPHCCA			BIT(6)

#define	REG_LBKMD_SEL			(RTL8712_CMDCTRL_ + 0x0003)
#define		LBK_NORMAL			0x00
#define		LBK_MAC_DLB			(BIT(0) | BIT(1))
#define		LBK_DMA_LB			(BIT(0) | BIT(1) | BIT(2))
#define		LBK_MAC_LB			(BIT(0) | BIT(1) | BIT(3))

#define REG_TCR				(RTL8712_CMDCTRL_ + 0x0004)
#define		IMEM_CODE_DONE			BIT(0)
#define		IMEM_CHK_RPT			BIT(1)
#define		EMEM_CODE_DONE			BIT(2)
#define		EXT_IMEM_CODE_DONE		BIT(2)	/* DUP */
#define		EMEM_CHK_RPT			BIT(3)
#define		EXT_IMEM_CHK_RPT		BIT(3)	/* DUP */
#define		DMEM_CODE_DONE			BIT(4)
#define		IMEM_RDY			BIT(5)
#define		IMEM				BIT(5)	/* DUP */
#define		BASECHG				BIT(6)
#define		FWRDY				BIT(7)
#define		TSFEN				BIT(8)
#define		TCR_TSFEN			BIT(8)  /* DUP */
#define		TSFRST				BIT(9)
#define		TCR_TSFRST			BIT(9)  /* DUP */
#define		FAKE_IMEM_EN			BIT(15)
#define		TCR_FAKE_IMEM_EN		BIT(15)  /* DUP */
#define		TCRCRC				BIT(16)
#define		TCR_CRC				BIT(16)  /* DUP */
#define		CFENDFORM			BIT(17)
#define		TCRICV				BIT(19)
#define		TCR_ICV				BIT(19)  /* DUP */
#define		DISCW				BIT(20)
#define		TCR_DISCW			BIT(20)  /* DUP */
#define		TXDMAPRE2FULL			BIT(23)
#define		HWPC_TX_EN			BIT(24)
#define		TCP_OFDL_EN			BIT(25)
#define		TCR_HWPC_TX_EN			BIT(24)  /* DUP */
#define		TCR_TCP_OFDL_EN			BIT(25)
#define		FWALLRDY			(BIT(0) | BIT(1) | BIT(2) | \
						BIT(3) | BIT(4) | BIT(5) | \
						BIT(6) | BIT(7))
#define		LOAD_FW_READY			(IMEM_CODE_DONE | \
						IMEM_CHK_RPT | \
						EMEM_CODE_DONE | \
						EMEM_CHK_RPT | \
						DMEM_CODE_DONE | \
						IMEM_RDY | \
						BASECHG | \
						FWRDY)
#define		TXDMA_INIT_VALUE		(IMEM_CHK_RPT | \
						EXT_IMEM_CHK_RPT)

#define REG_RCR				(RTL8712_CMDCTRL_ + 0x0008)
#define		RCR_AAP				BIT(0)
#define		RCR_APM				BIT(1)
#define		RCR_AM				BIT(2)
#define		RCR_AB				BIT(3)
#define		RCR_RXSHFT_EN			BIT(4)
#define		RCR_ACRC32			BIT(5)
#define		RCR_APP_BA_SSN			BIT(6)
#define		RCR_MXDMA_OFFSET		8
#define		RCR_RXDESC_LK_EN		BIT(11)
#define		RCR_AICV			BIT(12)
#define		RCR_RXFTH			BIT(13)
#define		RCR_FIFO_OFFSET			13	/* or 14? */
#define		RCR_APP_ICV			BIT(16)
#define		RCR_APP_MIC			BIT(17)
#define		RCR_ADF				BIT(18)
#define		RCR_ACF				BIT(19)
#define		RCR_AMF				BIT(20)
#define		RCR_ADD3			BIT(21)
#define		RCR_APWRMGT			BIT(22)
#define		RCR_CBSSID			BIT(23)
#define		RCR_APP_PHYST_STAFF		BIT(24)
#define		RCR_APP_PHYST_RXFF		BIT(25)
#define		RCR_RX_TCPOFDL_EN		BIT(26)
#define		RCR_ENMBID			BIT(27)
#define		RCR_HTC_LOC_CTRL		BIT(28)
#define		RCR_DIS_AES_2BYTE		BIT(29)
#define		RCR_DIS_ENC_2BYTE		BIT(30)
#define		RCR_APPFCS			BIT(31)

#define REG_MSR				(RTL8712_CMDCTRL_ + 0x000C)
#define		MSR_LINK_MASK			(MSR_LINK_NONE |	\
						 MSR_LINK_MANAGED |	\
						 MSR_LINK_ADHOC |	\
						 MSR_LINK_MASTER)
#define		MSR_LINK_NONE			0
#define		MSR_LINK_SHIFT			0
#define		MSR_LINK_ADHOC			BIT(0)
#define		MSR_LINK_MANAGED		BIT(1)
#define		MSR_LINK_MASTER			BIT(2)

#define REG_SYSF_CFG			(RTL8712_CMDCTRL_ + 0x000D)
#define REG_MBIDCTRL			(RTL8712_CMDCTRL_ + 0x000E)

/* ----------------------------------------------------- */
/* 0x10250050h ~ 0x1025007Fh MAC ID Settings */
/* ----------------------------------------------------- */
#define REG_MACID			(RTL8712_MACIDSETTING_ + 0x0000)
#define REG_BSSIDR			(RTL8712_MACIDSETTING_ + 0x0008)
#define REG_HWVID			(RTL8712_MACIDSETTING_ + 0x000E)
#define REG_MAR				(RTL8712_MACIDSETTING_ + 0x0010)
#define REG_MBIDCANCONTENT		(RTL8712_MACIDSETTING_ + 0x0018)
#define REG_MBIDCANCFG			(RTL8712_MACIDSETTING_ + 0x0020)
#define REG_BUILDTIME			(RTL8712_MACIDSETTING_ + 0x0024)
#define REG_BUILDUSER			(RTL8712_MACIDSETTING_ + 0x0028)

/* ----------------------------------------------------- */
/* 0x10250080h ~ 0x1025009Fh Timer Control */
/* ----------------------------------------------------- */
#define REG_TSFTR			(RTL8712_TIMECTRL_ + 0x00)
#define REG_USTIME			(RTL8712_TIMECTRL_ + 0x08)
#define REG_SLOT_TIME			(RTL8712_TIMECTRL_ + 0x09)
#define REG_TUBASE			(RTL8712_TIMECTRL_ + 0x0A)
#define REG_SIFS_CCK			(RTL8712_TIMECTRL_ + 0x0C)
#define REG_SIFS_OFDM			(RTL8712_TIMECTRL_ + 0x0E)
#define REG_PIFS			(RTL8712_TIMECTRL_ + 0x10)
#define REG_ACK_TIMEOUT			(RTL8712_TIMECTRL_ + 0x11)
#define REG_EIFS			(RTL8712_TIMECTRL_ + 0x12)
#define REG_BCN_INTERVAL		(RTL8712_TIMECTRL_ + 0x14)
#define REG_ATIMWND			(RTL8712_TIMECTRL_ + 0x16)
#define REG_DRVERLYINT			(RTL8712_TIMECTRL_ + 0x18)
#define REG_BCNDMATIM			(RTL8712_TIMECTRL_ + 0x1A)
#define REG_BCNERRTH			(RTL8712_TIMECTRL_ + 0x1C)
#define REG_MLT				(RTL8712_TIMECTRL_ + 0x1D)

/* ----------------------------------------------------- */
/* 0x102500A0h ~ 0x1025015Fh FIFO Control */
/* ----------------------------------------------------- */
#define REG_RQPN			(RTL8712_FIFOCTRL_ + 0x00)
#define REG_RXFF_BNDY			(RTL8712_FIFOCTRL_ + 0x0C)
#define REG_RXRPT_BNDY			(RTL8712_FIFOCTRL_ + 0x10)
#define REG_TXPKTBUF_PGBNDY		(RTL8712_FIFOCTRL_ + 0x14)
#define REG_PBP				(RTL8712_FIFOCTRL_ + 0x15)
#define		PBP_PAGE_128B			BIT(0)
#define REG_RX_DRVINFO_SZ		(RTL8712_FIFOCTRL_ + 0x16)
#define REG_TXFF_STATUS			(RTL8712_FIFOCTRL_ + 0x17)
#define REG_RXFF_STATUS			(RTL8712_FIFOCTRL_ + 0x18)
#define REG_TXFF_EMPTY_TH		(RTL8712_FIFOCTRL_ + 0x19)
#define REG_SDIO_RX_BLKSZ		(RTL8712_FIFOCTRL_ + 0x1C)
#define REG_RXDMA_RXCTRL		(RTL8712_FIFOCTRL_ + 0x1D)
#define		RXDMA_AGG_EN			BIT(7)

#define REG_RXPKT_NUM			(RTL8712_FIFOCTRL_ + 0x1E)
#define REG_RXPKT_NUM_C2H		(RTL8712_FIFOCTRL_ + 0x1F)
#define REG_C2HCMD_UDT_SIZE		(RTL8712_FIFOCTRL_ + 0x20)
#define REG_C2HCMD_UDT_ADDR		(RTL8712_FIFOCTRL_ + 0x22)
#define REG_FIFOPAGE2			(RTL8712_FIFOCTRL_ + 0x24)
/* PAGE1 after PAGE2?? */
#define REG_FIFOPAGE1			(RTL8712_FIFOCTRL_ + 0x28)
#define REG_FW_RSVD_PG_CTRL		(RTL8712_FIFOCTRL_ + 0x30)
#define	REG_FIFOPAGE5			(RTL8712_FIFOCTRL_ + 0x34)
#define	REG_FW_RSVD_PG_CRTL		(RTL8712_FIFOCTRL_ + 0x38)
#define	REG_RXDMA_AGG_PG_TH		(RTL8712_FIFOCTRL_ + 0x39)
#define	REG_TXDESC_MSK			(RTL8712_FIFOCTRL_ + 0x3C)
#define REG_TXRPTFF_RDPTR		(RTL8712_FIFOCTRL_ + 0x40)
#define REG_TXRPTFF_WTPTR		(RTL8712_FIFOCTRL_ + 0x44)
#define REG_C2HFF_RDPTR			(RTL8712_FIFOCTRL_ + 0x48)
#define REG_C2HFF_WTPTR			(RTL8712_FIFOCTRL_ + 0x4C)
#define REG_RXFF0_RDPTR			(RTL8712_FIFOCTRL_ + 0x50)
#define REG_RXFF0_WTPTR			(RTL8712_FIFOCTRL_ + 0x54)
#define REG_RXFF1_RDPTR			(RTL8712_FIFOCTRL_ + 0x58)
#define REG_RXFF1_WTPTR			(RTL8712_FIFOCTRL_ + 0x5C)
#define REG_RXRPT0FF_RDPTR		(RTL8712_FIFOCTRL_ + 0x60)
#define REG_RXRPT0FF_WTPTR		(RTL8712_FIFOCTRL_ + 0x64)
#define REG_RXRPT1FF_RDPTR		(RTL8712_FIFOCTRL_ + 0x68)
#define REG_RXRPT1FF_WTPTR		(RTL8712_FIFOCTRL_ + 0x6C)
#define REG_RX0PKTNUM			(RTL8712_FIFOCTRL_ + 0x72)
#define REG_RX1PKTNUM			(RTL8712_FIFOCTRL_ + 0x74)
#define REG_RXFLTMAP0			(RTL8712_FIFOCTRL_ + 0x76)
#define REG_RXFLTMAP1			(RTL8712_FIFOCTRL_ + 0x78)
#define REG_RXFLTMAP2			(RTL8712_FIFOCTRL_ + 0x7A)
#define REG_RXFLTMAP3			(RTL8712_FIFOCTRL_ + 0x7c)
#define REG_TBDA			(RTL8712_FIFOCTRL_ + 0x84)
#define REG_THPDA			(RTL8712_FIFOCTRL_ + 0x88)
#define REG_TCDA			(RTL8712_FIFOCTRL_ + 0x8C)
#define REG_TMDA			(RTL8712_FIFOCTRL_ + 0x90)
#define REG_HDA				(RTL8712_FIFOCTRL_ + 0x94)
#define REG_TVODA			(RTL8712_FIFOCTRL_ + 0x98)
#define REG_TVIDA			(RTL8712_FIFOCTRL_ + 0x9C)
#define REG_TBEDA			(RTL8712_FIFOCTRL_ + 0xA0)
#define REG_TBKDA			(RTL8712_FIFOCTRL_ + 0xA4)
#define REG_RCDA			(RTL8712_FIFOCTRL_ + 0xA8)
#define REG_RDSA			(RTL8712_FIFOCTRL_ + 0xAC)
#define REG_TXPKT_NUM_CTRL		(RTL8712_FIFOCTRL_ + 0xB0)
#define REG_TXQ_PGADD			(RTL8712_FIFOCTRL_ + 0xB3)
#define REG_TXFF_PG_NUM			(RTL8712_FIFOCTRL_ + 0xB4)

/* ----------------------------------------------------- */
/* 0x10250160h ~ 0x102501CFh Adaptive Control */
/* ----------------------------------------------------- */
#define REG_INIMCS_SEL			(RTL8712_RATECTRL_ + 0x00)
#define REG_INIRTSMCS_SEL		(RTL8712_RATECTRL_ + 0x20)
#define		RRSR_RSC_OFFSET			21
#define		RRSR_SHORT_OFFSET		23
#define		RRSR_1M				BIT(0)
#define		RRSR_2M				BIT(1)
#define		RRSR_5_5M			BIT(2)
#define		RRSR_11M			BIT(3)
#define		RRSR_6M				BIT(4)
#define		RRSR_9M				BIT(5)
#define		RRSR_12M			BIT(6)
#define		RRSR_18M			BIT(7)
#define		RRSR_24M			BIT(8)
#define		RRSR_36M			BIT(9)
#define		RRSR_48M			BIT(10)
#define		RRSR_54M			BIT(11)
#define		RRSR_MCS0			BIT(12)
#define		RRSR_MCS1			BIT(13)
#define		RRSR_MCS2			BIT(14)
#define		RRSR_MCS3			BIT(15)
#define		RRSR_MCS4			BIT(16)
#define		RRSR_MCS5			BIT(17)
#define		RRSR_MCS6			BIT(18)
#define		RRSR_MCS7			BIT(19)
#define		BRSR_ACKSHORTPMB		BIT(23)
#define		RATR_1M				0x00000001
#define		RATR_2M				0x00000002
#define		RATR_55M			0x00000004
#define		RATR_11M			0x00000008
#define		RATR_6M				0x00000010
#define		RATR_9M				0x00000020
#define		RATR_12M			0x00000040
#define		RATR_18M			0x00000080
#define		RATR_24M			0x00000100
#define		RATR_36M			0x00000200
#define		RATR_48M			0x00000400
#define		RATR_54M			0x00000800
#define		RATR_MCS0			0x00001000
#define		RATR_MCS1			0x00002000
#define		RATR_MCS2			0x00004000
#define		RATR_MCS3			0x00008000
#define		RATR_MCS4			0x00010000
#define		RATR_MCS5			0x00020000
#define		RATR_MCS6			0x00040000
#define		RATR_MCS7			0x00080000
#define		RATR_MCS8			0x00100000
#define		RATR_MCS9			0x00200000
#define		RATR_MCS10			0x00400000
#define		RATR_MCS11			0x00800000
#define		RATR_MCS12			0x01000000
#define		RATR_MCS13			0x02000000
#define		RATR_MCS14			0x04000000
#define		RATR_MCS15			0x08000000
#define		RATE_ALL_CCK			(RATR_1M | RATR_2M | \
						RATR_55M | RATR_11M)
#define		RATE_ALL_OFDM_AG		(RATR_6M | RATR_9M | \
						RATR_12M | RATR_18M | \
						RATR_24M | RATR_36M | \
						RATR_48M | RATR_54M)
#define		RATE_ALL_OFDM_1SS		(RATR_MCS0 | RATR_MCS1 | \
						RATR_MCS2 | RATR_MCS3 | \
						RATR_MCS4 | RATR_MCS5 | \
						RATR_MCS6 | RATR_MCS7)
#define		RATE_ALL_OFDM_2SS		(RATR_MCS8 | RATR_MCS9 | \
						RATR_MCS10 | RATR_MCS11 | \
						RATR_MCS12 | RATR_MCS13 | \
						RATR_MCS14 | RATR_MCS15)
#define REG_RRSR			(RTL8712_RATECTRL_ + 0x21)
/* that's a bit weird, why not use BIT(28)? */
#define		RRSR_RSC_LOWSUBCHNL		0x00200000
#define		RRSR_RSC_UPSUBCHNL		0x00400000
#define		RRSR_RSC_BW_40M			0x00600000
#define		RRSR_SHORT			0x00800000

#define REG_ARFR0			(RTL8712_RATECTRL_ + 0x24)
#define REG_ARFR1			(RTL8712_RATECTRL_ + 0x28)
#define REG_ARFR2			(RTL8712_RATECTRL_ + 0x2C)
#define REG_ARFR3			(RTL8712_RATECTRL_ + 0x30)
#define REG_ARFR4			(RTL8712_RATECTRL_ + 0x34)
#define REG_ARFR5			(RTL8712_RATECTRL_ + 0x38)
#define REG_ARFR6			(RTL8712_RATECTRL_ + 0x3C)
#define REG_ARFR7			(RTL8712_RATECTRL_ + 0x40)
#define REG_AGGLEN_LMT_H		(RTL8712_RATECTRL_ + 0x47)
#define REG_AGGLEN_LMT_L		(RTL8712_RATECTRL_ + 0x48)
#define REG_DARFRC			(RTL8712_RATECTRL_ + 0x50)
#define REG_RARFRC			(RTL8712_RATECTRL_ + 0x58)
#define REG_MCS_TXAGC0			(RTL8712_RATECTRL_ + 0x60)
#define REG_MCS_TXAGC1			(RTL8712_RATECTRL_ + 0x61)
#define REG_MCS_TXAGC2			(RTL8712_RATECTRL_ + 0x62)
#define REG_MCS_TXAGC3			(RTL8712_RATECTRL_ + 0x63)
#define REG_MCS_TXAGC4			(RTL8712_RATECTRL_ + 0x64)
#define REG_MCS_TXAGC5			(RTL8712_RATECTRL_ + 0x65)
#define REG_MCS_TXAGC6			(RTL8712_RATECTRL_ + 0x66)
#define REG_MCS_TXAGC7			(RTL8712_RATECTRL_ + 0x67)
#define REG_CCK_TXAGC			(RTL8712_RATECTRL_ + 0x68)

/* ----------------------------------------------------- */
/* 0x102501D0h ~ 0x102501FFh EDCA Configuration */
/* ----------------------------------------------------- */
#define REG_EDCA_VO_PARAM		(RTL8712_EDCASETTING_ + 0x00)
#define REG_EDCA_VI_PARAM		(RTL8712_EDCASETTING_ + 0x04)
#define REG_EDCA_BE_PARAM		(RTL8712_EDCASETTING_ + 0x08)
#define REG_EDCA_BK_PARAM		(RTL8712_EDCASETTING_ + 0x0C)
#define REG_BCNTCFG			(RTL8712_EDCASETTING_ + 0x10)
#define REG_CWRR			(RTL8712_EDCASETTING_ + 0x12)
#define REG_ACMAVG			(RTL8712_EDCASETTING_ + 0x16)
#define REG_ACMHWCTRL			(RTL8712_EDCASETTING_ + 0x17)
#define		ACMHW_HWEN			BIT(0)
#define		ACMHW_BEQEN			BIT(1)
#define		ACMHW_VIQEN			BIT(2)
#define		ACMHW_VOQEN			BIT(3)
#define		ACMHW_BEQSTATUS			BIT(4)
#define		ACMHW_VIQSTATUS			BIT(5)
#define		ACMHW_VOQSTATUS			BIT(6)

#define REG_VO_ADMTIME			(RTL8712_EDCASETTING_ + 0x18)
#define REG_VI_ADMTIME			(RTL8712_EDCASETTING_ + 0x1C)
#define REG_BE_ADMTIME			(RTL8712_EDCASETTING_ + 0x20)
#define REG_RETRY_LIMIT			(RTL8712_EDCASETTING_ + 0x24)
#define		RETRY_LIMIT_SHORT_SHIFT		8
#define		RETRY_LIMIT_LONG_SHIFT		0

#define	REG_SG_RATE			(RTL8712_EDCASETTING_ + 0x26)

/* ----------------------------------------------------- */
/* 0x10250200h ~ 0x1025023Fh Wireless MAC */
/* ----------------------------------------------------- */
#define REG_NAVCTRL			(RTL8712_WMAC_ + 0x00)
#define		NAV_UPPER_EN			BIT(16)
#define		NAV_UPPER			0xFF00
#define		NAV_RTSRST			0xFF

#define REG_BWOPMODE			(RTL8712_WMAC_ + 0x03)
#define		BW_OPMODE_11J			BIT(0)
#define		BW_OPMODE_5G			BIT(1)
#define		BW_OPMODE_20MHZ			BIT(2)

#define REG_BACAMCMD			(RTL8712_WMAC_ + 0x04)
#define REG_BACAMCONTENT		(RTL8712_WMAC_ + 0x08)
#define REG_LBDLY			(RTL8712_WMAC_ + 0x10)
#define REG_FWDLY			(RTL8712_WMAC_ + 0x11)
#define REG_HWPC_RX_CTRL		(RTL8712_WMAC_ + 0x18)
#define REG_MQ				(RTL8712_WMAC_ + 0x20)
#define REG_MA				(RTL8712_WMAC_ + 0x22)
#define REG_MS				(RTL8712_WMAC_ + 0x24)
#define REG_CLM_RESULT			(RTL8712_WMAC_ + 0x27)
#define REG_NHM_RPI_CNT			(RTL8712_WMAC_ + 0x28)
#define REG_RXERR_RPT			(RTL8712_WMAC_ + 0x30)
#define REG_NAV_PROT_LEN		(RTL8712_WMAC_ + 0x34)
#define REG_CFEND_TH			(RTL8712_WMAC_ + 0x36)
#define REG_AMPDU_MIN_SPACE		(RTL8712_WMAC_ + 0x37)
#define		MAX_MSS_OFFSET			3
#define		MAX_MSS_DENSITY_2T		0x13
#define		MAX_MSS_DENSITY_1T		0x0A
#define	REG_TXOP_STALL_CTRL		(RTL8712_WMAC_ + 0x38)

/* ----------------------------------------------------- */
/* 0x10250240h ~ 0x1025025Fh CAM / Security Engine */
/* ----------------------------------------------------- */
#define	REG_RWCAM			(RTL8712_SECURITY_ + 0x00)
#define	REG_WCAMI			(RTL8712_SECURITY_ + 0x04)
#define	REG_RCAMO			(RTL8712_SECURITY_ + 0x08)
#define	REG_CAMDBG			(RTL8712_SECURITY_ + 0x0C)
#define	REG_SECR			(RTL8712_SECURITY_ + 0x10)
#define		SCR_TXUSEDK			BIT(0)
#define		SCR_RXUSEDK			BIT(1)
#define		SCR_TXENCENABLE			BIT(2)
#define		SCR_RXENCENABLE			BIT(3)
#define		SCR_SKBYA2			BIT(4)
#define		SCR_NOSKMC			BIT(5)

/* ----------------------------------------------------- */
/* 0x10250260h ~ 0x102502BFh Powersave Control */
/* ----------------------------------------------------- */
#define REG_WOWCTRL			(RTL8712_POWERSAVE_ + 0x00)
#define REG_PSSTATUS			(RTL8712_POWERSAVE_ + 0x01)
#define REG_PSSWITCH			(RTL8712_POWERSAVE_ + 0x02)
#define REG_MIMOPS_WAITPERIOD		(RTL8712_POWERSAVE_ + 0x03)
#define REG_LPNAV_CTRL			(RTL8712_POWERSAVE_ + 0x04)
#define REG_WFM0			(RTL8712_POWERSAVE_ + 0x10)
#define REG_WFM1			(RTL8712_POWERSAVE_ + 0x20)
#define REG_WFM2			(RTL8712_POWERSAVE_ + 0x30)
#define REG_WFM3			(RTL8712_POWERSAVE_ + 0x40)
#define REG_WFM4			(RTL8712_POWERSAVE_ + 0x50)
#define REG_WFM5			(RTL8712_POWERSAVE_ + 0x60)
#define REG_WFCRC			(RTL8712_POWERSAVE_ + 0x70)
#define REG_RPWM			(RTL8712_POWERSAVE_ + 0x7C)
#define REG_CPWM			(RTL8712_POWERSAVE_ + 0x7D)

/* ----------------------------------------------------- */
/* 0x102502C0h ~ 0x102502CFh Base Band Control */
/* ----------------------------------------------------- */
#define REG_RF_BB_CMD_ADDR		(RTL8712_BB_ + 0x00)
#define REG_RF_BB_CMD_DATA		(RTL8712_BB_ + 0x04)

/* ----------------------------------------------------- */
/* 0x102502D0h ~ 0x102502DFh Offload Control */
/* ----------------------------------------------------- */

/* ----------------------------------------------------- */
/* 0x102502E0h ~ 0x102502FFh GPIO */
/* ----------------------------------------------------- */
#define REG_PSTIMER			(RTL8712_GP_ + 0x00)
#define REG_TIMER1			(RTL8712_GP_ + 0x04)
#define REG_TIMER2			(RTL8712_GP_ + 0x08)
#define REG_GPIO_CTRL			(RTL8712_GP_ + 0x0C)
#define		HAL_8192S_HW_GPIO_OFF_BIT	BIT(3)
#define		HAL_8192S_HW_GPIO_WPS_BIT	BIT(4)

#define REG_GPIO_IO_SEL			(RTL8712_GP_ + 0x0E)
#define		GPIOSEL_PHYDBG			1
#define		GPIOSEL_BT			2
#define		GPIOSEL_WLANDBG			3
#define		GPIOSEL_GPIO_MASK		(~(BIT(0)|BIT(1)))

#define REG_GPIO_INTCTRL		(RTL8712_GP_ + 0x10)
#define REG_MAC_PINMUX_CTRL		(RTL8712_GP_ + 0x11)
#define		GPIOSEL_GPIO			0
#define		GPIOMUX_EN			BIT(3)

#define REG_LEDCFG			(RTL8712_GP_ + 0x12)
#define REG_PHY_REG_RPT			(RTL8712_GP_ + 0x13)
#define REG_PHY_REG_DATA		(RTL8712_GP_ + 0x14)
#define REG_EFUSE_CLK			(RTL8712_GP_ + 0x18)

/* ----------------------------------------------------- */
/* 0x10250300h ~ 0x1025030Fh Interrupt Controller */
/* ----------------------------------------------------- */
#define REG_HIMR			(RTL8712_INTERRUPT_ + 0x08)

/* ----------------------------------------------------- */
/* 0x10250310h ~ 0x1025035Fh Debug */
/* ----------------------------------------------------- */
#define REG_BIST			(RTL8712_DEBUGCTRL_ + 0x00)
#define REG_DBS				(RTL8712_DEBUGCTRL_ + 0x04)
#define REG_LMS				(RTL8712_DEBUGCTRL_ + 0x05)
#define REG_CPUINST			(RTL8712_DEBUGCTRL_ + 0x08)
#define REG_CPUCAUSE			(RTL8712_DEBUGCTRL_ + 0x0C)
#define REG_LBUS_ERR_ADDR		(RTL8712_DEBUGCTRL_ + 0x10)
#define REG_LBUS_ERR_CMD		(RTL8712_DEBUGCTRL_ + 0x14)
#define REG_LBUS_ERR_DATA_L		(RTL8712_DEBUGCTRL_ + 0x18)
#define REG_LBUS_ERR_DATA_H		(RTL8712_DEBUGCTRL_ + 0x1C)
#define REG_LBUS_EXCEPTION_ADDR		(RTL8712_DEBUGCTRL_ + 0x20)
#define REG_WDG_CTRL			(RTL8712_DEBUGCTRL_ + 0x24)
#define REG_INTMTU			(RTL8712_DEBUGCTRL_ + 0x28)
#define REG_INTM			(RTL8712_DEBUGCTRL_ + 0x2A)
#define REG_FDLOCKTURN0			(RTL8712_DEBUGCTRL_ + 0x2C)
#define REG_FDLOCKTURN1			(RTL8712_DEBUGCTRL_ + 0x2D)
#define REG_FDLOCKFLAG0			(RTL8712_DEBUGCTRL_ + 0x2E)
#define REG_FDLOCKFLAG1			(RTL8712_DEBUGCTRL_ + 0x2F)
#define REG_TRXPKTBUF_DBG_DATA		(RTL8712_DEBUGCTRL_ + 0x30)
#define REG_TRXPKTBUF_DBG_CTRL		(RTL8712_DEBUGCTRL_ + 0x38)
#define REG_DPLL_MON			(RTL8712_DEBUGCTRL_ + 0x3A)

/* ----------------------------------------------------- */
/* 0x10250310h ~ 0x1025035Fh IO Command Control */
/* ----------------------------------------------------- */
#define REG_IOCMD_CTRL			(RTL8712_IOCMD_ + 0x00)
#define REG_IOCMD_DATA			(RTL8712_IOCMD_ + 0x04)

/* ----------------------------------------------------- */
/* 0x10250800 ~ 0x10250DFF PHY */
/* ----------------------------------------------------- */
#define REG_RFPGA0_RFMOD		(RTL8712_PHY_ + 0x00)
#define REG_RFPGA0_TXINFO		(RTL8712_PHY_ + 0x04)
#define REG_RFPGA0_PSDFUNCTION		(RTL8712_PHY_ + 0x08)
#define REG_RFPGA0_TXGAINSTAGE		(RTL8712_PHY_ + 0x0C)
#define REG_RFPGA0_RFTIMING1		(RTL8712_PHY_ + 0x10)
#define REG_RFPGA0_RFTIMING2		(RTL8712_PHY_ + 0x14)
#define REG_RFPGA0_XA_HSSIPARAMETER1	(RTL8712_PHY_ + 0x20)
#define REG_RFPGA0_XA_HSSIPARAMETER2	(RTL8712_PHY_ + 0x24)
#define REG_RFPGA0_XB_HSSIPARAMETER1	(RTL8712_PHY_ + 0x28)
#define REG_RFPGA0_XB_HSSIPARAMETER2	(RTL8712_PHY_ + 0x2C)
#define REG_RFPGA0_XC_HSSIPARAMETER1	(RTL8712_PHY_ + 0x30)
#define REG_RFPGA0_XC_HSSIPARAMETER2	(RTL8712_PHY_ + 0x34)
#define REG_RFPGA0_XD_HSSIPARAMETER1	(RTL8712_PHY_ + 0x38)
#define REG_RFPGA0_XD_HSSIPARAMETER2	(RTL8712_PHY_ + 0x3C)
#define REG_RFPGA0_XA_LSSIPARAMETER	(RTL8712_PHY_ + 0x40)
#define REG_RFPGA0_XB_LSSIPARAMETER	(RTL8712_PHY_ + 0x44)
#define REG_RFPGA0_XC_LSSIPARAMETER	(RTL8712_PHY_ + 0x48)
#define REG_RFPGA0_XD_LSSIPARAMETER	(RTL8712_PHY_ + 0x4C)
#define REG_RFPGA0_RFWAKEUP_PARAMETER	(RTL8712_PHY_ + 0x50)
#define REG_RFPGA0_RFSLEEPUP_PARAMETER	(RTL8712_PHY_ + 0x54)
#define REG_RFPGA0_XAB_SWITCHCONTROL	(RTL8712_PHY_ + 0x58)
#define REG_RFPGA0_XCD_SWITCHCONTROL	(RTL8712_PHY_ + 0x5C)
#define REG_RFPGA0_XA_RFINTERFACEOE	(RTL8712_PHY_ + 0x60)
#define REG_RFPGA0_XB_RFINTERFACEOE	(RTL8712_PHY_ + 0x64)
#define REG_RFPGA0_XC_RFINTERFACEOE	(RTL8712_PHY_ + 0x68)
#define REG_RFPGA0_XD_RFINTERFACEOE	(RTL8712_PHY_ + 0x6C)
#define REG_RFPGA0_XAB_RFINTERFACESW	(RTL8712_PHY_ + 0x70)
#define REG_RFPGA0_XCD_RFINTERFACESW	(RTL8712_PHY_ + 0x74)
#define REG_RFPGA0_XAB_RFPARAMETER	(RTL8712_PHY_ + 0x78)
#define REG_RFPGA0_XCD_RFPARAMETER	(RTL8712_PHY_ + 0x7C)
#define REG_RFPGA0_ANALOGPARAMETER1	(RTL8712_PHY_ + 0x80)
#define REG_RFPGA0_ANALOGPARAMETER2	(RTL8712_PHY_ + 0x84)
#define REG_RFPGA0_ANALOGPARAMETER3	(RTL8712_PHY_ + 0x88)
#define REG_RFPGA0_ANALOGPARAMETER4	(RTL8712_PHY_ + 0x8C)
#define REG_RFPGA0_XA_LSSIREADBACK	(RTL8712_PHY_ + 0xa0)
#define REG_RFPGA0_XB_LSSIREADBACK	(RTL8712_PHY_ + 0xA4)
#define REG_RFPGA0_XC_LSSIREADBACK	(RTL8712_PHY_ + 0xA8)
#define REG_RFPGA0_XD_LSSIREADBACK	(RTL8712_PHY_ + 0xAC)
#define REG_RFPGA0_PSDREPORT		(RTL8712_PHY_ + 0xB4)
#define REG_TRANSCEIVERA_HSPI_READBACK	(RTL8712_PHY_ + 0xB8)
#define REG_TRANSCEIVERB_HSPI_READBACK	(RTL8712_PHY_ + 0xBC)
#define REG_RFPGA0_XAB_RFINTERFACERB	(RTL8712_PHY_ + 0xE0)
#define REG_RFPGA0_XCD_RFINTERFACERB	(RTL8712_PHY_ + 0xE4)

/* ----------------------------------------------------- */
/* 0x10250900 ~ 0x102509FF PHY Page 9*/
/* ----------------------------------------------------- */
#define REG_RFPGA1_RFMOD		(RTL8712_PHY_P1_ + 0x00)
#define REG_RFPGA1_TXBLOCK		(RTL8712_PHY_P1_ + 0x04)
#define REG_RFPGA1_DEBUGSELECT		(RTL8712_PHY_P1_ + 0x08)
#define REG_RFPGA1_TXINFO		(RTL8712_PHY_P1_ + 0x0C)

/* ----------------------------------------------------- */
/* 0x10250A00 ~ 0x10250AFF PHY CCK */
/* ----------------------------------------------------- */
#define REG_RCCK0_SYSTEM		(RTL8712_PHY_CCK_ + 0x00)
#define REG_RCCK0_AFESETTING		(RTL8712_PHY_CCK_ + 0x04)
#define REG_RCCK0_CCA			(RTL8712_PHY_CCK_ + 0x08)
#define REG_RCCK0_RXAGC1		(RTL8712_PHY_CCK_ + 0x0C)
#define REG_RCCK0_RXAGC2		(RTL8712_PHY_CCK_ + 0x10)
#define REG_RCCK0_RXHP			(RTL8712_PHY_CCK_ + 0x14)
#define REG_RCCK0_DSPPARAMETER1		(RTL8712_PHY_CCK_ + 0x18)
#define REG_RCCK0_DSPPARAMETER2		(RTL8712_PHY_CCK_ + 0x1C)
#define REG_RCCK0_TXFILTER1		(RTL8712_PHY_CCK_ + 0x20)
#define REG_RCCK0_TXFILTER2		(RTL8712_PHY_CCK_ + 0x24)
#define REG_RCCK0_DEBUGPORT		(RTL8712_PHY_CCK_ + 0x28)
#define REG_RCCK0_FALSEALARMREPORT	(RTL8712_PHY_CCK_ + 0x2C)
#define REG_RCCK0_TRSSIREPORT		(RTL8712_PHY_CCK_ + 0x50)
#define REG_RCCK0_RXREPORT		(RTL8712_PHY_CCK_ + 0x54)
#define REG_RCCK0_FACOUNTERLOWER	(RTL8712_PHY_CCK_ + 0x5C)
#define REG_RCCK0_FACOUNTERUPPER	(RTL8712_PHY_CCK_ + 0x58)

/* ----------------------------------------------------- */
/* 0x10250C00 ~ 0x10250DFF PHY OFDM */
/* ----------------------------------------------------- */
#define REG_ROFDM0_LSTF			(RTL8712_PHY_OFDM_ + 0x00)
#define REG_ROFDM0_TRXPATHENABLE	(RTL8712_PHY_OFDM_ + 0x04)
#define REG_ROFDM0_TRMUXPAR		(RTL8712_PHY_OFDM_ + 0x08)
#define REG_ROFDM0_TRSWISOLATION	(RTL8712_PHY_OFDM_ + 0x0C)
#define REG_ROFDM0_XARXAFE		(RTL8712_PHY_OFDM_ + 0x10)
#define REG_ROFDM0_XARXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x14)
#define REG_ROFDM0_XBRXAFE		(RTL8712_PHY_OFDM_ + 0x18)
#define REG_ROFDM0_XBRXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x1C)
#define REG_ROFDM0_XCRXAFE		(RTL8712_PHY_OFDM_ + 0x20)
#define REG_ROFDM0_XCRXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x24)
#define REG_ROFDM0_XDRXAFE		(RTL8712_PHY_OFDM_ + 0x28)
#define REG_ROFDM0_XDRXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x2C)
#define REG_ROFDM0_RXDETECTOR1		(RTL8712_PHY_OFDM_ + 0x30)
#define REG_ROFDM0_RXDETECTOR2		(RTL8712_PHY_OFDM_ + 0x34)
#define REG_ROFDM0_RXDETECTOR3		(RTL8712_PHY_OFDM_ + 0x38)
#define REG_ROFDM0_RXDETECTOR4		(RTL8712_PHY_OFDM_ + 0x3C)
#define REG_ROFDM0_RXDSP		(RTL8712_PHY_OFDM_ + 0x40)
#define REG_ROFDM0_CFO_AND_DAGC		(RTL8712_PHY_OFDM_ + 0x44)
#define REG_ROFDM0_CCADROP_THRESHOLD	(RTL8712_PHY_OFDM_ + 0x48)
#define REG_ROFDM0_ECCA_THRESHOLD	(RTL8712_PHY_OFDM_ + 0x4C)
#define REG_ROFDM0_XAAGCCORE1		(RTL8712_PHY_OFDM_ + 0x50)
#define REG_ROFDM0_XAAGCCORE2		(RTL8712_PHY_OFDM_ + 0x54)
#define REG_ROFDM0_XBAGCCORE1		(RTL8712_PHY_OFDM_ + 0x58)
#define REG_ROFDM0_XBAGCCORE2		(RTL8712_PHY_OFDM_ + 0x5C)
#define REG_ROFDM0_XCAGCCORE1		(RTL8712_PHY_OFDM_ + 0x60)
#define REG_ROFDM0_XCAGCCORE2		(RTL8712_PHY_OFDM_ + 0x64)
#define REG_ROFDM0_XDAGCCORE1		(RTL8712_PHY_OFDM_ + 0x68)
#define REG_ROFDM0_XDAGCCORE2		(RTL8712_PHY_OFDM_ + 0x6C)
#define REG_ROFDM0_AGCPARAMETER1	(RTL8712_PHY_OFDM_ + 0x70)
#define REG_ROFDM0_AGCPARAMETER2	(RTL8712_PHY_OFDM_ + 0x74)
#define REG_ROFDM0_AGCRSSITABLE		(RTL8712_PHY_OFDM_ + 0x78)
#define REG_ROFDM0_HTSTFAGC		(RTL8712_PHY_OFDM_ + 0x7C)
#define REG_ROFDM0_XATXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x80)
#define REG_ROFDM0_XATXAFE		(RTL8712_PHY_OFDM_ + 0x84)
#define REG_ROFDM0_XBTXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x88)
#define REG_ROFDM0_XBTXAFE		(RTL8712_PHY_OFDM_ + 0x8C)
#define REG_ROFDM0_XCTXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x90)
#define REG_ROFDM0_XCTXAFE		(RTL8712_PHY_OFDM_ + 0x94)
#define REG_ROFDM0_XDTXIQIMBALANCE	(RTL8712_PHY_OFDM_ + 0x98)
#define REG_ROFDM0_XDTXAFE		(RTL8712_PHY_OFDM_ + 0x9C)
#define REG_ROFDM0_TXCOEFF1		(RTL8712_PHY_OFDM_ + 0xA4)
#define REG_ROFDM0_TXCOEFF2		(RTL8712_PHY_OFDM_ + 0xA8)
#define REG_ROFDM0_TXCOEFF3		(RTL8712_PHY_OFDM_ + 0xAC)
#define REG_ROFDM0_TXCOEFF4		(RTL8712_PHY_OFDM_ + 0xB0)
#define REG_ROFDM0_TXCOEFF5		(RTL8712_PHY_OFDM_ + 0xB4)
#define REG_ROFDM0_TXCOEFF6		(RTL8712_PHY_OFDM_ + 0xB8)
#define REG_ROFDM0_RXHP_PARAMETER	(RTL8712_PHY_OFDM_ + 0xe0)
#define REG_ROFDM0_TXPSEUDO_NOISE_WGT	(RTL8712_PHY_OFDM_ + 0xe4)
#define REG_ROFDM0_FRAME_SYNC		(RTL8712_PHY_OFDM_ + 0xf0)
#define REG_ROFDM0_DFSREPORT		(RTL8712_PHY_OFDM_ + 0xf4)

#define REG_ROFDM1_LSTF			(RTL8712_PHY_OFDM_ + 0x100)
#define REG_ROFDM1_TRXPATHENABLE	(RTL8712_PHY_OFDM_ + 0x104)
#define REG_ROFDM1_CFO			(RTL8712_PHY_OFDM_ + 0x108)

#define REG_ROFDM1_CSI1			(RTL8712_PHY_OFDM_ + 0x110)
#define REG_ROFDM1_SBD			(RTL8712_PHY_OFDM_ + 0x114)
#define REG_ROFDM1_CSI2			(RTL8712_PHY_OFDM_ + 0x118)

#define REG_ROFDM1_CFOTRACKING		(RTL8712_PHY_OFDM_ + 0x12C)

#define REG_ROFDM1_TRXMESAURE1		(RTL8712_PHY_OFDM_ + 0x134)

#define REG_ROFDM1_INTF_DET		(RTL8712_PHY_OFDM_ + 0x13C)

#define REG_ROFDM1_PSEUDO_NOISESTATEAB	(RTL8712_PHY_OFDM_ + 0x150)
#define REG_ROFDM1_PSEUDO_NOISESTATECD	(RTL8712_PHY_OFDM_ + 0x154)
#define REG_ROFDM1_RX_PSEUDO_NOISE_WGT	(RTL8712_PHY_OFDM_ + 0x158)

#define REG_ROFDM_PHYCOUNTER1		(RTL8712_PHY_OFDM_ + 0x1A0)
#define REG_ROFDM_PHYCOUNTER2		(RTL8712_PHY_OFDM_ + 0x1A4)
#define REG_ROFDM_PHYCOUNTER3		(RTL8712_PHY_OFDM_ + 0x1A8)
#define REG_ROFDM_SHORT_CFOAB		(RTL8712_PHY_OFDM_ + 0x1AC)
#define REG_ROFDM_SHORT_CFOCD		(RTL8712_PHY_OFDM_ + 0x1B0)
#define REG_ROFDM_LONG_CFOAB		(RTL8712_PHY_OFDM_ + 0x1B4)
#define REG_ROFDM_LONG_CFOCD		(RTL8712_PHY_OFDM_ + 0x1B8)
#define REG_ROFDM_TAIL_CFOAB		(RTL8712_PHY_OFDM_ + 0x1BC)
#define REG_ROFDM_TAIL_CFOCD		(RTL8712_PHY_OFDM_ + 0x1C0)
#define REG_ROFDM_PW_MEASURE1		(RTL8712_PHY_OFDM_ + 0x1C4)
#define REG_ROFDM_PW_MEASURE2		(RTL8712_PHY_OFDM_ + 0x1C8)
#define REG_ROFDM_BW_REPORT		(RTL8712_PHY_OFDM_ + 0x1CC)
#define REG_ROFDM_AGC_REPORT		(RTL8712_PHY_OFDM_ + 0x1D0)
#define REG_ROFDM_RXSNR			(RTL8712_PHY_OFDM_ + 0x1D4)
#define REG_ROFDM_RXEVMCSI		(RTL8712_PHY_OFDM_ + 0x1D8)
#define REG_ROFDM_SIG_REPORT		(RTL8712_PHY_OFDM_ + 0x1DC)

/* ----------------------------------------------------- */
/* 0x10250E00 ~ 0x10250EFF PHY Automatic Gain Control */
/* ----------------------------------------------------- */
#define REG_RTXAGC_RATE18_06		(RTL8712_PHY_RXAGC_ + 0x00)
#define REG_RTXAGC_RATE54_24		(RTL8712_PHY_RXAGC_ + 0x04)
#define REG_RTXAGC_CCK_MCS32		(RTL8712_PHY_RXAGC_ + 0x08)
#define REG_RTXAGC_MCS03_MCS00		(RTL8712_PHY_RXAGC_ + 0x10)
#define REG_RTXAGC_MCS07_MCS04		(RTL8712_PHY_RXAGC_ + 0x14)
#define REG_RTXAGC_MCS11_MCS08		(RTL8712_PHY_RXAGC_ + 0x18)
#define REG_RTXAGC_MCS15_MCS12		(RTL8712_PHY_RXAGC_ + 0x1c)

/* ----------------------------------------------------- */
/* 0x1025FE00 ~ 0x1025FEFF USB Configuration */
/* ----------------------------------------------------- */
#define REG_USB_INFO			(RTL8712_USB_ + 0x17)
#define REG_USB_MAGIC			(RTL8712_USB_ + 0x1C)
#define		USB_MAGIC_BIT7			BIT(7)

#define REG_USB_SPECIAL_OPTION		(RTL8712_USB_ + 0x55)
#define REG_USB_HCPWM			(RTL8712_USB_ + 0x57)
#define REG_USB_HRPWM			(RTL8712_USB_ + 0x58)
#define REG_USB_DMA_AGG_TO		(RTL8712_USB_ + 0x5B)
#define REG_USB_AGG_TO			(RTL8712_USB_ + 0x5C)
#define REG_USB_AGG_TH			(RTL8712_USB_ + 0x5D)

#define REG_USB_VID			(RTL8712_USB_ + 0x60)
#define REG_USB_PID			(RTL8712_USB_ + 0x62)
#define REG_USB_OPTIONAL		(RTL8712_USB_ +	0x64)
#define REG_USB_CHIRP_K			(RTL8712_USB_ + 0x65) /* REG_USB_EP */
#define REG_USB_PHY			(RTL8712_USB_ + 0x66) /* 0xFE68? */
#define REG_USB_MAC_ADDR		(RTL8712_USB_ + 0x70)
#define REG_USB_STRING			(RTL8712_USB_ + 0x80)

/* ----------------------------------------------------- */
/* 0x10300000 ~ 0x103FFFFF FIFOs for 8712 */
/* ----------------------------------------------------- */
#define REG_RTL8712_DMA_BCNQ		(RTL8712_IOBASE_FF + 0x10000)
#define REG_RTL8712_DMA_MGTQ		(RTL8712_IOBASE_FF + 0x20000)
#define REG_RTL8712_DMA_BMCQ		(RTL8712_IOBASE_FF + 0x30000)
#define REG_RTL8712_DMA_VOQ		(RTL8712_IOBASE_FF + 0x40000)
#define REG_RTL8712_DMA_VIQ		(RTL8712_IOBASE_FF + 0x50000)
#define REG_RTL8712_DMA_BEQ		(RTL8712_IOBASE_FF + 0x60000)
#define REG_RTL8712_DMA_BKQ		(RTL8712_IOBASE_FF + 0x70000)
#define REG_RTL8712_DMA_RX0FF		(RTL8712_IOBASE_FF + 0x80000)
#define REG_RTL8712_DMA_H2CCMD		(RTL8712_IOBASE_FF + 0x90000)
#define REG_RTL8712_DMA_C2HCMD		(RTL8712_IOBASE_FF + 0xA0000)

#define SET_VAL(reg, value, newvalue)					\
	(value = ((value) & ~(reg)) | (((newvalue) << (reg##_S)) & (reg)))

#define SET_CONSTVAL(reg, newvalue)					\
	(((newvalue) << (reg##_S)) & (reg))

#define MOD_VAL(reg, value, newvalue)					\
	(((value) & ~(reg)) | (((newvalue) << (reg##_S)) & (reg)))

#define GET_VAL(reg, value)						\
	(((value) & (reg)) >> (reg##_S))

#endif /* __R92SU_REG_H__*/
