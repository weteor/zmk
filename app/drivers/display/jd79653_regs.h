/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_JD79653_REGS_H_
#define ZEPHYR_DRIVERS_DISPLAY_JD79653_REGS_H_

#define JD79653_CMD_PSR				0x00
#define JD79653_CMD_PWR				0x01
#define JD79653_CMD_POF				0x02
#define JD79653_CMD_PFS				0x03
#define JD79653_CMD_PON				0x04
#define JD79653_CMD_PMES			0x05
#define JD79653_CMD_BTST			0x06
#define JD79653_CMD_DSLP			0x07
#define JD79653_CMD_DTM1			0x10
#define JD79653_CMD_DSP				0x11
#define JD79653_CMD_DRF				0x12
#define JD79653_CMD_DTM2			0x13
#define JD79653_CMD_AUTO			0x17
#define JD79653_CMD_BIST			0x18
#define JD79653_CMD_BIST_PS			0x19
#define JD79653_CMD_LUT1			0x20
#define JD79653_CMD_LUTWW			0x21
#define JD79653_CMD_LUTBWTR			0x22
#define JD79653_CMD_LUTWBTB			0x23
#define JD79653_CMD_LUTBBTB			0x24
#define JD79653_CMD_GRPFRAMERATE	0x25
#define JD79653_CMD_SET_GRP	        0x26
#define JD79653_CMD_LUTOPT			0x2A
#define JD79653_CMD_PLL				0x30
#define JD79653_CMD_PLL_MODE		0x31
#define JD79653_CMD_TSC				0x40
#define JD79653_CMD_TSE				0x41
#define JD79653_CMD_TSW				0x42
#define JD79653_CMD_TSR				0x43
#define JD79653_CMD_PBC				0x44
#define JD79653_CMD_FITIINT_4D      0x4D
#define JD79653_CMD_CDI				0x50
#define JD79653_CMD_LPD				0x51
#define JD79653_CMD_TCON			0x60
#define JD79653_CMD_TRES			0x61
#define JD79653_CMD_GSST			0x65
#define JD79653_CMD_VOTP			0x68
#define JD79653_CMD_REV				0x70
#define JD79653_CMD_FLG				0x71
#define JD79653_CMD_READRESBYTES	0x7F
#define JD79653_CMD_AMV				0x80
#define JD79653_CMD_VV				0x81
#define JD79653_CMD_VDCS			0x82
#define JD79653_CMD_PTL				0x90
#define JD79653_CMD_PTIN			0x91
#define JD79653_CMD_PTOUT			0x92
#define JD79653_CMD_CRCS			0x94
#define JD79653_CMD_CRCO			0x95
#define JD79653_CMD_CRCSTATUS		0x96
#define JD79653_CMD_WRITEOTPKEY		0x97
#define JD79653_CMD_PGM				0xA0
#define JD79653_CMD_APG				0xA1
#define JD79653_CMD_ROTP			0xA2
#define JD79653_CMD_FITIINT_AA      0xAA
#define JD79653_CMD_FITIINT_B6      0xB6
#define JD79653_CMD_CCSET			0xE0
#define JD79653_CMD_SET_OTP_BANK	0xE1
#define JD79653_CMD_PWS				0xE3
#define JD79653_CMD_LVSEL			0xE4
#define JD79653_CMD_TSSET			0xE5
#define JD79653_CMD_FITIINT_E9      0xE9
#define JD79653_CMD_FITIINT_F3      0xF3

#define JD79653_PSR_REG				BIT(5)
#define JD79653_PSR_KW_R			BIT(4)
#define JD79653_PSR_UD				BIT(3)
#define JD79653_PSR_SHL				BIT(2)
#define JD79653_PSR_SHD				BIT(1)
#define JD79653_PSR_RST				BIT(0)

#define JD79653_AUTO_PON_DRF_POF			0xA5
#define JD79653_AUTO_PON_DRF_POF_DSLP		0xA7

#define JD79653_CDI_REG_LENGTH		1U
#define JD79653_CDI_BDV1			BIT(7)
#define JD79653_CDI_BDV0			BIT(6)
#define JD79653_CDI_DDX1			BIT(5)
#define JD79653_CDI_DDX0			BIT(4)

#define JD79653_TRES_REG_LENGTH		3U
#define JD79653_TRES_HRES_IDX		0
#define JD79653_TRES_VRES_IDX		2

#define JD79653_PTL_REG_LENGTH		7U
#define JD79653_PTL_HRST_IDX		0
#define JD79653_PTL_HRED_IDX		1
#define JD79653_PTL_HRESERVED       2
#define JD79653_PTL_VRST_IDX		3
#define JD79653_PTL_VRESERVED       4
#define JD79653_PTL_VRED_IDX		5
#define JD79653_PTL_PT_SCAN			BIT(0)

/* Time constants in ms */
#define JD79653_RESET_DELAY			100U
#define JD79653_PON_DELAY			100U
#define JD79653_BUSY_DELAY			1U

#endif /* ZEPHYR_DRIVERS_DISPLAY_JD79653_REGS_H_ */