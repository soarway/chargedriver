/*
 * TI BQ25890 charger driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>

#include <linux/acpi.h>
#include <linux/of.h>

#define BQ25890_MANUFACTURER		"Texas Instruments"
#define BQ25890_IRQ_PIN			    "bq25890_irq"

#define BQ25890_ID			3
#define SYBA76_BC_ID		0x66

#define NAME          "BA76_70_7D"
#define DRIVER_NAME   NAME;
#define DEVICE_NAME   NAME
/*
enum _bq25890_fields_ {
	F_EN_HIZ, F_EN_ILIM, F_IILIM,				     		// Reg00 
	F_BHOT, F_BCOLD, F_VINDPM_OFS,				     		// Reg01 
	F_CONV_START, F_CONV_RATE, F_BOOSTF, F_ICO_EN, F_HVDCP_EN, F_MAXC_EN, F_FORCE_DPM, F_AUTO_DPDM_EN, // Reg02 
	F_BAT_LOAD_EN, F_WD_RST, F_OTG_CFG, F_CHG_CFG, F_SYSVMIN,    // Reg03 
	F_PUMPX_EN, F_ICHG,					     		// Reg04 
	F_IPRECHG, F_ITERM,					     		// Reg05 
	F_VREG, F_BATLOWV, F_VRECHG,				     // Reg06 
	F_TERM_EN, F_STAT_DIS, F_WD, F_TMR_EN, F_CHG_TMR,F_JEITA_ISET,// Reg07 
	F_BATCMP, F_VCLAMP, F_TREG,				     // Reg08 
	F_FORCE_ICO, F_TMR2X_EN, F_BATFET_DIS, F_JEITA_VSET,
	F_BATFET_DLY, F_BATFET_RST_EN, F_PUMPX_UP, F_PUMPX_DN,	     // Reg09 
	F_BOOSTV, F_BOOSTI,					     					// Reg0A 
	F_VBUS_STAT, F_CHG_STAT, F_PG_STAT, F_SDP_STAT, F_VSYS_STAT, // Reg0B 
	F_WD_FAULT, F_BOOST_FAULT, F_CHG_FAULT, F_BAT_FAULT,
	F_NTC_FAULT,						     					// Reg0C 
	F_FORCE_VINDPM, F_VINDPM,				     				// Reg0D 
	F_THERM_STAT, F_BATV,					     				// Reg0E 
	F_SYSV,							     						// Reg0F 
	F_TSPCT,						     						// Reg10 
	F_VBUS_GD, F_VBUSV,					     					// Reg11 
	F_ICHGR,						     						// Reg12 
	F_VDPM_STAT, F_IDPM_STAT, F_IDPM_LIM,			     		// Reg13 
	F_REG_RST, F_ICO_OPTIMIZED, F_PN, F_TS_PROFILE, F_DEV_REV,  // Reg14 

	F_MAX_FIELDS
};
*/
enum bq25890_fields {
	F_DEVICE_ID,				     		/* Reg00 */
	F_VAC_OVP_DIS, F_VAC_OVP, F_VBUS_OVP_DIS, F_VBUS_OVP,	/* Reg04 */
	F_TSBUS_FLT,/* Reg05 */
	F_TSBAT_FLT,/* Reg06 */
	F_ACDRV_EN, F_ACDRV_MANUAL_EN, F_WD_TIMER_RST, F_WD_TIMEOUT, /* Reg07 */
	F_REG_RST, F_VBUS_PD, F_VAC_PD, F_PMID_PD_EN, F_TSHUT_DIS, F_TSBUS_TSBAT_FLT_DIS,/* Reg08 */
	F_ADC_DONE_STAT, F_REGN_OK_STAT, F_VBAT_PRESENT_STAT, F_VBUS_PRESENT_STAT,F_VAC_PRESENT_STAT,/* Reg09 */
	F_POR_FLG, F_RESET_FLG, F_ADC_DONE_FLG, F_REGN_OK_FLG, F_VBAT_PRESENT_FLG, F_VBUS_PRESENT_FLG, F_VAC_PRESENT_FLG,/* Reg0A */
	F_POR_MASK, F_RESET_MASK, F_ADC_DONE_MASK, F_REGN_OK_MASK, F_VBAT_PRESENT_MASK, F_VBUS_PRESENT_MASK, F_VAC_PRESENT_MASK,/* Reg0B */
	F_TSHUT_STAT, F_TSBAT_HOT_STAT, F_TSBAT_COLD_STAT,F_TSBUS_FLT_STAT, F_VBUS_OVP_STAT, F_VAC_OVP_STAT,/* Reg0C */
	F_WD_TIMEOUT_FLG, F_TSHUT_FLG, F_TSBAT_HOT_FLG, F_TSBAT_COLD_FLG, F_TSBUS_FLT_FLG, F_VBUS_OVP_FLG, F_VAC_OVP_FLG,/* Reg0D */
	F_WD_TIMEOUT_MASK, F_TSHUT_MASK, F_TSBAT_HOT_MASK, F_TSBAT_COLD_MASK, F_TSBUS_FLT_MASK, F_VBUS_OVP_MASK, F_VAC_OVP_MASK,/* Reg0E */
	F_ADC_EN, F_ADC_RATE, F_BATID_ADC_EN, F_IBUS_ADC_DIS, F_VBUS_ADC_DIS, F_VAC_ADC_DIS,/* Reg0F */
	F_VBATSNS_ADC_DIS, F_VBAT_ADC_DIS, F_IBAT_ADC_DIS, F_VSYS_ADC_DIS, F_TSBUS_ADC_DIS, F_TSBAT_ADC_DIS, F_TDIE_ADC_DIS,/* Reg10 */
	F_IBUS_ADC_1,/* Reg11 */
	F_IBUS_ADC_2,/* Reg12 */
	F_VBUS_ADC_1,/* Reg13 */
	F_VBUS_ADC_2,/* Reg14 */
	F_VAC_ADC_1,/* Reg15 */
	F_VAC_ADC_2,/* Reg16 */
	F_VBATSNS_ADC_1,/* Reg17 */
	F_VBATSNS_ADC_2,/* Reg18 */
	F_VBAT_ADC_1,/* Reg19 */
	F_VBAT_ADC_2,/* Reg1A */
	F_IBAT_ADC_1,/* Reg1B */
	F_IBAT_ADC_2,/* Reg1C */
	F_VSYS_ADC_1,/* Reg1D */
	F_VSYS_ADC_2,/* Reg1E */
	F_TSBUS_ADC_1,/* Reg1F */
	F_TSBUS_ADC_2,/* Reg20 */
	F_TSBAT_ADC_1,/* Reg21 */
	F_TSBAT_ADC_2,/* Reg22 */
	F_TDIE_ADC_1,/* Reg23 */
	F_TDIE_ADC_2,/* Reg24 */
	F_BATID_ADC_1,/* Reg25 */
	F_BATID_ADC_2,/* Reg26 */
	F_SYS_MIN,/* Reg30 */
	F_BATSNS_EN,F_VBAT_REG,/* Reg31 */
	F_ICHG_CC,/* Reg32 */
	F_VINDPM_VBAT,F_VINDPM_DIS,F_VINDPM,/* Reg33 */
	F_IINDPM_DIS,F_IINDPM,/* Reg34 */
	F_FORCE_ICO,F_ICO_EN,F_IINDPM_ICO,/* Reg35 */
	F_VBAT_PRECHG,F_IPRECHG,/* Reg36 */
	F_TERM_EN,F_ITERM_LSB_SET,F_ITERM,/* Reg37 */
	F_RECHG_DIS,F_RECHG_DG,F_VRECHG,/* Reg38 */
	F_VBOOST,F_IBOOST_LIM,/* Reg39 */
	F_CONV_OCP_DIS,F_TSBAT_JEITA_DIS,F_IBAT_OCP_DIS,F_VPMID_OVP_OTG_DIS,F_VBAT_OVP_BUCK_DIS,/* Reg3A */
	F_T_BATFET_RST,F_BATFET_RST_EN,F_BATFET_DLY,F_BATFET_DIS,/* Reg3B */
	F_HIZ_EN,F_DIS_BUCK_PATH,F_QB_EN,F_BOOST_EN,F_CHG_EN,/* Reg3C */
	F_VBAT_TRACK,F_IBATOCP,F_VSYSOVP_DIS,F_VSYSOVP_TH,/* Reg3D */
	F_BAT_COMP,F_VCLAMP,F_JEITA_ISET_COOL,F_JEITA_VSET_WARM,/* Reg3E */
	F_TMR2X_EN,F_CHG_TIMER_EN,F_CHG_TIMER,F_TDIE_REG_DIS,F_TDIE_REG,F_PFM_DIS,/* Reg3F */
	F_VBAT_LOW_OTG,F_BOOST_FREQ,F_BUCK_FREQ,F_BAT_LOAD_EN,/* Reg40 */
	F_VSYS_SHORT_STAT,F_VSLEEP_BUCK_STAT,F_VBAT_DPL_STAT,F_VBAT_LOW_BOOST_STAT,F_VBUS_GOOD_STAT,/* Reg41 */
	F_CHG_STAT,F_BOOST_OK_STAT,F_VSYSMIN_REG_STAT,F_QB_ON_STAT,F_BATFET_STAT,/* Reg42 */
	F_TDIE_REG_STAT,F_TSBAT_COOL_STAT,F_TSBAT_WARM_STAT,F_ICO_STAT,F_IINDPM_STAT,F_VINDPM_STAT,/* Reg43 */
	F_VSYS_SHORT_FLG,F_VSLEEP_BUCK_FLG,F_VBAT_DPL_FLG,F_VBAT_LOW_BOOST_FLG,F_VBUS_GOOD_FLG,/* Reg44 */
	F_CHG_FLG,F_BOOST_OK_FLG,F_VSYSMIN_REG_FLG,F_QB_ON_FLG,F_BATFET_FLG,/* Reg45 */
	F_TDIE_REG_FLG,F_TSBAT_COOL_FLG,F_TSBAT_WARM_FLG,F_ICO_FLG,F_IINDPM_FLG,F_VINDPM_FLG,/* Reg46 */
	F_VSYS_SHORT_MASK,F_VSLEEP_BUCK_MASK,F_VBAT_DPL_MASK,F_VBAT_LOW_BOOST_MASK,F_VBUS_GOOD_MASK,/* Reg47 */
	F_CHG_MASK,F_BOOST_OK_MASK,F_VSYSMIN_REG_MASK,F_QB_ON_MASK,F_BATFET_MASK,/* Reg48 */
	F_TDIE_REG_MASK,F_TSBAT_COOL_MASK,F_TSBAT_WARM_MASK,F_ICO_MASK,F_IINDPM_MASK,F_VINDPM_MASK,/* Reg49 */
	F_CONV_OCP_STAT,F_VSYS_OVP_STAT,F_IBUS_RCP_STAT,F_IBAT_OCP_STAT,F_VBAT_OVP_BUCK_STAT,/* Reg50 */
	F_OTG_HICCUP_STAT,F_CHG_TIMEOUT_STAT,F_VPMID_SHORT_STAT,F_VPMID_OVP_OTG_STAT,/* Reg51 */
	F_CONV_OCP_FLG,F_VSYS_OVP_FLG,F_IBUS_RCP_FLG,F_IBAT_OCP_FLG,F_VBAT_OVP_BUCK_FLG,/* Reg52 */
	F_OTG_HICCUP_FLG,F_CHG_TIMEOUT_FLG,F_VPMID_SHORT_FLAG,F_VPMID_OVP_OTG_FLG,/* Reg53 */
	F_CONV_OCP_MASK,F_VSYS_OVP_MASK,F_IBUS_RCP_MASK,F_IBAT_OCP_MASK,F_VBAT_OVP_BUCK_MASK,/* Reg54 */
	F_OTG_HICCUP_MASK,F_CHG_TIMEOUT_MASK,F_VPMID_SHORT_MASK,F_VPMID_OVP_OTG_MASK,/* Reg55 */
	F_JEITA_COOL_TEMP,F_JEITA_WARM_TEMP,F_BOOST_NTC_HOT_TEMP,F_JEITA_COLD_TEMP,F_BOOST_NTC_COLD_TEMP,/* Reg56 */
	F_EN_PUMPX,F_Higher_OCP_BOOST,/* Reg57 */
	F_PUMPX_UP,F_PUMPX_DN,F_VDR_OVP_DEGLITCH,F_VDR_OVP_THRESHOLD,/* Reg58 */
	F_FLED1_EN,F_FLED2_EN,F_TLED1_EN,F_TLED2_EN,F_FL_TX_EN,F_TRPT,/* Reg80 */
	F_FLED1_BR,/* Reg81 */
	F_FLED2_BR,/* Reg82 */
	F_FTIMEOUT_EN,F_FRPT,F_FTIMEOUT,/* Reg83 */
	F_TLED1,/* Reg84 */
	F_TLED2,/* Reg85 */
	F_LED_POWER,F_VBAT_MIN_FLED_DEG,F_VBAT_MIN_FLED,F_PMID_FLED_OVP_DEG,/* Reg86 */
	F_FLED1_STAT,F_FLED2_STAT,F_TLED1_STAT,F_TLED2_STAT,/* Reg87 */
	F_FTIMIEOUT1_STAT,F_FTIMIEOUT2_STAT,F_PMID_FLED_OVP_STAT,F_LED1_SHORT_STAT,F_LED2_SHORT_STAT,F_PTORCH_UVP_STAT,F_PTORCH_OVP_STAT,F_VBAT_LED_LOW_STAT,/* Reg88 */
	F_FTIMIEOUT1_FLG,F_FTIMIEOUT2_FLG,F_PMID_FLED_OVP_FLG,F_LED1_SHORT_FLG,F_LED2_SHORT_FLG,F_PTORCH_UVP_FLG,F_PTORCH_OVP_FLG,F_VBAT_LED_LOW_FLG, /* Reg89 */
	F_FTIMIEOUT1_MASK,F_FTIMIEOUT2_MASK,F_PMID_FLED_OVP_MASK,F_LED1_SHORT_MASK,F_LED2_SHORT_MASK,F_PTORCH_UVP_MASK,F_PTORCH_OVP_MASK,F_VBAT_LED_LOW_MASK,/* Reg8A */
	F_FL_TX_STAT,F_FL_TX_FLG,F_FL_TX_MASK,/* Reg8B */
	F_FORCE_INDET,F_AUTO_INDET_EN,F_HVDCP_EN,F_FC_EN,/* Reg90 */
	F_DP_DRIVE,F_DM_DRIVE,/* Reg91 */
	F_QC35_2PLUS_2MINUS,F_QC35_3PLUS_MINUS,F_QC35_16_MINUS,F_QC35_16_PLUS,F_FC3_MINUS,F_FC3_PLUS,F_FC2,/* Reg92 */
	F_VBUS_STAT,F_INPUT_DET_DONE_FLAG,F_DP_OVP_FLAG,F_DM_OVP_FLAG,/* Reg94 */
	F_INPUT_DET_DONE_MASK,F_DP_OVP_MASK,F_DM_OVP_MASK,/* Reg95 */

	F_DP_OVP_STAT,F_DP_IN4,F_DP_IN3,F_DP_IN2,F_DP_IN1,F_DP_IN0,/* Reg98 */
	F_DM_OVP_STAT,F_DM_IN4,F_DM_IN3,F_DM_IN2,F_DM_IN1,F_DM_IN0,/* Reg99 */
	F_MAX_FIELDS
};


/* initial field values, converted to register values */
struct bq25890_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 boostv;	/* boost regulation voltage	*/
	u8 boosti;	/* boost current limit		*/
	u8 boostf;	/* boost frequency		*/
	u8 ilim_en;	/* enable ILIM pin		*/
	u8 treg;	/* thermal regulation threshold */
};

struct bq25890_state {
	u8 online;
	u8 chrg_status;
	u8 chrg_fault;
	u8 vsys_status;
	u8 boost_fault;
	u8 bat_fault;
};

struct bq25890_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *ps_charger;

	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;

	struct regmap *rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];

	int chip_id;
	struct bq25890_init_data init_data;
	struct bq25890_state state;

	struct mutex lock; /* protect state data */
};

//只读寄存器范围
static const struct regmap_range bq25890_readonly_reg_ranges[] = {
	regmap_reg_range(0x00, 0x00),
	regmap_reg_range(0x09, 0x0A),regmap_reg_range(0x0B, 0x0B),
	regmap_reg_range(0x0C, 0x0D),regmap_reg_range(0x0E, 0x0E),
	regmap_reg_range(0x11, 0x26),
	regmap_reg_range(0x41, 0x46),regmap_reg_range(0x47, 0x49),
	regmap_reg_range(0x50, 0x53),regmap_reg_range(0x54, 0x55),
	regmap_reg_range(0x87, 0x89),regmap_reg_range(0x92, 0x92),
	regmap_reg_range(0x94, 0x99),
	regmap_reg_range(0x9D, 0x9D),
};
//可写寄存器
static const struct regmap_access_table bq25890_writeable_regs = {
	.no_ranges = bq25890_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bq25890_readonly_reg_ranges),
};
//volatile寄存器范围
static const struct regmap_range bq25890_volatile_reg_ranges[] = {
	regmap_reg_range(0x3C, 0x3C),
	regmap_reg_range(0x35, 0x35),
	regmap_reg_range(0x42, 0x42),
	regmap_reg_range(0x58, 0x58),
	regmap_reg_range(0x94, 0x94),
	
};
//volatile寄存器
static const struct regmap_access_table bq25890_volatile_regs = {
	.yes_ranges = bq25890_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bq25890_volatile_reg_ranges),
};

static const struct regmap_config bq25890_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x9D,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bq25890_writeable_regs//,
	//.volatile_table = &bq25890_volatile_regs,
};

/*
struct reg_field {
	unsigned int reg;
	unsigned int lsb;
	unsigned int msb;
	unsigned int id_size;
	unsigned int id_offset;
};

#define REG_FIELD(_reg, _lsb, _msb) {		\
				.reg = _reg,	\
				.lsb = _lsb,	\
				.msb = _msb,	\
				}
*/

/*
static const struct reg_field _bq25890_reg_fields_[] = {
	// REG00 
	[F_EN_HIZ]		= REG_FIELD(0x00, 7, 7),
	[F_EN_ILIM]		= REG_FIELD(0x00, 6, 6),
	[F_IILIM]		= REG_FIELD(0x00, 0, 5),
	// REG01 
	[F_BHOT]		= REG_FIELD(0x01, 6, 7),
	[F_BCOLD]		= REG_FIELD(0x01, 5, 5),
	[F_VINDPM_OFS]	= REG_FIELD(0x01, 0, 4),
	// REG02 
	[F_CONV_START]	= REG_FIELD(0x02, 7, 7),
	[F_CONV_RATE]	= REG_FIELD(0x02, 6, 6),
	[F_BOOSTF]		= REG_FIELD(0x02, 5, 5),
	[F_ICO_EN]		= REG_FIELD(0x02, 4, 4),
	[F_HVDCP_EN]	= REG_FIELD(0x02, 3, 3),
	[F_MAXC_EN]		= REG_FIELD(0x02, 2, 2),
	[F_FORCE_DPM]	= REG_FIELD(0x02, 1, 1),
	[F_AUTO_DPDM_EN]= REG_FIELD(0x02, 0, 0),
	// REG03 
	[F_BAT_LOAD_EN]	= REG_FIELD(0x03, 7, 7),
	[F_WD_RST]		= REG_FIELD(0x03, 6, 6),
	[F_OTG_CFG]		= REG_FIELD(0x03, 5, 5),
	[F_CHG_CFG]		= REG_FIELD(0x03, 4, 4),
	[F_SYSVMIN]		= REG_FIELD(0x03, 1, 3),
	// REG04 
	[F_PUMPX_EN]	= REG_FIELD(0x04, 7, 7),
	[F_ICHG]		= REG_FIELD(0x04, 0, 6),
	// REG05 
	[F_IPRECHG]		= REG_FIELD(0x05, 4, 7),
	[F_ITERM]		= REG_FIELD(0x05, 0, 3),
	// REG06 
	[F_VREG]		= REG_FIELD(0x06, 2, 7),
	[F_BATLOWV]		= REG_FIELD(0x06, 1, 1),
	[F_VRECHG]		= REG_FIELD(0x06, 0, 0),
	// REG07 
	[F_TERM_EN]		= REG_FIELD(0x07, 7, 7),
	[F_STAT_DIS]	= REG_FIELD(0x07, 6, 6),
	[F_WD]			= REG_FIELD(0x07, 4, 5),
	[F_TMR_EN]		= REG_FIELD(0x07, 3, 3),
	[F_CHG_TMR]		= REG_FIELD(0x07, 1, 2),
	[F_JEITA_ISET]	= REG_FIELD(0x07, 0, 0),
	// REG08 
	[F_BATCMP]		= REG_FIELD(0x08, 6, 7),
	[F_VCLAMP]		= REG_FIELD(0x08, 2, 4),
	[F_TREG]		= REG_FIELD(0x08, 0, 1),
	// REG09 
	[F_FORCE_ICO]		= REG_FIELD(0x09, 7, 7),
	[F_TMR2X_EN]		= REG_FIELD(0x09, 6, 6),
	[F_BATFET_DIS]		= REG_FIELD(0x09, 5, 5),
	[F_JEITA_VSET]		= REG_FIELD(0x09, 4, 4),
	[F_BATFET_DLY]		= REG_FIELD(0x09, 3, 3),
	[F_BATFET_RST_EN]	= REG_FIELD(0x09, 2, 2),
	[F_PUMPX_UP]		= REG_FIELD(0x09, 1, 1),
	[F_PUMPX_DN]		= REG_FIELD(0x09, 0, 0),
	// REG0A 
	[F_BOOSTV]			= REG_FIELD(0x0A, 4, 7),
	[F_BOOSTI]			= REG_FIELD(0x0A, 0, 2),
	// REG0B 
	[F_VBUS_STAT]		= REG_FIELD(0x0B, 5, 7),
	[F_CHG_STAT]		= REG_FIELD(0x0B, 3, 4),
	[F_PG_STAT]			= REG_FIELD(0x0B, 2, 2),
	[F_SDP_STAT]		= REG_FIELD(0x0B, 1, 1),
	[F_VSYS_STAT]		= REG_FIELD(0x0B, 0, 0),
	// REG0C 
	[F_WD_FAULT]		= REG_FIELD(0x0C, 7, 7),
	[F_BOOST_FAULT]		= REG_FIELD(0x0C, 6, 6),
	[F_CHG_FAULT]		= REG_FIELD(0x0C, 4, 5),
	[F_BAT_FAULT]		= REG_FIELD(0x0C, 3, 3),
	[F_NTC_FAULT]		= REG_FIELD(0x0C, 0, 2),
	// REG0D 
	[F_FORCE_VINDPM]	= REG_FIELD(0x0D, 7, 7),
	[F_VINDPM]			= REG_FIELD(0x0D, 0, 6),
	// REG0E 
	[F_THERM_STAT]		= REG_FIELD(0x0E, 7, 7),
	[F_BATV]			= REG_FIELD(0x0E, 0, 6),
	// REG0F 
	[F_SYSV]			= REG_FIELD(0x0F, 0, 6),
	// REG10 
	[F_TSPCT]			= REG_FIELD(0x10, 0, 6),
	// REG11 
	[F_VBUS_GD]			= REG_FIELD(0x11, 7, 7),
	[F_VBUSV]			= REG_FIELD(0x11, 0, 6),
	// REG12 
	[F_ICHGR]			= REG_FIELD(0x12, 0, 6),
	// REG13 
	[F_VDPM_STAT]		= REG_FIELD(0x13, 7, 7),
	[F_IDPM_STAT]		= REG_FIELD(0x13, 6, 6),
	[F_IDPM_LIM]		= REG_FIELD(0x13, 0, 5),
	// REG14 
	[F_REG_RST]			= REG_FIELD(0x14, 7, 7),
	[F_ICO_OPTIMIZED]	= REG_FIELD(0x14, 6, 6),
	[F_PN]				= REG_FIELD(0x14, 3, 5),
	[F_TS_PROFILE]		= REG_FIELD(0x14, 2, 2),
	[F_DEV_REV]			= REG_FIELD(0x14, 0, 1)
};
*/

static const struct reg_field bq25890_reg_fields[] = {
	/* REG00 */
	[F_DEVICE_ID]	= REG_FIELD(0x00, 0, 7),
	/* REG04 */
	[F_VAC_OVP_DIS]		= REG_FIELD(0x04, 7, 7),
	[F_VAC_OVP]			= REG_FIELD(0x04, 4, 6),
	[F_VBUS_OVP_DIS]	= REG_FIELD(0x04, 2, 2),
	[F_VBUS_OVP]		= REG_FIELD(0x04, 0, 1),
	
	/* REG05 */
	[F_TSBUS_FLT]	= REG_FIELD(0x05, 0, 7),

	/* REG06 */
	[F_TSBAT_FLT]	= REG_FIELD(0x06, 0, 7),

	/* Reg07 */
	[F_ACDRV_EN]	= REG_FIELD(0x07, 5, 5),
	[F_ACDRV_MANUAL_EN]	= REG_FIELD(0x07, 4, 4),
	[F_WD_TIMER_RST]	= REG_FIELD(0x07, 3, 3),
	[F_WD_TIMEOUT]	= REG_FIELD(0x07, 0, 2),

	/* Reg08 */
	[F_REG_RST]	= REG_FIELD(0x08, 7, 7),
	[F_VBUS_PD]	= REG_FIELD(0x08, 6, 6),
	[F_VAC_PD]	= REG_FIELD(0x08, 5, 5),
	[F_PMID_PD_EN]	= REG_FIELD(0x08, 4, 4),
	[F_TSHUT_DIS]	= REG_FIELD(0x08, 1, 1),
	[F_TSBUS_TSBAT_FLT_DIS]	= REG_FIELD(0x08, 0, 0),

	/* Reg09 */
	[F_ADC_DONE_STAT]	= REG_FIELD(0x09, 4, 4),
	[F_REGN_OK_STAT]	= REG_FIELD(0x09, 3, 3),
	[F_VBAT_PRESENT_STAT]	= REG_FIELD(0x09, 2, 2),
	[F_VBUS_PRESENT_STAT]	= REG_FIELD(0x09, 1, 1),
	[F_VAC_PRESENT_STAT]	= REG_FIELD(0x09, 0, 0),

	/* Reg0A */
	[F_POR_FLG]			= REG_FIELD(0x0A, 6, 6),
	[F_RESET_FLG]		= REG_FIELD(0x0A, 5, 5),
	[F_ADC_DONE_FLG]	= REG_FIELD(0x0A, 4, 4),
	[F_REGN_OK_FLG]		= REG_FIELD(0x0A, 3, 3),
	[F_VBAT_PRESENT_FLG]	= REG_FIELD(0x0A, 2, 2),
	[F_VBUS_PRESENT_FLG]	= REG_FIELD(0x0A, 1, 1),
	[F_VAC_PRESENT_FLG]		= REG_FIELD(0x0A, 0, 0),

	/* Reg0B */
	[F_POR_MASK]	= REG_FIELD(0x0B, 6, 6),
	[F_RESET_MASK]	= REG_FIELD(0x0B, 5, 5),
	[F_ADC_DONE_MASK]	= REG_FIELD(0x0B, 4, 4),
	[F_REGN_OK_MASK]	= REG_FIELD(0x0B, 3, 3),
	[F_VBAT_PRESENT_MASK]	= REG_FIELD(0x0B, 2, 2),
	[F_VBUS_PRESENT_MASK]	= REG_FIELD(0x0B, 1, 1),
	[F_VAC_PRESENT_MASK]	= REG_FIELD(0x0B, 0, 0),

	/* Reg0C */
	[F_TSHUT_STAT]		= REG_FIELD(0x0C, 5, 5),
	[F_TSBAT_HOT_STAT]	= REG_FIELD(0x0C, 4, 4),
	[F_TSBAT_COLD_STAT]	= REG_FIELD(0x0C, 3, 3),
	[F_TSBUS_FLT_STAT]	= REG_FIELD(0x0C, 2, 2),
	[F_VBUS_OVP_STAT]	= REG_FIELD(0x0C, 1, 1),
	[F_VAC_OVP_STAT]	= REG_FIELD(0x0C, 0, 0),

	/* Reg0D */
	[F_WD_TIMEOUT_FLG]	= REG_FIELD(0x0D, 6, 6),
	[F_TSHUT_FLG]		= REG_FIELD(0x0D, 5, 5),
	[F_TSBAT_HOT_FLG]	= REG_FIELD(0x0D, 4, 4),
	[F_TSBAT_COLD_FLG]	= REG_FIELD(0x0D, 3, 3),
	[F_TSBUS_FLT_FLG]	= REG_FIELD(0x0D, 2, 2),
	[F_VBUS_OVP_FLG]	= REG_FIELD(0x0D, 1, 1),
	[F_VAC_OVP_FLG]		= REG_FIELD(0x0D, 0, 0),


	/* Reg0E */
	[F_WD_TIMEOUT_MASK]	= REG_FIELD(0x0E, 6, 6),
	[F_TSHUT_MASK]		= REG_FIELD(0x0E, 5, 5),
	[F_TSBAT_HOT_MASK]	= REG_FIELD(0x0E, 4, 4),
	[F_TSBAT_COLD_MASK]	= REG_FIELD(0x0E, 3, 3),
	[F_TSBUS_FLT_MASK]	= REG_FIELD(0x0E, 2, 2),
	[F_VBUS_OVP_MASK]	= REG_FIELD(0x0E, 1, 1),
	[F_VAC_OVP_MASK]	= REG_FIELD(0x0E, 0, 0),

	/* Reg0F */
	[F_ADC_EN]			= REG_FIELD(0x0F, 7, 7),
	[F_ADC_RATE]		= REG_FIELD(0x0F, 6, 6),
	[F_BATID_ADC_EN]	= REG_FIELD(0x0F, 3, 3),
	[F_IBUS_ADC_DIS]	= REG_FIELD(0x0F, 2, 2),
	[F_VBUS_ADC_DIS]	= REG_FIELD(0x0F, 1, 1),
	[F_VAC_ADC_DIS]		= REG_FIELD(0x0F, 0, 0),

	/* Reg10 */
	[F_VBATSNS_ADC_DIS]	= REG_FIELD(0x10, 6, 6),
	[F_VBAT_ADC_DIS]	= REG_FIELD(0x10, 5, 5),
	[F_IBAT_ADC_DIS]	= REG_FIELD(0x10, 4, 4),
	[F_VSYS_ADC_DIS]	= REG_FIELD(0x10, 3, 3),
	[F_TSBUS_ADC_DIS]	= REG_FIELD(0x10, 2, 2),
	[F_TSBAT_ADC_DIS]	= REG_FIELD(0x10, 1, 1),
	[F_TDIE_ADC_DIS]	= REG_FIELD(0x10, 0, 0),

	/* Reg11 */
	[F_IBUS_ADC_1]	= REG_FIELD(0x11, 0, 3),
	/* Reg12 */
	[F_IBUS_ADC_2]	= REG_FIELD(0x12, 0, 7),

	/* Reg13 */
	[F_VBUS_ADC_1]	= REG_FIELD(0x13, 0, 3),
	/* Reg14 */
	[F_VBUS_ADC_2]	= REG_FIELD(0x14, 0, 7),

	/* Reg15 */
	[F_VAC_ADC_1]	= REG_FIELD(0x15, 0, 3),
	/* Reg16 */
	[F_VAC_ADC_2]	= REG_FIELD(0x16, 0, 7),

	/* Reg17 */
	[F_VBATSNS_ADC_1]	= REG_FIELD(0x17, 0, 7),
	/* Reg18 */
	[F_VBATSNS_ADC_2]	= REG_FIELD(0x18, 0, 7),

	/* Reg19 */
	[F_VBAT_ADC_1]	= REG_FIELD(0x19, 0, 3),
	/* Reg1A */
	[F_VBAT_ADC_2]	= REG_FIELD(0x1A, 0, 7),
	/* Reg1B */
	[F_IBAT_ADC_1]	= REG_FIELD(0x1B, 0, 3),
	/* Reg1C */
	[F_IBAT_ADC_2]	= REG_FIELD(0x1C, 0, 7),

	/* Reg1D *//* Reg1E */
	[F_VSYS_ADC_1]	= REG_FIELD(0x1D, 0, 3),
	[F_VSYS_ADC_2]	= REG_FIELD(0x1E, 0, 7),

	/* Reg1F *//* Reg20 */
	[F_TSBUS_ADC_1]	= REG_FIELD(0x1F, 0, 1),
	[F_TSBUS_ADC_2]	= REG_FIELD(0x20, 0, 7),


	/* Reg21 *//* Reg22 */
	[F_TSBAT_ADC_1]	= REG_FIELD(0x21, 0, 1),
	[F_TSBAT_ADC_2]	= REG_FIELD(0x22, 0, 7),
	/* Reg23 *//* Reg24 */
	[F_TDIE_ADC_1] = REG_FIELD(0x23, 0, 0),
	[F_TDIE_ADC_2] = REG_FIELD(0x24, 0, 7),
	/* Reg25 *//* Reg26 */
	[F_BATID_ADC_1] = REG_FIELD(0x25, 0, 3),
	[F_BATID_ADC_2] = REG_FIELD(0x26, 0, 7),
	/* Reg30 */
	[F_SYS_MIN] = REG_FIELD(0x30, 0, 2),

	/* Reg31 */
	[F_BATSNS_EN] = REG_FIELD(0x31, 7, 7),
	[F_VBAT_REG] = REG_FIELD(0x31, 0, 6),

	/* Reg32 */
	[F_ICHG_CC] = REG_FIELD(0x32, 0, 6),

	/* Reg33 */
	[F_VINDPM_VBAT] = REG_FIELD(0x33, 5, 6),
	[F_VINDPM_DIS] 	= REG_FIELD(0x33, 4, 4),
	[F_VINDPM] 		= REG_FIELD(0x33, 0, 3),

	/* Reg34 */
	[F_IINDPM_DIS] = REG_FIELD(0x34, 5, 7),
	[F_IINDPM] = REG_FIELD(0x34, 0, 5),

	/* Reg35 */
	[F_FORCE_ICO] = REG_FIELD(0x35, 7, 7),
	[F_ICO_EN] = REG_FIELD(0x35, 6, 6),
	[F_IINDPM_ICO] = REG_FIELD(0x35, 0, 5),

	/* Reg36 */
	[F_VBAT_PRECHG] = REG_FIELD(0x36, 6, 7),
	[F_IPRECHG] 	= REG_FIELD(0x36, 0, 3),

	/* Reg37 */
	[F_TERM_EN] = REG_FIELD(0x37, 6, 7),
	[F_ITERM_LSB_SET] = REG_FIELD(0x37, 6, 6),
	[F_ITERM] = REG_FIELD(0x37, 0, 4),

	/* Reg38 */
	[F_RECHG_DIS] = REG_FIELD(0x38, 4, 4),
	[F_RECHG_DG] = REG_FIELD(0x38, 2, 3),
	[F_VRECHG] = REG_FIELD(0x38, 0, 1),

	/* Reg39 */
	[F_VBOOST] = REG_FIELD(0x39, 3, 7),
	[F_IBOOST_LIM] = REG_FIELD(0x39, 0, 2),

	/* Reg3A */
	[F_CONV_OCP_DIS] = REG_FIELD(0x3A, 4, 4),
	[F_TSBAT_JEITA_DIS] = REG_FIELD(0x3A, 3, 3),
	[F_IBAT_OCP_DIS] = REG_FIELD(0x3A, 2, 2),
	[F_VPMID_OVP_OTG_DIS] = REG_FIELD(0x3A, 1, 1),
	[F_VBAT_OVP_BUCK_DIS] = REG_FIELD(0x3A, 0, 0),

	/* Reg3B */
	[F_T_BATFET_RST] = REG_FIELD(0x3B, 5, 5),
	[F_BATFET_RST_EN] = REG_FIELD(0x3B, 3, 3),
	[F_BATFET_DLY] = REG_FIELD(0x3B, 2, 2),
	[F_BATFET_DIS] = REG_FIELD(0x3B, 1, 1),

	/* Reg3C */
	[F_HIZ_EN] = REG_FIELD(0x3C, 7, 7),
	[F_DIS_BUCK_PATH] = REG_FIELD(0x3C, 5, 5),
	[F_QB_EN] = REG_FIELD(0x3C, 2, 2),
	[F_BOOST_EN] = REG_FIELD(0x3C, 1, 1),
	[F_CHG_EN] = REG_FIELD(0x3C, 0, 0),
	/* Reg3D */
	[F_VBAT_TRACK] 	= REG_FIELD(0x3D, 5, 5),
	[F_IBATOCP] 	= REG_FIELD(0x3D, 3, 4),
	[F_VSYSOVP_DIS] = REG_FIELD(0x3D, 2, 2),
	[F_VSYSOVP_TH] 	= REG_FIELD(0x3D, 0, 1),

	/* Reg3E */
	[F_BAT_COMP] = REG_FIELD(0x3E, 5, 7),
	[F_VCLAMP] 	= REG_FIELD(0x3E, 2, 4),
	[F_JEITA_ISET_COOL] = REG_FIELD(0x3E, 1, 1),
	[F_JEITA_VSET_WARM] = REG_FIELD(0x3E, 0, 0),

	/* Reg3F */
	[F_TMR2X_EN] 	= REG_FIELD(0x3F, 7, 7),
	[F_CHG_TIMER_EN] = REG_FIELD(0x3F, 6, 6),
	[F_CHG_TIMER] 	= REG_FIELD(0x3F, 4, 5),
	[F_TDIE_REG_DIS] = REG_FIELD(0x3F, 3, 3),
	[F_TDIE_REG] 	= REG_FIELD(0x3F, 1, 2),
	[F_PFM_DIS] 	= REG_FIELD(0x3F, 0, 0),


	/* Reg40 */
	[F_VBAT_LOW_OTG] = REG_FIELD(0x40, 5, 5),
	[F_BOOST_FREQ] = REG_FIELD(0x40, 3, 4),
	[F_BUCK_FREQ] = REG_FIELD(0x40, 1, 2),
	[F_BAT_LOAD_EN] = REG_FIELD(0x40, 0, 0),

	/* Reg41 */
	[F_VSYS_SHORT_STAT] = REG_FIELD(0x41, 4, 4),
	[F_VSLEEP_BUCK_STAT] = REG_FIELD(0x41, 3, 3),
	[F_VBAT_DPL_STAT] 	= REG_FIELD(0x41, 2, 2),
	[F_VBAT_LOW_BOOST_STAT] = REG_FIELD(0x41, 1, 1),
	[F_VBUS_GOOD_STAT] 		= REG_FIELD(0x41, 0, 0),


	/* Reg42 */
	[F_CHG_STAT] = REG_FIELD(0x42, 5, 7),
	[F_BOOST_OK_STAT] = REG_FIELD(0x42, 4, 4),
	[F_VSYSMIN_REG_STAT] = REG_FIELD(0x42, 3, 3),
	[F_QB_ON_STAT] = REG_FIELD(0x42, 2, 2),
	[F_BATFET_STAT] = REG_FIELD(0x42, 0, 1),

	/* Reg43 */
	[F_TDIE_REG_STAT] = REG_FIELD(0x43, 6, 6),
	[F_TSBAT_COOL_STAT] = REG_FIELD(0x43, 5, 5),
	[F_TSBAT_WARM_STAT] = REG_FIELD(0x43, 4, 4),
	[F_ICO_STAT] 	= REG_FIELD(0x43, 2, 3),
	[F_IINDPM_STAT] = REG_FIELD(0x43, 1, 1),
	[F_VINDPM_STAT] = REG_FIELD(0x43, 0, 0),

	/* Reg44 */
	[F_VSYS_SHORT_FLG] = REG_FIELD(0x44, 4, 4),
	[F_VSLEEP_BUCK_FLG] = REG_FIELD(0x44, 3, 3),
	[F_VBAT_DPL_FLG] = REG_FIELD(0x44, 2, 2),
	[F_VBAT_LOW_BOOST_FLG] = REG_FIELD(0x44, 1, 1),
	[F_VBUS_GOOD_FLG] = REG_FIELD(0x44, 0, 0),
	/* Reg45 */
	[F_CHG_FLG] = REG_FIELD(0x45, 5, 5),
	[F_BOOST_OK_FLG] = REG_FIELD(0x45, 4, 4),
	[F_VSYSMIN_REG_FLG] = REG_FIELD(0x45, 3, 3),
	[F_QB_ON_FLG] = REG_FIELD(0x45, 2, 2),
	[F_BATFET_FLG] = REG_FIELD(0x45, 0, 0),


	/* Reg46 */
	[F_TDIE_REG_FLG] = REG_FIELD(0x46, 6, 6),
	[F_TSBAT_COOL_FLG] = REG_FIELD(0x46, 5, 5),
	[F_TSBAT_WARM_FLG] = REG_FIELD(0x46, 4, 4),
	[F_ICO_FLG] = REG_FIELD(0x46, 2, 2),
	[F_IINDPM_FLG] = REG_FIELD(0x46, 1, 1),
	[F_VINDPM_FLG] = REG_FIELD(0x46, 0, 0),

	/* Reg47 */
	[F_VSYS_SHORT_MASK] = REG_FIELD(0x47, 4, 4),
	[F_VSLEEP_BUCK_MASK] = REG_FIELD(0x47, 3, 3),
	[F_VBAT_DPL_MASK] = REG_FIELD(0x47, 2, 2),
	[F_VBAT_LOW_BOOST_MASK] = REG_FIELD(0x47, 1, 1),
	[F_VBUS_GOOD_MASK] = REG_FIELD(0x47, 0, 0),

	/* Reg48 */
	[F_CHG_MASK] = REG_FIELD(0x48, 5, 5),
	[F_BOOST_OK_MASK] = REG_FIELD(0x48, 4, 4),
	[F_VSYSMIN_REG_MASK] = REG_FIELD(0x48, 3, 3),
	[F_QB_ON_MASK] = REG_FIELD(0x48, 2, 2),
	[F_BATFET_MASK] = REG_FIELD(0x48, 0, 0),


	/* Reg49 */
	[F_TDIE_REG_MASK] = REG_FIELD(0x49, 6, 6),
	[F_TSBAT_COOL_MASK] = REG_FIELD(0x49, 5, 5),
	[F_TSBAT_WARM_MASK] = REG_FIELD(0x49, 4, 4),
	[F_ICO_MASK] = REG_FIELD(0x49, 2, 2),
	[F_IINDPM_MASK] = REG_FIELD(0x49, 1, 1),
	[F_VINDPM_MASK] = REG_FIELD(0x49, 0, 0),

	/* Reg50 */
	[F_CONV_OCP_STAT] = REG_FIELD(0x50, 4, 4),
	[F_VSYS_OVP_STAT] = REG_FIELD(0x50, 3, 3),
	[F_IBUS_RCP_STAT] = REG_FIELD(0x50, 2, 2),
	[F_IBAT_OCP_STAT] = REG_FIELD(0x50, 1, 1),
	[F_VBAT_OVP_BUCK_STAT] = REG_FIELD(0x50, 0, 0),

	/* Reg51 */
	[F_OTG_HICCUP_STAT] = REG_FIELD(0x51, 3, 3),
	[F_CHG_TIMEOUT_STAT] = REG_FIELD(0x51, 2, 2),
	[F_VPMID_SHORT_STAT] = REG_FIELD(0x51, 1, 1),
	[F_VPMID_OVP_OTG_STAT] = REG_FIELD(0x51, 0, 0),

	/* Reg52 */
	[F_CONV_OCP_FLG] = REG_FIELD(0x52, 4, 4),
	[F_VSYS_OVP_FLG] = REG_FIELD(0x52, 3, 3),
	[F_IBUS_RCP_FLG] = REG_FIELD(0x52, 2, 2),
	[F_IBAT_OCP_FLG] = REG_FIELD(0x52, 1, 1),
	[F_VBAT_OVP_BUCK_FLG] = REG_FIELD(0x52, 0, 0),

	/* Reg53 */
	[F_OTG_HICCUP_FLG] = REG_FIELD(0x53, 3, 3),
	[F_CHG_TIMEOUT_FLG] = REG_FIELD(0x53, 2, 2),
	[F_VPMID_SHORT_FLAG] = REG_FIELD(0x53, 1, 1),
	[F_VPMID_OVP_OTG_FLG] = REG_FIELD(0x53, 0, 0),

	/* Reg54 */
	[F_CONV_OCP_MASK] = REG_FIELD(0x54, 4, 4),
	[F_VSYS_OVP_MASK] = REG_FIELD(0x54, 3, 3),
	[F_IBUS_RCP_MASK] = REG_FIELD(0x54, 2, 2),
	[F_IBAT_OCP_MASK] = REG_FIELD(0x54, 1, 1),
	[F_VBAT_OVP_BUCK_MASK] = REG_FIELD(0x54, 0, 0),

	/* Reg55 */
	[F_OTG_HICCUP_MASK] = REG_FIELD(0x55, 3, 3),
	[F_CHG_TIMEOUT_MASK] = REG_FIELD(0x55, 2, 2),
	[F_VPMID_SHORT_MASK] = REG_FIELD(0x55, 1, 1),
	[F_VPMID_OVP_OTG_MASK] = REG_FIELD(0x55, 0, 0),

	/* Reg56 */
	[F_JEITA_COOL_TEMP] = REG_FIELD(0x56, 6, 7),
	[F_JEITA_WARM_TEMP] = REG_FIELD(0x56, 4, 5),
	[F_BOOST_NTC_HOT_TEMP] = REG_FIELD(0x56, 2, 3),
	[F_JEITA_COLD_TEMP] = REG_FIELD(0x56, 1, 1),
	[F_BOOST_NTC_COLD_TEMP] = REG_FIELD(0x56, 0, 0),

	/* Reg57 */
	[F_EN_PUMPX] = REG_FIELD(0x57, 4, 4),
	[F_Higher_OCP_BOOST] = REG_FIELD(0x57, 1, 3),

	/* Reg58 */
	[F_PUMPX_UP] = REG_FIELD(0x58, 5, 5),
	[F_PUMPX_DN] = REG_FIELD(0x58, 4, 4),
	[F_VDR_OVP_DEGLITCH] = REG_FIELD(0x58, 3, 3),
	[F_VDR_OVP_THRESHOLD] = REG_FIELD(0x58, 0, 2),

	/* Reg80 */
	[F_FLED1_EN] = REG_FIELD(0x80, 7, 7),
	[F_FLED2_EN] = REG_FIELD(0x80, 6, 6),
	[F_TLED1_EN] = REG_FIELD(0x80, 5, 5),
	[F_TLED2_EN] = REG_FIELD(0x80, 4, 4),
	[F_FL_TX_EN] = REG_FIELD(0x80, 3, 3),
	[F_TRPT] 	 = REG_FIELD(0x80, 0, 2),

	/* Reg81 */
	[F_FLED1_BR] = REG_FIELD(0x81, 0, 6),
	/* Reg82 */
	[F_FLED2_BR] = REG_FIELD(0x82, 0, 6),
	
	/* Reg83 */
	[F_FTIMEOUT_EN] = REG_FIELD(0x83, 7, 7),
	[F_FRPT] = REG_FIELD(0x83, 4, 6),
	[F_FTIMEOUT] = REG_FIELD(0x83, 0, 3),

	/* Reg84 */
	[F_TLED1] = REG_FIELD(0x84, 0, 6),
	/* Reg85 */
	[F_TLED2] = REG_FIELD(0x85, 0, 6),

	/* Reg86 */
	[F_LED_POWER] = REG_FIELD(0x86, 7, 7),
	[F_VBAT_MIN_FLED_DEG] = REG_FIELD(0x86, 5, 6),
	[F_VBAT_MIN_FLED] = REG_FIELD(0x86, 2, 4),
	[F_PMID_FLED_OVP_DEG] = REG_FIELD(0x86, 0, 1),

	/* Reg87 */
	[F_FLED1_STAT] = REG_FIELD(0x87, 3, 3),
	[F_FLED2_STAT] = REG_FIELD(0x87, 2, 2),
	[F_TLED1_STAT] = REG_FIELD(0x87, 1, 1),
	[F_TLED2_STAT] = REG_FIELD(0x87, 0, 0),
	/* Reg88 */
	[F_FTIMIEOUT1_STAT] = REG_FIELD(0x88, 7, 7),
	[F_FTIMIEOUT2_STAT] = REG_FIELD(0x88, 6, 6),
	[F_PMID_FLED_OVP_STAT] = REG_FIELD(0x88, 5, 5),
	[F_LED1_SHORT_STAT] = REG_FIELD(0x88, 4, 4),
	[F_LED2_SHORT_STAT] = REG_FIELD(0x88, 3, 3),
	[F_PTORCH_UVP_STAT] = REG_FIELD(0x88, 2, 2),
	[F_PTORCH_OVP_STAT] = REG_FIELD(0x88, 1, 1),
	[F_VBAT_LED_LOW_STAT] = REG_FIELD(0x88, 0, 0),

	/* Reg89 */
	[F_FTIMIEOUT1_FLG] = REG_FIELD(0x89, 7, 7),
	[F_FTIMIEOUT2_FLG] = REG_FIELD(0x89, 6, 6),
	[F_PMID_FLED_OVP_FLG] = REG_FIELD(0x89, 5, 5),
	[F_LED1_SHORT_FLG] = REG_FIELD(0x89, 4, 4),
	[F_LED2_SHORT_FLG] = REG_FIELD(0x89, 3, 3),
	[F_PTORCH_UVP_FLG] = REG_FIELD(0x89, 2, 2),
	[F_PTORCH_OVP_FLG] = REG_FIELD(0x89, 1, 1),
	[F_VBAT_LED_LOW_FLG] = REG_FIELD(0x89, 0, 0),

	/* Reg8A */
	[F_FTIMIEOUT1_MASK] = REG_FIELD(0x8A, 7, 7),
	[F_FTIMIEOUT2_MASK] = REG_FIELD(0x8A, 6, 6),
	[F_PMID_FLED_OVP_MASK] = REG_FIELD(0x8A, 5, 5),
	[F_LED1_SHORT_MASK] = REG_FIELD(0x8A, 4, 4),
	[F_LED2_SHORT_MASK] = REG_FIELD(0x8A, 3, 3),
	[F_PTORCH_UVP_MASK] = REG_FIELD(0x8A, 2, 2),
	[F_PTORCH_OVP_MASK] = REG_FIELD(0x8A, 1, 1),
	[F_VBAT_LED_LOW_MASK] = REG_FIELD(0x8A, 0, 0),


	/* Reg8B */
	[F_FL_TX_STAT] = REG_FIELD(0x8B, 2, 2),
	[F_FL_TX_FLG] = REG_FIELD(0x8B, 1, 1),
	[F_FL_TX_MASK] = REG_FIELD(0x8B, 0, 0),


	/* Reg90 */
	[F_FORCE_INDET] = REG_FIELD(0x90, 7, 7),
	[F_AUTO_INDET_EN] = REG_FIELD(0x90, 6, 6),
	[F_HVDCP_EN] = REG_FIELD(0x90, 5, 5),
	[F_FC_EN] = REG_FIELD(0x90, 0, 0),

	/* Reg91 */
	[F_DP_DRIVE] = REG_FIELD(0x91, 5, 7),
	[F_DM_DRIVE] = REG_FIELD(0x91, 2, 4),

	/* Reg92 */
	[F_QC35_2PLUS_2MINUS] = REG_FIELD(0x92, 7, 7),
	[F_QC35_3PLUS_MINUS] = REG_FIELD(0x92, 6, 6),
	[F_QC35_16_MINUS] = REG_FIELD(0x92, 5, 5),
	[F_QC35_16_PLUS] = REG_FIELD(0x92, 4, 4),
	[F_FC3_MINUS] = REG_FIELD(0x92, 3, 3),
	[F_FC3_PLUS] = REG_FIELD(0x92, 2, 2),
	[F_FC2] = REG_FIELD(0x92, 0, 1),

	/* Reg94 */
	[F_VBUS_STAT] = REG_FIELD(0x94, 5, 7),
	[F_INPUT_DET_DONE_FLAG] = REG_FIELD(0x94, 2, 2),
	[F_DP_OVP_FLAG] = REG_FIELD(0x94, 1, 1),
	[F_DM_OVP_FLAG] = REG_FIELD(0x94, 0, 0),

	/* Reg95 */
	[F_INPUT_DET_DONE_MASK] = REG_FIELD(0x95, 2, 2),
	[F_DP_OVP_MASK] = REG_FIELD(0x95, 1, 1),
	[F_DM_OVP_MASK] = REG_FIELD(0x95, 0, 0),

	/* Reg98 */
	[F_DP_OVP_STAT] = REG_FIELD(0x98, 7, 7),
	[F_DP_IN4] = REG_FIELD(0x98, 4, 4),
	[F_DP_IN3] = REG_FIELD(0x98, 3, 3),
	[F_DP_IN2] = REG_FIELD(0x98, 2, 2),
	[F_DP_IN1] = REG_FIELD(0x98, 1, 1),
	[F_DP_IN0] = REG_FIELD(0x98, 0, 0),
	/* Reg99 */
	[F_DM_OVP_STAT] = REG_FIELD(0x99, 7, 7),
	[F_DM_IN4] = REG_FIELD(0x99, 4, 4),
	[F_DM_IN3] = REG_FIELD(0x99, 3, 3),
	[F_DM_IN2] = REG_FIELD(0x99, 2, 2),
	[F_DM_IN1] = REG_FIELD(0x99, 1, 1),
	[F_DM_IN0] = REG_FIELD(0x99, 0, 0)

};

/*
 * Most of the val -> idx conversions can be computed, given the minimum,
 * maximum and the step between values. For the rest of conversions, we use
 * lookup tables.
 */
enum bq25890_table_ids {
	/* range tables */
	TBL_ICHG,
	TBL_ITERM,
	TBL_IPRECHG,
	TBL_VREG,
	TBL_BATCMP,
	TBL_VCLAMP,
	TBL_BOOSTV,
	TBL_SYSVMIN,

	/* lookup tables */
	TBL_TREG,
	TBL_BOOSTI,
};

/* Thermal Regulation Threshold lookup table, in degrees Celsius */
static const u32 bq25890_treg_tbl[] = { 60, 80, 100, 120 };

#define BQ25890_TREG_TBL_SIZE		ARRAY_SIZE(bq25890_treg_tbl)

/* Boost mode current limit lookup table, in uA */
static const u32 bq25890_boosti_tbl[] = {
	500000, 700000, 1100000, 1300000, 1600000, 1800000, 2100000, 2400000
};

#define BQ25890_BOOSTI_TBL_SIZE		ARRAY_SIZE(bq25890_boosti_tbl)

struct bq25890_range {
	u32 min;
	u32 max;
	u32 step;
};

struct bq25890_lookup {
	const u32 *tbl;
	u32 size;
};

static const union {
	struct bq25890_range  rt;
	struct bq25890_lookup lt;
} bq25890_tables[] = {
	/* range tables */
	[TBL_ICHG] =	{ .rt = {0,	  5056000, 64000} },	 /* uA */
	[TBL_ITERM] =	{ .rt = {64000,   1024000, 64000} }, /* uA */
	[TBL_VREG] =	{ .rt = {3840000, 4608000, 16000} }, /* uV */
	[TBL_BATCMP] =	{ .rt = {0,	  140,     20} },	 	/* mOhm */
	[TBL_VCLAMP] =	{ .rt = {0,	  224000,  32000} },	 /* uV */
	[TBL_BOOSTV] =	{ .rt = {4550000, 5510000, 64000} }, /* uV */
	[TBL_SYSVMIN] = { .rt = {3000000, 3700000, 100000} }, /* uV */

	/* lookup tables */
	[TBL_TREG] =	{ .lt = {bq25890_treg_tbl, BQ25890_TREG_TBL_SIZE} },
	[TBL_BOOSTI] =	{ .lt = {bq25890_boosti_tbl, BQ25890_BOOSTI_TBL_SIZE} }
};


//读取指定寄存器的值
static int bq25890_field_read(struct bq25890_device *bq, enum bq25890_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(bq->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}
//改写指定寄存器的值
static int bq25890_field_write(struct bq25890_device *bq, enum bq25890_fields field_id, u8 val)
{
	return regmap_field_write(bq->rmap_fields[field_id], val);
}
//查找 ？
static u8 bq25890_find_idx(u32 value, enum bq25890_table_ids id)
{
	u8 idx;

	if (id >= TBL_TREG) {
		const u32 *tbl = bq25890_tables[id].lt.tbl;
		u32 tbl_size = bq25890_tables[id].lt.size;

		for (idx = 1; idx < tbl_size && tbl[idx] <= value; idx++)
			;
	} else {
		const struct bq25890_range *rtbl = &bq25890_tables[id].rt;
		u8 rtbl_size;

		rtbl_size = (rtbl->max - rtbl->min) / rtbl->step + 1;

		for ( idx = 1; idx < rtbl_size && (idx * rtbl->step + rtbl->min <= value);  idx++ )
			;
	}

	return idx - 1;
}

static u32 bq25890_find_val(u8 idx, enum bq25890_table_ids id)
{
	const struct bq25890_range *rtbl;

	/* lookup table? */
	if (id >= TBL_TREG)
		return bq25890_tables[id].lt.tbl[idx];

	/* range table */
	rtbl = &bq25890_tables[id].rt;

	return (rtbl->min + idx * rtbl->step);
}

enum bq25890_status {
	STATUS_NOT_CHARGING,//没有充电
	STATUS_PRE_CHARGING,//预充
	STATUS_FAST_CHARGING,//快充
	STATUS_TERMINATION_DONE,//充电完成
};

enum bq25890_chrg_fault {
	CHRG_FAULT_NORMAL,//普通错误
	CHRG_FAULT_INPUT,//输入错误
	CHRG_FAULT_THERMAL_SHUTDOWN,
	CHRG_FAULT_TIMER_EXPIRED,
};

//获取指定的属性
static int bq25890_power_supply_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	int ret;
	int v1, v2;

	struct bq25890_device *bq = power_supply_get_drvdata(psy);
	struct bq25890_state state;

	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING || state.chrg_status == STATUS_FAST_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ25890_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (!state.chrg_fault && !state.bat_fault && !state.boost_fault)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else if (state.bat_fault)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (state.chrg_fault == CHRG_FAULT_TIMER_EXPIRED)
			val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		else if (state.chrg_fault == CHRG_FAULT_THERMAL_SHUTDOWN)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		//ret = bq25890_field_read(bq, F_ICHGR); /* read measured value */
		ret = bq25890_field_read(bq, F_ICHG_CC); /* 读取电流的测量值 */
		
		if (ret < 0)
			return ret;

		/* converted_val = ADC_val * 50mA (table 10.3.19) */
		val->intval = ret * 50000;//Bit step size: 50mA
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq25890_tables[TBL_ICHG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		if (!state.online) {
			val->intval = 0;
			break;
		}

		//ret = bq25890_field_read(bq, F_BATV); /* read measured value */
		v1 = bq25890_field_read(bq, F_VBAT_ADC_1); /* 改为两个字节合并 */
		v2 = bq25890_field_read(bq, F_VBAT_ADC_2); /* 改为两个字节合并 */
		
		if (v1 < 0 || v2 < 0)
			return v1<v2?v1:v2;

		ret = ((v1 & 0xF) << 8) | (v2 & 0xFF);

		/* converted_val = 2.304V + ADC_val * 20mV (table 10.3.15) */
		val->intval = 2304000 + ret * 20000;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq25890_tables[TBL_VREG].rt.max;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq25890_find_val(bq->init_data.iterm, TBL_ITERM);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int bq25890_get_chip_state(struct bq25890_device *bq, struct bq25890_state *state)
{
	int i, ret;

	struct {
		enum bq25890_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_VBUS_GOOD_STAT,		&state->online},//{F_PG_STAT,		&state->online},
		{F_VSYSMIN_REG_STAT,	&state->vsys_status},//{F_VSYS_STAT,	&state->vsys_status},
		
		//{F_BOOST_FAULT, &state->boost_fault},//{F_BOOST_FAULT, &state->boost_fault},
		
		{F_BATFET_STAT,	&state->bat_fault}//{F_BAT_FAULT,	&state->bat_fault},
		
		//{F_CHG_FAULT,	&state->chrg_fault}
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = bq25890_field_read(bq, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	//dev_dbg(bq->dev, "[OBEI_BC]S:CHG/PG/VSYS=%d/%d/%d, F:CHG/BOOST/BAT=%d/%d/%d\n", state->chrg_status, state->online, state->vsys_status, state->chrg_fault, state->boost_fault, state->bat_fault);

	return 0;
}
//判断状态是否发生改变
static bool bq25890_state_changed(struct bq25890_device *bq, struct bq25890_state *new_state)
{
	struct bq25890_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		    old_state.chrg_fault != new_state->chrg_fault	||
		    old_state.online != new_state->online		||
		    old_state.bat_fault != new_state->bat_fault	||
		    old_state.boost_fault != new_state->boost_fault ||
		    old_state.vsys_status != new_state->vsys_status);
}

static void bq25890_handle_state_change(struct bq25890_device *bq, struct bq25890_state *new_state)
{
	int ret;
	struct bq25890_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	if (!new_state->online) {  /* power removed */
		/* disable ADC */
		//ret = bq25890_field_write(bq, F_CONV_START, 0);
		ret = bq25890_field_write(bq, F_ADC_EN, 0);
		
		if (ret < 0)
			goto error;
	} else if (!old_state.online) {	  /* power inserted */
		/* enable ADC, to have control of charge current/voltage */
		//ret = bq25890_field_write(bq, F_CONV_START, 1);
		ret = bq25890_field_write(bq, F_ADC_EN, 1);
		if (ret < 0)
			goto error;
	}

	return;

error:
	dev_err(bq->dev, "Error communicating with the chip.\n");
}

static irqreturn_t bq25890_irq_handler_thread(int irq, void *private)
{
	struct bq25890_device *bq = private;
	int ret;
	struct bq25890_state state;

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0)
		goto handled;

	if (!bq25890_state_changed(bq, &state))
		goto handled;

	bq25890_handle_state_change(bq, &state);

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->ps_charger);

handled:
	return IRQ_HANDLED;
}

static int bq25890_chip_reset(struct bq25890_device *bq)
{
	int ret;
	int rst_check_counter = 10;

	ret = bq25890_field_write(bq, F_REG_RST, 1);//此行无需修改，新老文档名字一样，寄存器从REG14变REG08,
	if (ret < 0)
		return ret;

	do {
		ret = bq25890_field_read(bq, F_REG_RST);//此行无需修改，新老文档名字一样，寄存器从REG14变REG08,
		if (ret < 0)
			return ret;

		usleep_range(5, 10);
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
		return -ETIMEDOUT;

	return 0;
}

static int bq25890_hw_init(struct bq25890_device *bq)
{
	int ret;
	int i;
	struct bq25890_state state;

	const struct {
		enum bq25890_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG,	 bq->init_data.ichg},
		{F_VREG,	 bq->init_data.vreg},
		{F_ITERM,	 bq->init_data.iterm},
		{F_IPRECHG,	 bq->init_data.iprechg},
		{F_SYSVMIN,	 bq->init_data.sysvmin},
		{F_BOOSTV,	 bq->init_data.boostv},
		{F_BOOSTI,	 bq->init_data.boosti},
		{F_BOOSTF,	 bq->init_data.boostf},
		{F_EN_ILIM,	 bq->init_data.ilim_en},
		{F_TREG,	 bq->init_data.treg}
	};

	ret = bq25890_chip_reset(bq);
	if (ret < 0)
		return ret;

	/* disable watchdog */
	//ret = bq25890_field_write(bq, F_WD, 0);
	ret = bq25890_field_write(bq, F_WD_TIMEOUT, 0);//寄存器未变
	
	if (ret < 0)
		return ret;

	/* 初始化电流/电压和其他参数。 initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = bq25890_field_write(bq, init_data[i].id, init_data[i].value);
		if (ret < 0)
			return ret;
	}

	/* Configure ADC for continuous conversions. This does not enable it. */
	//ret = bq25890_field_write(bq, F_CONV_RATE, 1);
	ret = bq25890_field_write(bq, F_ADC_RATE, 1);
	
	if (ret < 0)
		return ret;

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	return 0;
}

static enum power_supply_property bq25890_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static char *bq25890_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc bq25890_power_supply_desc = {
	.name = DEVICE_NAME,
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bq25890_power_supply_props,
	.num_properties = ARRAY_SIZE(bq25890_power_supply_props),
	.get_property = bq25890_power_supply_get_property,
};
//注册power_supply就是注册设备么 ？
static int bq25890_power_supply_init(struct bq25890_device *bq)
{
	struct power_supply_config psy_cfg = { .drv_data = bq, };

	psy_cfg.supplied_to     = bq25890_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bq25890_charger_supplied_to);

	bq->ps_charger = power_supply_register(bq->dev, &bq25890_power_supply_desc, &psy_cfg);

	return PTR_ERR_OR_ZERO(bq->ps_charger);
}

static void bq25890_usb_work(struct work_struct *data)
{
	int ret;
	struct bq25890_device *bq = container_of(data, struct bq25890_device, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		/* Enable boost mode */
		//ret = bq25890_field_write(bq, F_OTG_CFG, 1);
		ret = bq25890_field_write(bq, F_BOOST_EN, 1);
		
		if (ret < 0)
			goto error;
		break;

	case USB_EVENT_NONE:
		/* Disable boost mode */
		//ret = bq25890_field_write(bq, F_OTG_CFG, 0);
		ret = bq25890_field_write(bq, F_BOOST_EN, 0);
		if (ret < 0)
			goto error;

		power_supply_changed(bq->ps_charger);
		break;
	}

	return;

error:
	dev_err(bq->dev, "[OBEI_BC]Error switching to boost/charger mode.\n");
}

static int bq25890_usb_notifier(struct notifier_block *nb, unsigned long val, void *priv)
{
	struct bq25890_device *bq = container_of(nb, struct bq25890_device, usb_nb);

	bq->usb_event = val;
	queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}

static int bq25890_irq_probe(struct bq25890_device *bq)
{
	struct gpio_desc *irq;

	irq = devm_gpiod_get(bq->dev, BQ25890_IRQ_PIN, GPIOD_IN);
	if (IS_ERR(irq)) {
		dev_err(bq->dev, "[OBEI_BC]Could not probe irq pin.\n");
		return PTR_ERR(irq);
	}

	return gpiod_to_irq(irq);
}

static int bq25890_fw_read_u32_props(struct bq25890_device *bq)
{
	int ret;
	u32 property;
	int i;
	struct bq25890_init_data *init = &bq->init_data;
	struct {
		char *name;
		bool optional;
		enum bq25890_table_ids tbl_id;
		u8 *conv_data; /* holds converted value from given property */
	} props[] = {
		/* required properties */
		{"ti,charge-current",             false, TBL_ICHG,    &init->ichg},
		{"ti,battery-regulation-voltage", false, TBL_VREG,    &init->vreg},
		{"ti,termination-current",        false, TBL_ITERM,   &init->iterm},
		{"ti,precharge-current",          false, TBL_ITERM,   &init->iprechg},
		{"ti,minimum-sys-voltage",        false, TBL_SYSVMIN, &init->sysvmin},
		{"ti,boost-voltage",              false, TBL_BOOSTV,  &init->boostv},
		{"ti,boost-max-current",          false, TBL_BOOSTI,  &init->boosti},

		/* optional properties */
		{"ti,thermal-regulation-threshold", true, TBL_TREG, &init->treg}
	};
	/*
	bq24150a: bq24150a@6b {
		compatible = "ti,bq24150a";
		reg = <0x6b>;
		ti,current-limit = <100>;
		ti,weak-battery-voltage = <3400>;
		ti,battery-regulation-voltage = <4200>;
		ti,charge-current = <650>;
		ti,termination-current = <100>;
		ti,resistor-sense = <68>;
		ti,usb-charger-detection = <&isp1707>;
	};
	*/
	*props[TBL_ICHG].conv_data = 650;
	/* initialize data for optional properties */
	init->treg = 3; /* 120 degrees Celsius */

	/*
	for (i = 0; i < ARRAY_SIZE(props); i++) {
		//从设备树中读取上面的属性值
		ret = device_property_read_u32(bq->dev, props[i].name, &property);
		if (ret < 0) {
			if (props[i].optional)
				continue;
			//读取失败直接返回错误
			return ret;
		}

		//读取成功.给conv_data变量赋值....
		*props[i].conv_data = bq25890_find_idx(property, props[i].tbl_id);
	}
	*/
	return 0;
}

static int bq25890_fw_probe(struct bq25890_device *bq)
{
	int ret;
	struct bq25890_init_data *init = &bq->init_data;

	ret = bq25890_fw_read_u32_props(bq);
	if (ret < 0)
		return ret;

	init->ilim_en = device_property_read_bool(bq->dev, "ti,use-ilim-pin");
	init->boostf = device_property_read_bool(bq->dev, "ti,boost-low-freq");

	return 0;
}

static int bq25890_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct bq25890_device *bq;
	int ret;
	int i;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "[OBEI_BC]No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);

	bq->rmap = devm_regmap_init_i2c(client, &bq25890_regmap_config);
	if (IS_ERR(bq->rmap)) {
		dev_err(dev, "[OBEI_BC]failed to allocate register map\n");
		return PTR_ERR(bq->rmap);
	}

	for (i = 0; i < ARRAY_SIZE(bq25890_reg_fields); i++) {
		const struct reg_field *reg_fields = bq25890_reg_fields;

		bq->rmap_fields[i] = devm_regmap_field_alloc(dev, bq->rmap, reg_fields[i]);
		if (IS_ERR(bq->rmap_fields[i])) {
			dev_err(dev, "[OBEI_BC]cannot allocate regmap field\n");
			return PTR_ERR(bq->rmap_fields[i]);
		}
	}

	i2c_set_clientdata(client, bq);

	//bq->chip_id = bq25890_field_read(bq, F_PN);
	bq->chip_id = bq25890_field_read(bq, F_DEVICE_ID);//读取设备编号 0x66

	if (bq->chip_id < 0) {
		dev_err(dev, "[OBEI_BC]Cannot read chip ID.\n");
		return bq->chip_id;
	}
	
	//if (bq->chip_id != BQ25890_ID) {
	if (bq->chip_id != SYBA76_BC_ID) {
		dev_err(dev, "[OBEI_BC]Chip with ID=%d, not supported!\n", bq->chip_id);
		return -ENODEV;
	}

	if (!dev->platform_data) {
		ret = bq25890_fw_probe(bq);
		if (ret < 0) {
			dev_err(dev, "[OBEI_BC]Cannot read device properties.\n");
			return ret;
		}
	} else {
		return -ENODEV;
	}

	ret = bq25890_hw_init(bq);
	if (ret < 0) {
		dev_err(dev, "[OBEI_BC]Cannot initialize the chip.\n");
		return ret;
	}

	if (client->irq <= 0)
		client->irq = bq25890_irq_probe(bq);

	if (client->irq < 0) {
		dev_err(dev, "[OBEI_BC]No irq resource found.\n");
		return client->irq;
	}

	/* OTG reporting */
	bq->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(bq->usb_phy)) {
		INIT_WORK(&bq->usb_work, bq25890_usb_work);
		bq->usb_nb.notifier_call = bq25890_usb_notifier;
		usb_register_notifier(bq->usb_phy, &bq->usb_nb);
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL, bq25890_irq_handler_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, BQ25890_IRQ_PIN, bq);
	if (ret)
		goto irq_fail;

	ret = bq25890_power_supply_init(bq);
	if (ret < 0) {
		dev_err(dev, "[OBEI_BC]Failed to register power supply\n");
		goto irq_fail;
	}

	return 0;

irq_fail:
	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);

	return ret;
}

static int bq25890_remove(struct i2c_client *client)
{
	struct bq25890_device *bq = i2c_get_clientdata(client);

	power_supply_unregister(bq->ps_charger);

	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);

	/* reset all registers to default values */
	bq25890_chip_reset(bq);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bq25890_suspend(struct device *dev)
{
	struct bq25890_device *bq = dev_get_drvdata(dev);

	/*
	 * If charger is removed, while in suspend, make sure ADC is diabled
	 * since it consumes slightly more power.
	 */
	//return bq25890_field_write(bq, F_CONV_START, 0);
	return bq25890_field_write(bq, F_ADC_EN, 0);
	
}

static int bq25890_resume(struct device *dev)
{
	int ret;
	struct bq25890_state state;
	struct bq25890_device *bq = dev_get_drvdata(dev);

	ret = bq25890_get_chip_state(bq, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	/* Re-enable ADC only if charger is plugged in. */
	if (state.online) {
		//ret = bq25890_field_write(bq, F_CONV_START, 1);
		ret = bq25890_field_write(bq, F_ADC_EN, 1);
		if (ret < 0)
			return ret;
	}

	/* signal userspace, maybe state changed while suspended */
	power_supply_changed(bq->ps_charger);

	return 0;
}
#endif

static const struct dev_pm_ops bq25890_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(bq25890_suspend, bq25890_resume)
};

static const struct i2c_device_id bq25890_i2c_ids[] = {
	{ DRIVER_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq25890_i2c_ids);

static const struct of_device_id bq25890_of_match[] = {
	{ .compatible = DRIVER_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, bq25890_of_match);

static const struct acpi_device_id bq25890_acpi_match[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, bq25890_acpi_match);

static struct i2c_driver bq25890_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(bq25890_of_match),
		.acpi_match_table = ACPI_PTR(bq25890_acpi_match),
		.pm = &bq25890_pm,
	},
	.probe  = bq25890_probe,
	.remove = bq25890_remove,
	.id_table = bq25890_i2c_ids,
};
module_i2c_driver(bq25890_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@intel.com>");
MODULE_DESCRIPTION("bq25890 charger driver");
MODULE_LICENSE("GPL");
