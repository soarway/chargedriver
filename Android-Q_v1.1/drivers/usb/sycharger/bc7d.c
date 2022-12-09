/**
 * @file bc7d.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/phy.h>
#include <linux/acpi.h>
#include <linux/of.h>


#include "symaster.h"
#include "bc7d.h"
/*
000 – Not charging
001 – Trickle charging
010 – Pre-charging
011 – Fast charging (CC mode)
100 – Fast charging (CV mode)
101 – Charge Termination
*/
enum bc7d_status {
	STATUS_NOT_CHARGING,
	STATUS_TRICKLE_CHARGING,
	STATUS_PRE_CHARGING,
	STATUS_FAST_CHARGING_CC,
	STATUS_FAST_CHARGING_CV,
	STATUS_TERMINATION_DONE,
};

enum BC7D_fields {
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



//只读寄存器范围
static const struct regmap_range BC7D_readonly_reg_ranges[] = {
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
static const struct regmap_access_table BC7D_writeable_regs = {
	.no_ranges = BC7D_readonly_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(BC7D_readonly_reg_ranges),
};
//volatile寄存器范围
static const struct regmap_range BC7D_volatile_reg_ranges[] = {
	regmap_reg_range(0x3C, 0x3C),
	regmap_reg_range(0x35, 0x35),
	regmap_reg_range(0x42, 0x42),
	regmap_reg_range(0x58, 0x58),
	regmap_reg_range(0x94, 0x94),
	
};
//volatile寄存器
static const struct regmap_access_table BC7D_volatile_regs = {
	.yes_ranges = BC7D_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(BC7D_volatile_reg_ranges),
};

static const struct regmap_config BC7D_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x9D,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &BC7D_writeable_regs//,
	//.volatile_table = &bq25890_volatile_regs,
};


static const struct reg_field BC7D_reg_fields[] = {
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
struct bc7d_state {
	u8 online;
	u8 chrg_status;
	u8 vsys_status;
	u8 boost_fault;
	u8 bat_fault;
};
/* initial field values, converted to register values */
struct bc7d_init_data {
	u8 ichg;	/* charge current		*/
	u8 vreg;	/* regulation voltage		*/
	u8 iterm;	/* termination current		*/
	u8 iprechg;	/* precharge current		*/
	u8 sysvmin;	/* minimum system voltage limit */
	u8 boostv;	/* boost regulation voltage	*/
	u8 boosti;	/* boost current limit		*/
	u8 boostf;	/* boost frequency		*/
	//u8 ilim_en;	/* enable ILIM pin		*/
	u8 treg;	/* thermal regulation threshold */
};
struct bc7d_chip {
	struct power_supply *ps_charger;
	struct power_supply_desc	 charger_desc;
	struct i2c_client			*client;
	struct device               *dev;
	struct mutex			lock;
	struct gpio_desc		*status_gpio;
	struct delayed_work		poll;
	u32						poll_interval;
	bool					charging;
	int                     chip_id;

	struct symaster_device  *sydev;
	struct regmap 			*rmap;
	struct regmap_field *rmap_fields[F_MAX_FIELDS];
	struct bc7d_init_data init_data;
	struct bc7d_state state;

	struct notifier_block   bc7d_nb;//3个子模块之间的事件通信
	struct usb_phy *usb_phy;
	struct notifier_block usb_nb;//USB的事件通信
	struct work_struct usb_work;
	unsigned long usb_event;
};

static DEFINE_MUTEX(bc7d_i2c_lock);

//消息处理函数
static int bc7d_event_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	struct symaster_device* sydev;
	struct bc7d_chip * chip = container_of(nb, struct bc7d_chip, bc7d_nb);

	//struct tcp_notify *tcp_noti = data;

	switch (event) {
	case SY_NOTIFY_BC7D_ENABLE_CHARGER:
		pr_info("[OBEI][bc7d]: event call enable charger\n");
		break;
	case SY_NOTIFY_BC7D_DISABLE_CHARGER:
		pr_info("[OBEI][bc7d]: event call disable charger\n");
		break;
	case SY_NOTIFY_BC7D_CHECK_DCP:
		if ( check_DCP_condition(chip) )
		{
			pr_info("[OBEI][bc7d]:event DCP is true\n");
		}
		else {
			pr_info("[OBEI][bc7d]:event DCP is false\n");
		}
		break;
	default:
		pr_info("[OBEI][bc7d]: unknown event = %d\n", event);
		break;
	};

	//--------------test-------------------
	//测试消息从子模块发送到主模块
	sydev = sy_dev_get_by_name(MASTER_DEVICE_NAME);
	if (sydev == NULL) {
		pr_info("[OBEI][bc7d]:event notify: get master dev fail\n");
		return NOTIFY_OK;
	}
	else {
		pr_info("[OBEI][bc7d]:event notify: get master dev success\n");
	}
	srcu_notifier_call_chain(&sydev->master_event, SY_NOTIFY_MASTER_ENABLE_CHARGER, chip);
	//------------- end--------------------
	return NOTIFY_OK;
}

//-------------------------------------------
//传统写法的读寄存器
static int bc7d_read_byte(struct bc7d_chip *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bc7d_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bc7d_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bc7d_i2c_lock);

	return 0;
}
//传统写法的写寄存器
static int bc7d_write_byte(struct bc7d_chip *bq, u8 reg, u8 data)
{
	int ret;
	mutex_lock(&bc7d_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bc7d_i2c_lock);
	return ret;
}
//传统写法的更新寄存器
static int bc7d_update_bits(struct bc7d_chip *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bc7d_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bc7d_write_byte(bq, reg, tmp);
}

//--------------------------------------------

//读指定寄存器的值
static int bc7d_field_read(struct bc7d_chip *bq, enum BC7D_fields field_id)
{
	int ret;
	int val;

	ret = regmap_field_read(bq->rmap_fields[field_id], &val);
	if (ret < 0)
		return ret;

	return val;
}
//写指定寄存器的值
static int bc7d_field_write(struct bc7d_chip *bq, enum BC7D_fields field_id, u8 val)
{
	return regmap_field_write(bq->rmap_fields[field_id], val);
}
//重置寄存器的默认值
/*
Register Reset:
0–Keep current register setting (default)
1–Reset to default register value and reset safety timer
*/
static int bc7d_chip_reset(struct bc7d_chip *bq)
{
	int ret;
	int rst_check_counter = 10;

	ret = bc7d_field_write(bq, F_REG_RST, 1);
	if (ret < 0)
	{
		pr_info("[OBEI][bc7d]field write fail");
		return ret;
	}	

	do {
		
		ret = bc7d_field_read(bq, F_REG_RST);
		if (ret < 0)
		{
			pr_info("[OBEI][bc7d]field read fail");
			return ret;
		}

		usleep_range(5, 10);
		//如果失败，重试10次
	} while (ret == 1 && --rst_check_counter);

	if (!rst_check_counter)
	{
		pr_info("[OBEI][bc7d]rst_check_counter fail");
		//return -ETIMEDOUT;

		bc7d_update_bits(bq, 0x08, 0x80, 1);
		usleep_range(5, 10);
		ret = bc7d_field_read(bq, F_REG_RST);
		pr_info("[OBEI][bc7d]rst_check_counter ret = %d", ret);
	}	
 
	return 0;
}

/*
//是否满足DCP特性
000 – No Input 
001 – USB Host SDP
010 – USB CDP (1.5A)
011 – USB DCP (3.25A)
100 – HVDCP 
101 – Unknown Adapter (500mA)
110 – Non-Standard Adapter (1A/2A/2.1A/2.4A/3A)
111 – OTG
*/
//返回true代表满足DCP条件
bool check_DCP_condition (struct bc7d_chip *bq)
{
	int val = 0;
	int ret;

	ret = bc7d_field_read(bq, F_INPUT_DET_DONE_FLAG);
	if (ret == 1) {
		ret = bc7d_field_read(bq, F_VBUS_STAT);
		if (ret == BC7D_VBUS_USB_DCP)
		{
			return true;
		}
	}

	return false;
}
static int bc7d_get_chip_state(struct bc7d_chip *bq, struct bc7d_state *state)
{
	int i, ret;

	struct {
		enum BC7D_fields id;
		u8 *data;
	} state_fields[] = {
		{F_CHG_STAT,	&state->chrg_status},
		{F_VBUS_STAT,		&state->online},//这个没有
		{F_VSYSMIN_REG_STAT,	&state->vsys_status},
		
		{F_BOOST_OK_STAT, &state->boost_fault},
		{F_BATFET_STAT,	&state->bat_fault}//,
		//{F_CHG_FAULT,	&state->chrg_fault}//这个没有
	};

	for (i = 0; i < ARRAY_SIZE(state_fields); i++) {
		ret = bc7d_field_read(bq, state_fields[i].id);
		if (ret < 0)
			return ret;

		*state_fields[i].data = ret;
	}

	//dev_dbg(bq->dev, "S:CHG/PG/VSYS=%d/%d/%d, F:CHG/BOOST/BAT=%d/%d/%d\n", state->chrg_status, state->online, state->vsys_status, state->chrg_fault, state->boost_fault, state->bat_fault);

	return 0;
}

//硬件初始化
static int bc7d_hw_init(struct bc7d_chip *bq)
{
	int ret;
	int i;
	struct bc7d_state state;

	const struct {
		enum BC7D_fields id;
		u32 value;
	} init_data[] = {
		{F_ICHG_CC,	 bq->init_data.ichg},
		{F_VBAT_REG, bq->init_data.vreg},
		{F_ITERM,	 bq->init_data.iterm},
		{F_IPRECHG,	 bq->init_data.iprechg},
		{F_SYS_MIN,	 bq->init_data.sysvmin},
		{F_VBOOST,	 bq->init_data.boostv},
		{F_IBOOST_LIM,	 bq->init_data.boosti},
		{F_BOOST_FREQ,	 bq->init_data.boostf},
		//{F_EN_ILIM,	 bq->init_data.ilim_en},//这个没有
		{F_TDIE_REG,	 bq->init_data.treg}
	};

	ret = bc7d_chip_reset(bq);
	if (ret < 0)
	{
		pr_info("[OBEI][bc7d]chip reset fail!");
		return ret;
	}

	/* 
	disable watchdog 
	I2C Watchdog Timer Setting:
	000 – Disable watchdog timer
	001 – 0.5s
	010 – 1s
	011 – 2s
	100 – 20s
	101 – 40s (default)
	110 – 80s
	111 – 160s
	*/
	ret = bc7d_field_write(bq, F_WD_TIMEOUT, 0);
	if (ret < 0)
	{
		pr_info("[OBEI][bc7d]field write Watchdog fail!");
		return ret;
	}
		

	/* initialize currents/voltages and other parameters */
	for (i = 0; i < ARRAY_SIZE(init_data); i++) {
		ret = bc7d_field_write(bq, init_data[i].id, init_data[i].value);
		if (ret < 0)
		{
			pr_info("[OBEI][bc7d]field write init data fail!");
			return ret;
		}	
	}

	/* Configure ADC for continuous conversions. This does not enable it. */
	ret = bc7d_field_write(bq, F_ADC_RATE, 0);//貌似要写0值才对
	if (ret < 0)
	{
		pr_info("[OBEI][bc7d]field write init ADC rate fail!");
		return ret;
	}	

	ret = bc7d_get_chip_state(bq, &state);
	if (ret < 0)
	{
		pr_info("[OBEI][bc7d]get chip state fail!");
		return ret;
	}	

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	pr_info("[OBEI][bc7d]hw init success!");
	return 0;
}

static int bc7d_irq_probe(struct bc7d_chip *bq, const char* name)
{
	struct gpio_desc *irq;

	irq = devm_gpiod_get(bq->dev, name, GPIOD_IN);
	//对应设备树中的：ba70irq-gpios = <&tlmm 57 0x00>; 

	if (IS_ERR(irq)) {
		pr_err("[OBEI][bc7d]Could not probe irq pin.\n");
		return PTR_ERR(irq);
	}

	return gpiod_to_irq(irq);
}


static void bc7d_usb_work(struct work_struct *data)
{
	int ret;
	struct bc7d_chip *bq = container_of(data, struct bc7d_chip, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		/* Enable boost mode */
		pr_err("[OBEI][bc7d]USB EVENT ID\n");
		break;

	case USB_EVENT_NONE:
		/* Disable boost mode */
		pr_err("[OBEI][bc7d]USB EVENT NONE\n");
		power_supply_changed(bq->ps_charger);
		break;
	}

	return;

error:
	pr_err("[OBEI][bc7d]Error switching to boost/charger mode.\n");
}

static int bc7d_usb_notifier(struct notifier_block *nb, unsigned long val, void *priv)
{
	struct bc7d_chip *bq = container_of(nb, struct bc7d_chip, usb_nb);

	bq->usb_event = val;
	queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}

static bool bc7d_state_changed(struct bc7d_chip *bq, struct bc7d_state *new_state)
{
	struct bc7d_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return (old_state.chrg_status != new_state->chrg_status ||
		//old_state.chrg_fault != new_state->chrg_fault	||
		old_state.online != new_state->online		||
		old_state.bat_fault != new_state->bat_fault	||
		old_state.boost_fault != new_state->boost_fault ||
		old_state.vsys_status != new_state->vsys_status);
}

static void bc7d_handle_state_change(struct bc7d_chip *bq, struct bc7d_state *new_state)
{
	int ret;
	struct bc7d_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	if (!new_state->online) {			     /* power removed */
		/* disable ADC */
		ret = bc7d_field_write(bq, F_ADC_EN, 0);
		if (ret < 0)
			goto error;
	} else if (!old_state.online) {			    /* power inserted */
		/* enable ADC, to have control of charge current/voltage */
		ret = bc7d_field_write(bq, F_ADC_EN, 1);
		if (ret < 0)
			goto error;
	}

	return;

error:
	dev_err(bq->dev, "Error communicating with the chip.\n");
}
static irqreturn_t bc7d_irq_handler_thread(int irq, void *private)
{
	struct bc7d_chip *bq = private;
	int ret;
	struct bc7d_state state;

	ret = bc7d_get_chip_state(bq, &state);
	if (ret < 0)
		goto handled;

	if (!bc7d_state_changed(bq, &state))
		goto handled;

	bc7d_handle_state_change(bq, &state);

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->ps_charger);

	pr_info("[OBEI][bc7d]irq handler thread\n");
handled:
	return IRQ_HANDLED;
}


static int bc7d_power_supply_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	int ret;
	struct bc7d_chip *bq = power_supply_get_drvdata(psy);
	struct bc7d_state state;

	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.online)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state.chrg_status == STATUS_NOT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (state.chrg_status == STATUS_PRE_CHARGING || state.chrg_status == STATUS_FAST_CHARGING_CC || state.chrg_status == STATUS_FAST_CHARGING_CV)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (state.chrg_status == STATUS_TERMINATION_DONE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

		break;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BC7D_MANUFACTURER;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}


static enum power_supply_property  bc7d_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	//POWER_SUPPLY_PROP_HEALTH,
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	//POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	//POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static char *bc7d_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc bc7d_power_supply_desc = {
	.name = "bc7d-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bc7d_power_supply_props,
	.num_properties = ARRAY_SIZE(bc7d_power_supply_props),
	.get_property = bc7d_power_supply_get_property,
};

static int bc7d_power_supply_init(struct bc7d_chip *bq)
{
	struct power_supply_config psy_cfg = { .drv_data = bq, };

	psy_cfg.supplied_to = bc7d_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(bc7d_charger_supplied_to);

	bq->ps_charger = power_supply_register(bq->dev, &bc7d_power_supply_desc, &psy_cfg);

	return PTR_ERR_OR_ZERO(bq->ps_charger);
}

static int bc7d_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret, i;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct bc7d_chip *charger;
	struct symaster_device* sydev;
	//struct power_supply_desc *supply_desc;

	//struct power_supply_config psy_cfg = {};
	char *name;

	printk("[OBEI][bc7d]call charger probe.\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("[OBEI][bc7d]No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}



	//分配自定义结构体的内存
	charger = devm_kzalloc(dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	
	charger->client = client;
	charger->dev = dev;

	mutex_init(&charger->lock);

	charger->rmap = devm_regmap_init_i2c(client, &BC7D_regmap_config);
	if (IS_ERR(charger->rmap)) {
		printk("[OBEI][bc7d]failed to allocate register map\n");
		return PTR_ERR(charger->rmap);
	}

	for (i = 0; i < ARRAY_SIZE(BC7D_reg_fields); i++) {
		const struct reg_field *reg_fields = BC7D_reg_fields;

		charger->rmap_fields[i] = devm_regmap_field_alloc(dev, charger->rmap, reg_fields[i]);
		if (IS_ERR(charger->rmap_fields[i])) {
			printk("[OBEI][bc7d]cannot allocate regmap field\n");
			return PTR_ERR(charger->rmap_fields[i]);
		}
	}

	i2c_set_clientdata(client, charger);

	charger->chip_id = bc7d_field_read(charger, F_DEVICE_ID);//读取设备编号 0x66
	if (charger->chip_id < 0) {
		printk("[OBEI][bc7d]read DEVICE ID fail\n");
		return -EINVAL;
	}
	
	pr_info("[OBEI][bc7d]read  DEVICE ID = %x\n", charger->chip_id);
	if (charger->chip_id != BC7D_CHIP_ID) {
		printk("[OBEI][bc7d]Chip ID is not match!\n");
		return -ENODEV;
	}

	//设备树变量初始化


	//硬件初始化
	ret = bc7d_hw_init(charger);
	if (ret < 0) {
		pr_err("[OBEI][bc7d]Cannot initialize the chip.\n");
		return ret;
	}

	//注册事件回调函数
	sydev = sy_dev_get_by_name(MASTER_DEVICE_NAME);
	if (sydev != NULL) {
		charger->sydev = sydev;

		charger->bc7d_nb.notifier_call = bc7d_event_notifer_call;
		ret = sy_register_notifier(charger->sydev, &charger->bc7d_nb);
		if (ret < 0) {
			printk("[OBEI][bc7d]register  notifer fail\n");
			return -EINVAL;
		}
	}
	else {
		printk("[OBEI][bc7d]sydev get by name fail!\n");
	}


	if (client->irq <= 0)
	{
		pr_info("[OBEI][bc7d]client -> irq =%d\n", client->irq);
		client->irq = bc7d_irq_probe(charger, BC7D_IRQ_PIN);
		
	}	

	pr_info("[OBEI][bc7d]irqn = %d\n", client->irq);

	if (client->irq < 0) {
		pr_err("[OBEI][bc7d]No irq resource found.\n");
		return client->irq;
	}
	

	/* OTG reporting */
	charger->usb_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(charger->usb_phy)) {
		INIT_WORK(&charger->usb_work, bc7d_usb_work);
		charger->usb_nb.notifier_call = bc7d_usb_notifier;
		usb_register_notifier(charger->usb_phy, &charger->usb_nb);
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL, bc7d_irq_handler_thread, 
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, BC7D_IRQ_PIN, charger);
	if (ret)
	{
		pr_err("[OBEI][bc7d]request threaded irq fail\n");
		goto irq_fail;
	}	

	ret = bc7d_power_supply_init(charger);
	if (ret < 0) {
		pr_err("[OBEI][bc7d]Failed to register power supply\n");
		goto irq_fail;
	}

	pr_info("[OBEI][bc7d]probe success!\n");
	return 0;
irq_fail:
	if (!IS_ERR_OR_NULL(charger->usb_phy))
		usb_unregister_notifier(charger->usb_phy, &charger->usb_nb);

	return ret;
}

static int bc7d_charger_remove(struct i2c_client *client)
{
	struct bc7d_chip *bq = i2c_get_clientdata(client);

	power_supply_unregister(bq->ps_charger);

	if (!IS_ERR_OR_NULL(bq->usb_phy))
		usb_unregister_notifier(bq->usb_phy, &bq->usb_nb);

	/* reset all registers to default values */
	bc7d_chip_reset(bq);

	printk("[OBEI][bc7d]bc7d charger remove.\n");
	return 0;
}

//--------------------------------------------BC7D-------------------------------------------


static struct of_device_id    bc7d_charger_match_table[] = {
	{.compatible = SY_BC7D,},
	{},
};

static const struct i2c_device_id    bc7d_charger_id[] = {
	{ SY_BC7D, 1 },
	{},
};

MODULE_DEVICE_TABLE(i2c,  bc7d_charger2_id);


static struct i2c_driver   bc7d_charger_driver = {
	.driver		= {
		.name	= SY_BC7D,
		.of_match_table = bc7d_charger_match_table,

	},

	.id_table	= bc7d_charger_id,

	.probe	= bc7d_charger_probe,
	.remove = bc7d_charger_remove,
	//.shutdown   = bc7d_charger_shutdown,
};



static struct i2c_board_info __initdata   i2c_bc7d_charger[] = {
	{
		I2C_BOARD_INFO(SY_BC7D, 0x7D),
	},
};



static int __init bc7d_charger_init(void)
{
	int ret;
	ret = i2c_register_board_info(1, i2c_bc7d_charger,   ARRAY_SIZE(i2c_bc7d_charger));
	if (ret) {
		printk("[OBEI][bc7d]register board info fail.\n");
	}


	//添加bc1.2驱动
	if (i2c_add_driver(&bc7d_charger_driver))
		printk("[OBEI][bc7d]failed to register bc7d_driver.\n");
	else
		printk("[OBEI][bc7d]driver register successfully!\n");



	return 0;
}

static void __exit bc7d_charger_exit(void)
{
	i2c_del_driver(&bc7d_charger_driver);

}

module_init(bc7d_charger_init);
module_exit(bc7d_charger_exit);

MODULE_DESCRIPTION("SY BC7D  Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");



