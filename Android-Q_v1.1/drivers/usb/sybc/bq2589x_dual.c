/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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
#include "bq2589x_reg.h"

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no {
	BQ25890 = 0x03,
	BQ25892 = 0x00,
	BQ25895 = 0x07,
};


#define BQ2589X_STATUS_PLUGIN		0x0001
#define BQ2589X_STATUS_PG			0x0002
#define BQ2589X_STATUS_FAULT		0x0008

#define BQ2589X_STATUS_EXIST			0x0100
#define BQ2589X_STATUS_CHARGE_ENABLE 	0x0200

struct bq2589x_config {
	bool	enable_auto_dpdm;
/*	bool	enable_12v;*/

	int		charge_voltage;
	int		charge_current;

	bool	enable_term;
	int		term_current;

	bool 	enable_ico;
	bool	enable_absolute_vindpm;
};


struct bq2589x {
	struct device *dev;
	struct i2c_client *client;

	enum   bq2589x_part_no part_no;
	int    revision;

	unsigned int    status;
	int		vbus_type;

	bool	enabled;

	bool    interrupt;


	int     vbus_volt;
	int     vbat_volt;

	int     rsoc;
	struct	bq2589x_config	cfg;
	struct work_struct irq_work;
	struct work_struct adapter_in_work;
	struct work_struct adapter_out_work;
	struct delayed_work monitor_work;
	struct delayed_work ico_work;
	struct delayed_work pe_volt_tune_work;
	struct delayed_work check_pe_tuneup_work;
	struct delayed_work charger2_enable_work;



	struct power_supply *usb;
	struct power_supply *wall;
	struct power_supply *batt_psy;


};

struct pe_ctrl {
	bool enable;
	bool tune_up_volt;
	bool tune_down_volt;
	bool tune_done;
	bool tune_fail;
	int  tune_count;
	int  target_volt;
	int	 high_volt_level;/* vbus volt > this threshold means tune up successfully */
	int  low_volt_level; /* vbus volt < this threshold means tune down successfully */
	int  vbat_min_volt;  /* to tune up voltage only when vbat > this threshold */
};

static struct bq2589x *g_bq1;
static struct bq2589x *g_bq2;
static struct pe_ctrl pe;


static DEFINE_MUTEX(bq2589x_i2c_lock);

//从寄存器读取1个字节
static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2589x_i2c_lock);

	return 0;
}
//往寄存器写入1个字节
static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;
	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq2589x_i2c_lock);
	return ret;
}

//读取然后写入
static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2589x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(bq, reg, tmp);
}

//通过读取寄存器0x0B的值，获取vbus的类型
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}

//让设备支持otg功能
static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

}
//取消设备的otg功能
static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);
//设置otg的电压
static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt)
{
	u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;


	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);
//设置otg的电流
static int bq2589x_set_otg_current(struct bq2589x *bq, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 700)
		temp = BQ2589X_BOOST_LIM_700MA;
	else if (curr == 1100)
		temp = BQ2589X_BOOST_LIM_1100MA;
	else if (curr == 1600)
		temp = BQ2589X_BOOST_LIM_1600MA;
	else if (curr == 1800)
		temp = BQ2589X_BOOST_LIM_1800MA;
	else if (curr == 2100)
		temp = BQ2589X_BOOST_LIM_2100MA;
	else if (curr == 2400)
		temp = BQ2589X_BOOST_LIM_2400MA;
	else
		temp = BQ2589X_BOOST_LIM_1300MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);
//使能充电器
static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status |= BQ2589X_STATUS_CHARGE_ENABLE;
	return ret;
}
//消能充电器
static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status &= ~BQ2589X_STATUS_CHARGE_ENABLE;
	return ret;
}


/* interfaces that can be called by other module
寄存器BQ2589X_REG_02的第7个比特位
ADC Conversion Start Control
0 – ADC conversion not active (default).
1 – Start ADC Conversion
This bit is read-only when CONV_RATE = 1. The bit stays high during
ADC conversion and during input source detection

 */
//ADC模式开启
int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);
//ADC模式关闭
int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);

//读取电池的电压值
int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);

//读取系统电压值
int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);
//读取VBUS电压值
int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);
//读取温度值
int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
	uint8_t val;
	int temp;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return ret;
	} else {
		temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		return temp;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);
//读取电感器电流值
int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else {
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);
//设置当前电流值
int bq2589x_set_chargecurrent(struct bq2589x *bq, int curr)
{

	u8 ichg;

	ichg = (curr - BQ2589X_ICHG_BASE) / BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);
//结束当前电流限制
/*
Termination Current Limit
Offset: 64mA
Range: 64mA – 1024mA
Default: 256mA (0011)
*/
int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);

//设置预充电电流
int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);
//设置预充电电压
int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VREG_BASE) / BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);

//设置输入电压的上限
int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;
	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);
//设置输入电流的上限
int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{

	u8 val;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

//设置offset值
int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	val = (offset - BQ2589X_VINDPMOS_BASE) / BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

//开始充电
void bq2589x_start_charging(struct bq2589x *bq)
{
	bq2589x_enable_charger(bq);
}
EXPORT_SYMBOL_GPL(bq2589x_start_charging);
//停止充电
void bq2589x_stop_charging(struct bq2589x *bq)
{
	bq2589x_disable_charger(bq);
}
EXPORT_SYMBOL_GPL(bq2589x_stop_charging);
//获取充电状态
int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s Failed to read register 0x0b:%d\n", __func__, ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);
//启用和禁用otg功能
void bq2589x_set_otg(struct bq2589x *bq, int enable)
{
	int ret;

	if (enable) {
		ret = bq2589x_enable_otg(bq);
		if (ret < 0) {
			dev_err(bq->dev, "%s:Failed to enable otg-%d\n", __func__, ret);
			return;
		}
	} else {
		ret = bq2589x_disable_otg(bq);
		if (ret < 0)
			dev_err(bq->dev, "%s:Failed to disable otg-%d\n", __func__, ret);
	}
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);
//设置看门狗定时器的超时时间
int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, (u8)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);
//取消看门狗定时器
int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);
//重启看门狗定时器
int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);
//强制使用dpdm模式
int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if (ret)
		return ret;

	msleep(10);
	return 0;

}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);
/*
函数功能：寄存器0x14的第七位设置位1
0 – Keep current register setting (default)  设为0不会有动作
1 – Reset to default register value and reset safety timer  设为1会安全的重启寄存器
Note:
Reset to 0 after register reset is completed   重启完成后，寄存器的值会恢复为0
*/
int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

/*
Force BATFET off to enable ship mode
0 – Allow BATFET turn on (default)
1 – Force BATFET off
*/
int bq2589x_enter_ship_mode(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);
/*
第7位为 EN_HIZ 模式
Enable HIZ Mode
0 – Disable (default)
1 – Enable
*/
int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);
//读取HIZ模式，从寄存器读取1个字节，然后取第7位的值 &0x80 就是取一个字节的最高位
int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);
/*
取第6位的值 &0x40
Enable ILIM Pin
0 – Disable
1 – Enable (default: Enable ILIM pin (1))
*/
int bq2589x_enable_ilim_pin(struct bq2589x *bq)
{
	u8 val = BQ2589X_ENILIM_ENABLE << BQ2589X_ENILIM_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ilim_pin);
/*
Enable ILIM Pin
0 – Disable
1 – Enable (default: Enable ILIM pin (1))
*/
int bq2589x_disable_ilim_pin(struct bq2589x *bq)
{
	u8 val = BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_ilim_pin);

/*
EN_PUMPX 寄存器的最高位
Current pulse control Enable
0 - Disable Current pulse control (default)
1 - Enable Current pulse control (PUMPX_UP and PUMPX_DN)
*/
int bq2589x_pumpx_enable(struct bq2589x *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);
/*
设置PUMPX_UP
Current pulse control voltage up enable
0 – Disable (default)
1 – Enable
Note:
This bit is can only be set when EN_PUMPX bit is set and returns to 0
after current pulse control sequence is completed
只有当EN_PUMPX位被设置 ，电流脉冲控制序列完成  EN_PUMPX返回0时，该位才能被重新设置
*/
int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);
//EN_PUMPX位是否设置完成，判断比特位是否为0， 0代表结束
int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);
/*
Current pulse control voltage down enable
0 – Disable (default)
1 – Enable
Note:
This bit is can only be set when EN_PUMPX bit is set and returns to 0
after current pulse control sequence is completed
*/
int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);
//EN_PUMPX位是否设置完成，判断比特位是否为0， 0代表结束
int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);
/*
FORCE_ICO 第7位设置为 1  
Force Start Input Current Optimizer (ICO)
0 – Do not force ICO (default)
1 – Force ICO
Note:
This bit is can only be set only and always returns to 0 after ICO starts
*/
static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);
//判断FORCE_ICO是否结束
static int bq2589x_check_force_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(bq2589x_check_force_ico_done);
/*
//EN_TERM  启用禁用终止
Charging Termination Enable
0 – Disable
1 – Enable (default)
*/
static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);
/*
AUTO_DPDM_EN
Automatic D+/D- Detection Enable
0 –Disable D+/D- or PSEL detection when VBUS is plugged-in
1 –Enable  D+/D- or PEL detection when VBUS is plugged-in (default)
*/
static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);
/*
VINDPM Threshold Setting Method
0 – Run Relative VINDPM Threshold (default)
1 – Run Absolute VINDPM Threshold
Note: Register is reset to default value when input source is plugged-in
*/
static int bq2589x_enable_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_absolute_vindpm);
/*
Input Current Optimizer (ICO) Enable
0 – Disable ICO Algorithm
1 – Enable  ICO Algorithm (default)
*/
static int bq2589x_enable_ico(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ico);
/*
Input Current Limit in effect while Input Current Optimizer
(ICO) is enabled
Offset: 100mA (default)
Range 100mA (0000000) – 3.25mA (1111111)
*/
static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else {                                   //0X3f=00111111
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);
/*
//读取2个比特位
CHRG_STAT[1]
CHRG_STAT[0]
00 – Not Charging
01 – Pre-charge ( < VBATLOWV)
10 – Fast Charging
11 – Charge Termination Done
*/
static bool bq2589x_is_charge_done(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);//11就是3
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);
//初始化设备
static int bq2589x_init_device(struct bq2589x *bq)
{

	int ret;

	/*common initialization*/

	bq2589x_disable_watchdog_timer(bq);

	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	bq2589x_enable_ico(bq, bq->cfg.enable_ico);
	bq2589x_enable_absolute_vindpm(bq, bq->cfg.enable_absolute_vindpm);


	ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set vindpm offset:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		dev_err(bq->dev, "%s:Failed to enable charger:%d\n", __func__, ret);
		return ret;
	}

	bq2589x_disable_ilim_pin(bq);

	bq2589x_adc_start(bq, false);

	if (bq == g_bq1) {/* charger 1 specific initialization*/

		ret = bq2589x_pumpx_enable(bq, 1);
		if (ret) {
			dev_err(bq->dev, "%s:Failed to enable pumpx:%d\n", __func__, ret);
			return ret;
		}

		bq2589x_set_watchdog_timer(bq, 160);

	} else if (bq == g_bq2) {/*charger2 specific initialization*/
		ret = bq2589x_enter_hiz_mode(bq);
		if (ret < 0) {
			dev_err(bq->dev, "%s:Failed to enter hiz charger 2:%d\n", __func__, ret);
			return ret;
		}

	}

	return ret;
}

/*
获取充电状态
Charging Status
00 – Not Charging
01 – Pre-charge ( < VBATLOWV)
10 – Fast Charging
11 – Charge Termination Done
*/
static int bq2589x_charge_status(struct bq2589x *bq)
{
	u8 val = 0;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static enum power_supply_property bq2589x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE, /* Charger status output */
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};

//获取指定属性值
static int bq2589x_usb_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{

	//struct bq2589x *bq = container_of(psy, struct bq2589x, usb);
	struct bq2589x *bq = power_supply_get_drvdata(psy);
	u8 type = bq2589x_get_vbus_type(bq);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (type == BQ2589X_VBUS_USB_SDP || type == BQ2589X_VBUS_USB_DCP)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_charge_status(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
//获取指定属性值
static int bq2589x_wall_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{

	//struct bq2589x *bq = container_of(psy, struct bq2589x, wall);
	struct bq2589x *bq = power_supply_get_drvdata(psy);
	u8 type = bq2589x_get_vbus_type(bq);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (type == BQ2589X_VBUS_MAXC || type == BQ2589X_VBUS_UNKNOWN || type == BQ2589X_VBUS_NONSTAND)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_charge_status(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *bq2589x_charger_supplied_to_usb[] = {
	"usb",
};
static char *bq2589x_charger_supplied_to_wall[] = {
	"battery",
};
static const struct power_supply_desc bq2589x_power_supply_desc_usb = {
	.name = "bq2589x-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = bq2589x_charger_props,
	.num_properties = ARRAY_SIZE(bq2589x_charger_props),
	.get_property = bq2589x_usb_get_property,
};

static const struct power_supply_desc bq2589x_power_supply_desc_wall = {
	.name = "bq2589x-Wall",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = bq2589x_charger_props,
	.num_properties = ARRAY_SIZE(bq2589x_charger_props),
	.get_property = bq2589x_wall_get_property,
};

//psy注册
static int bq2589x_psy_register(struct bq2589x *bq)
{
	int ret;

	
	// bq->usb.name = "bq2589x-usb";
	// bq->usb.type = POWER_SUPPLY_TYPE_USB;
	// bq->usb.properties = bq2589x_charger_props;
	// bq->usb.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	// bq->usb.get_property = bq2589x_usb_get_property;
	// bq->usb.external_power_changed = NULL;
//power_supply_register(struct device *parent, struct power_supply *psy);
//power_supply_register(struct device *parent, const struct power_supply_desc *desc, const struct power_supply_config *cfg);
	struct power_supply_config psy_cfg1 = { .drv_data = bq, };
	struct power_supply_config psy_cfg2 = { .drv_data = bq, };

	psy_cfg1.supplied_to = bq2589x_charger_supplied_to_usb;
	psy_cfg1.num_supplicants = ARRAY_SIZE(bq2589x_charger_supplied_to_usb);

	psy_cfg2.supplied_to = bq2589x_charger_supplied_to_wall;
	psy_cfg2.num_supplicants = ARRAY_SIZE(bq2589x_charger_supplied_to_wall);

	bq->usb = power_supply_register(bq->dev, &bq2589x_power_supply_desc_usb, &psy_cfg1);

	//bq->usb = power_supply_register(bq->dev, &bq->usb);
	if (bq->usb == NULL) {
		dev_err(bq->dev, "%s:failed to register usb psy:%d\n", __func__, ret);
		return ret;
	}


	// bq->wall.name = "bq2589x-Wall";
	// bq->wall.type = POWER_SUPPLY_TYPE_MAINS;
	// bq->wall.properties = bq2589x_charger_props;
	// bq->wall.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	// bq->wall.get_property = bq2589x_wall_get_property;
	// bq->wall.external_power_changed = NULL;
//power_supply_register(struct device *parent, const struct power_supply_desc *desc, const struct power_supply_config *cfg);
	bq->wall = power_supply_register(bq->dev, &bq2589x_power_supply_desc_wall, &psy_cfg2);
	if (bq->wall == NULL) {
		dev_err(bq->dev, "%s:failed to register _wall_ psy:%d\n", __func__, ret);
		goto fail_1;
	}

	return 0;

fail_1:
	power_supply_unregister(bq->usb);
	return ret;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
	power_supply_unregister(bq->usb);
	power_supply_unregister(bq->wall);
}

//显示所有寄存器的值
static ssize_t bq2589x_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE,"%s:\n", "Charger 1");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq1, &val, addr);
		if (ret == 0) {
			len = snprintf(&buf[idx], PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			idx += len;
		}
	}

	idx += snprintf(&buf[idx], PAGE_SIZE - idx, "%s:\n", "Charger 2");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq2, &val, addr);
		if (ret == 0) {
			len = snprintf(&buf[idx], PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			idx += len;
		}
	}

	return idx;
}

static DEVICE_ATTR(registers, S_IRUGO, bq2589x_show_registers, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};

//读取设备树
static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-high-level", &pe.high_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-low-level", &pe.low_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,vbat-min-volt-to-tuneup", &pe.vbat_min_volt);
	if (ret)
		return ret;

	bq->cfg.enable_auto_dpdm       = of_property_read_bool(np, "ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term            = of_property_read_bool(np, "ti,bq2589x,enable-termination");
	bq->cfg.enable_ico             = of_property_read_bool(np, "ti,bq2589x,enable-ico");
	bq->cfg.enable_absolute_vindpm = of_property_read_bool(np, "ti,bq2589x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",&bq->cfg.charge_voltage);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",&bq->cfg.charge_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,term-current",&bq->cfg.term_current);
	if (ret)
		return ret;
	return 0;
}
/*
PN[0]
PN[1]
PN[2]
Device Configuration
011: bq25890H
*/
//这个函数可以用来检测芯片是否是BQ25890
static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no  = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;//Device Revision: 11
	}

	return ret;
}


//获取电池电量
static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	union power_supply_propval ret = {0,};

	if (!bq->batt_psy) {
		return 50;
	}
	

	if (bq->batt_psy && bq->batt_psy->desc) {
		bq->batt_psy->desc->get_property(bq->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
		return ret.intval;
	} else {
		return 50;
	}
}

//调整vindpm电压的绝对值
static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	u16 vindpm_volt;
	int ret;

	msleep(1000);
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else
		vindpm_volt = vbus_volt - 1200;

	ret = bq2589x_set_input_volt_limit(bq, vindpm_volt);
	if (ret < 0)
		dev_err(bq->dev, "%s:Set absolute vindpm threshold %d Failed:%d\n", __func__, vindpm_volt, ret);
	else
		dev_info(bq->dev, "%s:Set absolute vindpm threshold %d successfully\n", __func__, vindpm_volt);

}

//工作流水线函数
static void bq2589x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_in_work);
	int ret;

	//进入HIZ模式
	ret = bq2589x_enter_hiz_mode(g_bq2);
	if (ret < 0) {
		dev_err(bq->dev, "%s: Charger 2 enter hiz mode failed\n", __func__);
	} else {
		dev_info(bq->dev, "%s:Charger 2 enter Hiz mode successfully\n", __func__);
		g_bq2->enabled = false;
	}

	if (bq->vbus_type == BQ2589X_VBUS_MAXC) {
		dev_info(bq->dev, "%s:HVDCP or Maxcharge adapter plugged in\n", __func__);
		schedule_delayed_work(&bq->ico_work, 0);
	} else if (bq->vbus_type == BQ2589X_VBUS_USB_DCP) {/* DCP, let's check if it is PE adapter*/
		dev_info(bq->dev, "%s:usb dcp adapter plugged in\n", __func__);
		schedule_delayed_work(&bq->check_pe_tuneup_work, 0);
	} else {
		dev_info(bq->dev, "%s:other adapter plugged in,vbus_type is %d\n", __func__, bq->vbus_type);
		schedule_delayed_work(&bq->ico_work, 0);
	}

	if (bq->cfg.enable_absolute_vindpm) {
		bq2589x_adjust_absolute_vindpm(bq);
		bq2589x_adjust_absolute_vindpm(g_bq2);
	}
	//执行monitor_work
	schedule_delayed_work(&bq->monitor_work, 0);
}
//工作流水线函数
static void bq2589x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_out_work);

	bq2589x_set_input_volt_limit(g_bq1, 4400);
	bq2589x_set_input_volt_limit(g_bq2, 4400);
	
	//取消monitor_work的执行
	cancel_delayed_work_sync(&bq->monitor_work);

}
//工作流水线函数for  ico_work
static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	int ret;
	u8 status;
	int curr;
	static bool ico_issued;

	if (!ico_issued) {
		/* Read VINDPM/IINDPM status */
		ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
		if (ret < 0) {
			schedule_delayed_work(&bq->ico_work, 2 * HZ);
			return;
		}

		ret = bq2589x_force_ico(bq);
		if (ret < 0) {
			schedule_delayed_work(&bq->ico_work, HZ); /* retry 1 second later*/
			dev_info(bq->dev, "%s:ICO command issued failed:%d\n", __func__, ret);
		} else {
			ico_issued = true;
			schedule_delayed_work(&bq->ico_work, 3 * HZ);
			dev_info(bq->dev, "%s:ICO command issued successfully\n", __func__);
		}
	} 
	else {
		ico_issued = false;
		ret = bq2589x_check_force_ico_done(bq);
//		if (ret) {/*ico done*/
//			dev_info(bq->dev, "%s:ICO done!\n", __func__);
			ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
			if (ret == 0) {
				curr = (status & BQ2589X_IDPM_LIM_MASK) * BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
				curr /= 2;
				ret = bq2589x_set_input_current_limit(g_bq2, curr);
				if (ret < 0)
					dev_info(bq->dev, "%s:Set IINDPM for charger 2:%d,failed with code:%d\n", __func__, curr, ret);
				else
					dev_info(bq->dev, "%s:Set IINDPM for charger 2:%d successfully\n", __func__, curr);

			}
//		}
		schedule_delayed_work(&bq->charger2_enable_work, 0);
	}
}

//工作流水线函数 for charger2_enable_work
static void bq2589x_charger2_enable_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, charger2_enable_work.work);
	int ret;

	//获取电池电量
	bq->rsoc = bq2589x_read_batt_rsoc(bq); 
	if ( (bq->vbus_type == BQ2589X_VBUS_MAXC || (bq->vbus_type == BQ2589X_VBUS_USB_DCP && pe.enable && pe.tune_up_volt && pe.tune_done)) 
		&& bq->rsoc < 95 ) {
		ret = bq2589x_exit_hiz_mode(g_bq2);
		if (ret) {
			dev_err(bq->dev, "%s: charger 2 exit hiz mode failed:%d\n", __func__, ret);
		} else {
			dev_info(bq->dev, "%s: charger 2 exit hiz mode successfully\n", __func__);
			g_bq2->enabled = true;
		}
	}
}

//工作流水线函数 for check_pe_tuneup_work
static void bq2589x_check_pe_tuneup_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, check_pe_tuneup_work.work);

	if (!pe.enable) {
		schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	g_bq1->vbat_volt = bq2589x_adc_read_battery_volt(g_bq1);//电池电压
	g_bq1->rsoc = bq2589x_read_batt_rsoc(g_bq1); //电池电流

	if (bq->vbat_volt > pe.vbat_min_volt && g_bq1->rsoc < 95) {
		dev_info(bq->dev, "%s:trying to tune up vbus voltage\n", __func__);
		pe.target_volt = pe.high_volt_level;
		pe.tune_up_volt = true;
		pe.tune_down_volt = false;
		pe.tune_done = false;
		pe.tune_count = 0;
		pe.tune_fail = false;
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	} else if (g_bq1->rsoc >= 95) {
		schedule_delayed_work(&bq->ico_work, 0);
	} else {
		/* wait battery voltage up enough to check again */
		schedule_delayed_work(&bq->check_pe_tuneup_work, 2 * HZ); 
	}
}
//工作流水线函数 for pe_volt_tune_work
static void bq2589x_pe_tune_volt_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pe_volt_tune_work.work);
	int ret;
	static bool pumpx_cmd_issued;

	g_bq1->vbus_volt = bq2589x_adc_read_vbus_volt(g_bq1);

	dev_info(bq->dev, "%s:vbus voltage:%d, Tune Target Volt:%d\n", __func__, g_bq1->vbus_volt, pe.target_volt);

	if ((pe.tune_up_volt && g_bq1->vbus_volt > pe.target_volt) ||
	    (pe.tune_down_volt && g_bq1->vbus_volt < pe.target_volt)) {
		dev_info(bq->dev, "%s:voltage tune successfully\n", __func__);
		pe.tune_done = true;
		bq2589x_adjust_absolute_vindpm(bq);
		bq2589x_adjust_absolute_vindpm(g_bq2);
		if (pe.tune_up_volt)
			schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	if (pe.tune_count > 10) {
		dev_info(bq->dev, "%s:voltage tune failed,reach max retry count\n", __func__);
		pe.tune_fail = true;
		bq2589x_adjust_absolute_vindpm(bq);
		bq2589x_adjust_absolute_vindpm(g_bq2);

		if (pe.tune_up_volt)
			schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	if (!pumpx_cmd_issued) {
		if (pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if (pe.tune_down_volt)
			ret =  bq2589x_pumpx_decrease_volt(bq);
		if (ret) {
			schedule_delayed_work(&bq->pe_volt_tune_work, HZ);
		} else {
			dev_info(bq->dev, "%s:pumpx command issued.\n", __func__);
			pumpx_cmd_issued = true;
			pe.tune_count++;
			schedule_delayed_work(&bq->pe_volt_tune_work, 3 * HZ);
		}
	} else {
		if (pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if (pe.tune_down_volt)
			ret = bq2589x_pumpx_decrease_volt_done(bq);
		if (ret == 0) {/*finished for one step*/
			dev_info(bq->dev, "%s:pumpx command finishedd!\n", __func__);
			bq2589x_adjust_absolute_vindpm(bq);
			bq2589x_adjust_absolute_vindpm(g_bq2);
			pumpx_cmd_issued = 0;
		}
		schedule_delayed_work(&bq->pe_volt_tune_work, HZ);
	}
}

//工作流水线函数 for monitor_work
static void bq2589x_monitor_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, monitor_work.work);
	int ret;
	int chg1_current;
	int chg2_current;

	dev_info(bq->dev, "%s\n", __func__);
	bq2589x_reset_watchdog_timer(bq);

	bq->rsoc = bq2589x_read_batt_rsoc(bq); 

	g_bq1->vbus_volt = bq2589x_adc_read_vbus_volt(g_bq1);
	g_bq1->vbat_volt = bq2589x_adc_read_battery_volt(g_bq1);

	g_bq2->vbus_volt = bq2589x_adc_read_vbus_volt(g_bq2);
	g_bq2->vbat_volt = bq2589x_adc_read_battery_volt(g_bq2);

	chg1_current = bq2589x_adc_read_charge_current(g_bq1);
	chg2_current = bq2589x_adc_read_charge_current(g_bq2);

	dev_info(bq->dev, "%s:charger1:vbus volt:%d,vbat volt:%d,charge current:%d\n",
		__func__,g_bq1->vbus_volt,g_bq1->vbat_volt,chg1_current);

	dev_info(bq->dev, "%s:charger2:vbus volt:%d,vbat volt:%d,charge current:%d\n",
		__func__,g_bq2->vbus_volt,g_bq2->vbat_volt,chg2_current);

	if (g_bq2->enabled && g_bq1->rsoc > 95) {
		ret = bq2589x_enter_hiz_mode(g_bq2);
		if (ret) {
			dev_err(g_bq1->dev, "%s: charger 2 enter hiz mode failed:%d\n", __func__, ret);
		} else {
			dev_info(g_bq1->dev, "%s: charger 2 enter hiz mode successfully\n", __func__);
			g_bq2->enabled = false;
		}
		if (pe.enable && bq->vbus_type == BQ2589X_VBUS_USB_DCP && !pe.tune_down_volt) {
			pe.tune_down_volt = true;
			pe.tune_up_volt = false;
			pe.target_volt = pe.low_volt_level;
			pe.tune_done = false;
			pe.tune_count = 0;
			pe.tune_fail = false;
			schedule_delayed_work(&bq->pe_volt_tune_work, 0);
		}
	}

	/* read temperature,or any other check if need to decrease charge current*/

	schedule_delayed_work(&bq->monitor_work, 10 * HZ);
}
//获取适配器的类型
static void check_adapter_type(struct bq2589x *bq)
{
	if (!bq->cfg.enable_auto_dpdm && bq2589x_force_dpdm(bq)) {
		dev_err(bq->dev,"failed to do force dpdm, vbus type is forced to DCP\n");
		bq->vbus_type = BQ2589X_VBUS_USB_DCP;
	} else{
		bq->vbus_type = bq2589x_get_vbus_type(bq);	
	}
}
//工作流水线函数 for irq_work
static void bq2589x_charger1_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;

	msleep(5);

	if (!(bq->status & BQ2589X_STATUS_PLUGIN))
		check_adapter_type(bq);
	else
		bq->vbus_type = bq2589x_get_vbus_type(bq);

	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret)
		return;

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret)
		return;


	if ((bq->vbus_type == BQ2589X_VBUS_NONE || bq->vbus_type  == BQ2589X_VBUS_OTG) && (bq->status & BQ2589X_STATUS_PLUGIN)) {
		dev_info(bq->dev, "%s:adapter removed\n", __func__);
		bq->status &= ~BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_out_work);
	} else if (bq->vbus_type != BQ2589X_VBUS_NONE && bq->vbus_type != BQ2589X_VBUS_OTG && !(bq->status & BQ2589X_STATUS_PLUGIN)) {
		dev_info(bq->dev, "%s:adapter plugged in\n", __func__);
		bq->status |= BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_in_work);
	}

	if ((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG))
		bq->status |= BQ2589X_STATUS_PG;
	else if (!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG))
		bq->status &= ~BQ2589X_STATUS_PG;

	if (fault && !(bq->status & BQ2589X_STATUS_FAULT))
		bq->status |= BQ2589X_STATUS_FAULT;
	else if (!fault && (bq->status & BQ2589X_STATUS_FAULT))
		bq->status &= ~BQ2589X_STATUS_FAULT;

	bq->interrupt = true;
}

//中断处理函数
static irqreturn_t bq2589x_charger1_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}


#define GPIO_IRQ    80
//驱动加载函数
static int bq2589x_charger1_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;

	int ret;

	//给自定义结构体分配内存
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);//设置自定义数据的指针

	//这个函数可以用来检测芯片是否是BQ25890
	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		bq->status |= BQ2589X_STATUS_EXIST;
		dev_info(bq->dev, "%s: charger device bq25890 detected, revision:%d\n", __func__, bq->revision);
	} else {
		dev_info(bq->dev, "%s: no bq25890 charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	//获取psy的指针
	bq->batt_psy = power_supply_get_by_name("battery");

	g_bq1 = bq;

#if 0
	 /*by default adapter output 5v, if >4.4v,it is ok after tune up*/
	pe.high_volt_level = 4400;
	/*by default adapter output 5v, if <5.5v,it is ok after tune down*/
	pe.low_volt_level = 5500;
	/* by default, tune up adapter output only when bat is >3000*/
	pe.vbat_min_volt = 3000;
#endif
	//解析设备树
	if (client->dev.of_node)
		 bq2589x_parse_dt(&client->dev, g_bq1);

	//初始化设备(给寄存器写入默认值)
	ret = bq2589x_init_device(g_bq1);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}
	//申请一个 GPIO 管脚
	ret = gpio_request(GPIO_IRQ, "bq2589x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, GPIO_IRQ);
		goto err_0;
	}
	//设置某个 GPIO 为输入
	gpio_direction_input(GPIO_IRQ);
	//将当前已经申请的GPIO号转换为IRQ号，也就是获取当前GPIO的中断线，函数调用成功后，将返回对应的IRQ号
	irqn = gpio_to_irq(GPIO_IRQ);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;

	//psy注册
	ret = bq2589x_psy_register(bq);
	if (ret)
		goto err_0;

	//初始化
	INIT_WORK(&bq->irq_work,         bq2589x_charger1_irq_workfunc);
	INIT_WORK(&bq->adapter_in_work,  bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	
	INIT_DELAYED_WORK(&bq->monitor_work,         bq2589x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work,             bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->pe_volt_tune_work,    bq2589x_pe_tune_volt_workfunc);
	INIT_DELAYED_WORK(&bq->check_pe_tuneup_work, bq2589x_check_pe_tuneup_workfunc);
	INIT_DELAYED_WORK(&bq->charger2_enable_work, bq2589x_charger2_enable_workfunc);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}
	//申请中断处理函数
	ret = request_irq(client->irq, bq2589x_charger1_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2589x_charger1_irq", bq);
	if (ret) {
		dev_err(bq->dev, "%s:Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		dev_info(bq->dev, "%s:irq = %d\n", __func__, client->irq);
	}


	pe.enable = true;
	/*in case of adapter has been in when power off*/
	schedule_work(&bq->irq_work);
	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
err_1:
	gpio_free(GPIO_IRQ);
err_0:
	g_bq1 = NULL;
	return ret;
}
//驱动关闭时调用
static void bq2589x_charger1_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	dev_info(bq->dev, "%s: shutdown\n", __func__);

	bq2589x_psy_unregister(bq);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	cancel_work_sync(&bq->irq_work);
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->charger2_enable_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);

	free_irq(bq->client->irq, NULL);
	gpio_free(GPIO_IRQ);

	g_bq1 = NULL;
}

/* interface for other module end */
static void bq2589x_charger2_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret)
		return;

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret)
		return;

	if (((status & BQ2589X_VBUS_STAT_MASK) == 0) && (bq->status & BQ2589X_STATUS_PLUGIN))
		bq->status &= ~BQ2589X_STATUS_PLUGIN;
    else if ((status & BQ2589X_VBUS_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PLUGIN))
		bq->status |= BQ2589X_STATUS_PLUGIN;

	if ((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG))
		bq->status |= BQ2589X_STATUS_PG;
	else if (!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG))
		bq->status &= ~BQ2589X_STATUS_PG;

	if (fault && !(bq->status & BQ2589X_STATUS_FAULT))
		bq->status |= BQ2589X_STATUS_FAULT;
	else if (!fault && (bq->status & BQ2589X_STATUS_FAULT))
		bq->status &= ~BQ2589X_STATUS_FAULT;

	bq->interrupt = true;

}
#if 0
static irqreturn_t bq2589x_charger2_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

#endif
//驱动2加载函数
static int bq2589x_charger2_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct bq2589x *bq;

	int ret;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);
	//这个函数可以用来检测芯片是否是BQ25890
	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25892) {
		bq->status |= BQ2589X_STATUS_EXIST;
		dev_info(bq->dev, "%s: charger device bq25892 detected, revision:%d\n", __func__, bq->revision);
	} else {
		dev_info(bq->dev, "%s: no charger device bq25892 found:%d\n", __func__, ret);
		return -ENODEV;
	}

	g_bq2 = bq;

    /*initialize bq2589x, disable charger 2 by default*/
	//解析设备树
	if (client->dev.of_node)
		 bq2589x_parse_dt(&client->dev, g_bq2);
		 
	//初始化设备
	ret = bq2589x_init_device(g_bq2);
	if (ret)
		dev_err(bq->dev, "%s:Failed to initialize bq2589x charger\n", __func__);
    else
		dev_info(bq->dev, "%s: Initialize bq2589x charger successfully!\n", __func__);
    /* platform setup, irq,...*/
	INIT_WORK(&bq->irq_work, bq2589x_charger2_irq_workfunc);

	return 0;
}

static void bq2589x_charger2_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	dev_info(bq->dev, "%s: shutdown\n", __func__);
	cancel_work_sync(&bq->irq_work);
	g_bq2 = NULL;
}

static struct of_device_id bq2589x_charger1_match_table[] = {
	{.compatible = "BA70",},
	{},
};


static const struct i2c_device_id bq2589x_charger1_id[] = {
	{ "BA70", BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger1_id);

static struct i2c_driver bq2589x_charger1_driver = {
	.driver		= {
		.name	= "BA70",
		.of_match_table = bq2589x_charger1_match_table,
	},
	.id_table	= bq2589x_charger1_id,

	.probe		= bq2589x_charger1_probe,
	.shutdown   = bq2589x_charger1_shutdown,
};

static struct of_device_id bq2589x_charger2_match_table[] = {
	{.compatible = "ti,bq2589x-2",},
	{},
};

static const struct i2c_device_id bq2589x_charger2_id[] = {
	{ "bq2589x-2", BQ25892 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger2_id);


static struct i2c_driver bq2589x_charger2_driver = {
	.driver		= {
		.name	= "bq2589x-2",
		.of_match_table = bq2589x_charger2_match_table,
	},

	.id_table	= bq2589x_charger2_id,

	.probe		= bq2589x_charger2_probe,
	.shutdown   = bq2589x_charger2_shutdown,
};
  //先注释掉，避免编译不通过
static struct i2c_board_info __initdata i2c_bq2589x_charger1[] = {
	{
		I2C_BOARD_INFO("BA70", 0x7D),
	},
};


static struct i2c_board_info __initdata i2c_bq2589x_charger2[] = {
	{
		I2C_BOARD_INFO("bq2589x-2", 0x6B),
	},
};

static int __init bq2589x_charger_init(void)
{

	//先注释掉下面2行代码，避免编译不通过
	i2c_register_board_info(0, i2c_bq2589x_charger1, ARRAY_SIZE(i2c_bq2589x_charger1));
	i2c_register_board_info(0, i2c_bq2589x_charger2, ARRAY_SIZE(i2c_bq2589x_charger2));

	if (i2c_add_driver(&bq2589x_charger2_driver))
		printk("[OBEI]%s, failed to register bq2589x_charger2_driver.\n", __func__);
	else
		printk("[OBEI]%s, bq2589x_charger2_driver register successfully!\n", __func__);

	if (i2c_add_driver(&bq2589x_charger1_driver))
		printk("[OBEI]%s, failed to register bq2589x_charger1_driver.\n", __func__);
	else
		printk("[OBEI]%s, bq2589x_charger1_driver register successfully!\n", __func__);

	return 0;
}

static void __exit bq2589x_charger_exit(void)
{
	i2c_del_driver(&bq2589x_charger1_driver);
	i2c_del_driver(&bq2589x_charger2_driver);
}

module_init(bq2589x_charger_init);
module_exit(bq2589x_charger_exit);

MODULE_DESCRIPTION("TI BQ2589x Dual Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
