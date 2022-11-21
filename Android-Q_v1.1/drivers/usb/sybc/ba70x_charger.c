/*
 * BA70X battery charging driver
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
#include "ba70x_reg.h"

enum ba70x_vbus_type {
	BA70X_VBUS_NONE,
	BA70X_VBUS_USB_SDP,
	BA70X_VBUS_USB_CDP,
	BA70X_VBUS_USB_DCP,
	BA70X_VBUS_MAXC,
	BA70X_VBUS_UNKNOWN,
	BA70X_VBUS_NONSTAND,
	BA70X_VBUS_OTG,
	BA70X_VBUS_TYPE_NUM,
};

enum ba70x_part_no {
	BA70  = 0x03,
	BA702 = 0x00,
	BA705 = 0x07,
};


#define BA70X_STATUS_PLUGIN		0x0001
#define BA70X_STATUS_PG			0x0002
#define BA70X_STATUS_FAULT		0x0008

#define BA70X_STATUS_EXIST			0x0100
#define BA70X_STATUS_CHARGE_ENABLE 	0x0200

struct ba70x_config {
	bool	enable_auto_dpdm;
/*	bool	enable_12v;*/

	int		charge_voltage;
	int		charge_current;

	bool	enable_term;
	int		term_current;

	bool 	enable_ico;
	bool	enable_absolute_vindpm;
};


struct ba70x {
	struct device *dev;
	struct i2c_client *client;

	enum   ba70x_part_no part_no;
	int    revision;

	unsigned int    status;
	int		vbus_type;

	bool	enabled;

	bool    interrupt;

	int     gpio_irq;
	int     vbus_volt;
	int     vbat_volt;

	int     rsoc;
	struct	ba70x_config	cfg;
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

static struct ba70x *g_ba1;
static struct ba70x *g_ba2;
static struct pe_ctrl pe;


static DEFINE_MUTEX(ba70x_i2c_lock);

//从寄存器读取1个字节
static int ba70x_read_byte(struct ba70x *ba, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&ba70x_i2c_lock);
	ret = i2c_smbus_read_byte_data(ba->client, reg);
	if (ret < 0) {
		dev_err(ba->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&ba70x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&ba70x_i2c_lock);

	return 0;
}
//往寄存器写入1个字节
static int ba70x_write_byte(struct ba70x *ba, u8 reg, u8 data)
{
	int ret;
	mutex_lock(&ba70x_i2c_lock);
	ret = i2c_smbus_write_byte_data(ba->client, reg, data);
	mutex_unlock(&ba70x_i2c_lock);
	return ret;
}

//读取然后写入
static int ba70x_update_bits(struct ba70x *ba, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = ba70x_read_byte(ba, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return ba70x_write_byte(ba, reg, tmp);
}

//通过读取寄存器0x0B的值，获取vbus的类型
static enum ba70x_vbus_type ba70x_get_vbus_type(struct ba70x *ba)
{
	u8 val = 0;
	int ret;
	ret = ba70x_read_byte(ba, &val, BA70X_REG_94);
	if (ret < 0)
		return 0;
	val &= BA70X_VBUS_STAT_MASK;
	val >>= BA70X_VBUS_STAT_SHIFT;

	return val;
}

//让设备支持otg功能
static int ba70x_enable_otg(struct ba70x *ba)
{
	u8 val = BA70X_OTG_ENABLE << BA70X_OTG_CONFIG_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_03, BA70X_OTG_CONFIG_MASK, val);

}
//取消设备的otg功能
static int ba70x_disable_otg(struct ba70x *ba)
{
	u8 val = BA70X_OTG_DISABLE << BA70X_OTG_CONFIG_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_03, BA70X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_disable_otg);
//设置otg的电压
static int ba70x_set_otg_volt(struct ba70x *ba, int volt)
{
	u8 val = 0;

	if (volt < BA70X_BOOSTV_BASE)
		volt = BA70X_BOOSTV_BASE;
	if (volt > BA70X_BOOSTV_BASE + (BA70X_BOOSTV_MASK >> BA70X_BOOSTV_SHIFT) * BA70X_BOOSTV_LSB)
		volt = BA70X_BOOSTV_BASE + (BA70X_BOOSTV_MASK >> BA70X_BOOSTV_SHIFT) * BA70X_BOOSTV_LSB;


	val = ((volt - BA70X_BOOSTV_BASE) / BA70X_BOOSTV_LSB) << BA70X_BOOSTV_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_39, BA70X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_set_otg_volt);
//设置otg的电流
static int ba70x_set_otg_current(struct ba70x *ba, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = BA70X_BOOST_LIM_500MA;
	else if (curr == 900)
		temp = BA70X_BOOST_LIM_900MA;
	else if (curr == 1300)
		temp = BA70X_BOOST_LIM_1300MA;
	else if (curr == 1500)
		temp = BA70X_BOOST_LIM_1500MA;
	else if (curr == 2100)
		temp = BA70X_BOOST_LIM_2100MA;
	else if (curr == 2500)
		temp = BA70X_BOOST_LIM_2500MA;
//uu-	else if (curr == 2400)
//uu-		temp = BA70X_BOOST_LIM_2400MA;
	else
		temp = BA70X_BOOST_LIM_1500MA;//uuc

	return ba70x_update_bits(ba, BA70X_REG_39, BA70X_BOOST_LIM_MASK, temp << BA70X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_otg_current);
//使能充电器
static int ba70x_enable_charger(struct ba70x *ba)
{
	int ret;
	u8 val = BA70X_CHG_ENABLE << BA70X_CHG_CONFIG_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_03, BA70X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		ba->status |= BA70X_STATUS_CHARGE_ENABLE;
	return ret;
}
//消能充电器
static int ba70x_disable_charger(struct ba70x *ba)
{
	int ret;
	u8 val = BA70X_CHG_DISABLE << BA70X_CHG_CONFIG_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_03, BA70X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		ba->status &= ~BA70X_STATUS_CHARGE_ENABLE;
	return ret;
}


/* interfaces that can be called by other module
寄存器BA70X_REG_02的第7个比特位
ADC Conversion Start Control
0 – ADC conversion not active (default).
1 – Start ADC Conversion
This bit is read-only when CONV_RATE = 1. The bit stays high during
ADC conversion and during input source detection

 */
//ADC模式开启
int ba70x_adc_start(struct ba70x *ba, bool oneshot)
{
	u8 val;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_0F);//uu 02-0f
	if (ret < 0) {
		dev_err(ba->dev, "%s failed to read register 0x0F:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BA70X_CONV_RATE_MASK) >> BA70X_CONV_RATE_SHIFT) == BA70X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = ba70x_update_bits(ba, BA70X_REG_0F, BA70X_CONV_START_MASK, BA70X_CONV_START << BA70X_CONV_START_SHIFT);
	else
		ret = ba70x_update_bits(ba, BA70X_REG_0F, BA70X_CONV_RATE_MASK, BA70X_ADC_CONTINUE_ENABLE << BA70X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(ba70x_adc_start);
//ADC模式关闭
int ba70x_adc_stop(struct ba70x *ba)
{
	return ba70x_update_bits(ba, BA70X_REG_0F, BA70X_CONV_RATE_MASK, BA70X_ADC_CONTINUE_DISABLE << BA70X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_adc_stop);

//读取电池的电压值
int ba70x_adc_read_battery_volt(struct ba70x *ba)
{
	uint8_t val;
	int volt;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_31);//uu:0E->31:mabe error???
	if (ret < 0) {
		dev_err(ba->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = BA70X_BATV_BASE + ((val & BA70X_BATV_MASK) >> BA70X_BATV_SHIFT) * BA70X_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(ba70x_adc_read_battery_volt);

//读取系统电压值
int ba70x_adc_read_sys_volt(struct ba70x *ba)
{
	uint8_t val;
	int volt;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_0F);
	if (ret < 0) {
		dev_err(ba->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else {
		volt = BA70X_SYSV_BASE + ((val & BA70X_SYSV_MASK) >> BA70X_SYSV_SHIFT) * BA70X_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(ba70x_adc_read_sys_volt);
//读取VBUS电压值
int ba70x_adc_read_vbus_volt(struct ba70x *ba)
{
	uint8_t val1;
	uint8_t val2;
	int volt;
	int ret1,ret2;


	ret1 = ba70x_read_byte(ba, &val1, BA70X_REG_13);
	ret2 = ba70x_read_byte(ba, &val2, BA70X_REG_14);

	if (ret1 < 0|| ret2 < 0) {
		dev_err(ba->dev, "read vbus voltage failed :%d\n", ret1);
		return ret1;
	}

	volt = (val1&0xF)<<8|val2;
	printk("read vbus voltage");
	dev_info(ba->dev, "read vbus voltage success :%d\n", volt);
	//volt = BA70X_VBUSV_BASE + ((val & BA70X_VBUSV_MASK) >> BA70X_VBUSV_SHIFT) * BA70X_VBUSV_LSB ;
	return volt;
	
}
EXPORT_SYMBOL_GPL(ba70x_adc_read_vbus_volt);
//读取温度值
int ba70x_adc_read_temperature(struct ba70x *ba)
{
	uint8_t val;
	int temp;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_10);
	if (ret < 0) {
		dev_err(ba->dev, "read temperature failed :%d\n", ret);
		return ret;
	} else {
		temp = BA70X_TSPCT_BASE + ((val & BA70X_TSPCT_MASK) >> BA70X_TSPCT_SHIFT) * BA70X_TSPCT_LSB ;
		return temp;
	}
}
EXPORT_SYMBOL_GPL(ba70x_adc_read_temperature);
//读取电感器电流值
int ba70x_adc_read_charge_current(struct ba70x *ba)
{
	uint8_t val;
	int volt;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_12);
	if (ret < 0) {
		dev_err(ba->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else {
		volt = (int)(BA70X_ICHGR_BASE + ((val & BA70X_ICHGR_MASK) >> BA70X_ICHGR_SHIFT) * BA70X_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(ba70x_adc_read_charge_current);
//设置当前电流值
int ba70x_set_chargecurrent(struct ba70x *ba, int curr)
{

	u8 ichg;

	ichg = (curr - BA70X_ICHG_BASE) / BA70X_ICHG_LSB;
	return ba70x_update_bits(ba, BA70X_REG_32, BA70X_ICHG_MASK, ichg << BA70X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(ba70x_set_chargecurrent);
//结束当前电流限制
/*
Termination Current Limit
Offset: 64mA
Range: 64mA – 1024mA
Default: 256mA (0011)
*/
int ba70x_set_term_current(struct ba70x *ba, int curr)
{
	u8 iterm;

	iterm = (curr - BA70X_ITERM_BASE) / BA70X_ITERM_LSB;

	return ba70x_update_bits(ba, BA70X_REG_37, BA70X_ITERM_MASK, iterm << BA70X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_term_current);

//设置预充电电流
int ba70x_set_prechg_current(struct ba70x *ba, int curr)
{
	u8 iprechg;

	iprechg = (curr - BA70X_IPRECHG_BASE) / BA70X_IPRECHG_LSB;

	return ba70x_update_bits(ba, BA70X_REG_36, BA70X_IPRECHG_MASK, iprechg << BA70X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_prechg_current);
//设置预充电电压
int ba70x_set_chargevoltage(struct ba70x *ba, int volt)
{
	u8 val;

	val = (volt - BA70X_VREG_BASE) / BA70X_VREG_LSB;
	return ba70x_update_bits(ba, BA70X_REG_31, BA70X_VREG_MASK, val << BA70X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_chargevoltage);

//设置输入电压的上限
int ba70x_set_input_volt_limit(struct ba70x *ba, int volt)
{
	u8 val;
	val = (volt - BA70X_VINDPM_BASE) / BA70X_VINDPM_LSB;
	return ba70x_update_bits(ba, BA70X_REG_33, BA70X_VINDPM_MASK, val << BA70X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_input_volt_limit);
//设置输入电流的上限
int ba70x_set_input_current_limit(struct ba70x *ba, int curr)
{

	u8 val;

	val = (curr - BA70X_IINLIM_BASE) / BA70X_IINLIM_LSB;
	return ba70x_update_bits(ba, BA70X_REG_34, BA70X_IINLIM_MASK, val << BA70X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_input_current_limit);

//设置offset值
int ba70x_set_vindpm_offset(struct ba70x *ba, int offset)
{
	u8 val;

	val = (offset - BA70X_VINDPMOS_BASE) / BA70X_VINDPMOS_LSB;
	return ba70x_update_bits(ba, BA70X_REG_33, BA70X_VINDPMOS_MASK, val << BA70X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_vindpm_offset);

//开始充电
void ba70x_start_charging(struct ba70x *ba)
{
	ba70x_enable_charger(ba);
}
EXPORT_SYMBOL_GPL(ba70x_start_charging);
//停止充电
void ba70x_stop_charging(struct ba70x *ba)
{
	ba70x_disable_charger(ba);
}
EXPORT_SYMBOL_GPL(ba70x_stop_charging);
//获取充电状态
int ba70x_get_charging_status(struct ba70x *ba)
{
	u8 val = 0;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_42);
	if (ret < 0) {
		dev_err(ba->dev, "%s Failed to read register 0x42:%d\n", __func__, ret);
		return ret;
	}
	val &= BA70X_CHRG_STAT_MASK;
	val >>= BA70X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(ba70x_get_charging_status);
//启用和禁用otg功能
void ba70x_set_otg(struct ba70x *ba, int enable)
{
	int ret;

	if (enable) {
		ret = ba70x_enable_otg(ba);
		if (ret < 0) {
			dev_err(ba->dev, "%s:Failed to enable otg-%d\n", __func__, ret);
			return;
		}
	} else {
		ret = ba70x_disable_otg(ba);
		if (ret < 0)
			dev_err(ba->dev, "%s:Failed to disable otg-%d\n", __func__, ret);
	}
}
EXPORT_SYMBOL_GPL(ba70x_set_otg);
//设置看门狗定时器的超时时间
int ba70x_set_watchdog_timer(struct ba70x *ba, u8 timeout)
{
	return ba70x_update_bits(ba, BA70X_REG_07, BA70X_WDT_MASK, (u8)((timeout - BA70X_WDT_BASE) / BA70X_WDT_LSB) << BA70X_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(ba70x_set_watchdog_timer);
//取消看门狗定时器
int ba70x_disable_watchdog_timer(struct ba70x *ba)
{
	u8 val = BA70X_WDT_DISABLE << BA70X_WDT_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_07, BA70X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(ba70x_disable_watchdog_timer);
//重启看门狗定时器
int ba70x_reset_watchdog_timer(struct ba70x *ba)
{
	u8 val = BA70X_WDT_RESET << BA70X_WDT_RESET_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_07, BA70X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(ba70x_reset_watchdog_timer);
//强制使用dpdm模式
int ba70x_force_dpdm(struct ba70x *ba)
{
	int ret;
	u8 val = BA70X_FORCE_DPDM << BA70X_FORCE_DPDM_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_90, BA70X_FORCE_DPDM_MASK, val);
	if (ret)
		return ret;

	msleep(10);
	return 0;

}
EXPORT_SYMBOL_GPL(ba70x_force_dpdm);
/*
函数功能：寄存器0x14的第七位设置位1
0 – Keep current register setting (default)  设为0不会有动作
1 – Reset to default register value and reset safety timer  设为1会安全的重启寄存器
Note:
Reset to 0 after register reset is completed   重启完成后，寄存器的值会恢复为0
*/
int ba70x_reset_chip(struct ba70x *ba)
{
	int ret;
	u8 val = BA70X_RESET << BA70X_RESET_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_14, BA70X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(ba70x_reset_chip);

/*
Force BATFET off to enable ship mode
0 – Allow BATFET turn on (default)
1 – Force BATFET off
*/
int ba70x_enter_ship_mode(struct ba70x *ba)
{
	int ret;
	u8 val = BA70X_BATFET_OFF << BA70X_BATFET_DIS_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_3B, BA70X_BATFET_DIS_MASK, val);
	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_enter_ship_mode);
/*
第7位为 EN_HIZ 模式
Enable HIZ Mode
0 – Disable (default)
1 – Enable
*/
int ba70x_enter_hiz_mode(struct ba70x *ba)
{
	u8 val = BA70X_HIZ_ENABLE << BA70X_HIZ_EN_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_3C, BA70X_HIZ_EN_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_enter_hiz_mode);

int ba70x_exit_hiz_mode(struct ba70x *ba)
{

	u8 val = BA70X_HIZ_DISABLE << BA70X_HIZ_EN_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_3C, BA70X_HIZ_EN_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_exit_hiz_mode);
//读取HIZ模式，从寄存器读取1个字节，然后取第7位的值 &0x80 就是取一个字节的最高位
int ba70x_get_hiz_mode(struct ba70x *ba, u8 *state)
{
	u8 val;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_3C);
	if (ret)
		return ret;
	*state = (val & BA70X_HIZ_EN_MASK) >> BA70X_HIZ_EN_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(ba70x_get_hiz_mode);
/*
取第6位的值 &0x40
Enable ILIM Pin
0 – Disable
1 – Enable (default: Enable ILIM pin (1))
*/
int ba70x_enable_ilim_pin(struct ba70x *ba)
{
	u8 val = BA70X_ENILIM_ENABLE << BA70X_ENILIM_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_34, BA70X_ENILIM_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_enable_ilim_pin);
/*
Enable ILIM Pin
0 – Disable
1 – Enable (default: Enable ILIM pin (1))
*/
int ba70x_disable_ilim_pin(struct ba70x *ba)
{
	u8 val = BA70X_ENILIM_DISABLE << BA70X_ENILIM_SHIFT;

	return ba70x_update_bits(ba, BA70X_REG_34, BA70X_ENILIM_MASK, val);

}
EXPORT_SYMBOL_GPL(ba70x_disable_ilim_pin);

/*
EN_PUMPX 寄存器的最高位
Current pulse control Enable
0 - Disable Current pulse control (default)
1 - Enable Current pulse control (PUMPX_UP and PUMPX_DN)
*/
int ba70x_pumpx_enable(struct ba70x *ba, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BA70X_PUMPX_ENABLE << BA70X_EN_PUMPX_SHIFT;
	else
		val = BA70X_PUMPX_DISABLE << BA70X_EN_PUMPX_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_57, BA70X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(ba70x_pumpx_enable);
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
int ba70x_pumpx_increase_volt(struct ba70x *ba)
{
	u8 val;
	int ret;

	val = BA70X_PUMPX_UP << BA70X_PUMPX_UP_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_58, BA70X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_pumpx_increase_volt);
//EN_PUMPX位是否设置完成，判断比特位是否为0， 0代表结束
int ba70x_pumpx_increase_volt_done(struct ba70x *ba)
{
	u8 val;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_58);//?09-58
	if (ret)
		return ret;

	if (val & BA70X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(ba70x_pumpx_increase_volt_done);
/*
Current pulse control voltage down enable
0 – Disable (default)
1 – Enable
Note:
This bit is can only be set when EN_PUMPX bit is set and returns to 0
after current pulse control sequence is completed
*/
int ba70x_pumpx_decrease_volt(struct ba70x *ba)
{
	u8 val;
	int ret;

	val = BA70X_PUMPX_DOWN << BA70X_PUMPX_DOWN_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_58, BA70X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_pumpx_decrease_volt);
//EN_PUMPX位是否设置完成，判断比特位是否为0， 0代表结束
int ba70x_pumpx_decrease_volt_done(struct ba70x *ba)
{
	u8 val;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_58);
	if (ret)
		return ret;

	if (val & BA70X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(ba70x_pumpx_decrease_volt_done);
/*
FORCE_ICO 第7位设置为 1  
Force Start Input Current Optimizer (ICO)
0 – Do not force ICO (default)
1 – Force ICO
Note:
This bit is can only be set only and always returns to 0 after ICO starts
*/
static int ba70x_force_ico(struct ba70x *ba)
{
	u8 val;
	int ret;

	val = BA70X_FORCE_ICO << BA70X_FORCE_ICO_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_35, BA70X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(ba70x_force_ico);
//判断FORCE_ICO是否结束
static int ba70x_check_force_ico_done(struct ba70x *ba)
{
	u8 val;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_14);
	if (ret)
		return ret;

	if (val & BA70X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(ba70x_check_force_ico_done);
/*
//EN_TERM  启用禁用终止
Charging Termination Enable
0 – Disable
1 – Enable (default)
*/
static int ba70x_enable_term(struct ba70x* ba, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BA70X_TERM_ENABLE << BA70X_EN_TERM_SHIFT;
	else
		val = BA70X_TERM_DISABLE << BA70X_EN_TERM_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_37, BA70X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(ba70x_enable_term);
/*
AUTO_DPDM_EN
Automatic D+/D- Detection Enable
0 –Disable D+/D- or PSEL detection when VBUS is plugged-in
1 –Enable  D+/D- or PEL detection when VBUS is plugged-in (default)
*/
static int ba70x_enable_auto_dpdm(struct ba70x* ba, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BA70X_AUTO_DPDM_ENABLE << BA70X_AUTO_DPDM_EN_SHIFT;
	else
		val = BA70X_AUTO_DPDM_DISABLE << BA70X_AUTO_DPDM_EN_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_90, BA70X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_enable_auto_dpdm);
/*
VINDPM Threshold Setting Method
0 – Run Relative VINDPM Threshold (default)
1 – Run Absolute VINDPM Threshold
Note: Register is reset to default value when input source is plugged-in
*/
static int ba70x_enable_absolute_vindpm(struct ba70x* ba, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BA70X_FORCE_VINDPM_ENABLE << BA70X_FORCE_VINDPM_SHIFT;
	else
		val = BA70X_FORCE_VINDPM_DISABLE << BA70X_FORCE_VINDPM_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_33, BA70X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_enable_absolute_vindpm);
/*
Input Current Optimizer (ICO) Enable
0 – Disable ICO Algorithm
1 – Enable  ICO Algorithm (default)
*/
static int ba70x_enable_ico(struct ba70x* ba, bool enable)
{
	u8 val;
	int ret;
	
	if (enable)
		val = BA70X_ICO_ENABLE << BA70X_ICOEN_SHIFT;
	else
		val = BA70X_ICO_DISABLE << BA70X_ICOEN_SHIFT;

	ret = ba70x_update_bits(ba, BA70X_REG_35, BA70X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(ba70x_enable_ico);
/*
Input Current Limit in effect while Input Current Optimizer
(ICO) is enabled
Offset: 100mA (default)
Range 100mA (0000000) – 3.25mA (1111111)
*/
static int ba70x_read_idpm_limit(struct ba70x *ba)
{
	uint8_t val;
	int curr;
	int ret;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_13);
	if (ret < 0) {
		dev_err(ba->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else {                                   //0X3f=00111111
		curr = BA70X_IDPM_LIM_BASE + ((val & BA70X_IDPM_LIM_MASK) >> BA70X_IDPM_LIM_SHIFT) * BA70X_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(ba70x_read_idpm_limit);
/*
//读取2个比特位
CHRG_STAT[1]
CHRG_STAT[0]
00 – Not Charging
01 – Pre-charge ( < VBATLOWV)
10 – Fast Charging
11 – Charge Termination Done
*/
static bool ba70x_is_charge_done(struct ba70x *ba)
{
	int ret;
	u8 val;

	ret = ba70x_read_byte(ba, &val, BA70X_REG_42);
	if (ret < 0) {
		dev_err(ba->dev, "%s:read REG42 failed :%d\n", __func__, ret);
		return false;
	}
	val &= BA70X_CHRG_STAT_MASK;
	val >>= BA70X_CHRG_STAT_SHIFT;

	return (val == BA70X_CHRG_STAT_CHGDONE);//11就是3
}
EXPORT_SYMBOL_GPL(ba70x_is_charge_done);
//初始化设备
static int ba70x_init_device(struct ba70x *ba)
{

	int ret;

	/*common initialization*/

	ba70x_disable_watchdog_timer(ba);

	ba70x_enable_auto_dpdm(ba, ba->cfg.enable_auto_dpdm);
	ba70x_enable_term(ba, ba->cfg.enable_term);
	ba70x_enable_ico(ba, ba->cfg.enable_ico);
	ba70x_enable_absolute_vindpm(ba, ba->cfg.enable_absolute_vindpm);

	printk("[OBEI]init device auto_dpdm\n");

	ret = ba70x_set_vindpm_offset(ba, 600);
	if (ret < 0) {
		dev_err(ba->dev, "%s:Failed to set vindpm offset:%d\n", __func__, ret);
		return ret;
	}

	ret = ba70x_set_term_current(ba, ba->cfg.term_current);
	if (ret < 0) {
		dev_err(ba->dev, "%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = ba70x_set_chargevoltage(ba, ba->cfg.charge_voltage);
	if (ret < 0) {
		dev_err(ba->dev, "%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = ba70x_set_chargecurrent(ba, ba->cfg.charge_current);
	if (ret < 0) {
		dev_err(ba->dev, "%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = ba70x_enable_charger(ba);
	if (ret < 0) {
		dev_err(ba->dev, "%s:Failed to enable charger:%d\n", __func__, ret);
		return ret;
	}

	ba70x_disable_ilim_pin(ba);

	ba70x_adc_start(ba, false);
	dev_info(ba->dev, "[OBEI]ADC START\n");
	if (ba == g_ba1) {/* charger 1 specific initialization*/

		ret = ba70x_pumpx_enable(ba, 1);
		if (ret) {
			dev_err(ba->dev, "%s:Failed to enable pumpx:%d\n", __func__, ret);
			return ret;
		}

		ba70x_set_watchdog_timer(ba, 160);

	} else if (ba == g_ba2) {/*charger2 specific initialization*/
		ret = ba70x_enter_hiz_mode(ba);
		if (ret < 0) {
			dev_err(ba->dev, "%s:Failed to enter hiz charger 2:%d\n", __func__, ret);
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
static int ba70x_charge_status(struct ba70x *ba)
{
	u8 val = 0;

	ba70x_read_byte(ba, &val, BA70X_REG_42);
	val &= BA70X_CHRG_STAT_MASK;
	val >>= BA70X_CHRG_STAT_SHIFT;

	switch (val) {
	case BA70X_CHRG_STAT_FASTCHG_CC:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;//uuc
	case BA70X_CHRG_STAT_FASTCHG_CV:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;//uuc

	case BA70X_CHRG_STAT_TRICKLE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;//uu+
	case BA70X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	case BA70X_CHRG_STAT_CHGDONE:
	case BA70X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static enum power_supply_property ba70x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE, /* Charger status output */
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};

//获取指定属性值
static int ba70x_usb_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{

	//struct ba70x *ba = container_of(psy, struct ba70x, usb);
	struct ba70x *ba = power_supply_get_drvdata(psy);
	u8 type = ba70x_get_vbus_type(ba);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (type == BA70X_VBUS_USB_SDP || type == BA70X_VBUS_USB_DCP)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ba70x_charge_status(ba);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
//获取指定属性值
static int ba70x_wall_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{

	//struct ba70x *ba = container_of(psy, struct ba70x, wall);
	struct ba70x *ba = power_supply_get_drvdata(psy);
	u8 type = ba70x_get_vbus_type(ba);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (type == BA70X_VBUS_MAXC || type == BA70X_VBUS_UNKNOWN || type == BA70X_VBUS_NONSTAND)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ba70x_charge_status(ba);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char *ba70x_charger_supplied_to_usb[] = {
	"usb",
};
static char *ba70x_charger_supplied_to_wall[] = {
	"battery",
};
static const struct power_supply_desc ba70x_power_supply_desc_usb = {
	.name = "ba70x-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ba70x_charger_props,
	.num_properties = ARRAY_SIZE(ba70x_charger_props),
	.get_property = ba70x_usb_get_property,
};

static const struct power_supply_desc ba70x_power_supply_desc_wall = {
	.name = "ba70x-Wall",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ba70x_charger_props,
	.num_properties = ARRAY_SIZE(ba70x_charger_props),
	.get_property = ba70x_wall_get_property,
};

//psy注册
static int ba70x_psy_register(struct ba70x *ba)
{
	int ret;

	
	// ba->usb.name = "ba70x-usb";
	// ba->usb.type = POWER_SUPPLY_TYPE_USB;
	// ba->usb.properties = ba70x_charger_props;
	// ba->usb.num_properties = ARRAY_SIZE(ba70x_charger_props);
	// ba->usb.get_property = ba70x_usb_get_property;
	// ba->usb.external_power_changed = NULL;
//power_supply_register(struct device *parent, struct power_supply *psy);
//power_supply_register(struct device *parent, const struct power_supply_desc *desc, const struct power_supply_config *cfg);
	struct power_supply_config psy_cfg1 = { .drv_data = ba, };
	struct power_supply_config psy_cfg2 = { .drv_data = ba, };

	psy_cfg1.supplied_to = ba70x_charger_supplied_to_usb;
	psy_cfg1.num_supplicants = ARRAY_SIZE(ba70x_charger_supplied_to_usb);

	psy_cfg2.supplied_to = ba70x_charger_supplied_to_wall;
	psy_cfg2.num_supplicants = ARRAY_SIZE(ba70x_charger_supplied_to_wall);

	ba->usb = power_supply_register(ba->dev, &ba70x_power_supply_desc_usb, &psy_cfg1);

	//ba->usb = power_supply_register(ba->dev, &ba->usb);
	if (ba->usb == NULL) {
		dev_err(ba->dev, "%s:failed to register usb psy:%d\n", __func__, ret);
		return ret;
	}


	// ba->wall.name = "ba70x-Wall";
	// ba->wall.type = POWER_SUPPLY_TYPE_MAINS;
	// ba->wall.properties = ba70x_charger_props;
	// ba->wall.num_properties = ARRAY_SIZE(ba70x_charger_props);
	// ba->wall.get_property = ba70x_wall_get_property;
	// ba->wall.external_power_changed = NULL;
//power_supply_register(struct device *parent, const struct power_supply_desc *desc, const struct power_supply_config *cfg);
	ba->wall = power_supply_register(ba->dev, &ba70x_power_supply_desc_wall, &psy_cfg2);
	if (ba->wall == NULL) {
		dev_err(ba->dev, "%s:failed to register _wall_ psy:%d\n", __func__, ret);
		goto fail_1;
	}

	return 0;

fail_1:
	power_supply_unregister(ba->usb);
	return ret;
}

static void ba70x_psy_unregister(struct ba70x *ba)
{
	power_supply_unregister(ba->usb);
	power_supply_unregister(ba->wall);
}

//显示所有寄存器的值
static ssize_t ba70x_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE,"%s:\n", "Charger 1");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = ba70x_read_byte(g_ba1, &val, addr);
		if (ret == 0) {
			len = snprintf(&buf[idx], PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			idx += len;
		}
	}

	idx += snprintf(&buf[idx], PAGE_SIZE - idx, "%s:\n", "Charger 2");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = ba70x_read_byte(g_ba2, &val, addr);
		if (ret == 0) {
			len = snprintf(&buf[idx], PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			idx += len;
		}
	}

	return idx;
}

static DEVICE_ATTR(registers, S_IRUGO, ba70x_show_registers, NULL);

static struct attribute *ba70x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group ba70x_attr_group = {
	.attrs = ba70x_attributes,
};

//读取设备树
static int ba70x_parse_dt(struct device *dev, struct ba70x *ba)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "sy,ba70x,vbus-volt-high-level", &pe.high_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "sy,ba70x,vbus-volt-low-level", &pe.low_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "sy,ba70x,vbat-min-volt-to-tuneup", &pe.vbat_min_volt);
	if (ret)
		return ret;

	ba->cfg.enable_auto_dpdm       = false;//of_property_read_bool(np, "sy,ba70x,enable-auto-dpdm");
	ba->cfg.enable_term            = of_property_read_bool(np, "sy,ba70x,enable-termination");
	ba->cfg.enable_ico             = of_property_read_bool(np, "sy,ba70x,enable-ico");
	ba->cfg.enable_absolute_vindpm = of_property_read_bool(np, "sy,ba70x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "sy,ba70x,charge-voltage",&ba->cfg.charge_voltage);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "sy,ba70x,charge-current",&ba->cfg.charge_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "sy,ba70x,term-current",&ba->cfg.term_current);
	if (ret)
		return ret;
	return 0;
}
/*
PN[0]
PN[1]
PN[2]
Device Configuration
011: ba25890H
*/
//这个函数可以用来检测芯片是否是BA70
static int ba70x_detect_device(struct ba70x *ba)
{
	int ret;
	u8 data;

	ret = ba70x_read_byte(ba, &data, BA70X_REG_00);
	if (ret == 0) {
		ba->part_no  = BA70;//(data & BA70X_PN_MASK) >> BA70X_PN_SHIFT;
		ba->revision = data;//(data & BA70X_DEV_REV_MASK) >> BA70X_DEV_REV_SHIFT;//Device Revision: 11
	}

	return ret;
}


//获取电池电量
static int ba70x_read_batt_rsoc(struct ba70x *ba)
{
	union power_supply_propval ret = {0,};

	if (!ba->batt_psy) {
		return 50;
	}
	

	if (ba->batt_psy && ba->batt_psy->desc) {
		ba->batt_psy->desc->get_property(ba->batt_psy,POWER_SUPPLY_PROP_CAPACITY,&ret);
		return ret.intval;
	} else {
		return 50;
	}
}

//调整vindpm电压的绝对值
static void ba70x_adjust_absolute_vindpm(struct ba70x *ba)
{
	u16 vbus_volt;
	u16 vindpm_volt;
	int ret;

	msleep(1000);
	vbus_volt = ba70x_adc_read_vbus_volt(ba);
	if (vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else
		vindpm_volt = vbus_volt - 1200;

	ret = ba70x_set_input_volt_limit(ba, vindpm_volt);
	if (ret < 0)
		dev_err(ba->dev, "%s:Set absolute vindpm threshold %d Failed:%d\n", __func__, vindpm_volt, ret);
	else
		dev_info(ba->dev, "%s:Set absolute vindpm threshold %d successfully\n", __func__, vindpm_volt);

}

//工作流水线函数
static void ba70x_adapter_in_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, adapter_in_work);
	int ret;

	//进入HIZ模式
	ret = ba70x_enter_hiz_mode(g_ba2);
	if (ret < 0) {
		dev_err(ba->dev, "%s: Charger 2 enter hiz mode failed\n", __func__);
	} else {
		dev_info(ba->dev, "%s:Charger 2 enter Hiz mode successfully\n", __func__);
		g_ba2->enabled = false;
	}

	if (ba->vbus_type == BA70X_VBUS_MAXC) {
		dev_info(ba->dev, "%s:HVDCP or Maxcharge adapter plugged in\n", __func__);
		schedule_delayed_work(&ba->ico_work, 0);
	} else if (ba->vbus_type == BA70X_VBUS_USB_DCP) {/* DCP, let's check if it is PE adapter*/
		dev_info(ba->dev, "%s:usb dcp adapter plugged in\n", __func__);
		schedule_delayed_work(&ba->check_pe_tuneup_work, 0);
	} else {
		dev_info(ba->dev, "%s:other adapter plugged in,vbus_type is %d\n", __func__, ba->vbus_type);
		schedule_delayed_work(&ba->ico_work, 0);
	}

	if (ba->cfg.enable_absolute_vindpm) {
		ba70x_adjust_absolute_vindpm(ba);
		ba70x_adjust_absolute_vindpm(g_ba2);
	}
	//执行monitor_work
	schedule_delayed_work(&ba->monitor_work, 0);
}
//工作流水线函数
static void ba70x_adapter_out_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, adapter_out_work);

	ba70x_set_input_volt_limit(g_ba1, 4400);
	ba70x_set_input_volt_limit(g_ba2, 4400);
	
	//取消monitor_work的执行
	cancel_delayed_work_sync(&ba->monitor_work);

}
//工作流水线函数for  ico_work
static void ba70x_ico_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, ico_work.work);
	int ret;
	u8 status;
	int curr;
	static bool ico_issued;

	if (!ico_issued) {
		/* Read VINDPM/IINDPM status */
		ret = ba70x_read_byte(ba, &status, BA70X_REG_13);
		if (ret < 0) {
			schedule_delayed_work(&ba->ico_work, 2 * HZ);
			return;
		}

		ret = ba70x_force_ico(ba);
		if (ret < 0) {
			schedule_delayed_work(&ba->ico_work, HZ); /* retry 1 second later*/
			dev_info(ba->dev, "%s:ICO command issued failed:%d\n", __func__, ret);
		} else {
			ico_issued = true;
			schedule_delayed_work(&ba->ico_work, 3 * HZ);
			dev_info(ba->dev, "%s:ICO command issued successfully\n", __func__);
		}
	} 
	else {
		ico_issued = false;
		ret = ba70x_check_force_ico_done(ba);
//		if (ret) {/*ico done*/
//			dev_info(ba->dev, "%s:ICO done!\n", __func__);
			ret = ba70x_read_byte(ba, &status, BA70X_REG_13);
			if (ret == 0) {
				curr = (status & BA70X_IDPM_LIM_MASK) * BA70X_IDPM_LIM_LSB + BA70X_IDPM_LIM_BASE;
				curr /= 2;
				ret = ba70x_set_input_current_limit(g_ba2, curr);
				if (ret < 0)
					dev_info(ba->dev, "%s:Set IINDPM for charger 2:%d,failed with code:%d\n", __func__, curr, ret);
				else
					dev_info(ba->dev, "%s:Set IINDPM for charger 2:%d successfully\n", __func__, curr);

			}
//		}
		schedule_delayed_work(&ba->charger2_enable_work, 0);
	}
}

//工作流水线函数 for charger2_enable_work
static void ba70x_charger2_enable_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, charger2_enable_work.work);
	int ret;

	//获取电池电量
	ba->rsoc = ba70x_read_batt_rsoc(ba); 
	if ( (ba->vbus_type == BA70X_VBUS_MAXC || (ba->vbus_type == BA70X_VBUS_USB_DCP && pe.enable && pe.tune_up_volt && pe.tune_done)) 
		&& ba->rsoc < 95 ) {
		ret = ba70x_exit_hiz_mode(g_ba2);
		if (ret) {
			dev_err(ba->dev, "%s: charger 2 exit hiz mode failed:%d\n", __func__, ret);
		} else {
			dev_info(ba->dev, "%s: charger 2 exit hiz mode successfully\n", __func__);
			g_ba2->enabled = true;
		}
	}
}

//工作流水线函数 for check_pe_tuneup_work
static void ba70x_check_pe_tuneup_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, check_pe_tuneup_work.work);

	if (!pe.enable) {
		schedule_delayed_work(&ba->ico_work, 0);
		return;
	}

	g_ba1->vbat_volt = ba70x_adc_read_battery_volt(g_ba1);//电池电压
	g_ba1->rsoc = ba70x_read_batt_rsoc(g_ba1); //电池电流

	if (ba->vbat_volt > pe.vbat_min_volt && g_ba1->rsoc < 95) {
		dev_info(ba->dev, "%s:trying to tune up vbus voltage\n", __func__);
		pe.target_volt = pe.high_volt_level;
		pe.tune_up_volt = true;
		pe.tune_down_volt = false;
		pe.tune_done = false;
		pe.tune_count = 0;
		pe.tune_fail = false;
		schedule_delayed_work(&ba->pe_volt_tune_work, 0);
	} else if (g_ba1->rsoc >= 95) {
		schedule_delayed_work(&ba->ico_work, 0);
	} else {
		/* wait battery voltage up enough to check again */
		schedule_delayed_work(&ba->check_pe_tuneup_work, 2 * HZ); 
	}
}
//工作流水线函数 for pe_volt_tune_work
static void ba70x_pe_tune_volt_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, pe_volt_tune_work.work);
	int ret;
	static bool pumpx_cmd_issued;

	g_ba1->vbus_volt = ba70x_adc_read_vbus_volt(g_ba1);

	dev_info(ba->dev, "%s:vbus voltage:%d, Tune Target Volt:%d\n", __func__, g_ba1->vbus_volt, pe.target_volt);

	if ((pe.tune_up_volt && g_ba1->vbus_volt > pe.target_volt) ||
	    (pe.tune_down_volt && g_ba1->vbus_volt < pe.target_volt)) {
		dev_info(ba->dev, "%s:voltage tune successfully\n", __func__);
		pe.tune_done = true;
		ba70x_adjust_absolute_vindpm(ba);
		ba70x_adjust_absolute_vindpm(g_ba2);
		if (pe.tune_up_volt)
			schedule_delayed_work(&ba->ico_work, 0);
		return;
	}

	if (pe.tune_count > 10) {
		dev_info(ba->dev, "%s:voltage tune failed,reach max retry count\n", __func__);
		pe.tune_fail = true;
		ba70x_adjust_absolute_vindpm(ba);
		ba70x_adjust_absolute_vindpm(g_ba2);

		if (pe.tune_up_volt)
			schedule_delayed_work(&ba->ico_work, 0);
		return;
	}

	if (!pumpx_cmd_issued) {
		if (pe.tune_up_volt)
			ret = ba70x_pumpx_increase_volt(ba);
		else if (pe.tune_down_volt)
			ret =  ba70x_pumpx_decrease_volt(ba);
		if (ret) {
			schedule_delayed_work(&ba->pe_volt_tune_work, HZ);
		} else {
			dev_info(ba->dev, "%s:pumpx command issued.\n", __func__);
			pumpx_cmd_issued = true;
			pe.tune_count++;
			schedule_delayed_work(&ba->pe_volt_tune_work, 3 * HZ);
		}
	} else {
		if (pe.tune_up_volt)
			ret = ba70x_pumpx_increase_volt_done(ba);
		else if (pe.tune_down_volt)
			ret = ba70x_pumpx_decrease_volt_done(ba);
		if (ret == 0) {/*finished for one step*/
			dev_info(ba->dev, "%s:pumpx command finishedd!\n", __func__);
			ba70x_adjust_absolute_vindpm(ba);
			ba70x_adjust_absolute_vindpm(g_ba2);
			pumpx_cmd_issued = 0;
		}
		schedule_delayed_work(&ba->pe_volt_tune_work, HZ);
	}
}

//工作流水线函数 for monitor_work
static void ba70x_monitor_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, monitor_work.work);
	int ret;
	int chg1_current;
	int chg2_current;

	dev_info(ba->dev, "[OBEI-Workfunc]%s\n", __func__);
	ba70x_reset_watchdog_timer(ba);

	ba->rsoc = ba70x_read_batt_rsoc(ba); 

	g_ba1->vbus_volt = ba70x_adc_read_vbus_volt(g_ba1);
	g_ba1->vbat_volt = ba70x_adc_read_battery_volt(g_ba1);

	g_ba2->vbus_volt = ba70x_adc_read_vbus_volt(g_ba2);
	g_ba2->vbat_volt = ba70x_adc_read_battery_volt(g_ba2);

	chg1_current = ba70x_adc_read_charge_current(g_ba1);
	chg2_current = ba70x_adc_read_charge_current(g_ba2);

	dev_info(ba->dev, "%s:charger1:vbus volt:%d,vbat volt:%d,charge current:%d\n",
		__func__,g_ba1->vbus_volt,g_ba1->vbat_volt,chg1_current);

	dev_info(ba->dev, "%s:charger2:vbus volt:%d,vbat volt:%d,charge current:%d\n",
		__func__,g_ba2->vbus_volt,g_ba2->vbat_volt,chg2_current);

	if (g_ba2->enabled && g_ba1->rsoc > 95) {
		ret = ba70x_enter_hiz_mode(g_ba2);
		if (ret) {
			dev_err(g_ba1->dev, "%s: charger 2 enter hiz mode failed:%d\n", __func__, ret);
		} else {
			dev_info(g_ba1->dev, "%s: charger 2 enter hiz mode successfully\n", __func__);
			g_ba2->enabled = false;
		}
		if (pe.enable && ba->vbus_type == BA70X_VBUS_USB_DCP && !pe.tune_down_volt) {
			pe.tune_down_volt = true;
			pe.tune_up_volt = false;
			pe.target_volt = pe.low_volt_level;
			pe.tune_done = false;
			pe.tune_count = 0;
			pe.tune_fail = false;
			schedule_delayed_work(&ba->pe_volt_tune_work, 0);
		}
	}

	/* read temperature,or any other check if need to decrease charge current*/

	schedule_delayed_work(&ba->monitor_work, 10 * HZ);
}
//获取适配器的类型
static void check_adapter_type(struct ba70x *ba)
{
	if (!ba->cfg.enable_auto_dpdm && !ba70x_force_dpdm(ba)) {
		dev_err(ba->dev,"failed to do force dpdm, vbus type is forced to DCP\n");
		ba->vbus_type = BA70X_VBUS_USB_DCP;
	} else{
		ba->vbus_type = ba70x_get_vbus_type(ba);	
	}
}
//工作流水线函数 for irq_work
static void ba70x_charger1_irq_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;
	printk("[OBEI]-call_charger1_irq_workfunc]");
	msleep(5);

	if (!(ba->status & BA70X_STATUS_PLUGIN))
	{
		check_adapter_type(ba);
		printk("[OBEI]-call_charger1_CHECK_TYPE]");
	}	
	else
	{
		ba->vbus_type = ba70x_get_vbus_type(ba);
		printk("[OBEI]-call_GET VBUS TYPE]");
	}	

	/* Read STATUS and FAULT registers */
//uu	ret = ba70x_read_byte(ba, &status, BA70X_REG_0B);//uu?pg:0b-41 42...
	ret = ba70x_read_byte(ba, &status, BA70X_REG_41);//uu?pg:0b-41 42...
	if (ret)
		return;

	ret = ba70x_read_byte(ba, &fault, BA70X_REG_0C);
	if (ret)
		return;


	if ((ba->vbus_type == BA70X_VBUS_NONE || ba->vbus_type  == BA70X_VBUS_OTG) && (ba->status & BA70X_STATUS_PLUGIN)) {
		dev_info(ba->dev, "%s:adapter removed\n", __func__);
		ba->status &= ~BA70X_STATUS_PLUGIN;
		schedule_work(&ba->adapter_out_work);
	} else if (ba->vbus_type != BA70X_VBUS_NONE && ba->vbus_type != BA70X_VBUS_OTG && !(ba->status & BA70X_STATUS_PLUGIN)) {
		dev_info(ba->dev, "%s:adapter plugged in\n", __func__);
		ba->status |= BA70X_STATUS_PLUGIN;
		schedule_work(&ba->adapter_in_work);
	}
//uu?pg:0b-41
	if ((status & BA70X_PG_STAT_MASK) && !(ba->status & BA70X_STATUS_PG))
		ba->status |= BA70X_STATUS_PG;
	else if (!(status & BA70X_PG_STAT_MASK) && (ba->status & BA70X_STATUS_PG))
		ba->status &= ~BA70X_STATUS_PG;
//uu?
	if (fault && !(ba->status & BA70X_STATUS_FAULT))
		ba->status |= BA70X_STATUS_FAULT;
	else if (!fault && (ba->status & BA70X_STATUS_FAULT))
		ba->status &= ~BA70X_STATUS_FAULT;

	ba->interrupt = true;
}

//中断处理函数
static irqreturn_t ba70x_charger1_interrupt(int irq, void *data)
{
	struct ba70x *ba = data;

	schedule_work(&ba->irq_work);
	return IRQ_HANDLED;
}


#define GPIO_IRQ    1159

//驱动加载函数
static int ba70x_charger1_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct ba70x *ba;
	int irqn;
	int irq_gpio = 0;
	int ret;
	//struct device_node *np;

	printk("[OBEI]-call_function_charger1_probe]");
	//给自定义结构体分配内存
	ba = devm_kzalloc(&client->dev, sizeof(struct ba70x), GFP_KERNEL);
	if (!ba) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	ba->dev = &client->dev;
	ba->client = client;
	i2c_set_clientdata(client, ba);//设置自定义数据的指针

	//这个函数可以用来检测芯片是否是BA70
	ret = ba70x_detect_device(ba);
	if (!ret && ba->part_no == BA70) {
		ba->status |= BA70X_STATUS_EXIST;
		dev_info(ba->dev, "[OBEI]%s: charger device ba25890 detected, revision:%d\n", __func__, ba->revision);
	} else {
		dev_info(ba->dev, "[OBEI]%s: no ba25890 charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}
	printk("[OBEI]-call_supply_get_by_name]");
	//获取psy的指针
	ba->batt_psy = power_supply_get_by_name("battery");

	g_ba1 = ba;

    //np = client->dev->of_node;
	//解析设备树
	if (client->dev.of_node)
		 ba70x_parse_dt(&client->dev, g_ba1);

	//初始化设备(给寄存器写入默认值)
	ret = ba70x_init_device(g_ba1);
	if (ret) {
		dev_err(ba->dev, "device init failure: %d\n", ret);
		goto err_0;
	}
	printk("[OBEI]-call_REQ_IO]");
	//申请一个 GPIO 管脚
	//if (np)
	//	irq_gpio = of_get_named_gpio(np, "ba70,irq_pin", 0);
	//else
		irq_gpio = GPIO_IRQ;
		ba->gpio_irq = irq_gpio;
	dev_info(ba->dev, "gpio = %d \n", irq_gpio);
	// if (irq_gpio > 0) {
	// 	ba->gpio_irq = irq_gpio;
	// 	ret = gpio_request(ba->gpio_irq, "ba70x_irq_pin");
	// 	if (ret) {
	// 		dev_err(ba->dev, "%s: %d gpio request failed\n", __func__, ba->gpio_irq);
	// 		goto err_0;
	// 	}
	// }
	// else{
	// 	printk("[OBEI]-call_of_get_named_gpio_fail]");
	// 	goto err_0;
	// }
	ret = gpio_request(ba->gpio_irq, "ba70x_irq_pin");
	if (ret) {
	 	dev_err(ba->dev, "%s: %d gpio request failed\n", __func__, ba->gpio_irq);
	 	goto err_0;
	}
	//设置某个 GPIO 为输入
	gpio_direction_input(ba->gpio_irq);
	//将当前已经申请的GPIO号转换为IRQ号，也就是获取当前GPIO的中断线，函数调用成功后，将返回对应的IRQ号
	irqn = gpio_to_irq(ba->gpio_irq);
	if (irqn < 0) {
		dev_err(ba->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;

	printk("[OBEI]-call_PSY_REGISTER]");
	//psy注册
	ret = ba70x_psy_register(ba);
	if (ret)
		goto err_0;

	//初始化
	INIT_WORK(&ba->irq_work,         ba70x_charger1_irq_workfunc);
	INIT_WORK(&ba->adapter_in_work,  ba70x_adapter_in_workfunc);
	INIT_WORK(&ba->adapter_out_work, ba70x_adapter_out_workfunc);
	
	INIT_DELAYED_WORK(&ba->monitor_work,         ba70x_monitor_workfunc);
	INIT_DELAYED_WORK(&ba->ico_work,             ba70x_ico_workfunc);
	INIT_DELAYED_WORK(&ba->pe_volt_tune_work,    ba70x_pe_tune_volt_workfunc);
	INIT_DELAYED_WORK(&ba->check_pe_tuneup_work, ba70x_check_pe_tuneup_workfunc);
	INIT_DELAYED_WORK(&ba->charger2_enable_work, ba70x_charger2_enable_workfunc);

	ret = sysfs_create_group(&ba->dev->kobj, &ba70x_attr_group);
	if (ret) {
		dev_err(ba->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}
	printk("[OBEI]-call_REQUEST_IRQ]");
	//申请中断处理函数
	ret = request_irq(client->irq, ba70x_charger1_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ba70x_charger1_irq", ba);
	if (ret) {
		dev_err(ba->dev, "%s:Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		dev_info(ba->dev, "%s:irq = %d\n", __func__, client->irq);
	}


	pe.enable = true;
	/*in case of adapter has been in when power off*/
	printk("[OBEI]-call_IRQ_WORK]");
	schedule_work(&ba->irq_work);
	return 0;

err_irq:
	cancel_work_sync(&ba->irq_work);
err_1:
	gpio_free(ba->gpio_irq);
err_0:
	g_ba1 = NULL;
	printk("[OBEI]-PROBE_FAIL]");
	return ret;
}
//驱动关闭时调用
static void ba70x_charger1_shutdown(struct i2c_client *client)
{
	struct ba70x *ba = i2c_get_clientdata(client);

	dev_info(ba->dev, "%s: shutdown\n", __func__);

	ba70x_psy_unregister(ba);

	sysfs_remove_group(&ba->dev->kobj, &ba70x_attr_group);
	cancel_work_sync(&ba->irq_work);
	cancel_work_sync(&ba->adapter_in_work);
	cancel_work_sync(&ba->adapter_out_work);
	cancel_delayed_work_sync(&ba->monitor_work);
	cancel_delayed_work_sync(&ba->ico_work);
	cancel_delayed_work_sync(&ba->check_pe_tuneup_work);
	cancel_delayed_work_sync(&ba->charger2_enable_work);
	cancel_delayed_work_sync(&ba->pe_volt_tune_work);

	free_irq(ba->client->irq, NULL);
	gpio_free(ba->gpio_irq);

	g_ba1 = NULL;
}

/* interface for other module end */
static void ba70x_charger2_irq_workfunc(struct work_struct *work)
{
	struct ba70x *ba = container_of(work, struct ba70x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;


	/* Read STATUS and FAULT registers */
	ret = ba70x_read_byte(ba, &status, BA70X_REG_94);//uuc 0b-94??? bus_stat
	if (ret)
		return;

	ret = ba70x_read_byte(ba, &fault, BA70X_REG_0C);
	if (ret)
		return;

	if (((status & BA70X_VBUS_STAT_MASK) == 0) && (ba->status & BA70X_STATUS_PLUGIN))
		ba->status &= ~BA70X_STATUS_PLUGIN;
    else if ((status & BA70X_VBUS_STAT_MASK) && !(ba->status & BA70X_STATUS_PLUGIN))
		ba->status |= BA70X_STATUS_PLUGIN;

	if ((status & BA70X_PG_STAT_MASK) && !(ba->status & BA70X_STATUS_PG))
		ba->status |= BA70X_STATUS_PG;
	else if (!(status & BA70X_PG_STAT_MASK) && (ba->status & BA70X_STATUS_PG))
		ba->status &= ~BA70X_STATUS_PG;

	if (fault && !(ba->status & BA70X_STATUS_FAULT))
		ba->status |= BA70X_STATUS_FAULT;
	else if (!fault && (ba->status & BA70X_STATUS_FAULT))
		ba->status &= ~BA70X_STATUS_FAULT;

	ba->interrupt = true;

}



static void ba70x_charger2_shutdown(struct i2c_client *client)
{
	struct ba70x *ba = i2c_get_clientdata(client);

	dev_info(ba->dev, "%s: shutdown\n", __func__);
	cancel_work_sync(&ba->irq_work);
	g_ba2 = NULL;
}

static struct of_device_id ba70x_charger1_match_table[] = {
	{.compatible = "sy,ba70x",},
	{},
};


static const struct i2c_device_id ba70x_charger1_id[] = {
	{ "ba70x", 0x7D },
	{},
};

MODULE_DEVICE_TABLE(i2c, ba70x_charger1_id);

static struct i2c_driver ba70x_charger1_driver = {
	.driver		= {
		.name	= "ba70x",
		.of_match_table = ba70x_charger1_match_table,
	},
	.id_table	= ba70x_charger1_id,

	.probe		= ba70x_charger1_probe,
	.shutdown   = ba70x_charger1_shutdown,
};





MODULE_DEVICE_TABLE(i2c, ba70x_charger2_id);



  //先注释掉，避免编译不通过
static struct i2c_board_info __initdata i2c_ba70x_charger1[] = {
	{
		I2C_BOARD_INFO("ba70x", 0x7D),
	},
};



static int __init ba70x_charger_init(void)
{

	//先注释掉下面2行代码，避免编译不通过
	i2c_register_board_info(0, i2c_ba70x_charger1, ARRAY_SIZE(i2c_ba70x_charger1));
	

	if (i2c_add_driver(&ba70x_charger1_driver))
		printk("[OBEI]%s, failed to register ba70x_charger1_driver.\n", __func__);
	else
		printk("[OBEI]%s, ba70x_charger1_driver register successfully!\n", __func__);



	return 0;
}

static void __exit ba70x_charger_exit(void)
{
	i2c_del_driver(&ba70x_charger1_driver);
	
}

module_init(ba70x_charger_init);
module_exit(ba70x_charger_exit);

MODULE_DESCRIPTION("SY BA70X Dual Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Silergy");
