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


#include "pd7e.h"
#include "symaster.h"

enum pd7e_fields {
	E_VID_LBYTE, /* Reg00 */
	E_VID_HBYTE, /* Reg01 */
	E_PID_LBYTE,/* Reg02 */
	E_PID_HBYTE,/* Reg03 */
	E_DID_LBYTE,/* Reg04 */
	E_DID_HBYTE,/* Reg05 */
	E_USBTYPEC_REV,/* Reg06 */

	E_USBPD_VER,/* Reg08 */
	E_USBPD_REV,/* Reg09 */

	E_PDIF_VER,/* Reg0A */
	E_PDIF_REV,/* Reg0B */

	TX_SUCCESS,TX_DISCARD,TX_FAIL,RX_HARD_RESET,RX_SOP_MSG_STATUS,POWER_STATUS,CC_STATUS,PDIF_REV,/* Reg10 */
	RXBUF_OVFLOW,FAULT,/* Reg11 */
	M_TX_SUCCESS,M_TX_DISCARD,M_TX_FAIL,M_RX_HARD_RESET,M_RX_SOP_MSG_STATUS,M_POWER_STATUS,M_CC_STATUS,/* Reg12 */
	M_RXBUF_OVFLOW,M_FAULT,/* Reg13 */
	M_TCPC_INITIAL,M_VBUS_PRESENT,M_VCONN_PRESENT,/* Reg14 */
	M_VCON_OV,M_VCON_OC,M_I2C_ERROR,/* Reg15 */
	WDT_MON_EN,BIST_TEST_MODE,PLUG_ORIENT,/* Reg19 */
	DRP,RP_VALUE,CC2,CC1,/* Reg1A */
	DIS_VCON_OV,DIS_VCON_OC,/* Reg1B */
	EN_VCONN,/* Reg1C */
	DRP_STATUS,DRP_RESULT,CC2_STATUS,CC1_STATUS,/* Reg1D */
	TCPC_INITIAL,VBUS_PRESENT,VCONN_PRESENT,/* Reg1E */
	VCONN_OV,VCONN_OC,I2C_ERROR,/* Reg1F */
	COMMAND,/* Reg23 */
	ROLES_SUPPORT,ALL_SOP_SUPPORT,SOURCE_VCONN,CPB_SINK_VBUS,SOURCE_HV_VBUS,/* Reg24 */
	CPB_VBUS_OC,CPB_VBUS_OV,CPB_BLEED_DISC,CPB_FORCE_DISC,VBUS_MEASURE_ALARM,SOURCE_RPSUPPORT,/* Reg25 */
	SINK_DISCONNECT_DET,STOP_DISC_THD,VBUS_VOL_ALARM_LSB,VCONN_POWER,VCONN_OCF, /* Reg26 */
	WDT_TMR,/* Reg27 */
	VBUS_EXT_OVF,VBUS_EXT_OCF,FORCE_OFF_VBUS_IN,/* Reg28 */
	CPB_DBG_ACC_IND,CPB_VBUS_PRESENT_MN,CPB_AUDIO_ADT_ACC_IND,CPB_ACTIVE_CABLE_INDET,CPB_MUX_CFG_CTRL,CPB_CONNECT_PRESENT,CPB_CONNECT_ORIENT,/* Reg29 */

	CABLE_PLUG,DATA_ROLE,USBPD_SPECREV,POWER_ROLE,/* Reg2E */
	EN_CABLE_RST,EN_HARD_RST,EN_SOP2DB,EN_SOP1DB,EN_SOP2,EN_SOP1,EN_SOP0,/* Reg2F */
	RX_BYTE_COUNT,/* Reg30 */
	RX_BUF_FRAME_TYPE,/* Reg31 */
	RX_HEAD_0,/* Reg32 */
	RX_HEAD_1,/* Reg33 ------------*/
	RX_OBJ1_0,/* Reg34 */
	RX_OBJ1_1,/* Reg35   OBJ1*/
	RX_OBJ1_2,/* Reg36 */
	RX_OBJ1_3,/* Reg37 ------------*/
	RX_OBJ2_0,/* Reg38 */
	RX_OBJ2_1,/* Reg39   OBJ2*/
	RX_OBJ2_2,/* Reg3A */
	RX_OBJ2_3,/* Reg3B ------------*/
	RX_OBJ3_0,/* Reg3C */
	RX_OBJ3_1,/* Reg3D   OBJ3*/
	RX_OBJ3_2,/* Reg3E */
	RX_OBJ3_3,/* Reg3F ------------*/
	RX_OBJ4_0,/* Reg40 */
	RX_OBJ4_1,/* Reg41   OBJ4*/
	RX_OBJ4_2,/* Reg42 */
	RX_OBJ4_3,/* Reg43 ------------*/
	RX_OBJ5_0,/* Reg44 */
	RX_OBJ5_1,/* Reg45   OBJ5*/
	RX_OBJ5_2,/* Reg46 */
	RX_OBJ5_3,/* Reg47 ------------*/
	RX_OBJ6_0,/* Reg48 */
	RX_OBJ6_1,/* Reg49   OBJ6*/
	RX_OBJ6_2,/* Reg4A */
	RX_OBJ6_3,/* Reg4B ------------*/
	RX_OBJ7_0,/* Reg4C */
	RX_OBJ7_1,/* Reg4D   OBJ7*/
	RX_OBJ7_2,/* Reg4E */
	RX_OBJ7_3,/* Reg4F ------------*/
	TX_RETRY_CNT,TX_BUF_FRAME_TYPE,/* Reg50 */
	TX_BYTE_COUNT,/* Reg51 */
	TX_HEAD_0,/* Reg52 */
	TX_HEAD_1,/* Reg53 ------------*/
	TX_OBJ1_0,/* Reg54 */
	TX_OBJ1_1,/* Reg55   OBJ1*/
	TX_OBJ1_2,/* Reg56 */
	TX_OBJ1_3,/* Reg57 ------------*/
	TX_OBJ2_0,/* Reg58 */
	TX_OBJ2_1,/* Reg59   OBJ2*/
	TX_OBJ2_2,/* Reg5A */
	TX_OBJ2_3,/* Reg5B ------------*/
	TX_OBJ3_0,/* Reg5C */
	TX_OBJ3_1,/* Reg5D   OBJ3*/
	TX_OBJ3_2,/* Reg5E */
	TX_OBJ3_3,/* Reg5F ------------*/
	TX_OBJ4_0,/* Reg60 */
	TX_OBJ4_1,/* Reg61   OBJ4*/
	TX_OBJ4_2,/* Reg62 */
	TX_OBJ4_3,/* Reg63 ------------*/
	TX_OBJ5_0,/* Reg64 */
	TX_OBJ5_1,/* Reg65   OBJ5*/
	TX_OBJ5_2,/* Reg66 */
	TX_OBJ5_3,/* Reg67 ------------*/
	TX_OBJ6_0,/* Reg68 */
	TX_OBJ6_1,/* Reg69   OBJ6*/
	TX_OBJ6_2,/* Reg6A */
	TX_OBJ6_3,/* Reg6B ------------*/
	TX_OBJ7_0,/* Reg6C */
	TX_OBJ7_1,/* Reg6D   OBJ7*/
	TX_OBJ7_2,/* Reg6E */
	TX_OBJ7_3,/* Reg6F ------------*/

	VCONN_DISCHARGE_EN,BMCIO_LPEN,/* Reg90 */
	VCONN_OCP,/* Reg93 */
	VBUS_80,/* Reg97 */
	INT_RESET,INT_RA_DETACH,CC_OVP_INT,INT_VBUS_80,INT_WAKEUP,/* Reg98 */
	M_RESET,M_RA_DETACH,M_CC_OVP,M_VBUS_80,M_WAKEUP,/* Reg99 */
	CK_300K_SEL,SHUTDOWN_OFF,ENEXTMSG,AUTOIDLE_EN,AUTOIDLE_TIMEOUT,/* Reg9B */
	F_MAX_FIELDS
};


static const struct reg_field pd7e_reg_fields[] = {
	/* REG00 */
	[E_VID_LBYTE]	= REG_FIELD(0x00, 0, 7),
	/* REG01 */
	[E_VID_HBYTE]	= REG_FIELD(0x04, 7, 7),
	/* REG02 */
	[E_PID_LBYTE]	= REG_FIELD(0x04, 4, 6),
	/* REG03 */
	[E_PID_HBYTE]	= REG_FIELD(0x04, 2, 2),
	/* REG04 */
	[E_DID_LBYTE]	= REG_FIELD(0x04, 0, 1),
	/* Reg05 */
	[E_DID_HBYTE] = REG_FIELD(0x99, 7, 7),
	/* REG06 */
	[E_USBTYPEC_REV] = REG_FIELD(0x99, 4, 4),
	/* REG07 */
	[E_USBPD_VER] = REG_FIELD(0x99, 3, 3),
	/* REG08 */
	[E_USBPD_REV] = REG_FIELD(0x99, 2, 2),
	/* REG09 */
	[E_PDIF_VER] = REG_FIELD(0x99, 1, 1),
	/* REG0A*/
	[E_PDIF_REV] = REG_FIELD(0x99, 0, 0)


};



struct pd7e {
	struct power_supply			*charger;
	struct power_supply_desc	 charger_desc;
	struct i2c_client			*client;
	struct device               *dev;
	struct mutex			lock;
	struct gpio_desc		*status_gpio;
	struct delayed_work		poll;
	u32						poll_interval;
	bool					charging;

	struct notifier_block   pd7e_nb;
	struct symaster_device  *sydev;
};


/* Please handle the notification in notifier call function,
 * User should control the Power here when you got SOURCE_VBUS notification
 * and SINK_VBUS notification
 */
static int pd7e_event_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	//struct tcp_notify *tcp_noti = data;

	switch (event) {
	case SY_NOTIFY_PD7E_ENABLE_CHARGER:
		pr_info("[OBEI][pd7e]: event call enable charger\n");
		break;
	case SY_NOTIFY_PD7E_DISABLE_CHARGER:
		pr_info("[OBEI][pd7e]: event call disable charger\n");
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}



static int pd7e_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct pd7e *charger;
	struct power_supply_desc *supply_desc;
	struct power_supply_config psy_cfg = {};
	char *name;

	pr_info("[OBEI][pd7e]pd7e charger probe.\n");

	//分配自定义结构体的内存
	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	//注册事件回调函数
	charger->pd7e_nb.notifier_call = pd7e_event_notifer_call;
	ret = sy_register_notifier(charger->sydev, &charger->pd7e_nb);
	if (ret < 0) {
		pr_info("[OBEI][pd7e]register notifer fail\n");
		return -EINVAL;
	}


	return 0;
}

static int pd7e_charger_remove(struct i2c_client *client)
{
	struct pd7e *charger = i2c_get_clientdata(client);

	//if (charger->poll_interval)
	//	cancel_delayed_work_sync(&charger->poll);//取消定时器

	pr_info("[OBEI][pd7e]pd7e charger remove.\n");
	return 0;
}





//------------------------------------------------PD7E--------------------------------------------
static struct of_device_id     pd7e_charger_match_table[] = {
	{.compatible = SY_PD7E,},
	{},
};

static const struct i2c_device_id   pd7e_charger_id[] = {
	{ SY_PD7E, 2 },
	{},
};

MODULE_DEVICE_TABLE(i2c, pd7e_charger_id);


static struct i2c_driver   pd7e_charger_driver = {
	.driver		= {
		.name	= SY_PD7E,
		.of_match_table = pd7e_charger_match_table,
	},

	.id_table	= pd7e_charger_id,

	.probe	= pd7e_charger_probe,
	.remove = pd7e_charger_remove,
};

static struct i2c_board_info __initdata i2c_pd7e_charger[] = {
	{
		I2C_BOARD_INFO(SY_PD7E, 0x7E),
	},
};




static int __init pd7e_charger_init(void)
{
	int ret;
	ret = i2c_register_board_info(2, i2c_pd7e_charger,   ARRAY_SIZE(i2c_pd7e_charger));
	if (ret) {
		printk("[OBEI][pd7e]register board info fail.\n");
	}


	//添加bc1.2驱动
	if (i2c_add_driver(&pd7e_charger_driver))
		printk("[OBEI][pd7e]failed to register pd7e_driver.\n");
	else
		printk("[OBEI][pd7e]driver register successfully!\n");



	return 0;
}

static void __exit pd7e_charger_exit(void)
{
	i2c_del_driver(&pd7e_charger_driver);

}

module_init(pd7e_charger_init);
module_exit(pd7e_charger_exit);

MODULE_DESCRIPTION("SY PD7E  Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");




