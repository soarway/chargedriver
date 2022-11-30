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

#include "pd7e.h"
#include "symaster.h"



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
		pr_info("[OBEI]register tcpc notifer fail\n");
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

	i2c_register_board_info(0, i2c_pd7e_charger,   ARRAY_SIZE(i2c_pd7e_charger));



	//添加bc1.2驱动
	if (i2c_add_driver(&pd7e_charger_driver))
		printk("[OBEI][pd7e] failed to register pd7e_driver.\n");
	else
		printk("[OBEI][pd7e] pd7e_driver register successfully!\n");



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




