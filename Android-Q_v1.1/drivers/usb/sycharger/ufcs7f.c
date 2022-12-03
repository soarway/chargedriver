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

#include "symaster.h"
#include "ufcs7f.h"




struct uf7f {
	struct power_supply			*charger;
	struct power_supply_desc	 charger_desc;
	struct i2c_client			*client;
	struct device               *dev;
	struct mutex			lock;
	struct gpio_desc		*status_gpio;
	struct delayed_work		poll;
	u32						poll_interval;
	bool					charging;

	struct notifier_block   uf7f_nb;
	struct symaster_device  *sydev;
};

/* Please handle the notification in notifier call function,
 * User should control the Power here when you got SOURCE_VBUS notification
 * and SINK_VBUS notification
 */
static int uf7f_event_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	//struct tcp_notify *tcp_noti = data;

	switch (event) {
	case SY_NOTIFY_UF7F_ENABLE_CHARGER:
		pr_info("[OBEI][uf7f]: event call enable charger\n");
		break;
	case SY_NOTIFY_UF7F_DISABLE_CHARGER:
		pr_info("[OBEI][uf7f]: event call disable charger\n");
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}


static int uf7f_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct uf7f *charger;
	//struct power_supply_desc *supply_desc;
	//struct power_supply_config psy_cfg = {};
	//char *name;

	pr_info("[OBEI][uf7f]charger probe.\n");

	//分配自定义结构体的内存
	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	//注册事件回调函数
	charger->uf7f_nb.notifier_call = uf7f_event_notifer_call;
	ret = sy_register_notifier(charger->sydev, &charger->uf7f_nb);
	if (ret < 0) {
		pr_info("[OBEI][uf7f]register  notifer fail\n");
		return -EINVAL;
	}

	return 0;
}

static int uf7f_charger_remove(struct i2c_client *client)
{
	//struct uf7f *charger = i2c_get_clientdata(client);

	//if (charger->poll_interval)
	//	cancel_delayed_work_sync(&charger->poll);//取消定时器

	printk("[OBEI][uf7f]uf7f charger remove.\n");
	return 0;
}


//------------------------------------------------UFCS7F--------------------------------------------
static struct of_device_id    uf7f_charger_match_table[] = {
	{.compatible = SY_UF7F,},
	{},
};

static const struct i2c_device_id   uf7f_charger_id[] = {
	{ SY_UF7F, 3 },
	{},
};

MODULE_DEVICE_TABLE(i2c, uf7f_charger_id);


static struct i2c_driver   uf7f_charger_driver = {
	.driver		= {
		.name	= SY_UF7F,
		.of_match_table = uf7f_charger_match_table,
	},

	.id_table	= uf7f_charger_id,

	.probe	= uf7f_charger_probe,
	.remove = uf7f_charger_remove,
};

static struct i2c_board_info __initdata i2c_uf7f_charger[] = {
	{
		I2C_BOARD_INFO(SY_UF7F, 0x7F),
	},
};


static int __init uf7f_charger_init(void)
{
	int ret;

	ret = i2c_register_board_info(3, i2c_uf7f_charger, ARRAY_SIZE(i2c_uf7f_charger));
	if (ret) {
		printk("[OBEI][uf7f]register board info fail.\n");
	}

	//添加bc1.2驱动
	if (i2c_add_driver(&uf7f_charger_driver))
		printk("[OBEI][uf7f]failed to register uf7f_driver.\n");
	else
		printk("[OBEI][uf7f]driver register successfully!\n");



	return 0;
}

static void __exit uf7f_charger_exit(void)
{
	i2c_del_driver(&uf7f_charger_driver);

}

module_init(uf7f_charger_init);
module_exit(uf7f_charger_exit);

MODULE_DESCRIPTION("SY UF7F  Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");







