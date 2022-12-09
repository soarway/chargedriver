#include <linux/miscdevice.h>
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


struct master_platform {
	uint32_t charge_current;
	uint32_t charge_voltage;
	uint32_t input_current;
	uint32_t protocol_priority;//协议优先级
	const char *name;

	bool ext_control;

	char **supplied_to;
	size_t num_supplicants;
};

struct symaster {
	struct power_supply			*charger;
	struct power_supply_desc	 charger_desc;
	struct i2c_client			*client;
	struct device               *dev;
	struct master_platform		*pdata;
	struct mutex			lock;
	struct gpio_desc		*status_gpio;
	struct delayed_work		poll;
	u32						poll_interval;
	bool					charging;

	struct symaster_device *master_dev;
	struct notifier_block   master_nb;
};



//使能充电器：先初始化寄存器的值，然后再获取对充电器的控制，就是往BQ24735_CHG_OPT=0x12寄存器写入数据
static inline int master_enable_charging(struct symaster *charger)
{
	//struct tcp_notify tcp_noti;

	struct symaster_device* sydev = charger->master_dev;

	return srcu_notifier_call_chain(&sydev->master_event, SY_NOTIFY_BC7D_ENABLE_CHARGER, sydev);
}

//消能充电器：
static inline int master_disable_charging(struct symaster *charger)
{
	struct symaster_device* sydev = charger->master_dev;

	return srcu_notifier_call_chain(&sydev->master_event, SY_NOTIFY_BC7D_DISABLE_CHARGER, sydev);
}


static inline struct symaster *to_master(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

static enum power_supply_property master_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

//https://zhuanlan.zhihu.com/p/512249351
//判断充电器是否存在，查看pdf文件中的 0x12H 寄存器的值
static bool master_charger_is_present(struct symaster *charger)
{
	return false;
}

//更新充电设备状态
static void master_update(struct symaster *charger)
{
	mutex_lock(&charger->lock);

	if (master_charger_is_present(charger))
		master_enable_charging(charger);
	else
		master_disable_charging(charger);

	mutex_unlock(&charger->lock);

	power_supply_changed(charger->charger);
}



//中断处理回调函数
static irqreturn_t master_irq_handler(int irq, void *devid)
{
	struct power_supply *psy = devid;
	struct symaster *charger = to_master(psy);
	struct symaster_device* sydev = charger->master_dev;
	//printk("[OBEI][master]irq handler\n");
	//master_update(charger);
	

	//发一个消息到BC7D, 让其返回是否满足DCP特性
	srcu_notifier_call_chain(&sydev->master_event, SY_NOTIFY_BC7D_CHECK_DCP, sydev);

	return IRQ_HANDLED;
}

//定时器处理函数(没有设置中断的时候会使用此定时器函数)
static void master_work_poll(struct work_struct *work)
{
	struct symaster *charger = container_of(work, struct symaster, poll.work);

	master_update(charger);
	printk("[OBEI][master]work poll\n");
	schedule_delayed_work(&charger->poll, msecs_to_jiffies(charger->poll_interval));
}



//获取充电设备的属性值
static int master_charger_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	struct symaster *charger = to_master(psy);
	return 0;
}

//设置充电器属性：设置充电状态，取消充电状态
static int master_charger_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val)
{
	struct symaster *charger = to_master(psy);

	return 0;
}



//解析设备树，读取设备树的属性值
static struct master_platform *parse_dt_data(struct i2c_client *client)
{
	struct master_platform *pdata = 0;
	struct device_node *np = client->dev.of_node;
	u32 val;
	int ret;


	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		printk("[OBEI][master]Memory alloc for pdata failed\n");
		return NULL;
	}

	pdata->protocol_priority = 0;
	pdata->charge_voltage = 0;
	
	ret = of_property_read_u32(np, "priority_proto", &val);
	if (!ret)
	{
		pdata->protocol_priority = val;
	}
	

	pdata->charge_current = 2000;

	ret = of_property_read_u32(np, "sy,ba70x,charge-voltage", &val);
	if (!ret)
		pdata->charge_voltage = val;

	//ret = of_property_read_u32(np, "ti,input-current", &val);
	//if (!ret)
		pdata->input_current = 500;

	pdata->ext_control = true;//of_property_read_bool(np, "ti,external-control");

	printk("[OBEI][master]read protocol_priority=%d, charge_voltage=%d\n", val, pdata->charge_voltage);

	return pdata;
}

static int master_charger_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return 0;
}




static int master_event_notifer_call(struct notifier_block *nb, unsigned long event, void *data)
{
	//struct symaster_device* sydev;
	//struct symaster * chip = container_of(nb, struct symaster, master_nb);

	//struct tcp_notify *tcp_noti = data;

	switch (event) {
	case SY_NOTIFY_MASTER_ENABLE_CHARGER:
		pr_info("[OBEI][master]: event call enable charger\n");
		break;
	case SY_NOTIFY_MASTER_DISABLE_CHARGER:
		pr_info("[OBEI][master]: event call disable charger\n");
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

//设备初始化
static int master_device_init(struct symaster *chip, struct device *dev)
{
	struct symaster_device *mdev;
	//struct bc7d_device* bc7ddev;


	pr_info("[OBEI][master] device init (%s)\n",  MASTER_DEVICE_NAME);

	mdev = sy_device_register(dev, MASTER_DEVICE_NAME, chip);
	if (!mdev) {
		pr_err("[OBEI][master] : allocate mdev memeory failed\n");
		return -1;
	}
	chip->master_dev = mdev;


	return 0;

}


//驱动匹配后被系统内核调用的初始化函数
static int master_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret, irqn;
	struct symaster *charger;
	struct power_supply_desc *supply_desc;
	struct power_supply_config psy_cfg = {};
	char *name;

	pr_info("[OBEI][master]call probe start!\n");
	//分配自定义结构体的内存
	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	//初始化锁
	mutex_init(&charger->lock);
	charger->charging = true;//要求进入充电状态
	charger->pdata = client->dev.platform_data;
	
	if (IS_ENABLED(CONFIG_OF) && !charger->pdata && client->dev.of_node)
	{
		charger->pdata = parse_dt_data(client);
		printk("[OBEI][master]pdata success\n");
	}

	if (!charger->pdata) {
		printk("[OBEI][master]no platform data provided\n");
		return -EINVAL;
	}
	

	name = (char *)charger->pdata->name;
	if (!name) {
		name = devm_kasprintf(&client->dev, GFP_KERNEL, "symaster@%s", dev_name(&client->dev));
		if (!name) {
			printk("[OBEI][master]Failed to alloc device name\n");
			return -ENOMEM;
		}
		else {
			printk("[OBEI][master]device name=%s\n", name);
		}
	}

	charger->client = client;
	charger->dev = &client->dev;

	supply_desc = &charger->charger_desc;

	supply_desc->name = name;
	supply_desc->type = POWER_SUPPLY_TYPE_MAINS;
	supply_desc->properties = master_charger_properties;
	supply_desc->num_properties = ARRAY_SIZE(master_charger_properties);
	supply_desc->get_property = master_charger_get_property;
	supply_desc->set_property = master_charger_set_property;
	supply_desc->property_is_writeable = master_charger_property_is_writeable;

	psy_cfg.supplied_to = charger->pdata->supplied_to;
	psy_cfg.num_supplicants = charger->pdata->num_supplicants;
	psy_cfg.of_node  = client->dev.of_node;
	psy_cfg.drv_data = charger;

	//注册结构体的数据指针到系统内核中
	i2c_set_clientdata(client, charger);


	ret = master_device_init(charger, &client->dev);
	if (ret < 0) {
		printk("[OBEI][master] dev init fail\n");
		return ret;
	}

	
	/*
	//获取GPIO的状态
	charger->status_gpio = devm_gpiod_get_optional(&client->dev, "ti,ac-detect", GPIOD_IN);
	if (IS_ERR(charger->status_gpio)) {
		ret = PTR_ERR(charger->status_gpio);
		printk("[OBEI][master]Getting gpio failed: %d\n", ret);
		return ret;
	}
	else {
		printk("[OBEI][master]Getting gpio success\n");
	}
	*/
	//充电器是否存在
	if (master_charger_is_present(charger)) {
		/*
		//使能充电器
		ret = bq24735_enable_charging(charger);
		if (ret < 0) {
			printk("Failed to enable charging\n");
			return ret;
		}
		*/
	}

	//注册驱动为一个power_supply设备
	charger->charger = devm_power_supply_register(&client->dev, supply_desc, &psy_cfg);
	if (IS_ERR(charger->charger)) {
		ret = PTR_ERR(charger->charger);
		printk("[OBEI][master]Failed to register power supply: %d\n", ret);
		return ret;
	}
	else
	{
		printk("[OBEI][master]Success to register power supply\n");
	}

	//中断处理
	ret = gpio_request(GPIO_IRQ, "symaster_irq_pin");
	if (ret) {
		printk("[OBEI][master] gpio request failed\n");
		return ret;
	}
	gpio_direction_input(GPIO_IRQ);

	irqn = gpio_to_irq(GPIO_IRQ);
	if (irqn < 0) {
		printk("[OBEI][master]%d gpio_to_irq failed\n", irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;


	//支持中断
	//内核提供 request_threaded_irq 和 devm_request_threaded_irq 为中断分配内核线程
	//若flags中设置IRQF_ONESHOT标志，内核会自动在中断上下文中屏蔽该中断号
	//bq24735_charger_isr 为中断处理函数
	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, master_irq_handler, 
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, supply_desc->name, charger->charger);
		if (ret) {
			printk("[OBEI][master]Unable to request IRQ %d err %d\n", client->irq, ret);
			return ret;
		}
		else {
			pr_info("[OBEI][master]request IRQ success!\n");
		}
	} 
	else {
		//不支持中断，通过定时轮询来进入主循环

		charger->poll_interval = 100;

		INIT_DELAYED_WORK(&charger->poll, master_work_poll);
		schedule_delayed_work(&charger->poll, msecs_to_jiffies(charger->poll_interval));
	}


	//注册事件回调函数
	charger->master_nb.notifier_call = master_event_notifer_call;
	ret = sy_register_notifier(charger->master_dev, &charger->master_nb);
	if (ret < 0) {
		printk("[OBEI][master]register master_dev notifer fail\n");
		return -EINVAL;
	}

	pr_info("[OBEI][master]call probe success!\n");
	return 0;

err_1:
	gpio_free(GPIO_IRQ);
	return 0;
}

static int master_charger_remove(struct i2c_client *client)
{
	struct symaster *charger = i2c_get_clientdata(client);

	if (charger->poll_interval)
		cancel_delayed_work_sync(&charger->poll);//取消定时器

	return 0;
}

//---------------------------------------symaster--------------------------------------------
static struct of_device_id   master_charger_match_table[] = {
	{.compatible = SY_MASTER,},
	{},
};

static const struct i2c_device_id  master_charger_id[] = {
	{ SY_MASTER, 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, master_charger_id);

static struct i2c_driver   master_charger_driver = {
	.driver		= {
		.name	= SY_MASTER,
		.of_match_table = master_charger_match_table,
	},
	.id_table	= master_charger_id,

	.probe	= master_charger_probe,
	.remove = master_charger_remove
	//.shutdown   = master_charger_shutdown,
};

/*
static struct file_operations dev_fops = {
	.owner=THIS_MODULE
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "symasterba76",
	.fops = &dev_fops,
};
*/
//---------------------------------------------------------------------------------------------------

static int __init master_charger_init(void)
{
	int ret;

	struct device_node *np2;
	

	np2 = of_find_node_by_name(NULL, "symaster");
	if (np2 != NULL)
		pr_info("[OBEI][master]symaster node found...\n");
	else
		pr_info("[OBEI][master]symaster node not found...\n");

	//添加主驱动
	if (i2c_add_driver(&master_charger_driver))
		printk("[OBEI][master]failed to register symaster_driver.\n");
	else
		printk("[OBEI][master]driver register successfully!\n");

	

	return 0;
}

static void __exit master_charger_exit(void)
{
	i2c_del_driver(&master_charger_driver);

	//misc_deregister(&misc);
}

subsys_initcall(master_charger_init);
module_exit(master_charger_exit);

MODULE_DESCRIPTION("SY MASTER Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");
