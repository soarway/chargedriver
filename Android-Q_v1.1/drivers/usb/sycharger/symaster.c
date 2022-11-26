
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




//---------------------------------------symaster--------------------------------------------
static struct of_device_id   master_charger_match_table[] = {
	{.compatible = SY_MASTER,},
	{},
};

static const struct i2c_device_id  master_charger_id[] = {
	{ SY_MASTER, BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, master_charger_id);

static struct i2c_driver   master_charger_driver = {
	.driver		= {
		.name	= SY_MASTER,
		.of_match_table = master_charger_match_table,
	},
	.id_table	= master_charger_id,

	.probe		= master_charger_probe,
	.shutdown   = master_charger_shutdown,
};



//---------------------------------------------------------------------------------------------------

static int __init bq2589x_charger_init(void)
{

	//添加主驱动
	if (i2c_add_driver(&master_charger_driver))
		printk("[OBEI] failed to register symaster_driver.\n");
	else
		printk("[OBEI] symaster_driver register successfully!\n");

	

	return 0;
}

static void __exit master_charger_exit(void)
{
	i2c_del_driver(&master_charger_driver);

}

module_init(master_charger_init);
module_exit(master_charger_exit);

MODULE_DESCRIPTION("SY MASTER Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");
