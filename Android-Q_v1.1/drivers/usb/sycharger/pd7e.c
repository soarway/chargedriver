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



//------------------------------------------------PD7E--------------------------------------------
static struct of_device_id     pd7e_charger_match_table[] = {
	{.compatible = SY_PD7E,},
	{},
};

static const struct i2c_device_id   pd7e_charger_id[] = {
	{ SY_PD7E, BQ25892 },
	{},
};

MODULE_DEVICE_TABLE(i2c, pd7e_charger_id);


static struct i2c_driver   pd7e_charger_driver = {
	.driver		= {
		.name	= SY_PD7E,
		.of_match_table = pd7e_charger_match_table,
	},

	.id_table	= pd7e_charger_id,

	.probe		= pd7e_charger_probe,
	.shutdown   = pd7e_charger_shutdown,
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
		printk("[OBEI] failed to register pd7e_driver.\n");
	else
		printk("[OBEI] pd7e_driver register successfully!\n");



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




