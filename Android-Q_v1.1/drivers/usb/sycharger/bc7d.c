
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

#include "bc7d.h"
#include "symaster.h"




//--------------------------------------------BC7D-------------------------------------------
static struct of_device_id    bc7d_charger_match_table[] = {
	{.compatible = SY_BC7D,},
	{},
};

static const struct i2c_device_id    bc7d_charger_id[] = {
	{ SY_BC7D, BQ25892 },
	{},
};

MODULE_DEVICE_TABLE(i2c,  bc7d_charger2_id);


static struct i2c_driver   bc7d_charger_driver = {
	.driver		= {
		.name	= SY_BC7D,
		.of_match_table = bc7d_charger_match_table,
	},

	.id_table	= bc7d_charger_id,

	.probe		= bc7d_charger_probe,
	.shutdown   = bc7d_charger_shutdown,
};

static struct i2c_board_info __initdata   i2c_bc7d_charger[] = {
	{
		I2C_BOARD_INFO(SY_BC7D, 0x7D),
	},
};



static int __init bc7d_charger_init(void)
{

	i2c_register_board_info(0, i2c_bc7d_charger,   ARRAY_SIZE(i2c_bc7d_charger));



	//添加bc1.2驱动
	if (i2c_add_driver(&bc7d_charger_driver))
		printk("[OBEI] failed to register bc7d_driver.\n");
	else
		printk("[OBEI] bc7d_driver register successfully!\n");



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



