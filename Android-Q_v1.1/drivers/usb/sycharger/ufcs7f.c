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

//------------------------------------------------UFCS7F--------------------------------------------
static struct of_device_id    uf7f_charger_match_table[] = {
	{.compatible = SY_UF7F,},
	{},
};

static const struct i2c_device_id   uf7f_charger_id[] = {
	{ SY_UF7F, BQ25892 },
	{},
};

MODULE_DEVICE_TABLE(i2c, uf7f_charger_id);


static struct i2c_driver   uf7f_charger_driver = {
	.driver		= {
		.name	= SY_UF7F,
		.of_match_table = uf7f_charger_match_table,
	},

	.id_table	= uf7f_charger_id,

	.probe		= uf7f_charger_probe,
	.shutdown   = uf7f_charger_shutdown,
};

static struct i2c_board_info __initdata i2c_uf7f_charger[] = {
	{
		I2C_BOARD_INFO(SY_UF7F, 0x7F),
	},
};






static int __init uf7f_charger_init(void)
{

	i2c_register_board_info(0, i2c_uf7f_charger,   ARRAY_SIZE(i2c_uf7f_charger));



	//添加bc1.2驱动
	if (i2c_add_driver(&uf7f_charger_driver))
		printk("[OBEI] failed to register pd7e_driver.\n");
	else
		printk("[OBEI] pd7e_driver register successfully!\n");



	return 0;
}

static void __exit pd7e_charger_exit(void)
{
	i2c_del_driver(&uf7f_charger_driver);

}

module_init(uf7f_charger_init);
module_exit(uf7f_charger_exit);

MODULE_DESCRIPTION("SY UF7F  Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("DAVID");







