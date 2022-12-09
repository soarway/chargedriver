
#ifndef __SYMASTER_HEADER__
#define __SYMASTER_HEADER__

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
//#include <linux/wakelock.h>
#include <linux/notifier.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>


#define  BC7D_CHIP_ID   0x66

#define  BC7D_IRQ_PIN   "ba70irq"
#define  PD7E_IRQ_PIN   "ba41irq"
#define  UF7F_IRQ_PIN   "ba52irq"



#define  BC7D_MANUFACTURER		"silergy"

#define  SY_MASTER  "silergy,masterba76"
#define  SY_BC7D    "silergy,ba70x"
#define  SY_PD7E    "silergy,ba41a"
#define  SY_UF7F    "silergy,ba52a"

#define  GPIO_IRQ     1159
#define  MASTER_DEVICE_NAME           "SY_MASTER_DEVICE"
#define  SY_VERSION                   "0.0.9"

enum {
	SY_NOTIFY_MASTER_ENABLE_CHARGER = 1,
	SY_NOTIFY_MASTER_DISABLE_CHARGER,
	SY_NOTIFY_BC7D_ENABLE_CHARGER,
	SY_NOTIFY_BC7D_DISABLE_CHARGER,
	SY_NOTIFY_BC7D_CHECK_DCP,
	SY_NOTIFY_PD7E_ENABLE_CHARGER,
	SY_NOTIFY_PD7E_DISABLE_CHARGER,
	SY_NOTIFY_UF7F_ENABLE_CHARGER,
	SY_NOTIFY_UF7F_DISABLE_CHARGER,

	
};


struct symaster_device {
	struct i2c_client *client;
	struct device dev;
    struct srcu_notifier_head  master_event;
	void *drv_data;
};

struct bc7d_device {
	struct i2c_client* client;
	struct device dev;
	void* drv_data;
};

extern struct symaster_device *sy_dev_get_by_name(const char *name);
extern struct symaster_device *sy_device_register(struct device *parent, const char* name, void *drv_data);
extern int sy_register_notifier(struct symaster_device *sydev, struct notifier_block *nb);
extern int sy_unregister_notifier(struct symaster_device *sydev, struct notifier_block *nb);


#endif


