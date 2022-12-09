
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/list.h>

#include "symaster.h"

static struct class *sy_class = NULL;
EXPORT_SYMBOL_GPL(sy_class);

static struct device_type sy_dev_type;

static int sy_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	//用来返回driver的私有数据
	//struct symaster_device *sydev = dev_get_drvdata(dev);

	return strcmp(MASTER_DEVICE_NAME, name) == 0;
}

struct symaster_device *sy_dev_get_by_name(const char *name)
{
	struct symaster_device * sydev = 0;
	struct device *dev = class_find_device(sy_class, NULL, (const void *)name, sy_match_device_by_name);
	sydev = dev ? dev_get_drvdata(dev) : NULL;
	if (sydev != NULL) {
		pr_info("[OBEI][class]class find device name=%s success\n", name);
	}
	else {
		pr_info("[OBEI][class]class find device name=%s fail\n", name);
	}
	return sydev;
}

//作为函数指针赋值给dev.release
static void sy_device_release(struct device *pdev)
{
	struct symaster_device *sydev = container_of(pdev, struct symaster_device, dev);

	pr_info("[OBEI][class]%s :sy device release\n",  dev_name(pdev));

	devm_kfree(pdev, sydev);
}


//设备注册
struct symaster_device *sy_device_register(struct device *parent, const char* name, void *drv_data)
{
	struct symaster_device *symdev;
	int ret = 0;

	pr_info("[OBEI][class] sy register  device (%s) start!\n", name);
	symdev = devm_kzalloc(parent, sizeof(*symdev), GFP_KERNEL);
	if (!symdev) {
		pr_err("[OBEI][class] : allocate dev reg memeory failed\n");
		return NULL;
	}

	symdev->dev.class = sy_class;
	symdev->dev.type = &sy_dev_type;
	symdev->dev.parent = parent;
	symdev->dev.release = sy_device_release;

	dev_set_drvdata(&symdev->dev, symdev);
	symdev->drv_data = drv_data;
	dev_set_name(&symdev->dev, name);


	ret = device_register(&symdev->dev);
	if (ret) {
		kfree(symdev);
		return ERR_PTR(ret);
	}

	srcu_init_notifier_head(&symdev->master_event);

	pr_info("[OBEI][class]sy register device success!\n");
	return symdev;
}
EXPORT_SYMBOL(sy_device_register);
//设备取消注册
void sy_device_unregister(struct device *dev, struct symaster_device *sydev)
{
	if (!sydev)
		return;
	device_unregister(&sydev->dev);
}
EXPORT_SYMBOL(sy_device_unregister);
//注册通知事件
int sy_register_notifier(struct symaster_device *sydev, struct notifier_block *nb)
{
	int ret;
	if (sydev == NULL || nb == NULL) {
		pr_info("[OBEI][class]register notifier error is NULL!\n");
		return -1;
	}
	ret = srcu_notifier_chain_register(&sydev->master_event, nb);
	if (ret != 0)
	{
		pr_info("[OBEI][class]notifier chain register error!\n");
		return ret;
	}	


	return ret;
}
EXPORT_SYMBOL(sy_register_notifier);
//取消注册通知事件
int sy_unregister_notifier(struct symaster_device *sydev, struct notifier_block *nb)
{
	return srcu_notifier_chain_unregister(&sydev->master_event, nb);
}
EXPORT_SYMBOL(sy_unregister_notifier);




void *sy_get_dev_data(struct symaster_device *tcpc)
{
	return tcpc->drv_data;
}
EXPORT_SYMBOL(sy_get_dev_data);



static int __init sy_class_init(void)
{
	sy_class = class_create(THIS_MODULE, "SYCHARGER");
	if (IS_ERR(sy_class)) {
		pr_info("[OBEI]Unable to create class; errno = %ld\n", PTR_ERR(sy_class));
		return PTR_ERR(sy_class);
	}

	pr_info("[OBEI][class]sy class init OK\n");
	return 0;
}

static void __exit sy_class_exit(void)
{
	class_destroy(sy_class);
	pr_info("[OBEI][class]sy class un-init OK\n");
}

subsys_initcall(sy_class_init);
module_exit(sy_class_exit);

MODULE_DESCRIPTION("SY_DEV_CLASS");
MODULE_AUTHOR("DAVID");
MODULE_VERSION(SY_VERSION);
MODULE_LICENSE("GPL");

//这个类做3件事， 一个是创建class，二个是notify event的初始化，三是设备注册


