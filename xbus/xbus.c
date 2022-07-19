#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>



int xbus_match(struct device *dev, struct device_driver *drv)
{
    if (strcmp(dev_name(dev), drv->name) == 0)   /* success to match */
    {
        printk("xbus device & driver match success\r\n");
        return 1;        
    }

    return 0;
}

static struct bus_type xbus = {
    .name = "xbus",
    .match = xbus_match,
};

EXPORT_SYMBOL(xbus);  

static int __init  xbus_init(void)
{
    int ret = bus_register(&xbus);       
    printk("register xbus\r\n");
    return ret;
}

static void __exit xbus_exit(void)
{
    bus_unregister(&xbus);
    printk("unregsiter xbus\r\n");
}

module_init(xbus_init);
module_exit(xbus_exit);

MODULE_LICENSE("GPL"); 





