#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

static int int_param = 250;
module_param(int_param, int, S_IRUGO);

static char *char_param = "module param test";
module_param(char_param, charp, S_IRUGO);

static int __init param_init(void)
{
    printk(KERN_INFO "int param : %d\r\n", int_param);
    printk(KERN_INFO "char pointer param : %s\r\n", char_param);
    return 0;
}

static void __exit param_exit(void)
{
    printk(KERN_INFO "module param exit\r\n");
}

module_init(param_init);
module_exit(param_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("module param test");
