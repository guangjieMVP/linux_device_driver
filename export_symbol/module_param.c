/**********************************************************************
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

static int int_param = 250;
module_param(int_param, int, S_IRUGO);

static char charparam = 20;
module_param(charparam, byte, S_IRUGO);

static char *char_param = "module param test";
module_param(char_param, charp, S_IRUGO);

EXPORT_SYMBOL(int_param);     //导出符号
EXPORT_SYMBOL(charparam);     //导出符号

int my_add(int a, int b)
{
    return a + b;
}
EXPORT_SYMBOL(my_add);

int my_sub(int a, int b)
{
    return a - b;
}
EXPORT_SYMBOL(my_sub);

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
