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

#include "export_symbol.h"

static int __init export_symbol_init(void)
{
    printk("export symbol init\r\n");
    printk(KERN_INFO"my_add : %d\r\n", my_add(int_param, charparam));
    printk(KERN_INFO"my_sub : %d\r\n", my_sub(int_param, charparam));
    return 0;
}

static void __exit export_symbol_exit(void)
{
    printk("export symbol exit\r\n");
}



module_init(export_symbol_init)
module_exit(export_symbol_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("symbol test");