#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

static int __init hello_init(void)   
{
  printk("hello world, init\r\n");
  return 0;
}

static void __exit hello_exit(void)  
{
  printk("hello world, exit\r\n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("GPL2");
MODULE_AUTHOR("ares");
MODULE_DESCRIPTION("hello module");
MODULE_ALIAS("test_module");