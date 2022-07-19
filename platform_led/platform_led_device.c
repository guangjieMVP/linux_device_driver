/**********************************************************************
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
************************************************************************/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define  CCM_CCGR1               0x20C406C
#define  MUX_PAD_GPIO1_IO04      0x20E006C
#define  GPIO1_DR                0x209C000
#define  GPIO1_GDIR              0x209C004

#define  REG_LEN                 4


static struct resource red_led_resource[] = {
    [0] = {
      .start =  CCM_CCGR1,
      .end   = (CCM_CCGR1 + REG_LEN - 1),
      .flags = IORESOURCE_MEM,
    },
    [1] = {
      .start =  MUX_PAD_GPIO1_IO04,
      .end   = (MUX_PAD_GPIO1_IO04 + REG_LEN - 1),
      .flags = IORESOURCE_MEM,
    },
    [2] = {
      .start =  GPIO1_DR,
      .end   = (GPIO1_DR + REG_LEN - 1),
      .flags = IORESOURCE_MEM,
    },
    [3] = {
      .start =  GPIO1_GDIR,
      .end   = (GPIO1_GDIR + REG_LEN - 1),
      .flags = IORESOURCE_MEM,
    },
};

static void red_led_platform_release(struct device *dev)
{

}

static struct platform_device  red_led_device = {
    .name = "red_led",
    .id = -1,
    .resource = red_led_resource,
    .num_resources = ARRAY_SIZE(red_led_resource),
    .dev.release = red_led_platform_release,
};


static int __init red_led_platform_device_init(void)
{
    printk("red_led_platform_device_init\r\n");
    int ret = platform_device_register(&red_led_device);
    return ret;
}

static void __exit red_led_platform_device_exit(void)
{
    printk("red_led_platform_device_exit\r\n");
    platform_device_unregister(&red_led_device);
}


module_init(red_led_platform_device_init);
module_exit(red_led_platform_device_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");


