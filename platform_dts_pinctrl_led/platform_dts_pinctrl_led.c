/**********************************************************************
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <asm/mach/map.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include <linux/device.h>

#include <linux/platform_device.h>

#define RED_LED_DTS_NODE   "/red_led"       //red_led设备树节点路径

 struct red_led_dev {
     struct cdev chrdev;               //字符设备结构体
     dev_t dev_no;                     //设备号
     struct class *led_class;
     struct device_node *dev_node;
     unsigned int *virtual_ccgr1;
     unsigned int *virtual_gpio1_io4;
     unsigned int *virtual_dr;
     unsigned int *virtual_gdir;
 };

static struct red_led_dev led_dev;

static int red_led_drv_open (struct inode *node, struct file *file)
{
    int val = 0;

    printk(KERN_INFO"open red led dev\n");
    val = ioread32(led_dev.virtual_ccgr1);      
    val &= ~(3 << 26);   
    val |= (3 << 26);   
    iowrite32(val, led_dev.virtual_ccgr1);    

    // val = 0;
    // val = ioread32(led_dev.virtual_gpio1_io4);   /* 配置复用为 GPIO1_IO4 的GPIO功能 */
    // val &= ~(0xf << 0);    
    // val |= (0x5 << 0);  
    // iowrite32(val, led_dev.virtual_gpio1_io4);   /* 去掉GPIO复用部分代码， 这部分配置让pinctrl子系统去做 */

    val = 0;
    val = ioread32(led_dev.virtual_gdir);       /* 配置为输出 */
    val |= (0x1 << 4); 
    iowrite32(val, led_dev.virtual_gdir);   
    return 0;
}


static ssize_t red_led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    int err;
	char status;
    int val = 0;
	
	printk("write %s %s line %d\r\n", __FILE__, __FUNCTION__, __LINE__);

	err = copy_from_user(&status, buf, 1);

	if (status)      
    {  /* 点灯 */
        val &= ~(1 << 4);
        iowrite32(val, led_dev.virtual_dr);      
    }
    else                      
    {   /* 熄灯 */
        val |= (1 << 4);                 
        iowrite32(val, led_dev.virtual_dr);       
    }
	
	return 1;
}

static struct file_operations red_led_drv = {
	.owner	 = THIS_MODULE,
	.open    = red_led_drv_open,
	.write   = red_led_drv_write,
};

//设备树的匹配条件
static struct of_device_id dts_match_table[] = {
    {.compatible = "red_led", },                     //通过设备树来匹配
};


static int led_red_driver_probe(struct platform_device *dev)
{
    int err;
    int ret;
    u32 regdata[8];
    int i;
    struct device *tmpdev;

    led_dev.dev_node = of_find_node_by_path(RED_LED_DTS_NODE);         //找到red_led的设备树节点
    if (!led_dev.dev_node) {          
        printk("red led can not found!\r\n"); 
        return -EINVAL; 
    }

    ret = of_property_read_u32_array(led_dev.dev_node,       
					                "reg",                                        //获取设备中寄存器属性值
					                 regdata, 8);
    if (ret < 0) {
        printk(KERN_INFO"Failed to get red led reg property\n");
    } else {
        for (i = 0; i < 8; i++) {
            printk("reg%d : %x\n", i, regdata[i]);
        }
    }
#if 1
    led_dev.virtual_ccgr1 = ioremap(regdata[0], regdata[1]);        //将寄存器物理地址转换成虚拟地址
    led_dev.virtual_gpio1_io4 = ioremap(regdata[2], regdata[3]); 
    led_dev.virtual_dr = ioremap(regdata[4], regdata[5]); 
    led_dev.virtual_gdir = ioremap(regdata[6], regdata[7]); 
#else 
    led_dev.virtual_ccgr1 = of_iomap(led_dev.dev_node, 0); 
    led_dev.virtual_gpio1_io4 = of_iomap(led_dev.dev_node, 1); 
    led_dev.virtual_dr = of_iomap(led_dev.dev_node, 2); 
    led_dev.gdir = of_iomap(led_dev.dev_node, 3); 
#endif

    ret = alloc_chrdev_region(&led_dev.dev_no, 0, 1, "red_led");
	if (ret < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", ret);
		return ret;
	}

	cdev_init(&led_dev.chrdev, &red_led_drv);

	cdev_add(&led_dev.chrdev, led_dev.dev_no, 1);

    led_dev.led_class = class_create(THIS_MODULE, "red_led_class");
	err = PTR_ERR(led_dev.led_class);
	if (IS_ERR(led_dev.led_class)) {
        goto failed1;
	}

    //在 /dev目录下创建red_led设备节点
    tmpdev = device_create(led_dev.led_class , NULL, led_dev.dev_no, NULL, "red_led"); 
    if (IS_ERR(tmpdev)) {
        ret = -EINVAL;
		goto failed2;
	}

   	printk(KERN_INFO"%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    
    return 0;
failed2:
    device_destroy(led_dev.led_class, led_dev.dev_no);
    class_destroy(led_dev.led_class);
failed1:
    cdev_del(&led_dev.chrdev);
	unregister_chrdev_region(led_dev.dev_no, 1);
    return ret;
}

int led_red_driver_remove(struct platform_device *dev)
{
    iounmap(led_dev.virtual_ccgr1);         //解除寄存器地址映射
    iounmap(led_dev.virtual_gpio1_io4);
    iounmap(led_dev.virtual_dr);
    iounmap(led_dev.virtual_gdir);
    
    printk(KERN_INFO"driver remove %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    device_destroy(led_dev.led_class, led_dev.dev_no);
	class_destroy(led_dev.led_class);
	unregister_chrdev_region(led_dev.dev_no, 1);
    cdev_del(&led_dev.chrdev);
     
    return 0;
}

static struct platform_driver red_led_platform_driver = {
      .probe = led_red_driver_probe,
      .remove = led_red_driver_remove,
      .driver = {
        .name = "fire-rgb-led",
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         //通过设备树匹配
      },
};


static int __init red_led_driver_init(void)
{
    int ret;
    printk(" %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    ret = platform_driver_register(&red_led_platform_driver);   //注册platform驱动
    return ret;
}

static void __exit red_led_driver_exit(void)
{
    printk(" %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    platform_driver_unregister(&red_led_platform_driver);
}

module_init(red_led_driver_init);
module_exit(red_led_driver_exit);


MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















