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

#include <asm/atomic.h>  
#include <asm/spinlock.h>                  //自旋锁头文件
#include <linux/semaphore.h>

#define RED_LED_DTS_NODE   "/red_led"       //red_led设备树节点路径
#define RED_LED_GPIO_NAME  "red_led_gpio"   //red led 节点中 gpio子系统相关属性名

struct red_led_dev {
    struct cdev chrdev;               //字符设备结构体
    dev_t dev_no;                     //设备号
    struct class *led_class;
    struct device_node *dev_node;     //red led 设备节点
    unsigned int red_led_gpio;
    struct semaphore sem;             //信号量
};

static struct red_led_dev led_dev;

static int red_led_drv_open (struct inode *node, struct file *filp)
{
    int ret;

    if (filp->f_flags & O_NONBLOCK)
    {
        if (down_trylock(&led_dev.sem))    //以非阻塞的方式获取信号量 获取失败
        {
            return -EBUSY;             //发挥一个错误信息
        }
    }
    else
        down(&led_dev.sem);  //信号量P操作  减1
    printk("sem P operation\n");
    printk(KERN_INFO"success to open red led dev\n set gpio is ouput\n");

    ret = gpio_direction_output(led_dev.red_led_gpio, 1);    //设置gpio为输出，默认输出高电平
    return ret;
}

static ssize_t red_led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    int err;
	char status;
	
	err = copy_from_user(&status, buf, 1);

	if (status)      
    {  /* 点灯 */
        printk("led on\n");
        gpio_set_value(led_dev.red_led_gpio, 0);   
    }
    else                      
    {   /* 熄灯 */
        printk("led off\n");
        gpio_set_value(led_dev.red_led_gpio, 1);
    }
	
	return 1;
}

int red_led_drv_release(struct inode *node, struct file *file)
{
    up(&led_dev.sem);    //信号量V操作  加1

    printk("led release \n sem V operation \n");
    return 1;
}

static struct file_operations red_led_drv = {
	.owner	 = THIS_MODULE,
	.open    = red_led_drv_open,
	.write   = red_led_drv_write,
    .release = red_led_drv_release,
};

//设备树的匹配条件
static struct of_device_id dts_match_table[] = {
    {.compatible = "red_led", },                     //通过设备树来匹配
};


static int led_red_driver_probe(struct platform_device *dev)  
{
    int err;
    int ret;
    struct device *tmpdev;

    sema_init(&led_dev.sem, 1);     //初始化信号量的值为1

    led_dev.dev_node = of_find_node_by_path(RED_LED_DTS_NODE);         //找到red_led的设备树节点
    if (!led_dev.dev_node) {          
        printk("red led device node can not found!\r\n"); 
        return -EINVAL; 
    }

    led_dev.red_led_gpio = of_get_named_gpio(led_dev.dev_node, RED_LED_GPIO_NAME, 0);   //获取gpio的编号
    if ( led_dev.red_led_gpio < 0) {
        printk("red led gpio can not found!\r\n"); 
        return -EINVAL;
    }

    // ret = gpio_request(led_dev.red_led_gpio, RED_LED_GPIO_NAME);   //请求gpio
    // if (ret < 0) {
    //     printk("rcan not request gpio!\r\n"); 
    //     return -EINVAL;
    // }

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

   	printk("led driver probe\n");
    
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
    device_destroy(led_dev.led_class, led_dev.dev_no);
	class_destroy(led_dev.led_class);
	unregister_chrdev_region(led_dev.dev_no, 1);
    cdev_del(&led_dev.chrdev);
    printk("led driver remove\n");
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

    ret = platform_driver_register(&red_led_platform_driver);   //注册platform驱动
    printk("led platform driver init\n");
    return ret;
}

static void __exit red_led_driver_exit(void)
{
    platform_driver_unregister(&red_led_platform_driver);
    printk("led platform driver exit\n");
}

module_init(red_led_driver_init);
module_exit(red_led_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















