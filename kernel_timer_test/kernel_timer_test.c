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
#include <linux/timer.h>
#include <asm/spinlock.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>

#define RED_LED_DTS_NODE   "/red_led"       //red_led设备树节点路径
#define RED_LED_GPIO_NAME  "red_led_gpio"   //red led 节点中 gpio子系统相关属性名

#define TIMER_MAGIC_NUM   0xFF
#define CLOSE_TIMER   (_IO(TIMER_MAGIC_NUM, 0))
#define OPEN_TIMER    (_IO(TIMER_MAGIC_NUM, 1))
#define MOD_TIMER     (_IO(TIMER_MAGIC_NUM, 2))

enum led_stat {
    LED_ON = 0,
    LED_OFF = !LED_ON,
};

 struct red_led_dev {
    struct cdev chrdev;               //字符设备结构体
    dev_t dev_no;                     //设备号
    struct class *led_class;
    struct device_node *dev_node;
    unsigned int red_led_gpio;
    struct timer_list *timer;
    unsigned long timeout;
    spinlock_t lock;
    // unsigned int *virtual_ccgr1;
    // unsigned int *virtual_gpio1_io4;
    // unsigned int *virtual_dr;
    // unsigned int *virtual_gdir;
 };

static struct red_led_dev led_dev;

static void timeout_callback(struct timer_list *t);
DEFINE_TIMER(led_timer, timeout_callback);

static int red_led_drv_open (struct inode *node, struct file *file)
{
    int ret;

    printk(KERN_INFO"open red led dev\n");

    ret = gpio_direction_output(led_dev.red_led_gpio, 1);    //设置gpio为输出，默认输出高电平
    return ret;
}


static ssize_t red_led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    int err;
	char status;
	
	printk("red_led_drv_write\r\n");

	err = copy_from_user(&status, buf, 1);

	if (status)      
    {  /* 点灯 */
        gpio_set_value(led_dev.red_led_gpio, LED_ON);   
    }
    else                      
    {   /* 熄灯 */
        gpio_set_value(led_dev.red_led_gpio, LED_OFF);
    }
	
	return 1;
}

static long red_led_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned long flags;

    switch (cmd)
    {
        case CLOSE_TIMER:
        {
            del_timer(led_dev.timer);
            break;
        }
        case OPEN_TIMER:
        {
            add_timer(led_dev.timer);
//            mod_timer(led_dev.timer, jiffies + msecs_to_jiffies(led_dev.timeout));        //默认定时1000ms
            break;
        }
        case MOD_TIMER:
        {
            spin_lock_irqsave(&led_dev.lock, flags); 
            led_dev.timeout = arg;
            spin_unlock_irqrestore(&led_dev.lock, flags);
            mod_timer(led_dev.timer, jiffies + msecs_to_jiffies(arg));         //修改定时时间
            break;
        }
    }

    return 0;
}

static struct file_operations red_led_drv = {
	.owner	 = THIS_MODULE,
	.open    = red_led_drv_open,
	.write   = red_led_drv_write,
    .unlocked_ioctl = red_led_unlocked_ioctl,
};

//设备树的匹配条件
static struct of_device_id dts_match_table[] = {
    {.compatible = "red_led", },                     //通过设备树来匹配
};

static void timeout_callback(struct timer_list *t)
{
    static char status = 0;

    status = !status;

    gpio_set_value(led_dev.red_led_gpio, status);

    mod_timer(led_dev.timer, jiffies + msecs_to_jiffies(led_dev.timeout)); 
}

static int led_red_driver_probe(struct platform_device *dev)
{
    int err;
    int ret;
    struct device *tmpdev;

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

    led_dev.timer = &led_timer;
    led_dev.timeout = 1000;
    led_dev.timer->expires = jiffies + msecs_to_jiffies(led_dev.timeout);

    spin_lock_init(&led_dev.lock);      //初始化自旋锁

   	printk(KERN_INFO"led_red_driver_probe success\n");
    
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
    gpio_set_value(led_dev.red_led_gpio, LED_OFF);     
    device_destroy(led_dev.led_class, led_dev.dev_no);
	class_destroy(led_dev.led_class);
	unregister_chrdev_region(led_dev.dev_no, 1);
    cdev_del(&led_dev.chrdev);
    del_timer(led_dev.timer);
    printk(KERN_INFO"led_red_driver_remove success\n");

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





















