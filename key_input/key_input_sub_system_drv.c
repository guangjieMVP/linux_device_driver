/*
 * @brief :  使用输入子系统编写按键驱动
 * @date :   2021-1-17
 * @version : v1.0.0
 * @Change Logs:   
 * @date         author         notes:  
 * 2022/1/18     guangjieMVP    first version
 */

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
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/sched/signal.h> 
#include <linux/input.h>
#include <linux/poll.h>

#define XGJ_KEY_DTS_PATH   "/xgj_key"        /* 设备树节点路径，在根目录下*/
#define XGJ_KEY_COMPATILE  "xgj-key"
#define XGJ_KEY_GPIO_NAME  "xgj-key-gpios"   /* 设备树中描述按键GPIO的属性字段 */

struct keydevice {
    int irq;                           /* GPIO */
    int gpio;
    unsigned long timeout;
    dev_t dev_no;                      /* 设备号 */
    struct cdev chrdev;                  
    struct class *class;
    struct device_node *dev_node;
    struct timer_list timer;
	struct input_dev *inputdev;    /* input 结构体 */ 
};

static struct keydevice *keydev;

static void timeout_callback(struct timer_list *t);
DEFINE_TIMER(key_timer, timeout_callback);

static irqreturn_t key_isr(int irq, void *dev_id)
{
    mod_timer(&key_timer,  jiffies + msecs_to_jiffies(10));    /* jiffies全局变量，内核记录心跳时钟变量，启动定时器消抖，10ms */
    return IRQ_RETVAL(IRQ_HANDLED); 
}

static void timeout_callback(struct timer_list *t)
{
    if (gpio_get_value(keydev->gpio))     /* 再次判断按键是否按下 */
    {
        /* 向输入子系统上报按键按下 */
        input_report_key(keydev->inputdev, KEY_0, 1);  
    }
    else
    {
        /* 向输入子系统上报按键弹起 */
        input_report_key(keydev->inputdev, KEY_0, 0);  
    }
    input_sync(keydev->inputdev);          /* 上报同步事件，告诉输入子系统上报结束 */
}

static int key_device_io_init(void)
{
    int ret = 0;

    keydev->dev_node = of_find_node_by_path(XGJ_KEY_DTS_PATH);         /* 找到按键的设备树节点 */
    if (!keydev->dev_node) {          
        printk("key driver dts node can not found!\r\n"); 
        return -EINVAL; 
    }

    keydev->gpio = of_get_named_gpio(keydev->dev_node, XGJ_KEY_GPIO_NAME, 0);   /* 获取gpio的编号 */
    if ( keydev->gpio < 0) {
        printk("key driver gpio can not found!\r\n"); 
        return -EINVAL;
    }

    printk("key-gpio %d\n", keydev->gpio);

    ret = gpio_request(keydev->gpio, "xgj-key-gpio");
    if (ret) return ret;
       
	gpio_direction_input(keydev->gpio);    /* 设置为输入 */
	
    keydev->irq = gpio_to_irq(keydev->gpio);

    printk("key-irq %d\n",  keydev->irq);

    ret = request_irq(keydev->irq, key_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "xgj-key-irq", NULL);    /* 申请中断 */
    if (ret) {
        printk("failed to request key's irq");
    }

    return ret;
}

static int __init key_driver_init(void)
{
    int ret = 0;
    printk(" %s\n", __FUNCTION__);

    keydev = (struct keydevice*)kzalloc(sizeof(struct keydevice), GFP_KERNEL);
	if (!keydev) 
    {
        ret = -EINVAL;
        return ret;
    }

    ret = key_device_io_init();
    if (ret) 
    {
        printk(KERN_ERR "%s: Failed to regist key device.\n", __func__);
        goto out2;
    }

	keydev->inputdev = input_allocate_device();    /* 申请输入设备结构 */
	if (!keydev->inputdev) 
    {
        printk(KERN_ERR "%s: Failed to allocate input device.\n", __func__);
        ret = -ENOMEM;
        goto out2;
    }
    keydev->inputdev->name = "xgj-key";

    set_bit(EV_KEY, keydev->inputdev->evbit);     /* 设置产生按键事件    */ 
    set_bit(EV_REP, keydev->inputdev->evbit);     /* 设置重复事件  */ 
    set_bit(KEY_0, keydev->inputdev->keybit);     /* 设置产生的按键值 */

    ret = input_register_device(keydev->inputdev);             /* 注册输入设备 */
    if (ret) 
    {
        printk(KERN_ERR "%s: Failed to regist key device.\n", __func__);
        goto out1;
    }

    goto out;

out1:
    input_unregister_device(keydev->inputdev);              /* 卸载输入设备 */
    input_free_device(keydev->inputdev);                    /* 释放输入设备 */
out2:
    kfree(keydev);
    keydev = NULL;
out:
    return ret;
}

static void __exit key_driver_exit(void)
{
    printk(" %s\n", __FUNCTION__);

    free_irq(keydev->irq, NULL);        /* 释放中断 */

    gpio_free(keydev->gpio);

    del_timer_sync(&key_timer);

    input_unregister_device(keydev->inputdev);   /* 卸载 */
    input_free_device(keydev->inputdev);         /* 释放 */

    if (keydev)
    {
        kfree(keydev);                          /* 释放内存 */
        keydev = NULL;
    }
}

module_init(key_driver_init);
module_exit(key_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















