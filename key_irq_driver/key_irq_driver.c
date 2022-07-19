/**********************************************************************
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief:       中断 等待队列 按键读取
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
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/sched/signal.h> 
#include <linux/poll.h>

#define ARES_KEY_DTS_NODE   "/my_key"       //red_led设备树节点路径
#define ARES_KEY_GPIO_NAME  "my_key_gpio"   //red led 节点中 gpio子系统相关属性名


struct keydev {
    struct cdev chrdev;               //字符设备结构体
    dev_t dev_no;                     //设备号
    struct class *class;
    struct device_node *dev_node;
    unsigned int key_gpio;
    struct timer_list *timer;
    unsigned long timeout;
    struct mutex  m_lock;;
    wait_queue_head_t r_wait_head;             //读等待队列
};

struct key_irq {
    int irqnum;
    char val;
    char is_press;
    char name[20];
    irqreturn_t (*irq_handler_t)(int, void *);
};

static struct keydev g_keydev;

static irqreturn_t irq_handler(int num, void *dev);   //中断
static struct key_irq my_key = {
    .irq_handler_t = irq_handler,
    .val = 0x00,
    .name = "mykey",
    .is_press = 0,
};

static void timeout_callback(struct timer_list *t);
DEFINE_TIMER(key_timer, timeout_callback);

static int key_drv_open (struct inode *node, struct file *file)
{
    printk(KERN_INFO"open key dev\n");

    gpio_direction_input(g_keydev.key_gpio);    //设置为输入
    printk(KERN_INFO"%s success\n", __FUNCTION__);

    return 0;
}


static ssize_t key_drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    char status = 0x00;
    int ret;

    DECLARE_WAITQUEUE(wait, current);       //定义并初始化等待队列项
    add_wait_queue(&g_keydev.r_wait_head, &wait);      //将等待队列项加入等待队列

    mutex_lock(&g_keydev.m_lock);
    if (my_key.val == 0)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }

        __set_current_state(TASK_INTERRUPTIBLE);     //只是标记为未睡眠状态
        mutex_unlock(&g_keydev.m_lock);              //解锁
        schedule();                                  //让出CPU
        
        if (signal_pending(current))               //判断被信号唤醒
        {
            ret = -ERESTARTSYS;
            goto out2;
        }                     

        mutex_lock(&g_keydev.m_lock);
    }
   
    if (size >= 1)
        size = 1;

    status = my_key.val;   

    if (copy_to_user(buf, &status, size)) 
    {
        ret = -EFAULT;
    } 
    else
    {
        ret = size;
    }

out:
    mutex_unlock(&g_keydev.m_lock);
out2:
    remove_wait_queue(&g_keydev.r_wait_head, &wait);  
    set_current_state(TASK_RUNNING);          //设置进程为运行态
	return ret;
}

__poll_t key_drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    __poll_t mask = 0;

    mutex_lock(&g_keydev.m_lock);

    poll_wait(filp, &g_keydev.r_wait_head, wait); 

    if (my_key.is_press)
    {
        my_key.is_press = 0;
        mask |= POLLIN | POLLRDNORM;
    }

    mutex_unlock(&g_keydev.m_lock);

    return mask;
}

static struct file_operations key_drv_ops = {
	.owner	= THIS_MODULE,
	.open   = key_drv_open,
    .read   = key_drv_read,
    .poll   = key_drv_poll,
};

//设备树的匹配列表
static struct of_device_id dts_match_table[] = {
    {.compatible = "my_key", },                     //通过设备树来匹配
};

static irqreturn_t irq_handler(int num, void *dev)
{
    if (gpio_get_value(g_keydev.key_gpio))     //按键按下
    {
        mod_timer(g_keydev.timer,  jiffies + msecs_to_jiffies(10));    //启动定时器，20ms
    }
    
    return IRQ_RETVAL(IRQ_HANDLED); 
}

static void timeout_callback(struct timer_list *t)
{
    my_key.val = 0;
    my_key.is_press = 0;

    if (gpio_get_value(g_keydev.key_gpio))     //再次判断按键是否按下
    {
        my_key.val = 0xff;
        my_key.is_press = 1;
        wake_up_interruptible(&g_keydev.r_wait_head);           //唤醒等待队列中进入休眠的进程
    }
}

static int key_driver_probe(struct platform_device *dev)
{
    int err;
    int ret;
    struct device *tmpdev;

    g_keydev.dev_node = of_find_node_by_path(ARES_KEY_DTS_NODE);         //找到red_led的设备树节点  
    if (!g_keydev.dev_node) {          
        printk("key driver dts node can not found!\r\n"); 
        return -EINVAL; 
    }

    g_keydev.key_gpio = of_get_named_gpio(g_keydev.dev_node, ARES_KEY_GPIO_NAME, 0);   //获取gpio的编号
    if ( g_keydev.key_gpio < 0) {
        printk("key driver gpio can not found!\r\n"); 
        return -EINVAL;
    }
    my_key.irqnum = irq_of_parse_and_map(g_keydev.dev_node, 0);         //根据设备树节点获取中断号
    printk("irq number = %d\n", my_key.irqnum);

    ret = request_irq(my_key.irqnum, my_key.irq_handler_t, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, my_key.name, NULL);    //申请中断
    if (ret < 0) {
        printk("failed to request irq");
        return ret;
    }

    ret = alloc_chrdev_region(&g_keydev.dev_no, 0, 1, "my_key");
	if (ret < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", ret);
		return ret;
	}

	cdev_init(&g_keydev.chrdev, &key_drv_ops);

	cdev_add(&g_keydev.chrdev, g_keydev.dev_no, 1);

    g_keydev.class = class_create(THIS_MODULE, "my_key");
	err = PTR_ERR(g_keydev.class);
	if (IS_ERR(g_keydev.class)) {
        goto failed1;
	}

    //在 /dev目录下创建red_led设备节点
    tmpdev = device_create(g_keydev.class , NULL, g_keydev.dev_no, NULL, "my_key"); 
    if (IS_ERR(tmpdev)) {
        ret = -EINVAL;
		goto failed2;
	}

    g_keydev.timer = &key_timer;

    init_waitqueue_head(&g_keydev.r_wait_head);     //初始化等待队列头 
    mutex_init(&g_keydev.m_lock);

   	printk(KERN_INFO"key_driver_probe success\n");
    
    return 0;
failed2:
    device_destroy(g_keydev.class, g_keydev.dev_no);
    class_destroy(g_keydev.class);
failed1:
    cdev_del(&g_keydev.chrdev);
	unregister_chrdev_region(g_keydev.dev_no, 1);
    return ret;
}

static int key_driver_remove(struct platform_device *dev)
{
    device_destroy(g_keydev.class, g_keydev.dev_no);
	class_destroy(g_keydev.class);
	unregister_chrdev_region(g_keydev.dev_no, 1);
    cdev_del(&g_keydev.chrdev);
    del_timer(g_keydev.timer);
    free_irq(my_key.irqnum, NULL);
    printk(KERN_INFO"key_driver_remove success\n");

    return 0;
}

static struct platform_driver key_platform_driver = {
      .probe = key_driver_probe,
      .remove = key_driver_remove,
      .driver = {
        .name = "my_key",
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         //通过设备树匹配
      },
};


static int __init key_driver_init(void)
{
    int ret;
    printk(" %s\n", __FUNCTION__);
    ret = platform_driver_register(&key_platform_driver);   //注册platform驱动
    return ret;
}

static void __exit key_driver_exit(void)
{
    printk(" %s\n", __FUNCTION__);
    platform_driver_unregister(&key_platform_driver);
}

module_init(key_driver_init);
module_exit(key_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















