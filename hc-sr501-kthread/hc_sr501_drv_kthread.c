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
#include <linux/kthread.h>

#define HC_SR501_DTS_NAME    "hc_sr501"

#define DEV_NAME    "hc-sr501"

struct hc_sr501 {
    int gpio;
    int irq;
    enum of_gpio_flags flag;
    struct gpio_desc *sr501_gpio;
    dev_t dev_no;
    struct cdev chrdev;
    struct class *class;
    struct mutex  m_lock;
    wait_queue_head_t  wq;
};

static struct hc_sr501  sr501;
static int sr501_val = 0;

struct task_struct *sr501_task;

static int hc_sr501_drv_open (struct inode *node, struct file *file)
{

    return 0;
}

static ssize_t hc_sr501_drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret;
#if 0
    mutex_lock(&sr501.m_lock);

    if (filp->f_flags & O_NONBLOCK)            /* 使用非阻塞方式 */  
    {
        if (sr501_val == 0)                     /* 无数据 */
        {
            ret = -EAGAIN;
        }
        else
        {
            size = size > 4 ? 4 : size;
            if (copy_to_user(buf, &sr501_val, size))
            {
                ret = -EFAULT;
            } 
            else
            {
                ret = size;
            }
            sr501_val = 0;
        }
        goto out;
    }

    ret = wait_event_interruptible(sr501.wq, sr501_val);	
    if (ret == -ERESTARTSYS)                  /* 被信号唤醒 */
    {
        goto out;
    }

    size = size > 4 ? 4 : size;
	if (copy_to_user(buf, &sr501_val, size)) 
    {
        ret = -EFAULT;
    } 
    else 
    {
        ret = size;
    }
    sr501_val = 0;
out:
    mutex_unlock(&sr501.m_lock);            /* 解锁 */
    return ret;
#else
    DECLARE_WAITQUEUE(wait, current);      /* 定义并初始化等待队列项 */
    add_wait_queue(&sr501.wq, &wait);      /* 将等待队列项加入等待队列 */

    mutex_lock(&sr501.m_lock);
    if (sr501_val == 0)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }

        __set_current_state(TASK_INTERRUPTIBLE);     /* 只是标记为未睡眠状态 */
        mutex_unlock(&sr501.m_lock);                 /* 解锁 */
        schedule();                                  /* 让出CPU */
        
        /* 当线程被唤醒从以下开始语句(断点)运行 */
        if (signal_pending(current)) /* 如果是被信号唤醒，则跳转到out2进行唤醒线程和设置线程状态为TASK_RUNNING */
        {
            ret = -ERESTARTSYS;
            goto out2;
        }                     

        /* 没有执行 if (signal_pending(current)) 说明不是信号唤醒，说明有数据 */
        mutex_lock(&sr501.m_lock);                   /* 加锁 */
    }
   
    size = size > 4 ? 4 : size; 
    // sr501_val |= 0xA0;
    if (copy_to_user(buf, &sr501_val, size)) 
    {
        ret = -EFAULT;
    } 
    else
    {
        ret = size;
    }
    sr501_val = 0;
out:
    mutex_unlock(&sr501.m_lock);
out2:
    remove_wait_queue(&sr501.wq, &wait);  
    set_current_state(TASK_RUNNING);                       /* 设置进程为运行态 */
	return ret;
#endif
}

static int is_data_already = 0;

/* 使驱动支持多路复用IO */
__poll_t hc_sr501_drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    __poll_t mask = 0;

    mutex_lock(&sr501.m_lock);

    poll_wait(filp, &sr501.wq, wait); 

    if (is_data_already)
    {
        is_data_already = 0;
        mask |= POLLIN | POLLRDNORM;
    }

    mutex_unlock(&sr501.m_lock);

    return mask;
}

static struct file_operations sr501_drv_ops = { 
	.owner	= THIS_MODULE,
	.open   = hc_sr501_drv_open,
    .read   = hc_sr501_drv_read,
    .poll   = hc_sr501_drv_poll,
};

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = HC_SR501_DTS_NAME, },                     /* 通过设备树来匹配 */
};

static int sr501_capture_thread(void *data)
{
    int prev = 0;

    do {
        mutex_lock(&sr501.m_lock);
        sr501_val = gpiod_get_value(sr501.sr501_gpio);
        mutex_unlock(&sr501.m_lock);
        if (prev != sr501_val) 
        {
            is_data_already = 1;
            wake_up(&sr501.wq);                  /* 唤醒线程*/
            prev = sr501_val;            
        }      
#if 1
        msleep(1000);     
        // printk("%s\n", __FUNCTION__);
#else
        // printk(" %s %s %d\n", __FILE__, __FUNCTION__, cnt++);
        // set_current_state(TASK_INTERRUPTIBLE);       /* 设置内核线程状态 */
        // schedule_timeout(HZ*5);                       /* 阻塞延时 */
        
#endif
    } while (!kthread_should_stop());

    return 0;
}


static int hc_sr501_driver_probe(struct platform_device *pdev)
{ 
    int err;
    struct device *hc_sr501_dev;
    int count;
    
    struct device_node *node = pdev->dev.of_node;

    if (!node) {          
        printk("hc-sr501 dts node can not found!\r\n");    
        return -EINVAL; 
    }

    count = of_gpio_count(node);  
    printk("gpio count %d\r\n", count);  

    sr501.sr501_gpio = gpiod_get(&pdev->dev, NULL, 0);
     if (IS_ERR(sr501.sr501_gpio)) {              
        dev_err(&pdev->dev, "Failed to get GPIO for hc-sr501\n");             
        return PTR_ERR(sr501.sr501_gpio);      
    }

	gpiod_direction_input(sr501.sr501_gpio);

    err = alloc_chrdev_region(&sr501.dev_no, 0, 1, DEV_NAME);          /* 内核自动分配设备号 */
	if (err < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", err);
		return err;
	}

	cdev_init(&sr501.chrdev, &sr501_drv_ops);

	cdev_add(&sr501.chrdev, sr501.dev_no, 1);

    sr501.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(sr501.class)) { 
        err = PTR_ERR(sr501.class);
        goto failed1;
	}

    /* 创建设备节点 */
    hc_sr501_dev = device_create(sr501.class , NULL, sr501.dev_no, NULL, "sr501"); 
    if (IS_ERR(hc_sr501_dev)) {
        err = PTR_ERR(hc_sr501_dev);
		goto failed2;
	}

    init_waitqueue_head(&sr501.wq);           /* 初始化等待队列头  */
    mutex_init(&sr501.m_lock);                 /* 初始化互斥锁  */   
#if 0
    sr501_task = kthread_create(sr501_capture_thread, NULL, "hc-sr501d");     /* 创建内核线程 */
    if (IS_ERR(sr501_task))
    {
        err = PTR_ERR(sr501_task);
        goto failed2;
    }
    wake_up_process(sr501_task);      /* 唤醒内核线程 sr501_task */
#else
    sr501_task = kthread_run(sr501_capture_thread, NULL, "hc-sr501d");     /* 创建内核线程 */
    if (IS_ERR(sr501_task))
    {
        err = PTR_ERR(sr501_task);
        goto failed2;
    }
#endif

    return 0;
failed2:
    device_destroy(sr501.class, sr501.dev_no);
    class_destroy(sr501.class);
failed1:
    unregister_chrdev_region(sr501.dev_no, 1);
    cdev_del(&sr501.chrdev);
    return err;
}

static int hc_sr501_driver_remove(struct platform_device *pdev)
{
    device_destroy(sr501.class, sr501.dev_no);
	class_destroy(sr501.class);
	unregister_chrdev_region(sr501.dev_no, 1);
    cdev_del(&sr501.chrdev);
    gpiod_put(sr501.sr501_gpio);           /* 释放gpio*/
    kthread_stop(sr501_task);              /* 停止内核线程 */
    printk(KERN_INFO"hc-sr501 drv remove success\n");
    return 0;
}

static struct platform_driver hc_sr501_platform_driver = {
      .probe = hc_sr501_driver_probe,
      .remove = hc_sr501_driver_remove,
      .driver = {
        .name = HC_SR501_DTS_NAME,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         /* 通过设备树匹配 */
      },
};

// 入口函数
static int __init hc_sr501_driver_init(void)
{
    int err;
    printk(" %s\n", __FUNCTION__);

    err = platform_driver_register(&hc_sr501_platform_driver);   //注册platform驱动
    return err;
}

// 出口函数
static void __exit hc_sr501_driver_exit(void)
{
    printk(" %s\n", __FUNCTION__);

    platform_driver_unregister(&hc_sr501_platform_driver);
}

module_init(hc_sr501_driver_init);
module_exit(hc_sr501_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















