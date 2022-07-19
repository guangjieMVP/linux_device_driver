

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
// #include <asm/spinlock.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/sched/signal.h> 
#include <linux/poll.h>
// #include <asm/atomic.h>  
#include <linux/atomic.h>

#define HC_SR04_DTS_NAME    "hc_sr04"

#define DEV_NAME            "hc-sr04"

#define USE_GPIO_LIB        0

struct hc_sr04 {
    int irq;                        /* 中断号 */
    enum of_gpio_flags flag;
    struct gpio_desc *trig_gpio;    /* trig-gpio */
    struct gpio_desc *echo_gpio;    /* echo-gpio */
    dev_t dev_no;                   /* 设备号 */    
    struct cdev chrdev;             
    struct class *class;
    struct mutex  m_lock;           
    wait_queue_head_t  wq;          /* 等待队列 */

};

static struct hc_sr04  sr04;
static int sr04_trig_gpio;
static int sr04_echo_gpio;

static atomic_t sr04_atomic = ATOMIC_INIT(1);   /*  定义原子变量 */

/* 使设备只能被一个进程打开 */
static int _drv_open (struct inode *node, struct file *file)
{
    if (!atomic_dec_and_test(&sr04_atomic))  {
       atomic_inc(&sr04_atomic);
       return  -EBUSY;               /*  已经打开 */
    }

    gpio_direction_input(sr04_echo_gpio);   
    gpio_direction_output(sr04_trig_gpio, 0);
  
    return 0;
}


static ssize_t _drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret;
    int time_us = 0;
    int timeout = 1000000;

    unsigned long flags;
    /* 中断屏蔽 */
    // local_irq_save(flags);
    local_irq_disable();
    /* 启动触发信号 */
    gpio_set_value(sr04_trig_gpio, 1);  //gpiod_set_value(sr04.trig_gpio, 1);
    udelay(40);
    gpio_set_value(sr04_trig_gpio, 0);//gpiod_set_value(sr04.trig_gpio, 0);    

    /* 等接收信号GPIO变为高电平*/
    while (gpio_get_value(sr04_echo_gpio)==0 && timeout)      //while (!gpiod_get_value(sr04.echo_gpio) && timeout)
    {
        udelay(1);
        timeout--;
    }   
    // printk("echo_gpio %d, %d", gpio_get_value(sr04_echo_gpio), timeout);
    if (timeout == 0) 
    {
        local_irq_restore(flags);
        return -EAGAIN;
    }

    timeout = 1000000;
    while (gpio_get_value(sr04_echo_gpio)==1 && timeout)   // while (gpiod_get_value(sr04.echo_gpio) && timeout) 
    {
        udelay(1);  
        time_us++;                                /* 计算信号高电平时间 */
        timeout--; 
    }
    // printk("time_us %d\r\n", time_us);
    if (timeout == 0) 
    {
        printk("timeout 2\r\n");
        local_irq_restore(flags);
        return -EAGAIN;   
    }    

    /* 恢复中断 */
    // local_irq_restore(flags);
    local_irq_enable();

    size = size > 4 ? 4 : size;
	if (copy_to_user(buf, &time_us, size)) 
    {
        ret = -EFAULT;
    } 
    else 
    {
        ret = size;
    }
    
    return ret;
}

/* 使驱动支持多路复用IO */
static __poll_t _drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    __poll_t mask = 0;

    // // wait_event_interruptible
    // mutex_lock(&sr04.m_lock);

    // poll_wait(filp, &sr04.wq, wait); 

    // if (sr04_val)
    // {
    //     mask |= POLLIN | POLLRDNORM;
    // }

    // mutex_unlock(&sr501.m_lock);

    return mask;
}

static int _drv_release(struct inode *node, struct file *file)
{
    atomic_set(&sr04_atomic, 1);      /* 释放时设置原子变量值为1 */
    printk("hc-sr04 release\n");
    return 0;
}


static struct file_operations sr04_drv_ops = { 
	.owner	= THIS_MODULE,
	.open   = _drv_open,
    .read   = _drv_read,
    .poll   = _drv_poll,
    .release = _drv_release,
};

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = HC_SR04_DTS_NAME, },                     /* 通过设备树来匹配 */
};

static irqreturn_t hc_sr04_isr(int irq_num, void *dev)
{
    int ret = 0;

    // ret = gpiod_get_value(sr04.sr04_gpio);
    printk("hc-sr04 gpio %d\r\n", ret);
    // wake_up_interruptible(&sr501.wq);           /* 唤醒等待队列中进入休眠的进程 */
    wake_up(&sr04.wq);                             /* 唤醒等待队列中进入休眠的进程 */
 
    printk("hc-sr04 irq\r\n");
    return IRQ_RETVAL(IRQ_HANDLED);   
}

static int _driver_probe(struct platform_device *pdev)
{ 
    int err;
    struct device *sr04_dev;
    int count;
    
    struct device_node *node = pdev->dev.of_node;

    if (!node) {          
        printk("hc-sr501 dts node can not found!\r\n");    
        return -EINVAL; 
    }

    // count = of_gpio_count(node);  
    // printk("gpio count %d\r\n", count);  
#if USE_GPIO_LIB
    sr04.trig_gpio = gpiod_get(&pdev->dev, "trig", GPIOD_OUT_LOW);
    if (IS_ERR(sr04.trig_gpio)) {              
        dev_err(&pdev->dev, "Failed to get trig-gpio for hc-sr04\n");             
        return PTR_ERR(sr04.trig_gpio);      
    }

    sr04.echo_gpio = gpiod_get(&pdev->dev, "echo", GPIOD_IN);
    if (IS_ERR(sr04.echo_gpio)) {              
        dev_err(&pdev->dev, "Failed to get trig-gpio for hc-sr04\n");     
        gpiod_put(sr04.trig_gpio);           /* 释放trig-gpio */        
        return PTR_ERR(sr04.echo_gpio);      
    }
#else
    struct device_node *dev_node = of_find_node_by_path("/hc_sr04");       /* 找到hc-sr04的设备树节点  */
    if (IS_ERR(dev_node)) {          
        printk("hc-sr04 DTS Node not found!\r\n"); 
        return PTR_ERR(dev_node); 
    }

    sr04_trig_gpio = of_get_named_gpio(dev_node, "trig-gpios", 0);   /* 获取trig-gpio的编号 */
    if (sr04_trig_gpio < 0) {
        printk("trig-gpio not found!\r\n"); 
        return -EINVAL;
    }

    err = gpio_request(sr04_trig_gpio, "trig-gpios");  
	if(err) 
    {
		printk("gpio_request trig-gpios is failed!\n");
        return -EINVAL;
	}

    sr04_echo_gpio = of_get_named_gpio(dev_node, "echo-gpios", 0);   /* 获取echo-gpio的编号 */
    if ( sr04_echo_gpio < 0) {
        printk("echo-gpio not found!\r\n"); 
        return -EINVAL;
    }
    err = gpio_request(sr04_echo_gpio, "echo-gpios");  
    if(err) 
    {
        gpio_free(sr04_trig_gpio);
		printk("gpio_request echo-gpios is failed!\n");
        return -EINVAL;
	}

    printk("trig-gpio %d  echo-gpio %d\n", sr04_trig_gpio, sr04_echo_gpio);
#endif
    // gpiod_direction_output(sr04.trig_gpio, 0);
    // gpiod_direction_input(sr04.echo_gpio);
	// sr04.irq = gpiod_to_irq(sr04.echo_gpio);

    // err = request_irq(sr04.irq, hc_sr501_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DEV_NAME, NULL);    /* 申请中断 */
    // if (err < 0) {
    //     printk(KERN_INFO"failed to request irq %d\r\n", sr04.irq);
    //     return err;
    // }
    /* 内核自动分配设备号 */
    err = alloc_chrdev_region(&sr04.dev_no, 0, 1, DEV_NAME);        
	if (err < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", err);
		return err;
	}

	cdev_init(&sr04.chrdev, &sr04_drv_ops);

	cdev_add(&sr04.chrdev, sr04.dev_no, 1);

    sr04.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(sr04.class)) { 
        err = PTR_ERR(sr04.class);
        goto failed1;
	}

    /* 创建设备节点 */
    sr04_dev = device_create(sr04.class , NULL, sr04.dev_no, NULL, DEV_NAME); 
    if (IS_ERR(sr04_dev)) {       /* 判断指针是否合法 */
        err = PTR_ERR(sr04_dev);
		goto failed2;
	}

    init_waitqueue_head(&sr04.wq);     /* 初始化等待队列头  */
    mutex_init(&sr04.m_lock);                 /* 初始化互斥锁  */   

    printk("hc-sr04 probe success\r\n");
    return 0;
failed2:
    device_destroy(sr04.class, sr04.dev_no);
    class_destroy(sr04.class);
failed1:
    unregister_chrdev_region(sr04.dev_no, 1);
    cdev_del(&sr04.chrdev);
#if USE_GPIO_LIB
    gpiod_put(sr04.echo_gpio);           /* 释放echo-gpio*/
    gpiod_put(sr04.trig_gpio);           /* 释放trig-gpio*/
#else
    gpio_free(sr04_trig_gpio);
    gpio_free(sr04_echo_gpio);
#endif
    return err;
}

static int _driver_remove(struct platform_device *pdev)
{
    device_destroy(sr04.class, sr04.dev_no);
	class_destroy(sr04.class);
	unregister_chrdev_region(sr04.dev_no, 1);
    cdev_del(&sr04.chrdev);
    // free_irq(sr04.irq, NULL);             /* 释放中断*/
#if USE_GPIO_LIB
    gpiod_put(sr04.echo_gpio);           /* 释放echo-gpio*/
    gpiod_put(sr04.trig_gpio);           /* 释放trig-gpio*/
#else
    gpio_free(sr04_trig_gpio);
    gpio_free(sr04_echo_gpio);
#endif
    printk(KERN_INFO"hc-sr04 drv remove success\n");

    return 0;
}

static struct platform_driver _platform_driver = {
      .probe = _driver_probe,
      .remove = _driver_remove,
      .driver = {
        .name = HC_SR04_DTS_NAME,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         /* 通过设备树匹配 */
      },
};

// 入口函数
static int __init _driver_init(void)
{
    int ret;
    printk("hc-sr04 %s\n", __FUNCTION__);
    
    ret = platform_driver_register(&_platform_driver);   //注册platform驱动
    return ret;
}

// 出口函数
static void __exit _driver_exit(void)
{
    printk("hc-sr04  %s\n", __FUNCTION__);
    platform_driver_unregister(&_platform_driver);
}

module_init(_driver_init);
module_exit(_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















