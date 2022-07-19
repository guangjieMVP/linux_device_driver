
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
#include <linux/atomic.h>

#define DEV_DTS_NODE_PATH    "/ds18b20"                /* 设备树节点的路径，在根节点下 */
#define DEV_PIN_DTS_NAME     "ds18b20-gpios"           /* GPIO引脚的属性名 */
#define DEV_NAME             "ds18b20"                 /* 设备名  /dev/ds18b20 */
#define DEV_DTS_COMPATIBLE   "xgj,ds18b20"             /* 设备匹配属性 compatible */

#define USE_GPIO_LIB        0

struct ds18b20 {
    int gpio;                       /* gpio */ 
    dev_t dev_no;                   /* 设备号 */    
    struct cdev chrdev;             
    struct class *class;
    spinlock_t lock;
};

#define CMD_CONVERT_TEMP         0x44
#define CMD_READ_DATA            0xBE
#define CMD_SKIP_ROM_ID          0xCC
#define CMD_MATCH_ROM_ID         0x55
#define CMD_READ_ROM_ID          0x33
#define CMD_SEARCH_ROM_ID        0xF0

#define DS18B20_IO_OUT()    gpio_direction_output(ds18b20_dev.gpio, 1)
#define DS18B20_IO_IN()     gpio_direction_input(ds18b20_dev.gpio)
#define DS18B20_WRITE(bit)  gpio_set_value(ds18b20_dev.gpio, bit)
#define DS18B20_READ()      gpio_get_value(ds18b20_dev.gpio)

static struct ds18b20  ds18b20_dev;

/* ds18b20复位，既检测ds18b20是否存在 0 - ds18b20存在， 1 - ds18b20不存在 */
static int ds18b20_reset(void)
{
    int ret = 1;
    unsigned long flags;

    spin_lock_irqsave(&ds18b20_dev.lock, flags);
    DS18B20_IO_OUT();     /* 设置为输出 */
    DS18B20_WRITE(0);
    udelay(480);                                         /* udelay 延时可能不准 */
    DS18B20_WRITE(1);
    udelay(75);
    DS18B20_IO_IN();    /* 设置为输入 */
    ret = DS18B20_READ();
    udelay(10);
    DS18B20_IO_OUT();
    DS18B20_WRITE(1);       /* 释放总线 */
    spin_unlock_irqrestore(&ds18b20_dev.lock, flags);

    return ret;
}

static void _ds18b20_write_bit(uint8_t bit)   
{
    DS18B20_IO_OUT();     /* 设置为输出 */

    bit = bit > 1 ? 1 : bit;
    DS18B20_WRITE(0);
    udelay(1);  
    DS18B20_WRITE(bit);
    udelay(60);           /* 65 us */
 
    DS18B20_WRITE(1);   /* 释放总线 */
    udelay(16);
}

/* 整个读周期最少需要60us，启动读开始信号后必须15us内读取IO电平，否则就会被上拉拉高 */
static uint8_t _ds18b20_read_bit(void)
{
    uint8_t bit;
 
    DS18B20_IO_OUT();     /* 设置为输出 */
    DS18B20_WRITE(0);
    udelay(1); 
    DS18B20_IO_IN();    /* 设置为输入 */
    udelay(5);
    bit = DS18B20_READ();    /* 读取结果 */
    udelay(55);

    return bit;
}

static void ds18b20_wrte_byte(uint8_t byte)
{
    int i;
    unsigned long flags;

    spin_lock_irqsave(&ds18b20_dev.lock, flags);
    for (i = 0; i < 8; i++) 
    {
        // _ds18b20_write_bit(byte & 0x01);  //(byte >> i) & 0x01
        // byte = byte >> 1;  
        _ds18b20_write_bit((byte >> i) & 0x01);  
    }
    spin_unlock_irqrestore(&ds18b20_dev.lock, flags);
}

static uint8_t ds18b20_read_byte(void)
{
    int i;
    uint8_t bit;
    uint8_t byte = 0;

    unsigned long flags;

    spin_lock_irqsave(&ds18b20_dev.lock, flags);

    for (i = 0; i < 8; i++)
    {
        bit = _ds18b20_read_bit();
        if (bit) byte |= (0x01 << i);
    }
    spin_unlock_irqrestore(&ds18b20_dev.lock, flags);

    return byte;
}   

/* 使设备只能被一个进程打开 */
static int _drv_open (struct inode *node, struct file *file)
{
    printk("ds18b20 open\n");
    return 0;
}

static ssize_t _drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret;
    uint8_t temp_L = 0, temp_H = 0;
    short temp = 0;

    if (ds18b20_reset() != 0)
    {
        printk("%d ds18b20 reset failed", __LINE__);
        return -EFAULT;
    }

    ds18b20_wrte_byte(CMD_SKIP_ROM_ID);
    ds18b20_wrte_byte(CMD_CONVERT_TEMP);

    msleep(750);            /* 等待一段时间，让DS18B20进行温度转换 */

    if (ds18b20_reset() != 0)
    {
        printk("%d ds18b20 reset failed", __LINE__);
        return -EFAULT;
    }

    ds18b20_wrte_byte(CMD_SKIP_ROM_ID);   
    ds18b20_wrte_byte(CMD_READ_DATA);      /* 发送读数据命令 */

    temp_L =  ds18b20_read_byte();
    temp_H =  ds18b20_read_byte(); 
    /* 组合温度数据 */
    temp = temp_H << 8;
    temp += temp_L;
    if (temp < 0)    /* 负温度 */
    {
        temp = ~temp + 1;       /* 转换得到负数源码的绝对值 */
    }
    
	if (copy_to_user(buf, &temp, sizeof(temp))) 
    {
        ret = -EFAULT;
    } 
    else 
    {
        ret = sizeof(temp);
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
    printk("ds18b20 release\n");
    return 0;
}

static struct file_operations drv_file_ops = { 
	.owner	= THIS_MODULE,
	.open   = _drv_open,
    .read   = _drv_read,
    .poll   = _drv_poll,
    .release = _drv_release,
};

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = DEV_DTS_COMPATIBLE, },                     /* 通过设备树来匹配 */
};


static int _driver_probe(struct platform_device *pdev)
{ 
    int err;
    struct device *ds_dev;
    struct device_node *dev_node;
    
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

    // gpiod_direction_output(sr04.trig_gpio, 0);
    // gpiod_direction_input(sr04.echo_gpio);
	// sr04.irq = gpiod_to_irq(sr04.echo_gpio);
#else
    dev_node = of_find_node_by_path(DEV_DTS_NODE_PATH);       /* 找到ds18b20的设备树节点  */
    if (IS_ERR(dev_node)) {          
        printk("hc-sr04 DTS Node not found!\r\n"); 
        return PTR_ERR(dev_node); 
    }

    ds18b20_dev.gpio = of_get_named_gpio(dev_node, DEV_PIN_DTS_NAME, 0);   /* 获取ds18b20 的gpio编号 */
    if ( ds18b20_dev.gpio < 0) {
        printk("trig-gpio not found!\r\n"); 
        return -EINVAL;
    }

    err = gpio_request(ds18b20_dev.gpio, DEV_PIN_DTS_NAME);  
	if(err) 
    {
		printk("gpio_request gpio is failed!\n");
        return -EINVAL;
	}

    printk("ds18b20 gpio %d\n", ds18b20_dev.gpio);
#endif
  
    /* 内核自动分配设备号 */
    err = alloc_chrdev_region(&ds18b20_dev.dev_no, 0, 1, DEV_NAME);        
	if (err < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", err);
		return err;
	}

	cdev_init(&ds18b20_dev.chrdev, &drv_file_ops);

	cdev_add(&ds18b20_dev.chrdev, ds18b20_dev.dev_no, 1);

    ds18b20_dev.class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(ds18b20_dev.class)) {   
        err = PTR_ERR(ds18b20_dev.class);
        goto failed1;
	}

    /* 创建设备节点 */
    ds_dev = device_create(ds18b20_dev.class , NULL, ds18b20_dev.dev_no, NULL, DEV_NAME); 
    if (IS_ERR(ds_dev)) {       /* 判断指针是否合法 */
        err = PTR_ERR(ds_dev);
		goto failed2;
	}

    spin_lock_init(&ds18b20_dev.lock);   /* 初始化自旋锁 */
    printk("ds18b20 probe success\r\n");
    return 0;
failed2:
    device_destroy(ds18b20_dev.class, ds18b20_dev.dev_no);
    class_destroy(ds18b20_dev.class);
failed1:
    unregister_chrdev_region(ds18b20_dev.dev_no, 1);
    cdev_del(&ds18b20_dev.chrdev);
    gpio_free(ds18b20_dev.gpio);
    return err;
}

static int _driver_remove(struct platform_device *pdev)
{
    device_destroy(ds18b20_dev.class, ds18b20_dev.dev_no);
	class_destroy(ds18b20_dev.class);
	unregister_chrdev_region(ds18b20_dev.dev_no, 1);
    cdev_del(&ds18b20_dev.chrdev);
    gpio_free(ds18b20_dev.gpio);

    printk(KERN_INFO"ds18b20 remove success\n");

    return 0;
}

static struct platform_driver _platform_driver = {
      .probe = _driver_probe,
      .remove = _driver_remove,
      .driver = {
        .name = DEV_DTS_COMPATIBLE,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         /* 通过设备树匹配 */
      },
};

/* 入口函数 */ 
static int __init _driver_init(void)
{
    int ret;
    printk("ds18b20 %s\n", __FUNCTION__);
    
    ret = platform_driver_register(&_platform_driver);   //注册platform驱动
    return ret;
}

/*  出口函数 */
static void __exit _driver_exit(void)
{
    printk("ds18b20  %s\n", __FUNCTION__);
    platform_driver_unregister(&_platform_driver);
}

module_init(_driver_init);
module_exit(_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















