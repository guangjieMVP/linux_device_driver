
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
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/sched/signal.h> 
#include <linux/poll.h>
#include <linux/atomic.h>
#include <linux/i2c.h>

#include "ap3216c.h"

#define AP3216C_DTS_NODE_PATH    "xgj_ap3216c"
#define AP3216C_DTS_COMPATIBLE   "xgj-ap3216c"
#define AP3216C_READY_GPIO_NAME  "ready-gpios"
#define DEV_NAME                 "ap3216c"

#define USE_GPIO_LIB        0

struct ap3216c_data {
    short int als;        //环境光亮度传感器数据
    short int ps;         //接近传感器数据
    short int ir;         //红外LED
};

struct ap3216c_device {
    int irq;                        /* 中断号 */
    int gpio;
    dev_t dev_no;                   /* 设备号 */    
    struct cdev chrdev;             
    struct class *class;
    struct mutex  m_lock;           
    wait_queue_head_t  wq;          /* 等待队列 */
    struct ap3216c_data  data;
    struct i2c_client *client;
};

static struct ap3216c_device  *ap3216c_dev;


static int ap3216c_i2c_write_reg(struct i2c_client *client, uint8_t reg_addr, uint8_t data)
{
    int ret = 0;
    
#if 0    
    struct i2c_msg msgs[2];
    msgs[0].addr = client->addr;    
    msgs[0].buf = &reg_addr;           /* 发送寄存器地址 */
    msgs[0].flags = 0;                  /* I2C方向 ：写数据 */
    msgs[0].len = sizeof(reg_addr);

    msgs[1].addr = client->addr;
    msgs[1].buf = &data;                /* 写数据 */    
    msgs[1].flags = 0;                  /* I2C方向 ：写数据 */
    msgs[1].len = sizeof(data);
#else
    uint8_t buf[2];
    
    struct i2c_msg msg;

    buf[0] = reg_addr;
    buf[1] = data;

    msg.addr = client->addr;    
    msg.buf = buf;           /* 发送寄存器地址 */
    msg.flags = 0;                  /* I2C方向 ：写数据 */
    msg.len = 2;
#endif
    ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;
    
    return 0;
}

static int ap3216c_i2c_read_reg(struct i2c_client *client, uint8_t reg_addr, uint8_t *data)
{
    int ret = 0;
    struct i2c_msg msgs[2];
    
    msgs[0].addr = client->addr;
    msgs[0].buf = &reg_addr;              /* 发送寄存器地址 */
    msgs[0].flags = 0;                    /* I2C方向 ：写数据 */
    msgs[0].len = sizeof(reg_addr);

    msgs[1].addr = client->addr;
    msgs[1].buf = data;                    /* 读取寄存器数据 */                            
    msgs[1].flags = I2C_M_RD;              /* I2C方向 ：读数据 */
    msgs[1].len = 1;

    ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		return ret;
	else if (ret != 2)
		return -EIO;
    
    return 0;
}

static int ap3216c_init(void)
{
    int ret; 
    
    ret = ap3216c_i2c_write_reg(ap3216c_dev->client, AP3216C_REG_SYS_CONFIG, AP3216C_SYS_CONF_SW_RESET);           /* 软复位 */
    msleep(50);
    ret = ap3216c_i2c_write_reg(ap3216c_dev->client, AP3216C_REG_SYS_CONFIG, AP3216C_SYS_CONF_ALS_PS_IR_ACTIVE);   /* 开启ALS、PS、IR*/
    return ret;
}

static int ap3216c_read_als(short int *als)
{
    int ret;
    uint8_t low, high;

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_ALS_DATA_LOW, &low);

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_ALS_DATA_HIGH, &high);

    *als = (high << 8) + low;

//    printk("low %d high %d\n", low, high);

    return ret;
}

static int ap3216c_read_ps(short int *ps)
{
    int ret;
    uint8_t low, high;

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_PS_DATA_LOW, &low);

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_PS_DATA_HIGH, &high);

    *ps = (high << 8) + low;

    return ret;
}

static int ap3216c_read_ir(short int *ir)
{
    int ret;
    uint8_t low, high;

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_IR_DATA_LOW, &low);

    ret = ap3216c_i2c_read_reg(ap3216c_dev->client, AP3216C_REG_IR_DATA_HIGH, &high);

    *ir = (high << 8) + low;

    return ret;
}


static int _drv_open (struct inode *node, struct file *file)
{
    int ret = 0;
    file->private_data = ap3216c_dev;

    if ((ret = ap3216c_init()) != 0)
    {
        printk("ap3216c init failed %d\r\n", ret);
    }

    printk("%s open\r\n", DEV_NAME);
    return ret;
}

static ssize_t _drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret = 0;
    struct ap3216c_device *tmp_ap3216c = filp->private_data;

    /* 中断屏蔽 */
    if (size != sizeof(struct ap3216c_data)) return -EINVAL;

    ret = ap3216c_read_als(&ap3216c_dev->data.als);
    ret = ap3216c_read_ps(&ap3216c_dev->data.ps);
    ret = ap3216c_read_ir(&ap3216c_dev->data.ir);

	if (copy_to_user(buf, &tmp_ap3216c->data, sizeof(struct ap3216c_data))) 
    {
        ret = -EFAULT;
    } 
    else 
    {
        ret = sizeof(struct ap3216c_data);
    }
    return ret;
}

/* 使驱动支持多路复用IO */
static __poll_t _drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    __poll_t mask = 0;

    // mutex_lock(&ap3216c_dev->m_lock);

    // poll_wait(filp, &ap3216c_dev->wq, wait); 


    // mutex_unlock(&sr501.m_lock);

    return mask;
}

static long _drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    return ret;
}

static int _drv_release(struct inode *node, struct file *filp)
{
    struct ap3216c_device *tmp_ap3216c = filp->private_data;

    printk("%s close\n", DEV_NAME);
    return 0;
}

static struct file_operations ap3216c_drv_ops = { 
	.owner	= THIS_MODULE,
	.open   = _drv_open,
    .read   = _drv_read,
    .poll   = _drv_poll,
    .release = _drv_release,
};


static irqreturn_t ap3216c_isr(int irq, void *dev)
{
    printk("%s %d %s\n", __FILE__, __LINE__, __FUNCTION__);
    return IRQ_RETVAL(IRQ_HANDLED);   
}

static int _driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{ 
    int err = 0;
    int ret;
    struct device *_dev;
    struct device_node *_dts_node;

    ap3216c_dev = (struct ap3216c_device *) kzalloc(sizeof(struct ap3216c_device), GFP_KERNEL);
    if (!ap3216c_dev) 
    {
        printk("can't kzalloc ap3216c\n");
        return -ENOMEM;
    }

    _dts_node = client->dev.of_node; 
    if (!_dts_node) {          
        printk("AP3216C dts node can not found!\r\n");    
        err = -EINVAL; 
        goto out_free_dev;
    }

    ap3216c_dev->client = client;

    printk("AP3216C irq %d !\r\n", client->irq);    
#if 1
    // ap3216c_dev->gpio = of_get_named_gpio(_dts_node, AP3216C_READY_GPIO_NAME, 0);   /* 获取gpio */
    // if (!gpio_is_valid( ap3216c_dev->gpio)) {
    //     printk("don't get ap3216c ready gpio!\n"); 
	// 	err = -EINVAL;
    //     goto out_free_dev;
	// }

    // ap3216c_dev->irq = gpio_to_irq( ap3216c_dev->gpio);                              /* 通过gpio得到irq */

    // gpiod_direction_input(ap3216c_dev->gpio);          /* 设置gpio为输入 */
    ap3216c_dev->irq = client->irq;
    err = request_irq(ap3216c_dev->irq, ap3216c_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DEV_NAME, NULL);    /* 申请中断 */
    if (err) {
        printk(KERN_INFO"failed to request irq %d\r\n", ap3216c_dev->irq);
        goto out_free_dev;
    }
    /* 内核自动分配设备号 */
    err = alloc_chrdev_region(&ap3216c_dev->dev_no, 0, 1, DEV_NAME);        
	if (err < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", err);

        goto out_free_irq;
	}

	cdev_init(&ap3216c_dev->chrdev, &ap3216c_drv_ops);

	err = cdev_add(&ap3216c_dev->chrdev, ap3216c_dev->dev_no, 1);
    if (err) {
        goto out_unregister;
    }

    ap3216c_dev->class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(ap3216c_dev->class)) { 
        err = PTR_ERR(ap3216c_dev->class);
        goto out_cdev_del;
	}

    /* 创建设备节点 */
    _dev = device_create(ap3216c_dev->class , NULL, ap3216c_dev->dev_no, NULL, DEV_NAME); 
    if (IS_ERR(_dev)) {       /* 判断指针是否合法 */
        err = PTR_ERR(_dev);
		goto out_class_del;
	}

    // mutex_init(&ap3216c_dev->m_lock);                 /* 初始化互斥锁  */   

    printk("ap3216c probe success\r\n");
    goto out;

out_class_del:
    class_destroy(ap3216c_dev->class);
out_cdev_del:
    cdev_del(&ap3216c_dev->chrdev);
out_unregister:
    unregister_chrdev_region(ap3216c_dev->dev_no, 1);      /* 注销设备 */
out_free_irq:
    free_irq(ap3216c_dev->irq, NULL);
out_free_dev:
    kfree(ap3216c_dev);
    ap3216c_dev = NULL;
out:
#endif
    return err;
}

static int _driver_remove(struct i2c_client *client)
{
    device_destroy(ap3216c_dev->class, ap3216c_dev->dev_no);
	class_destroy(ap3216c_dev->class);
	unregister_chrdev_region(ap3216c_dev->dev_no, 1);      /* 注销设备 */
    cdev_del(&ap3216c_dev->chrdev);
    free_irq(ap3216c_dev->irq, NULL);                      /* 释放中断*/
    kfree(ap3216c_dev);
    printk(KERN_INFO"%s success\n", DEV_NAME);

    return 0;
}

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = AP3216C_DTS_COMPATIBLE, },                     /* 通过设备树来匹配 */
};

/* 传统匹配方式 ID 列表 ，即使不使用也要添加，不然probe匹配不成功 */ 
static const struct i2c_device_id ap3216c_id[] = { 
   {AP3216C_DTS_COMPATIBLE, 0},   
}; 

static struct i2c_driver ap3216c_driver = {
    .probe = _driver_probe,
    .remove = _driver_remove,
    .driver = {
        .name = AP3216C_DTS_COMPATIBLE,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,                      /* 通过设备树匹配 */
    },
    .id_table = ap3216c_id,
};

/* 入口函数 */ 
static int __init _driver_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&ap3216c_driver);                  /* 注册I2C驱动 */
    printk("%s %s\n", DEV_NAME, __FUNCTION__);
    return ret;
}

/*  出口函数 */
static void __exit _driver_exit(void)
{
    printk("%s driver %s\n", DEV_NAME, __FUNCTION__);
    i2c_del_driver(&ap3216c_driver);                      /* 注销I2C驱动 */
}

module_init(_driver_init);
module_exit(_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















