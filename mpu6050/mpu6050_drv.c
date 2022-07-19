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
#include <linux/input.h>

#define HY46XX_DTS_NODE_PATH "xgj-mpu6050"
#define HY46XX_DTS_COMPATIBLE "xgj,hy46xx"
#define HY46XX_READY_GPIO_NAME "irq-gpios"

#define MPU6050_USE_INT      1

struct mpu6050_device
{
    int irq; /* 中断号 */
    int gpio;
    dev_t dev_no; /* 设备号 */
    struct cdev chrdev;
    struct class *class;
    struct mutex m_lock;
    wait_queue_head_t wq; /* 等待队列 */
    struct mpu6050_data data;
    struct i2c_client *client;
};

static struct mpu6050_device *mpu6050_dev;

static int mpu6050_i2c_write_reg(struct i2c_client *client, uint8_t reg_addr, uint8_t data)
{
    int ret = 0;
    uint8_t w_buf[2];

    struct i2c_msg msg;

    w_buf[0] = reg_addr;
    w_buf[1] = data;

    msg.addr = client->addr;
    msg.buf = w_buf;
    msg.flags = 0; /* I2C direction ： write */
    msg.len = sizeof(w_buf);

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
        return ret;
    else if (ret != 1)
        return -EIO;

    return 0;
}

static int mpu6050_i2c_read_reg(struct i2c_client *client, uint8_t reg_addr, uint8_t *data)
{
    int ret = 0;
    struct i2c_msg msgs[2];

    msgs[0].addr = client->addr;
    msgs[0].buf = &reg_addr; /* send regsiter */
    msgs[0].flags = 0;       /* I2C direction ： write */
    msgs[0].len = 1;

    msgs[1].addr = client->addr;
    msgs[1].buf = data;       /* 读取寄存器数据 */
    msgs[1].flags = I2C_M_RD; /* I2C direction ： read */
    msgs[1].len = 1;

    ret = i2c_transfer(client->adapter, msgs, 2); /* */
    if (ret < 0)
        return ret;
    else if (ret != 2)
        return -EIO;

    return 0;
}

static int mpu6050_init(void)
{
    uint8_t reg_val = 0;
    int ret = 0;

    /* 解除休眠状态 */
    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_PWR_MGMT_1, 0x00); /*  */

    /*  陀螺仪采样频率输出设置 */
    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_SMPLRT_DIV, 0x07);

    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_CONFIG, 0x06);

    /* 配置加速度传感器工作在 16G 模式, 不自检 */
    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_ACCEL_CONFIG, 0x18);

    /* 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_GYRO_CONFIG, 0x18);
#if MPU6050_USE_INT 
    /* 配置中断产生时中断引脚为低电平 */
    ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_INT_PIN_CFG, &reg_val);
    reg_val |= 0xC0;
    mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_INT_PIN_CFG, reg_val);

    /* 开启数据就绪中断 */
    ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_INT_ENABLE, &reg_val);
    reg_val |= 0x01;
    mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_INT_ENABLE, reg_val);
#endif
    return ret;
}

static int mpu6050_deinit(void)
{
    int ret = 0;
    /* mpu6050复位, 寄存器恢复默认值 */
    ret = mpu6050_i2c_write_reg(mpu6050_dev->client, MPU6050_PWR_MGMT_1, 0x80);
    return ret;
}

static int mpu6050_read_id(uint8_t *id)
{
    uint8_t data;
    int ret = 0;

    ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_WHO_AM_I, &data);

    if (id != NULL)
        *id = data;

    if (data != MPU6050_IIC_ADDR)
        ret = -1;

    return ret;
}

static int mpu6050_read_accel(struct mpu6050_accel *acc)
{
    int i = 0;
    int ret = 0;
    uint8_t data[6] = {0};

    for (i = 0; i < 6; i++)
    {
        ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_ACCEL_XOUT_H+i, &data[i]);
    }
    acc->x = (data[0] << 8) + data[1];
    acc->y = (data[2] << 8) + data[3];
    acc->z = (data[4] << 8) + data[5];
    return ret;
}

static int mpu6050_read_gyro(struct mpu6050_gyro *gyro)
{
    int i = 0;
    int ret = 0;
    uint8_t data[6] = {0};

    for (i = 0; i < 6; i++)
    {
        ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_GYRO_XOUT_H+i, &data[i]);
    }
    gyro->x = (data[0] << 8) + data[1];
    gyro->y = (data[2] << 8) + data[3];
    gyro->z = (data[4] << 8) + data[5];
    return ret;
}

static int mpu6050_read_temp(short *temp)
{
    int i = 0;
    int ret = 0;
    uint8_t data[2] = {0};

    for (i = 0; i < 2; i++)
    {
        ret = mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_TEMP_OUT_H+i, &data[i]);
    }
    *temp = (data[0] << 8) + data[1];
    return ret;
}

static int _drv_open(struct inode *node, struct file *file)
{
    uint8_t id;
    int ret = 0;

    file->private_data = mpu6050_dev;

    ret = mpu6050_init();
    if (ret != 0)
    {
        printk("%d-%s init failed %d\r\n", __LINE__, __FUNCTION__, ret);
        return -ENXIO;
    }

    if (mpu6050_read_id(&id) != 0)
    {
        printk("don't find %s %d\r\n", DEV_NAME, id);
        return -ENXIO;
    }

    gpio_direction_input(mpu6050_dev->gpio);

    printk("%s open %d\r\n", DEV_NAME, __LINE__);
    return 0;
}

static ssize_t _drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret;
    int data_size;
    struct mpu6050_device *tmp_mpu6050 = filp->private_data;
    struct mpu6050_data data;
    
    data_size = sizeof(data);
    if (size != data_size)
        return -EINVAL;
#if MPU6050_USE_INT == 0
    ret = mpu6050_read_accel(&data.accel);

    ret = mpu6050_read_gyro(&data.gyro);
#else

#endif
    if (copy_to_user(buf, &data, sizeof(data)))
    {
        ret = -EFAULT;
    }
    else
    {
        ret = data_size;
    }
    return ret;
}

/* 使驱动支持多路复用IO */
static __poll_t _drv_poll(struct file *filp, struct poll_table_struct *wait)
{
    __poll_t mask = 0;

    // mutex_lock(&mpu6050_dev->m_lock);

    // poll_wait(filp, &mpu6050_dev->wq, wait);

    // mutex_unlock(&sr501.m_lock);

    return mask;
}

static long _drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    uint8_t result = 0;
    short buf[3];
    void __user *user_space = (void __user *)arg;

    switch (cmd)
    {
        case MPU6050_CMD_GET_ID:
            ret = mpu6050_read_id(&result);
            ret = copy_to_user(user_space, &result, sizeof(result));
            break;
        case MPU6050_CMD_GET_ACCEL:
            ret = mpu6050_read_accel((struct mpu6050_accel *)&buf);
            ret = copy_to_user(user_space, &buf, sizeof(buf));
            break;
        case MPU6050_CMD_GET_GYRO:
            ret = mpu6050_read_gyro((struct mpu6050_gyro *)&buf);
            ret = copy_to_user(user_space, &buf, sizeof(buf));
            break;
        case MPU6050_CMD_GET_TEMP:
            ret = mpu6050_read_temp(&buf[0]);
            ret = copy_to_user(user_space, &buf[0], 2);
            break;
    }

    return ret;
}

static int _drv_release(struct inode *node, struct file *filp)
{
    int ret = 0;
    struct mpu6050_device *tmp_mpu6050 = filp->private_data;

    ret = mpu6050_deinit();  
    return ret;
}

static struct file_operations mpu6050_drv_ops = {
    .owner = THIS_MODULE,
    .open = _drv_open,
    .read = _drv_read,
    .poll = _drv_poll,
    .release = _drv_release,
};

static irqreturn_t mpu6050_isr(int irq, void *dev)
{
    uint8_t reg_val;
    // printk("%s %d %s\n", __FILE__, __LINE__, __FUNCTION__);
    mpu6050_i2c_read_reg(mpu6050_dev->client, MPU6050_INT_STATUS, &reg_val);
    if (reg_val & 0x01)     
    {
        /* 唤醒进程去读取数据 */

        printk("data ready\n");
    }
    return IRQ_RETVAL(IRQ_HANDLED);
}

static int _driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    // int ret = 0;
    struct device *_dev;
    struct device_node *_dts_node;

    mpu6050_dev = (struct mpu6050_device *)kzalloc(sizeof(struct mpu6050_device), GFP_KERNEL);
    if (!mpu6050_dev)
    {
        printk("can't kzalloc mpu6050 dev\n");
        return -ENOMEM;
    }

    _dts_node = client->dev.of_node;
    if (!_dts_node)
    {
        printk("mpu6050 dts node can not found!\r\n");
        err = -EINVAL;
        goto out_free_dev;
    }

    mpu6050_dev->client = client;

    printk("mpu6050 dtsirq %d !\r\n", client->irq);
#if 1
    // mpu6050_dev->gpio = of_get_named_gpio(_dts_node, AP3216C_READY_GPIO_NAME, 0);   /* 获取gpio */
    // if (!gpio_is_valid( mpu6050_dev->gpio)) {
    //     printk("don't get mpu6050 ready gpio!\n");
    // 	err = -EINVAL;
    //     goto out_free_dev;
    // }

    // mpu6050_dev->irq = gpio_to_irq( mpu6050_dev->gpio);                              /* 通过gpio得到irq */

    // gpiod_direction_input(mpu6050_dev->gpio);          /* 设置gpio为输入 */
    mpu6050_dev->irq = client->irq;
    // mpu6050_dev->gpio = irq_to_gpio(mpu6050_dev->irq);
    err = request_irq(mpu6050_dev->irq, mpu6050_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, DEV_NAME, NULL); /* 申请中断 */
    if (err)
    {
        printk(KERN_INFO "failed to request irq %d\r\n", mpu6050_dev->irq);
        goto out_free_dev;
    }
    /* 内核自动分配设备号 */
    err = alloc_chrdev_region(&mpu6050_dev->dev_no, 0, 1, DEV_NAME);
    if (err < 0)
    {
        pr_err("Error: failed to register mbochs_dev, err: %d\n", err);

        goto out_free_irq;
    }

    cdev_init(&mpu6050_dev->chrdev, &mpu6050_drv_ops);
    err = cdev_add(&mpu6050_dev->chrdev, mpu6050_dev->dev_no, 1);
    if (err)
    {
        goto out_unregister;
    }

    mpu6050_dev->class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(mpu6050_dev->class))
    {
        err = PTR_ERR(mpu6050_dev->class);
        goto out_cdev_del;
    }

    /* 创建设备节点 */
    _dev = device_create(mpu6050_dev->class, NULL, mpu6050_dev->dev_no, NULL, DEV_NAME);
    if (IS_ERR(_dev))
    { /* 判断指针是否合法 */
        err = PTR_ERR(_dev);
        goto out_class_del;
    }

    // mutex_init(&mpu6050_dev->m_lock);                 /* 初始化互斥锁  */

    printk("%s probe success\r\n", DEV_NAME);

    goto out;

out_class_del:
    class_destroy(mpu6050_dev->class);
out_cdev_del:
    cdev_del(&mpu6050_dev->chrdev);
out_unregister:
    unregister_chrdev_region(mpu6050_dev->dev_no, 1); /* 注销设备 */
out_free_irq:
    free_irq(mpu6050_dev->irq, NULL);
out_free_dev:
    kfree(mpu6050_dev);
    mpu6050_dev = NULL;
out:
#endif
    return err;
}

static int _driver_remove(struct i2c_client *client)
{
    device_destroy(mpu6050_dev->class, mpu6050_dev->dev_no);
    class_destroy(mpu6050_dev->class);
    cdev_del(&mpu6050_dev->chrdev);
    unregister_chrdev_region(mpu6050_dev->dev_no, 1); /* 注销设备 */
    free_irq(mpu6050_dev->irq, NULL); /* 释放中断*/
    kfree(mpu6050_dev);
    printk(KERN_INFO "%s success\n", DEV_NAME);

    return 0;
}

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = MPU6050_DTS_COMPATIBLE}, /* 通过设备树来匹配 */
    {},
};

/* 传统匹配方式 ID 列表 ，即使不使用也要添加，不然probe匹配不成功 */
static const struct i2c_device_id mpu6050_id[] = {
    {MPU6050_DTS_COMPATIBLE, 0},
    {},
};

static struct i2c_driver mpu6050_driver = {
    .probe = _driver_probe,
    .remove = _driver_remove,
    .driver = {
        .name = MPU6050_DTS_COMPATIBLE,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table, /* 通过设备树匹配 */
    },
    .id_table = mpu6050_id,
};

/* 入口函数 */
static int __init _driver_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&mpu6050_driver); /* 注册I2C驱动 */
    printk("%s %s\n", DEV_NAME, __FUNCTION__);
    return ret;
}

/*  出口函数 */
static void __exit _driver_exit(void)
{
    printk("%s driver %s\n", DEV_NAME, __FUNCTION__);
    i2c_del_driver(&mpu6050_driver); /* 注销I2C驱动 */
}

module_init(_driver_init);
module_exit(_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");