/*
 * @brief : HY46xx Touch IC Driver   
 * @date :  2022-02-xx
 * @version : v1.0.0
 * @Change Logs:   
 * @date         author         notes:  
 * 2022/3/8      guangjieMVP    first version 
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
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/wait.h>
#include <linux/sched/signal.h>
#include <linux/poll.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define HY_CHIP_ID_REG          0xA9
#define HY_TP_RUN_MODE_REG        0x00        /* 0x00 工作模式  0xc0 测试模式 */
#define HY_FITLTER_REG          0x8A        /* 滤波器  0-5 */
#define HY_PWR_NOISE_REG        0x89        /* 电源噪声 0 off  1 on */
#define HY_FW_VERSION_REG       0xA6       
#define HY_LIB_VERSION_REG      0xA7    
#define HY_PWR_MODE_REG         0xA5        /* 0x03 tp enter sleep  需要 reset pin 拉 low 喚醒 */
#define HY_REPORT_SPEED_REG     0x88        /* 报点率设置 0x64 */

#define HY_GSTID_REG 		    0x02   	    /* 当前检测到的触摸情况 */
#define HY_TP1_XH_REG 		    0X03  	    /* 第一个触摸点数据地址 */
#define HY_TP1_XL_REG 		    0X04  	    /* 第一个触摸点数据地址 */
#define HY_TP1_YH_REG 		    0X05  	    /* 第一个触摸点数据地址 */
#define HY_TP1_YL_REG 		    0X06  	    /* 第一个触摸点数据地址 */

#define HY_TP2_REG 		        0X09		/* 第二个触摸点数据地址 */
#define HY_TP3_REG 		        0X0F		/* 第三个触摸点数据地址 */
#define HY_TP4_REG 		        0X15		/* 第四个触摸点数据地址 */
#define HY_TP5_REG 		        0X1B		/* 第五个触摸点数据地址 */  

#define HY_MAX_SUPPORT_POINTS           5

#define HY46XX_HOR_RES    1024
#define HY46XX_VER_RES    600

#ifndef HY_COUNTOF
#define  HY_COUNTOF(a)           (sizeof(a)/sizeof(a[0]))
#endif

#define  DEV_NAME                   "hy46xx,ts"
#define  HY46XX_DTS_COMPATIBLE       "hy46xx,ts"

#define  HY46XX_DTS_IRQ_GPIO_NAME     "irq-gpios"
#define  HY46XX_DTS_RST_GPIO_NAME     "reset-gpios"

/*  
    reset-gpios = <&gpio3 4 GPIO_ACTIVE_LOW>;
    irq-gpios = <&gpio5 9 GPIO_ACTIVE_HIGH>;
    //interrupt­
    interrupt-parent = <&gpio5>;
    interrupts = <9 IRQ_TYPE_EDGE_FALLING>;
    irq-flags = <2>;        /*1:rising 2: falling*/*/

enum hy46xx_ts_state {
	HY46XX_TS_DOWN    = 0x00,      /* 按下 */
	HY46XX_TS_UP      = 0x01,        /* 弹起 */
	HY46XX_TS_CONTACT = 0x02,       /* 持续性按下 */
    HY46XX_TS_RESERVED = 0x03,  
};

struct touch_data {
    short x;
    short y;
    uint8_t state;
    uint8_t id;
};

struct hy46xx_device {
    int irq; /* 中断号 */
    int irq_gpio;
    int rst_gpio;
    dev_t dev_no; /* 设备号 */
    struct i2c_client *i2c;
    struct input_dev *inputdev;    /* input 结构体 */ 
    struct touch_data ts_data[HY_MAX_SUPPORT_POINTS];
};

static struct hy46xx_device *hy46xx_dev;

static int hy46xx_write_regs(struct i2c_client *i2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    int ret = 0;
    uint8_t w_buf[300] = {0};

    struct i2c_msg msg;

    w_buf[0] = reg;
    memcpy(&w_buf[1], buf, len);

    msg.addr = i2c->addr;
    msg.buf = w_buf;
    msg.flags = 0;               /* I2C direction ： write */
    msg.len = len + 1;

    ret = i2c_transfer(i2c->adapter, &msg, 1);
    if (ret < 0)
        return ret;
    else if (ret != 1)
        return -EIO;

    return 0;
}

static int hy46xx_write_reg(struct i2c_client *client, uint8_t reg, uint8_t data)
{
    return hy46xx_write_regs(client, reg, &buf, 1);
}

static int hy46xx_read_regs(struct i2c_client *client, uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msgs[2];
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 1;
	msgs[0].buf   = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret < 0)
        return ret;
    else if (ret != 2)
        return -EIO;
    return 0;
}

static int hy46xx_i2c_read_reg(struct i2c_client *client, uint8_t reg, uint8_t *data)
{
    return hy46xx_read_regs(client, reg, data, 1);
}

static int hy46xx_read_touch_data(struct hy46xx_device *dev)
{
    uint8_t buf[29];
    uint8_t *ts;
    int ret;

    ret =  hy46xx_read_regs(dev->i2c, HY_TP1_XH_REG, buf, 29);
    if (ret != 0) return ret;

    for (i = 0; i < HY_MAX_SUPPORT_POINTS; i++)
    {
        ts = &buf[i * 6];
        dev->ts_data.state = (ts[0] & 0xC0) >> 6;
        dev->ts_data->x = ((ts[0] & 0x0f) << 8) | ts[1];
        dev->ts_data->y = ((ts[2] & 0x0f) << 8) | ts[3];
        dev->ts_data.id = (ts[2] & 0xf0) >> 4;            
    }

    return 0;
}

static int hy46xx_set_tp_run_mode(struct hy46xx_device *dev)
{
    return hy46xx_write_reg(dev->i2c, HY_TP_RUN_MODE_REG, 0x00);
}


static int hy46xx_ts_reset(struct hy46xx_device *dev)
{
    if (gpio_is_valid(dev->rst_gpio))   /* 检查 IO 是否有效 */ 
    {
        gpio_set_value(dev->rst_gpio, 0);
        msleep(5);
        gpio_set_value(dev->rst_gpio, 1);
        msleep(1000);
    }
}

static void hy46xx_report_events(struct hy46xx_device *dev)
{
	int i;
	bool touch;
	unsigned int x, y;

	for (i = 0; i < HY_MAX_SUPPORT_POINTS; i++) 
    {
		input_mt_slot(dev->inputdev, dev->ts_data[i].id);                  /* ABS_MT_SLOT事件  上报触摸ID  */

        if (dev->ts_data[i].state == HY46XX_TS_RESERVED) continue;         

		touch = dev->ts_data[i].state != HY46XX_TS_UP;                     /* 当触摸按下touch为true，弹起时touch为false */

		input_mt_report_slot_state(dev->inputdev, MT_TOOL_FINGER, touch);    /* ABS_MT_TRACKING_ID 事件 */

		if (touch)                                                        
        {
			input_report_abs(dev->inputdev, ABS_MT_POSITION_X, dev->ts_data[i].x);    /* 上报触摸点的x坐标 */
			input_report_abs(dev->inputdev, ABS_MT_POSITION_Y, dev->ts_data[i].y);    /* 上报触摸点的y坐标 */
		}
	}

	input_mt_report_pointer_emulation(dev->inputdev, true);
	input_sync(dev->inputdev);                                            
}

static irqreturn_t hy46xx_ts_isr(int irq, void *dev)
{
    struct hy46xx_device *dev = (struct hy46xx_device *)dev;

    /* 上报触摸点数据 */
    hy46xx_read_touch_data(dev);
    hy46xx_report_events(dev);

    return IRQ_RETVAL(IRQ_HANDLED);
}

static int hy46xx_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    struct device *_dev;
    struct device_node *dev_node;

    hy46xx_dev = (struct hy46xx_device *)kzalloc(sizeof(struct hy46xx_device), GFP_KERNEL);
    if (!hy46xx_dev)
    {
        printk("can't kzalloc mpu6050 dev\n");
        return -ENOMEM;
    }

    dev_node = client->dev.of_node;
    if (!dev_node)
    {
        printk("hy46xx ts dts node can't found!\r\n");
        err = -EINVAL;
        goto exit_free_dev;
    }

    hy46xx_dev->i2c = client;
    printk("hy46xx dts irq %d !\r\n", client->irq);

    hy46xx_dev->irq_gpio = of_get_named_gpio(dev_node, HY46XX_DTS_IRQ_GPIO_NAME, 0);   /* 获取irq-gpios */
    if (!gpio_is_valid( hy46xx_dev->irq_gpio)) {
        printk("don't get %s %s!\n", DEV_NAME, HY46XX_DTS_IRQ_GPIO_NAME);
    	err = -EINVAL;
        goto exit_free_dev;
    }
    hy46xx_dev->irq = gpio_to_irq( hy46xx_dev->irq_gpio);                              /* 通过gpio得到irq */
    
    hy46xx_dev->rst_gpio = of_get_named_gpio(dev_node, HY46XX_DTS_RST_GPIO_NAME, 0);   /* 获取rst-gpios */
    if (!gpio_is_valid( hy46xx_dev->rst_gpio)) {
        printk("don't get %s %s!\n", DEV_NAME, HY46XX_DTS_RST_GPIO_NAME);
    	err = -EINVAL;
        goto exit_free_dev;
    }

    printk("%s %d, %s %d", HY46XX_DTS_IRQ_GPIO_NAME, hy46xx_dev->irq_gpio, HY46XX_DTS_RST_GPIO_NAME, hy46xx_dev->rst_gpio);

#if 0
    err = request_irq(hy46xx_dev->irq, hy46xx_ts_isr, RQF_TRIGGER_FALLING, DEV_NAME, NULL);    
#else
   /* 中断线程化 */
    err = devm_request_threaded_irq(&client->dev, client->irq, NULL, 
                                    hy46xx_ts_isr, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, 
                                    client->name, hy46xx_dev);
#endif
    if (err)
    {
        printk(KERN_INFO "failed to request irq %d\r\n", hy46xx_dev->irq);
        goto exit_free_dev;
    }

    hy46xx_dev->inputdev =  devm_input_allocate_device(&client->dev);;    /* 申请输入设备结构 */
    if (!hy46xx_dev->inputdev) 
    {
        printk(KERN_ERR "%s: Failed to allocate input device.\n", __func__);
        err = -ENOMEM;
        goto exit_free_irq;
    }
    hy46xx_dev->inputdev->name = DEV_NAME;
    hy46xx_dev->inputdev->id.bustype = BUS_I2C;      /* 触摸屏通信总线类型 */
    input->dev.parent = &client->dev;

    /* Single touch */
	input_set_abs_params(hy46xx_dev->inputdev, ABS_X, 0, HY46XX_HOR_RES, 0, 0);
	input_set_abs_params(hy46xx_dev->inputdev, ABS_Y, 0, HY46XX_VER_RES, 0, 0);

    input_set_abs_params(hy46xx_dev->inputdev, ABS_MT_POSITION_X, 0, HY46XX_HOR_RES, 0, 0);
	input_set_abs_params(hy46xx_dev->inputdev, ABS_MT_POSITION_Y, 0, HY46XX_VER_RES, 0, 0);

    err = input_mt_init_slots(hy46xx_dev->inputdev, HY_MAX_SUPPORT_POINTS, 0);        /* 初始化触摸屏，5点触摸 */
	if (err) {
		dev_err(&client->dev,
			"Failed to initialize MT slots: %d", error);
		goto exit_free_irq;
	}

    err = input_register_device(hy46xx_dev->inputdev);           /* 注册input_device */                
    if (err) 
    {
        printk(KERN_ERR "%s: Failed to regist key device.\n", __func__);
        goto exit_free_irq;
    }

    gpio_direction_input(hy46xx_dev->irq_gpio);
    gpio_direction_output(hy46xx_dev->rst_gpio, 0); 

    hy46xx_ts_reset();     /* 复位 */

    hy46xx_set_tp_run_mode();

    goto exit;
exit_free_irq:                                   
    free_irq(hy46xx_dev->irq, NULL);             /* 释放中断*/
exit_free_dev:
    kfree(hy46xx_dev);
    hy46xx_dev = NULL;
exit:
    return err;
}

static int hy46xx_driver_remove(struct i2c_client *i2c)
{
    /* 释放内存 */    
    kfree(hy46xx_dev);

    printk(KERN_INFO "%s success\n", DEV_NAME);

    return 0;
}

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = HY46XX_DTS_COMPATIBLE}, /* 通过设备树来匹配 */
    {},
};

/* 传统匹配方式 ID 列表 ，即使不使用也要添加，不然probe匹配不成功 */
static const struct i2c_device_id id_table[] = {
    {.name = HY46XX_DTS_COMPATIBLE, 0},
    {},
};

static struct i2c_driver hy46xx_driver = {
    .probe = hy46xx_driver_probe,
    .remove = hy46xx_driver_remove,
    .driver = {
        .name = HY46XX_DTS_COMPATIBLE,
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table, /* 通过设备树匹配 */
    },
    .id_table = id_table,
};

#if 1
module_i2c_driver(hy46xx_driver);
#else
/* 入口函数 */
static int __init _driver_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&hy46xx_driver); /* 注册I2C驱动 */
    printk("%s %s\n", DEV_NAME, __FUNCTION__);
    return ret;
}

/*  出口函数 */
static void __exit _driver_exit(void)
{
    printk("%s driver %s\n", DEV_NAME, __FUNCTION__);
    i2c_del_driver(&hy46xx_driver); /* 注销I2C驱动 */
}

module_init(_driver_init);
module_exit(_driver_exit);
#endif

MODULE_AUTHOR("xuguangjie");
MODULE_LICENSE("GPL");