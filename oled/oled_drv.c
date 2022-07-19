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
#include <linux/spi/spi.h>

#include "oled_def.h"

#define DEV_NAME "oled" /* /dev/oled */
#define OLED_DTS_NODE_PATH "xgj-oled"
#define OLED_DTS_COMPATIBLE "xgj,oled"
#define DC_GPIO_DEV_NODE_PATH "/ecspi1/xgj_oled"
#define DC_GPIO_DTS_NAME "dc-gpios"
#define RST_GPIO_DTS_NAME "rst-gpios"

enum {
    OLED_CMD = 0x00,
    OLED_DATA = 0x01,
};
typedef unsigned char oled_cmd_t;

struct oled_device
{
    int dc_gpio;
    int rst_gpio;
    dev_t dev_no; /* 设备号 */
    struct cdev chrdev;
    struct class *class;
    struct mutex m_lock;
    wait_queue_head_t wq; /* 等待队列 */
    struct spi_device *spi;
    uint8_t databuf[1024];
};

static struct oled_device *oled_dev;

static int oled_spi_write(char *buf, uint16_t len)
{
    int status;
#if 0
    struct spi_message msg;
	struct spi_transfer xfer = {
		.len = len,
		.tx_buf = buf,
	};

	spi_message_init(&msg);                       /* 初始化spi_message */
	spi_message_add_tail(&xfer, &msg);             /* 添加到传输队列 */
    status = spi_sync(oled_dev->spi, &msg);          /* 同步发送 */
#else
    status = spi_write(oled_dev->spi, buf, len);
#endif
    return status;
}

static int oled_write_cmd_data(uint8_t data, oled_cmd_t cmd)
{
    int ret = 0;

    if (cmd == OLED_CMD)
        gpio_set_value(oled_dev->dc_gpio, OLED_CMD); /* 拉低，表示写入的是指令 */
    else
        gpio_set_value(oled_dev->dc_gpio, OLED_DATA); /* 拉高，表示写入数据 */
    ret = oled_spi_write(&data, sizeof(data));
    return ret;
}

static int oled_write_datas(uint8_t *datas, uint16_t len)
{
    int ret = 0;

    gpio_set_value(oled_dev->dc_gpio, OLED_DATA);
    ret = oled_spi_write(datas, len);
    return ret;
}

static int oled_set_pos(uint16_t x, uint16_t y)
{
    int ret = 0;

    ret = oled_write_cmd_data(0xb0 + y, OLED_CMD);
    ret = oled_write_cmd_data((x & 0x0f), OLED_CMD);
    ret = oled_write_cmd_data(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    return ret;
}

static void oled_set_rst(uint8_t on_off)
{
    gpio_set_value(oled_dev->rst_gpio, on_off);
}

static void oled_reset(void)
{
    oled_set_rst(0);
    mdelay(50);
    oled_set_rst(1);
}

static void oled_disp_on_off(uint8_t on_off)
{
    if (on_off)
        oled_write_cmd_data(0xaf, OLED_CMD); /* set dispkay on */
    else
        oled_write_cmd_data(0xae, OLED_CMD); /* set dispkay off */
}

static void oled_disp_clear(void)
{
    uint8_t x, y;

    for (y = 0; y < 8; y++)
    {
        oled_set_pos(0, y);
        for (x = 0; x < 128; x++)
            oled_write_cmd_data(0, OLED_DATA); /* 清零 */
    }
}

static void oled_disp_test(void)
{
    uint8_t x, y;

    for (y = 0; y < 8; y++)
    {
        oled_set_pos(0, y);
        for (x = 0; x < 128; x++)
        {
            if (x % 2 == 0)
                oled_write_cmd_data(0, OLED_DATA);
            else
                oled_write_cmd_data(1, OLED_DATA);
        }
    }
}
static int oled_init(void)
{
    int ret = 0;

    oled_reset();

    ret = oled_write_cmd_data(0xae, OLED_CMD); //关闭显示

    ret = oled_write_cmd_data(0x00, OLED_CMD); //设置 lower column address
    ret = oled_write_cmd_data(0x10, OLED_CMD); //设置 higher column address

    ret = oled_write_cmd_data(0x40, OLED_CMD); //设置 display start line

    ret = oled_write_cmd_data(0xB0, OLED_CMD); //设置page address

    ret = oled_write_cmd_data(0x81, OLED_CMD); // contract control
    ret = oled_write_cmd_data(0x66, OLED_CMD); // 128

    ret = oled_write_cmd_data(0xa1, OLED_CMD); //设置 segment remap

    ret = oled_write_cmd_data(0xa6, OLED_CMD); // normal /reverse

    ret = oled_write_cmd_data(0xa8, OLED_CMD); // multiple ratio
    ret = oled_write_cmd_data(0x3f, OLED_CMD); // duty = 1/64

    ret = oled_write_cmd_data(0xc8, OLED_CMD); // com scan direction

    ret = oled_write_cmd_data(0xd3, OLED_CMD); // set displat offset
    ret = oled_write_cmd_data(0x00, OLED_CMD); //

    ret = oled_write_cmd_data(0xd5, OLED_CMD); // set osc division
    ret = oled_write_cmd_data(0x80, OLED_CMD); //

    ret = oled_write_cmd_data(0xd9, OLED_CMD); // ser pre-charge period
    ret = oled_write_cmd_data(0x1f, OLED_CMD); //

    ret = oled_write_cmd_data(0xda, OLED_CMD); // set com pins
    ret = oled_write_cmd_data(0x12, OLED_CMD); //

    ret = oled_write_cmd_data(0xdb, OLED_CMD); // set vcomh
    ret = oled_write_cmd_data(0x30, OLED_CMD); //

    ret = oled_write_cmd_data(0x8d, OLED_CMD); // set charge pump disable
    ret = oled_write_cmd_data(0x14, OLED_CMD); //

    ret = oled_write_cmd_data(0xaf, OLED_CMD); // set dispkay on

    oled_disp_clear();
    oled_set_pos(0, 0);

    return ret;
}

/*
 *回环测试函数
 *spi_device，指定oled 设备驱动的spi 结构体
 */
static void loop_back_test(struct spi_device *spi_device)
{
    uint8_t tx_buffer[2] = {0x66, 0x77};
    uint8_t rx_buffer[2];
    struct spi_message *message;   //定义发送的消息
    struct spi_transfer *transfer; //定义传输结构体

    message = kzalloc(sizeof(struct spi_message), GFP_KERNEL);
    transfer = kzalloc(sizeof(struct spi_transfer), GFP_KERNEL);
    printk("message size=%d,  transfer=%d \n", sizeof(struct spi_message), sizeof(struct spi_transfer));

    transfer->tx_buf = tx_buffer;
    transfer->rx_buf = rx_buffer;
    transfer->len = 2;

    spi_message_init(message);               /* 初始化spi_message */
    spi_message_add_tail(transfer, message); /* 将spi_transfer添加到spi_message队列 */
    spi_sync(spi_device, message);           /* 同步发送 */

    printk("tx_buffer=%02X, %02X \n", tx_buffer[0], tx_buffer[1]);
    printk("rx_buffer=%02X, %02X \n", rx_buffer[0], rx_buffer[1]);

    kfree(message);
    kfree(transfer);
}

static int _drv_open(struct inode *node, struct file *file)
{
    int ret = 0;
    ret = oled_init();
    printk("%s %s open\r\n", __FUNCTION__, DEV_NAME);
    return 0;
}

static ssize_t _drv_read(struct file *filp, char __user *buf, size_t size, loff_t *offset)
{
    int ret = 0;

    oled_disp_test();

    printk("%s %s\r\n", __FUNCTION__, DEV_NAME);

    ret = size;
    return ret;
}

static long _drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    uint8_t buf[3];
    uint16_t size;
    const void __user *userspace = (const void __user *)arg;

    switch (cmd & 0x0f)                /* 最低字节存放命令字段 */
    {
    case OLED_CMD_DISP_ON_OFF:
        ret = copy_from_user(&buf[0], userspace, 1);
        oled_disp_on_off(buf[0]);
        break;
    case OLED_CMD_SET_XY:
        ret = copy_from_user(&buf, userspace, 2);
        if (ret > 0) {
            ret = -EFAULT;
            goto exit;
        }
        // printk("x %d, y %d\r", buf[0], buf[1]);
        oled_set_pos(buf[0], buf[1]);
        break;
    case OLED_CMD_WRITE_DATAS:
        size = (uint16_t)(cmd & 0xffffff00);        /* 前三字节存放数据大小 */
        size >>= 8;
        // printk("size %d\r", size);
        ret = copy_from_user(oled_dev->databuf, userspace, size);
        if (ret > 0) {
            ret = -EFAULT;
            goto exit;
        }
        oled_write_datas(oled_dev->databuf, size);
    case OLED_CMD_SET_XY_WRITE_DATAS:
        ret = copy_from_user(buf, userspace, size);
        if (ret > 0) {
            ret = -EFAULT;
            goto exit;
        }
        break;
    }

exit:
    return ret;
}

static int _drv_release(struct inode *node, struct file *filp)
{
    struct oled_device *tmp_oled = filp->private_data;

    oled_disp_on_off(0);

    oled_reset();

    return 0;
}

static struct file_operations oled_drv_ops = {
    .owner = THIS_MODULE,
    .open = _drv_open,
    .read = _drv_read,
    .unlocked_ioctl = _drv_ioctl,
    .release = _drv_release,
};

static int _driver_probe(struct spi_device *spi)
{
    int err = 0;
    struct device *_dev;
    struct device_node *_dts_node;
    // struct device_node *oled_dev_node;

    oled_dev = (struct oled_device *)kzalloc(sizeof(struct oled_device), GFP_KERNEL);
    if (!oled_dev)
    {
        printk("can't kzalloc mpu6050 dev\n");
        return -ENOMEM;
    }

    _dts_node = spi->dev.of_node;
    if (!_dts_node)
    {
        printk("oled espi can not found!\r\n");
        err = -EINVAL;
        goto exit_free_dev;
    }

    oled_dev->dc_gpio = of_get_named_gpio(_dts_node, DC_GPIO_DTS_NAME, 0); /* 获取dc_gpio */
    if (!gpio_is_valid(oled_dev->dc_gpio))
    {
        printk("don't get oled %s!!!\n", DC_GPIO_DTS_NAME);
        err = -EINVAL;
        goto exit_free_dev;
    }
    printk("oled dc-gpio %d", oled_dev->dc_gpio);
    gpio_direction_output(oled_dev->dc_gpio, 1); /* 设置gpio为输入 */

    oled_dev->rst_gpio = of_get_named_gpio(_dts_node, RST_GPIO_DTS_NAME, 0); /* 获取rst_gpio */
    if (!gpio_is_valid(oled_dev->rst_gpio))
    {
        printk("don't get oled %s!!!\n", RST_GPIO_DTS_NAME);
        err = -EINVAL;
        goto exit_free_dev;
    }
    printk("oled dc-gpio %d", oled_dev->rst_gpio);
    gpio_direction_output(oled_dev->rst_gpio, 1); /* 设置gpio为输入 */

    /* 内核自动分配设备号 */
    err = alloc_chrdev_region(&oled_dev->dev_no, 0, 1, DEV_NAME);
    if (err < 0)
    {
        pr_err("Error: failed to register oled, err: %d\n", err);
        goto exit_free_dev;
    }

    cdev_init(&oled_dev->chrdev, &oled_drv_ops);
    err = cdev_add(&oled_dev->chrdev, oled_dev->dev_no, 1);
    if (err)
    {
        printk("cdev add failed\r\n");
        goto exit_unregister;
    }

    oled_dev->class = class_create(THIS_MODULE, DEV_NAME);
    if (IS_ERR(oled_dev->class))
    {
        err = PTR_ERR(oled_dev->class);
        goto exit_cdev_del;
    }

    /* 创建设备节点 */
    _dev = device_create(oled_dev->class, NULL, oled_dev->dev_no, NULL, DEV_NAME);
    if (IS_ERR(_dev))
    { /* 判断指针是否合法 */
        err = PTR_ERR(_dev);
        goto exit_class_del;
    }

    oled_dev->spi = spi;

    mutex_init(&oled_dev->m_lock); /* 初始化互斥锁  */

    printk("%s probe success\r\n", DEV_NAME);

    goto exit;

exit_class_del:
    class_destroy(oled_dev->class);
exit_cdev_del:
    cdev_del(&oled_dev->chrdev);
exit_unregister:
    unregister_chrdev_region(oled_dev->dev_no, 1); /* 注销设备 */
exit_free_dev:
    kfree(oled_dev);
    oled_dev = NULL;
exit:
    return err;
}

static int _driver_remove(struct spi_device *spi)
{
    int ret = 0;

    device_destroy(oled_dev->class, oled_dev->dev_no);
    class_destroy(oled_dev->class);
    cdev_del(&oled_dev->chrdev);
    unregister_chrdev_region(oled_dev->dev_no, 1); /* 注销设备 */
    kfree(oled_dev);

    printk(KERN_INFO "%s remove success\n", DEV_NAME);

    return ret;
}

/* 设备树的匹配列表 */
static struct of_device_id dts_match_table[] = {
    {.compatible = OLED_DTS_COMPATIBLE}, /* 通过设备树来匹配 */
    {},
};

/* 传统匹配方式 ID 列表 */
static const struct spi_device_id spi_dev_id[] = {
    {.name = OLED_DTS_COMPATIBLE, 0},
    {}};

/* SPI 驱动结构体 */
static struct spi_driver oled_driver = {
    .probe = _driver_probe,
    .remove = _driver_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = OLED_DTS_COMPATIBLE,
        .of_match_table = dts_match_table,
    },
    .id_table = spi_dev_id,
};

module_spi_driver(oled_driver);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");