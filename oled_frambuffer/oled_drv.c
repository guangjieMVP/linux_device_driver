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
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/kthread.h>

#include "oled_def.h"

#define DEV_NAME                   "oled" /* /dev/oled */
#define OLED_DTS_NODE_PATH         "xgj-oled"
#define OLED_DTS_COMPATIBLE        "xgj,oled"
#define DC_GPIO_DTS_NAME           "dc-gpios"
#define RST_GPIO_DTS_NAME          "rst-gpios"

#define OLED_HOR_RES         128
#define OLED_VER_RES         64
#define OLED_BPP             1

enum {
    OLED_CMD = 0x00,
    OLED_DATA = 0x01,
};
typedef unsigned char oled_cmd_t;

struct oled_device {
    int dc_gpio;
    int rst_gpio;
    struct spi_device *spi;
    struct fb_info *fb;
    struct task_struct *task;
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

static int oled_refresh_thread(void *data)
{
    do {
#if 1
        msleep(1000);     
#else
        // printk(" %s %s %d\n", __FILE__, __FUNCTION__, cnt++);
        // set_current_state(TASK_INTERRUPTIBLE);       /* 设置内核线程状态 */
        // schedule_timeout(HZ*5);                       /* 阻塞延时 */
        
#endif
    } while (!kthread_should_stop());

    return 0;
}

/* 设置调色板 */
static int oled_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	return 1; /* unkown type */
}

static unsigned int pseudo_palette[16];  /* 设置调色板 */

static struct fb_ops oled_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= oled_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

static int _driver_probe(struct spi_device *spi)
{
    int err = 0;
    struct device_node *_dts_node;
    dma_addr_t fb_phy_addr;

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
    printk("oled rst-gpio %d", oled_dev->rst_gpio);
    gpio_direction_output(oled_dev->rst_gpio, 1); /* 设置gpio为输入 */

    oled_dev->spi = spi;

    /* 1、申请分配fb_info */
    oled_dev->fb = framebuffer_alloc(0, &spi->dev);
    if (!oled_dev->fb) {
		err = -ENOMEM;
		goto exit_free_dev;
	}

    /* 2、设置fb_info,var可变参数 */
    /* 2.1 设置可变参数：分辨率、颜色格式、色彩深度等 */
    oled_dev->fb->var.xres_virtual = oled_dev->fb->var.xres = OLED_HOR_RES;
	oled_dev->fb->var.yres_virtual = oled_dev->fb->var.yres = OLED_VER_RES;
	oled_dev->fb->var.bits_per_pixel = OLED_BPP;  
	// oled_dev->fb->var.red.offset = 11;    OLED是单色屏，不需要设置颜色格式
	// oled_dev->fb->var.red.length = 5;

	// oled_dev->fb->var.green.offset = 5;
	// oled_dev->fb->var.green.length = 6;

	// oled_dev->fb->var.blue.offset = 0;
	// oled_dev->fb->var.blue.length = 5;
    /* 3、 设置fb_info, fix固定参数 */
    strcpy(oled_dev->fb->fix.id, "xgj,oled");
	oled_dev->fb->fix.smem_len = oled_dev->fb->var.xres * oled_dev->fb->var.yres * oled_dev->fb->var.bits_per_pixel / 8;

    /* 3.1 分配显存 kzalloc申请的内存控件可能不连续，作为frambuffer设备的内存必须是连续的，所以需要使用dma_alloc_wc来申请 */
    oled_dev->fb->screen_base = dma_alloc_wc(NULL, oled_dev->fb->fix.smem_len, &fb_phy_addr, GFP_KERNEL);
	oled_dev->fb->fix.smem_start = fb_phy_addr;  /* fb的物理地址 */
	
	oled_dev->fb->fix.type = FB_TYPE_PACKED_PIXELS;
	oled_dev->fb->fix.visual = FB_VISUAL_MONO10;
	oled_dev->fb->fix.line_length = oled_dev->fb->var.xres * oled_dev->fb->var.bits_per_pixel / 8;

    /* 2.4 跟具体显示设备的硬件操作 */
    oled_dev->fb->fbops = &oled_fb_ops;
	oled_dev->fb->pseudo_palette = pseudo_palette;	
    /* 3、注册fb_info */
    if (register_framebuffer(oled_dev->fb) < 0) {
		err = -EINVAL;
		goto exit_free_frambuf;
	}

    oled_init();

    oled_dev->task = kthread_run(oled_refresh_thread, NULL, "oled,update");     /* 创建内核线程 */
    if (IS_ERR(oled_dev->task))
    {
        err = PTR_ERR(oled_dev->task);
        goto exit_free_frambuf;
    }

    printk("%s probe success\r\n", DEV_NAME);
    goto exit;

exit_free_frambuf:
    framebuffer_release(oled_dev->fb);
exit_free_dev:
    kfree(oled_dev);
    oled_dev = NULL;
exit:
    return err;
}

static int _driver_remove(struct spi_device *spi)
{
    int ret = 0;

    /* 注销fb_info */
    unregister_framebuffer(oled_dev->fb);

    /* 释放fb_info */
    framebuffer_release(oled_dev->fb);  

    /* 停止内核线程 */
    kthread_stop(oled_dev->task); 

    /* 释放oled_dev */
    kfree(oled_dev);

    oled_write_cmd_data(0xae, OLED_CMD); //关闭显示
    oled_reset();

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