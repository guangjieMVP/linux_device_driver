#include <linux/module.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/gfp.h>
#include <asm/io.h>
#include <linux/device.h>

#include "led_opr.h"

struct led_gpio_reg{
    unsigned int pin;
    
    unsigned int __iomem *va_ccgr;
    unsigned int __iomem *va_mux_gpio;
    unsigned int __iomem *va_dr;
    unsigned int __iomem *va_gdir;   

    unsigned int ccgr;
    unsigned int mux_gpio;
    unsigned int dr;
    unsigned int gdir; 
};


static struct led_gpio_reg led_red = {
    .ccgr =  0x20C406C,
    .mux_gpio =  0x20E006C,
    .dr  = 0x209C000,
    .gdir = 0x209C004,
    .pin = 4
};


// GPIO4_IO20
static struct led_gpio_reg led_green = {
    .ccgr  = 0x20C4074,
    .mux_gpio =  0x20E01E0,
    .dr  = 0x20A8000,
    .gdir = 0x20A8004,
    .pin = 20
};

//GPIO4_IO19
static struct led_gpio_reg led_blue = {
    .ccgr  = 0x20C4074,
    .mux_gpio = 0x20E01DC,
    .dr  = 0x20A8000,
    .gdir = 0x20A8004,
    .pin = 19
};
 
static struct led_gpio_reg led_grp[3] = {
    {.ccgr =  0x20C406C,
    .mux_gpio =  0x20E006C,
    .dr  = 0x209C000,
    .gdir = 0x209C004,
    .pin = 4},
  { .ccgr  = 0x20C4074,
    .mux_gpio =  0x20E01E0,
    .dr  = 0x20A8000,
    .gdir = 0x20A8004,
    .pin = 20},
  {  .ccgr  = 0x20C4074,
    .mux_gpio = 0x20E01DC,
    .dr  = 0x20A8000,
    .gdir = 0x20A8004,
    .pin = 19}
};

static void led_reg_remap(struct led_gpio_reg *led)
{
    led->va_ccgr = ioremap(led->ccgr, 4);                    
    led->va_mux_gpio = ioremap(led->mux_gpio, 4);
    led->va_dr = ioremap(led->dr, 4);
    led->va_gdir = ioremap(led->gdir, 4); 
}

static void led_reg_unremap(struct led_gpio_reg *led)
{
    iounmap(led->va_ccgr);                    
    iounmap(led->va_mux_gpio);
    iounmap(led->va_dr);
    iounmap(led->va_gdir); 
}

static void led_red_init(void) 
{
    int val = 0;
    led_reg_remap(&led_red);
    val = ioread32(led_red.va_ccgr);
    val &= ~(3 << 26);     //清零
    val |= (3 << 26);   
    iowrite32(val, led_red.va_ccgr);    //打开时钟

    val = 0;
    val = ioread32(led_red.va_mux_gpio);     
    val &= ~(0xf << 0);     //清零
    val |= (0x5 << 0);     //
    iowrite32(val, led_red.va_mux_gpio);    //复用GPIO1_IO4为GPIO功能

    val = 0;
    val = ioread32(led_red.va_gdir);     
    val |= (0x1 << led_red.pin); 
    iowrite32(val, led_red.va_gdir);    //设置GPIO1_IO4为输出模式     
}

static void led_green_init(void)
{
    int val = 0;
    led_reg_remap(&led_green);

    val = ioread32(led_green.va_ccgr);
    val &= ~(3 << 12);     //清零
    val |= (3 << 12);   
    iowrite32(val, led_green.va_ccgr);    //打开时钟

    val = 0;
    val = ioread32(led_green.va_mux_gpio);     
    val &= ~(0xf << 0);     //清零
    val |= (0x5 << 0);     //
    iowrite32(val, led_green.va_mux_gpio);    //复用GPIO1_IO4为GPIO功能

    val = 0;
    val = ioread32(led_green.va_gdir);     
    val |= (0x1 << led_green.pin); 
    iowrite32(val, led_green.va_gdir);    //设置GPIO1_IO4为输出模式   
}

static void led_blue_init(void)
{
    int val = 0;
    led_reg_remap(&led_blue);

    val = ioread32(led_blue.va_ccgr);
    val &= ~(3 << 12);     //清零
    val |= (3 << 12);   
    iowrite32(val, led_blue.va_ccgr);    //打开时钟

    val = 0;
    val = ioread32(led_blue.va_mux_gpio);     
    val &= ~(0xf << 0);     //清零
    val |= (0x5 << 0);     //
    iowrite32(val, led_blue.va_mux_gpio);    //复用GPIO1_IO19为GPIO功能

    val = 0;
    val = ioread32(led_blue.va_gdir);     
    val |= (0x1 << led_blue.pin); 
    iowrite32(val, led_blue.va_gdir);    //设置GPIO1_IO19为输出模式   
}

static void led_ctrl(struct led_gpio_reg *led, int status)
{
    int val = 0;

    if (status)
    {
        val &= ~(1 << led->pin);
        iowrite32(val, led->va_dr);    //设置GPIO1_IO19为输出模式   
    }
    else
    {
        val |= (1 << led->pin);
        iowrite32(val, led->va_dr);    //设置GPIO1_IO19为输出模式 
    }
}


static int board_fire_led_init (int which) /* 初始化LED, which-哪个LED */       
{
    printk("board led init %s %d\r\n", __FUNCTION__, which);
    if (which == 0)
    {
       led_red_init();
    }
    else if (which == 1)
    {
        led_green_init();
    }
    else if (which == 2)
    {
        led_blue_init();
    }
    
    return 0;
}

static int board_fire_led_deinit (int which) /* 初始化LED, which-哪个LED */       
{
    printk("board led init %s %d\r\n", __FUNCTION__, which);
    //printk("%s %s line %d, led %d\n", __FILE__, __FUNCTION__, __LINE__, which);

    if (which == 0)
    {
       led_reg_unremap(&led_red);
    }
    else if (which == 1)
    {
        led_reg_unremap(&led_green);
    }
    else if (which == 2)
    {
        led_reg_unremap(&led_blue);   
    }    
//    led_reg_unremap(&led_grp[which]);

    return 0;
}


static int board_fire_led_ctrl (int which, char status) /* 控制LED, which-哪个LED, status:1-亮,0-灭 */
{
    //printk("%s %s line %d, led %d, %s\n", __FILE__, __FUNCTION__, __LINE__, which, status ? "on" : "off");
    printk("board led init %s %d\r\n", __FUNCTION__, which);
    if (which == 0)
    {
        led_ctrl(&led_red, status);
    }
    else if (which == 1)
    {
        led_ctrl(&led_green, status);
    }
    else if (which == 2)
    {
        led_ctrl(&led_blue, status);
    }   
//    led_ctrl(&led_grp[which], status);
    return 0;
}

static struct led_operations board_demo_led_opr = {
    .num  = 3,
    .init = board_fire_led_init,
    .deinit = board_fire_led_deinit,
    .ctrl  = board_fire_led_ctrl,
};

struct led_operations *get_board_led_opr(void)
{
    return &board_demo_led_opr;
}

