#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>

static unsigned int __iomem  *_CCM_CCGR1;
static unsigned int __iomem  *_MUX_PAD_GPIO1_IO04;
static unsigned int __iomem  *_GPIO1_DR;
static unsigned int __iomem  *_GPIO1_GDIR;

static struct resource *ccm_ccgr1;
static struct resource *mux_pad_gpio1_io4;
static struct resource *gpio1_dr;
static struct resource *gpio1_gdir;

static int major = 0;    
static struct class *led_class;

static int led_drv_open (struct inode *node, struct file *file)
{
    int val;
	  
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

//    unsigned int ccm = *(unsigned int *)ccm_ccgr1;
//    unsigned int mux_pad = *(unsigned int *)mux_pad_gpio1_io4;
//    unsigned int dr = *(unsigned int *)gpio1_dr;
//   unsigned int gdir = *(unsigned int *)gpio1_gdir;

    val = ioread32(_CCM_CCGR1);
    val &= ~(3 << 26);   
    val |= (3 << 26);   
    iowrite32(val, _CCM_CCGR1);    

    val = 0;
    val = ioread32(_MUX_PAD_GPIO1_IO04);     
    val &= ~(0xf << 0);    
    val |= (0x5 << 0);  
    iowrite32(val, _MUX_PAD_GPIO1_IO04);   

    val = 0;
    val = ioread32(_GPIO1_GDIR);     
    val |= (0x1 << 4); 
    iowrite32(val, _GPIO1_GDIR);   

	return 0;
}


static ssize_t led_drv_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	printk("read %s %s line %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	return -EFAULT;
}

/* write(fd, &val, 1); */
static ssize_t led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	int err;
	char status;
    int val = 0;
	// struct inode *inode = file_inode(file);    
	// int minor = iminor(inode);
	
	printk("write %s %s line %d\r\n", __FILE__, __FUNCTION__, __LINE__);

	err = copy_from_user(&status, buf, 1);

	if (status)
    {
        val &= ~(1 << 4);
        iowrite32(val, _GPIO1_DR);   
    }
    else
    {
        val |= (1 << 4);
        iowrite32(val, _GPIO1_DR);       
    }
	
	return 1;
}

static int led_drv_close (struct inode *node, struct file *file)
{
	printk("%s %s line %d\r\n", __FILE__, __FUNCTION__, __LINE__);

	return 0;
}

static struct file_operations led_drv = {
	.owner	 = THIS_MODULE,
	.open    = led_drv_open,
	.read    = led_drv_read,
	.write   = led_drv_write,
	.release = led_drv_close,
};

static int led_red_driver_probe(struct platform_device *dev)
{
    struct resource *ccm_ccgr1 = platform_get_resource(dev, IORESOURCE_MEM, 0);  //获取设备中定义的文件资源
    struct resource *mux_pad_gpio1_io4 = platform_get_resource(dev, IORESOURCE_MEM, 1);
    struct resource *gpio1_dr = platform_get_resource(dev, IORESOURCE_MEM, 2);
    struct resource *gpio1_gdir = platform_get_resource(dev, IORESOURCE_MEM, 3);
    
    _CCM_CCGR1 = ioremap(ccm_ccgr1->start, resource_size(ccm_ccgr1));                    
	  _MUX_PAD_GPIO1_IO04 = ioremap(mux_pad_gpio1_io4->start, resource_size(mux_pad_gpio1_io4));
    _GPIO1_DR = ioremap(gpio1_dr->start, resource_size(gpio1_dr));
    _GPIO1_GDIR = ioremap(gpio1_gdir->start, resource_size(gpio1_gdir));  

    int err;
    major = register_chrdev(0, "my_led", &led_drv);

    led_class = class_create(THIS_MODULE, "my_led_class");
	err = PTR_ERR(led_class);
	if (IS_ERR(led_class)) {
	
		unregister_chrdev(major, "my_led"); 
		return -1;
	}
    device_create(led_class, NULL, MKDEV(major, 0), NULL, "my_led"); 
   	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    
    return 0;
}

int led_red_driver_remove(struct platform_device *dev)
{
    iounmap(_CCM_CCGR1);
    iounmap(_MUX_PAD_GPIO1_IO04);
    iounmap(_GPIO1_DR);
    iounmap(_GPIO1_GDIR);
    
    printk(" %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    device_destroy(led_class, MKDEV(major, 0));
	  class_destroy(led_class);
	  unregister_chrdev(major, "my_led");
     
    return 0;
}

///static struct platform_device_id led_pdev_ids[] = {
//	{.name = "red_led"},
//	{}
//};

//MODULE_DEVICE_TABLE(platform, led_pdev_ids);

static struct platform_driver red_led_platform_driver = {
      .probe = led_red_driver_probe,
      .remove = led_red_driver_remove,
      .driver.name = "red_led",
//      .id_table = led_pdev_ids,           //platform匹配的关键
};


static int __init red_led_driver_init(void)
{
    printk(" %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    int ret = platform_driver_register(&red_led_platform_driver);
    return ret;
}

static void __exit red_led_driver_exit(void)
{
    printk(" %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    platform_driver_unregister(&red_led_platform_driver);
}

module_init(red_led_driver_init);
module_exit(red_led_driver_exit);


MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");




