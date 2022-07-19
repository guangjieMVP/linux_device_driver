#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/device.h>

#define  CCM_CCGR1                0x20C406C                           
#define  MUX_PAD_GPIO1_IO04       0x20E006C    

#define  GPIO1_DR                 0x209C000
#define  GPIO1_GDIR               0x209C004

static unsigned int __iomem *_CCM_CCGR1;
static unsigned int __iomem *_MUX_PAD_GPIO1_IO04;
static unsigned int __iomem *_GPIO1_DR;
static unsigned int __iomem *_GPIO1_GDIR;

static int major = 0;    
static struct class *led_class;

static int led_drv_open (struct inode *node, struct file *file)
{
    int val;
	  
	  printk("open %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	
    _CCM_CCGR1 = ioremap(CCM_CCGR1, 4);                    
	  _MUX_PAD_GPIO1_IO04 = ioremap(MUX_PAD_GPIO1_IO04, 4);
    _GPIO1_DR = ioremap(GPIO1_DR, 4);
    _GPIO1_GDIR = ioremap(GPIO1_GDIR, 4);  

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
	printk("read %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	return 0;
}

/* write(fd, &val, 1); */
static ssize_t led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
	int err;
	char status;
  int val = 0;
	// struct inode *inode = file_inode(file);    
	// int minor = iminor(inode);
	
	printk("write %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

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
	printk("close %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

    iounmap(_CCM_CCGR1);
    iounmap(_MUX_PAD_GPIO1_IO04);
    iounmap(_GPIO1_DR);
    iounmap(_GPIO1_GDIR);
	return 0;
}

static struct file_operations led_drv = {
	.owner	 = THIS_MODULE,
	.open    = led_drv_open,
	.read    = led_drv_read,
	.write   = led_drv_write,
	.release = led_drv_close,
};


static int __init led_init(void)
{
  int err;
    major = register_chrdev(0, "my_led", &led_drv);

    led_class = class_create(THIS_MODULE, "my_led_class");
	err = PTR_ERR(led_class);
	if (IS_ERR(led_class)) {
	
		unregister_chrdev(major, "my_led"); 
		return -1;
	}
    device_create(led_class, NULL, MKDEV(major, 0), NULL, "my_led"); 
   	printk("init %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    return 0;
}

static void __exit led_exit(void)
{
    printk("exit %s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    device_destroy(led_class, MKDEV(major, 0));
	class_destroy(led_class);
	unregister_chrdev(major, "my_led");
}

module_init(led_init);
module_exit(led_exit);


MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");


