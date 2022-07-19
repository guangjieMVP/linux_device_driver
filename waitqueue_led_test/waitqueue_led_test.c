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
#include <linux/platform_device.h>

#include <asm/atomic.h>  
#include <asm/spinlock.h>                  //������ͷ�ļ�
#include <linux/semaphore.h>
#include <linux/wait.h>

#define RED_LED_DTS_NODE   "/red_led"       //red_led�豸���ڵ�·��
#define RED_LED_GPIO_NAME  "red_led_gpio"   //red led �ڵ��� gpio��ϵͳ���������

struct red_led_dev {
    struct cdev chrdev;               //�ַ��豸�ṹ��
    dev_t dev_no;                     //�豸��tou 
    struct class *led_class;
    struct device_node *dev_node;     //red led �豸�ڵ�
    unsigned int red_led_gpio;
    struct semaphore sem;             //�ź���
    
};

static struct red_led_dev led_dev;
// wait_queue_head_t wait_head;     //�ȴ�����ͷ
DECLARE_WAIT_QUEUE_HEAD(wait_head); 
char status;

static int red_led_drv_open (struct inode *node, struct file *file)
{
    int ret;

    printk(KERN_INFO"success to open red led dev\n set gpio is ouput\n");

    ret = gpio_direction_output(led_dev.red_led_gpio, 1);    //����gpioΪ�����Ĭ������ߵ�ƽ
    return ret;
}

static ssize_t red_led_drv_write (struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    int err;
	
	err = copy_from_user(&status, buf, 1);

    if (status)
    {
        wake_up(&wait_head);
        return status;
    }
    else
    {
        wait_event(wait_head, status); 
    }
    
	if (status)      
    {  /* ��� */
        printk("led on\n");
        gpio_set_value(led_dev.red_led_gpio, 0);      
    }
    else                      
    {   /* Ϩ�� */
        printk("led off\n");
        gpio_set_value(led_dev.red_led_gpio, 1);
    }
	
	return 1;
}

int red_led_drv_release(struct inode *node, struct file *file)
{
    printk("led release \n sem V operation \n");
    return 1;
}

static struct file_operations red_led_drv = {
	.owner	 = THIS_MODULE,
	.open    = red_led_drv_open,
	.write   = red_led_drv_write,
    .release = red_led_drv_release,
};

//�豸����ƥ������
static struct of_device_id dts_match_table[] = {
    {.compatible = "red_led", },                     //ͨ���豸����ƥ��
};


static int led_red_driver_probe(struct platform_device *dev)  
{
    int err;
    int ret;
    struct device *tmpdev;

    // init_waitqueue_head(&wait_head);    //��ʼ���ȴ�����ͷ

    led_dev.dev_node = of_find_node_by_path(RED_LED_DTS_NODE);         //�ҵ�red_led���豸���ڵ�
    if (!led_dev.dev_node) {          
        printk("red led device node can not found!\r\n"); 
        return -EINVAL; 
    }

    led_dev.red_led_gpio = of_get_named_gpio(led_dev.dev_node, RED_LED_GPIO_NAME, 0);   //��ȡgpio�ı��
    if ( led_dev.red_led_gpio < 0) {
        printk("red led gpio can not found!\r\n"); 
        return -EINVAL;
    }

    // ret = gpio_request(led_dev.red_led_gpio, RED_LED_GPIO_NAME);   //����gpio
    // if (ret < 0) {
    //     printk("rcan not request gpio!\r\n"); 
    //     return -EINVAL;
    // }

    ret = alloc_chrdev_region(&led_dev.dev_no, 0, 1, "red_led");
	if (ret < 0) {
		pr_err("Error: failed to register mbochs_dev, err: %d\n", ret);
		return ret;
	}

	cdev_init(&led_dev.chrdev, &red_led_drv);

	cdev_add(&led_dev.chrdev, led_dev.dev_no, 1);

    led_dev.led_class = class_create(THIS_MODULE, "red_led_class");
	err = PTR_ERR(led_dev.led_class);
	if (IS_ERR(led_dev.led_class)) {
        goto failed1;
	}

    //�� /devĿ¼�´���red_led�豸�ڵ�
    tmpdev = device_create(led_dev.led_class , NULL, led_dev.dev_no, NULL, "red_led"); 
    if (IS_ERR(tmpdev)) {
        ret = -EINVAL;
		goto failed2;
	}

   	printk("led driver probe\n");
    
    return 0;
failed2:
    device_destroy(led_dev.led_class, led_dev.dev_no);
    class_destroy(led_dev.led_class);
failed1:
    cdev_del(&led_dev.chrdev);
	unregister_chrdev_region(led_dev.dev_no, 1);
    return ret;
}

int led_red_driver_remove(struct platform_device *dev)
{
    device_destroy(led_dev.led_class, led_dev.dev_no);
	class_destroy(led_dev.led_class);
	unregister_chrdev_region(led_dev.dev_no, 1);
    cdev_del(&led_dev.chrdev);
    printk("led driver remove\n");
    return 0;
}

static struct platform_driver red_led_platform_driver = {
      .probe = led_red_driver_probe,
      .remove = led_red_driver_remove,
      .driver = {
        .name = "fire-rgb-led",
        .owner = THIS_MODULE,
        .of_match_table = dts_match_table,         //ͨ���豸��ƥ��
      },
};


static int __init red_led_driver_init(void)
{
    int ret;

    ret = platform_driver_register(&red_led_platform_driver);   //ע��platform����
    printk("led platform driver init\n");
    return ret;
}

static void __exit red_led_driver_exit(void)
{
    platform_driver_unregister(&red_led_platform_driver);
    printk("led platform driver exit\n");
}

module_init(red_led_driver_init);
module_exit(red_led_driver_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");





















