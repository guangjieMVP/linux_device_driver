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

#define  DEVICE_NAME   "globalmem"

#define GLOBAL_MEM_SIZE   0X1000
#define MEM_CLEAR         0x1
#define GLOBALMEM_MAJOR   230       //主色设备号

static int global_major = GLOBALMEM_MAJOR;   //主设备号
module_param(global_major, int, S_IRUGO);    //到处模块参数


/* global mem 结构体 */
struct globalmem_dev {
    dev_t devno;
    struct cdev cdev;
    struct class *class;
    struct mutex  m_lock;
    unsigned char mem[GLOBAL_MEM_SIZE];  
};

struct globalmem_dev g_globalmem_obj;        
 
static int globalmem_open (struct inode *node, struct file *filp)
{
    filp->private_data = &g_globalmem_obj;         //将globalmem设备结构体存放在struct file的文件结构体的私有数据中
    return 0;
}

int globalmem_release(struct inode *node, struct file *filp)
{
    return 0;
}

static long globalmem_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct globalmem_dev *dev = filp->private_data;  

    switch (cmd)
    {
        case MEM_CLEAR: 
        {
            mutex_lock(&g_globalmem_obj.m_lock);        //枷锁
            memset(dev->mem, 0, GLOBAL_MEM_SIZE);           //清空内存设备
            mutex_unlock(&g_globalmem_obj.m_lock);        //枷锁mutex_unlock(&g_globalmem_obj.m_lock);        //枷锁
            printk(KERN_INFO"globalmem is set zero\n");
            break;
        }
        default :
        {
            return -EINVAL;
            break;
        }
    }
    return  0;
}


static ssize_t globalmem_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{   
    int ret;
    struct globalmem_dev *dev = filp->private_data;      

    unsigned long  p = *ppos;
    unsigned int count = size;

    if (p >= GLOBAL_MEM_SIZE)     //如果读取的偏移开始位置大于内存的大小
        return 0;                //读取失败

    if (count > (GLOBAL_MEM_SIZE - p))   //如果读取的大小大于剩余的空间大小
    {
        count = GLOBAL_MEM_SIZE - p;      //那就读取剩余的大小
    }

    mutex_lock(&g_globalmem_obj.m_lock);        //枷锁
    if (copy_to_user(buf, dev->mem + p, count))    //如果将读取的数据拷贝到用户空间是失败
    {
        return -EFAULT;
    }
    else
    {
        *ppos += count;       //标记已经读取的位置
        ret = count;
        printk(KERN_INFO"read %d bytes from %ld\n", count, p);         
    }
    mutex_unlock(&g_globalmem_obj.m_lock);        //枷锁

    return ret;   //返回实际读取的字节数
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    int ret;
    struct globalmem_dev *dev = filp->private_data;

    unsigned long p = *ppos;
    unsigned int count = size;

    if (p >= GLOBAL_MEM_SIZE)
        return 0;

    if (count >= (GLOBAL_MEM_SIZE-p))
        count = GLOBAL_MEM_SIZE - p;

    mutex_lock(&g_globalmem_obj.m_lock);        //枷锁
    if (copy_from_user(dev->mem + p, buf, count))   //将用户空间写入的数据拷贝进内核空间失败
    {
        return -EFAULT;
    }
    else
    {
        *ppos += count;  //读出了count字节，修改文件的读位置
        ret = count;
        printk(KERN_INFO"write %d bytes from %ld\n", count, p); 
    }
    mutex_unlock(&g_globalmem_obj.m_lock);        //枷锁

    return ret;    //返回写入字节数
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset, int origin)
{
    loff_t ret;

    switch(origin)
    {
        case 0 :       //从文件头开始定位   SEEK_SET : 0
        {
            if (offset < 0 || offset > GLOBAL_MEM_SIZE)
            {
                ret = -EINVAL;         // invalid value  无效参数
                break;
            }
            
            filp->f_pos = (unsigned int)offset;      //改变文件读写偏移位置
            ret = filp->f_pos;
            break;
        }
        case 1 :          //从当前位置定位，SEEK_CUR : 1
        {
            if ((filp->f_pos + offset) > GLOBAL_MEM_SIZE || (filp->f_pos + offset) < 0)
            {
                ret = -EINVAL;
                break;
            }
            filp->f_pos += offset;
            ret = filp->f_pos;
            break;
        }
        default:       //从文件尾SEEK_END ： 2定位 以及其他情况 都直接返回-EINVAL  
        {
            ret = -EINVAL;
            break;;
        }
    }
    return ret;
}

static struct file_operations globalmem_file_opr = {
	.owner	 = THIS_MODULE,
	.open    = globalmem_open,
    .read    = globalmem_read,
	.write   = globalmem_write,
    .unlocked_ioctl = globalmem_unlocked_ioctl,
    .release = globalmem_release,
    .llseek = globalmem_llseek,
};


static int __init globalmem_init(void)
{
    int ret;
    int err;
    struct device *tmpdev;

    g_globalmem_obj.devno = MKDEV(global_major, 0);    //创建设备号
    
    if (global_major)
    {
        ret = register_chrdev_region(g_globalmem_obj.devno, 1, DEVICE_NAME);
    }
    else
    {
        ret = alloc_chrdev_region(&g_globalmem_obj.devno, 0, 1, DEVICE_NAME);
    }

    cdev_init(&g_globalmem_obj.cdev, &globalmem_file_opr);
	cdev_add(&g_globalmem_obj.cdev, g_globalmem_obj.devno, 1);
    
    g_globalmem_obj.class = class_create(THIS_MODULE, "globalmem_class");
	err = PTR_ERR(g_globalmem_obj.class);
	if (IS_ERR(g_globalmem_obj.class)) {
        goto failed1;
	}

    //在 /dev目录下创建red_led设备节点
    tmpdev = device_create(g_globalmem_obj.class , NULL, g_globalmem_obj.devno, NULL, "globalmem"); 
    if (IS_ERR(tmpdev)) {
        ret = -EINVAL;
		goto failed2;
	}

    mutex_init(&g_globalmem_obj.m_lock);    //初始化互斥锁
    printk(KERN_INFO"globalmem init\n");

    return 0;
failed2:
    device_destroy(g_globalmem_obj.class, g_globalmem_obj.devno);
    class_destroy(g_globalmem_obj.class);
failed1:
    cdev_del(&g_globalmem_obj.cdev);
	unregister_chrdev_region(g_globalmem_obj.devno, 1);
    return ret;
}

static void __exit globalmem_exit(void)
{
    device_destroy(g_globalmem_obj.class, g_globalmem_obj.devno);
	class_destroy(g_globalmem_obj.class);
	unregister_chrdev_region(g_globalmem_obj.devno, 1);
    cdev_del(&g_globalmem_obj.cdev);
    printk(KERN_INFO"globalmem exit\n");
}

module_init(globalmem_init);
module_exit(globalmem_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");



