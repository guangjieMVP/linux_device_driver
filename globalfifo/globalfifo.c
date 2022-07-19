#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
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
#include <linux/wait.h>
#include <linux/sched/signal.h> 
#include <asm/uaccess.h> 
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poll.h>

#define  DEVICE_NAME   "globalmem"

#define GLOBAL_MEM_SIZE   0X1000
#define MEM_CLEAR         0x1
#define GLOBALMEM_MAJOR   230       //主色设备号

static int global_major = GLOBALMEM_MAJOR;   //主设备号
module_param(global_major, int, S_IRUGO);    //到处模块参数


/* global mem 结构体 */
struct globalfifo_dev {
    dev_t devno;
    struct cdev cdev;
    struct class *class;
    struct mutex  m_lock;
    unsigned int buf_len;
    unsigned char mem[GLOBAL_MEM_SIZE];
    wait_queue_head_t read_wait;             //读等待队列
    wait_queue_head_t write_wait;            //写等待队列
};

struct globalfifo_dev g_globalfifo;        

 
static int globalfifo_open (struct inode *node, struct file *filp)
{
    filp->private_data = &g_globalfifo;         //将globalmem设备结构体存放在struct file的文件结构体的私有数据中
    return 0;
}

int globalfifo_release(struct inode *node, struct file *filp)
{
    return 0;
}

static long globalfifo_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
    struct globalfifo_dev *dev = filp->private_data;  

    switch (cmd)
    {
        case MEM_CLEAR: 
        {
            mutex_lock(&g_globalfifo.m_lock);        //枷锁
            memset(dev->mem, 0, GLOBAL_MEM_SIZE);           //清空内存设备
            mutex_unlock(&g_globalfifo.m_lock);        //枷锁mutex_unlock(&g_globalmem_obj.m_lock);        //枷锁
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


static ssize_t globalfifo_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{   
    int ret = 0;
    struct globalfifo_dev *dev = filp->private_data; 

    DECLARE_WAITQUEUE(wait, current);      //current 可以说是一个全局变量  表示当前进程

    add_wait_queue(&dev->read_wait, &wait);

    mutex_lock(&g_globalfifo.m_lock);        //枷锁
    while (dev->buf_len == 0)     //数据未零的时候
    {
        if (filp->f_flags & O_NONBLOCK)   //非阻塞
        {
            ret = -EAGAIN;
            goto out;
        }

        __set_current_state(TASK_INTERRUPTIBLE);     //标记为未睡眠状态
        mutex_unlock(&g_globalfifo.m_lock);     //解锁
        schedule();     //主动让出CPU，进程进行休眠    这里是进程的休眠点  进程恢复运行会继续执行下面的代码

        if (signal_pending(current))               //判断被信号唤醒
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&g_globalfifo.m_lock);  //不是信号唤醒，枷锁保护下面的读操作
    }

    if (size > dev->buf_len)           //如果读取的字节数大于buffer现有的数据长度，就读取现有长度
        size = dev->buf_len;

   
    if (copy_to_user(buf, dev->mem, size))    //如果将读取的数据拷贝到用户空间是失败
    {
        ret = -EFAULT;
        goto out;
    }
    else
    {
        memcpy(dev->mem, dev->mem + size, dev->buf_len - size);        
        printk(KERN_INFO"read %d bytes current len : %d", size, dev->buf_len);
        wake_up_interruptible(&dev->write_wait);
        
        ret = size;
    }
    
out:
    mutex_unlock(&g_globalfifo.m_lock);        //枷锁
out2:
    remove_wait_queue(&dev->read_wait, &wait);    
    set_current_state(TASK_RUNNING);          //设置进程为运行态
    return ret;   //返回实际读取的字节数
}

static ssize_t globalfifo_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    int ret;
    struct globalfifo_dev *dev = filp->private_data;

    DECLARE_WAITQUEUE(wait, current);      //current 可以说是一个全局变量  表示当前进程

    mutex_lock(&g_globalfifo.m_lock);        //枷锁
    add_wait_queue(&g_globalfifo.write_wait, &wait);   //将等待队列项加入等待队列     

    while (dev->buf_len == GLOBAL_MEM_SIZE)
    {
        if (filp->f_flags & O_NONBLOCK)
        {
            ret = -EAGAIN;
            goto out;
        }

        __set_current_state(TASK_INTERRUPTIBLE);     //标记为未睡眠状态
        mutex_unlock(&g_globalfifo.m_lock);         //解锁
        schedule();     //主动让出CPU，进程进行休眠

        if (signal_pending(current))                 //被信号唤醒
        {
            ret = -ERESTARTSYS;
            goto out2;
        }

        mutex_lock(&g_globalfifo.m_lock);  //枷锁
    }

    if (size > (GLOBAL_MEM_SIZE - dev->buf_len) )
        size = (GLOBAL_MEM_SIZE - dev->buf_len);

    if (copy_from_user(dev->mem + dev->buf_len, buf, size))
    {
        ret = -EFAULT;
        goto out;
    }
    else
    {
        dev->buf_len += size;
        printk(KERN_INFO"write %d bytes current len = %d\n", size, dev->buf_len);
        wake_up_interruptible(&dev->read_wait);        //有数据写入，唤醒写进程读取数据
        
        ret = size; 
    }

out:    
    mutex_unlock(&g_globalfifo.m_lock);                //枷锁
out2:
    remove_wait_queue(&dev->write_wait, &wait);
    set_current_state(TASK_RUNNING);                   //设置当前进程未运行态
    return ret;    //返回写入字节数
}

static loff_t globalfifo_llseek(struct file *filp, loff_t offset, int origin)
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

__poll_t globalfifo_poll(struct file *filp, struct poll_table_struct *wait)
{
    unsigned int mask = 0;

    mutex_lock(&g_globalfifo.m_lock);       //枷锁

    poll_wait(filp, &g_globalfifo.write_wait, wait);
    poll_wait(filp, &g_globalfifo.read_wait, wait);

    if (g_globalfifo.buf_len != 0)          //有数据可读
    {
        mask |= POLLIN | POLLRDNORM;
    }

    if (g_globalfifo.buf_len != GLOBAL_MEM_SIZE)      //可以写数据
    {
        mask |= POLLOUT;
    }
    mutex_unlock(&g_globalfifo.m_lock);

    return mask;
}

static struct file_operations globalfifo_file_opr = {
	.owner	 = THIS_MODULE,
	.open    = globalfifo_open,
    .read    = globalfifo_read,
	.write   = globalfifo_write,
    .unlocked_ioctl = globalfifo_unlocked_ioctl,
    .release = globalfifo_release,
    .llseek = globalfifo_llseek,
    .poll = globalfifo_poll,
};


static int __init globalfifo_init(void)
{
    int ret;
    int err;
    struct device *tmpdev;

    g_globalfifo.devno = MKDEV(global_major, 0);    //创建设备号
    
    if (global_major)
    {
        ret = register_chrdev_region(g_globalfifo.devno, 1, DEVICE_NAME);
    }
    else
    {
        ret = alloc_chrdev_region(&g_globalfifo.devno, 0, 1, DEVICE_NAME);
    }

    cdev_init(&g_globalfifo.cdev, &globalfifo_file_opr);
	cdev_add(&g_globalfifo.cdev, g_globalfifo.devno, 1);
    
    g_globalfifo.class = class_create(THIS_MODULE, "globalmem_class");
	err = PTR_ERR(g_globalfifo.class);
	if (IS_ERR(g_globalfifo.class)) {
        goto failed1;
	}

    //在 /dev目录下创建red_led设备节点
    tmpdev = device_create(g_globalfifo.class , NULL, g_globalfifo.devno, NULL, "globalmem"); 
    if (IS_ERR(tmpdev)) {
        ret = -EINVAL;
		goto failed2;
	}

    mutex_init(&g_globalfifo.m_lock);    //初始化互斥锁

    init_waitqueue_head(&g_globalfifo.read_wait);
    init_waitqueue_head(&g_globalfifo.write_wait);

    printk(KERN_INFO"globalmem init\n");

    return 0;
failed2:
    device_destroy(g_globalfifo.class, g_globalfifo.devno);
    class_destroy(g_globalfifo.class);
failed1:
    cdev_del(&g_globalfifo.cdev);
	unregister_chrdev_region(g_globalfifo.devno, 1);
    return ret;
}

static void __exit globalfifo_exit(void)
{
    device_destroy(g_globalfifo.class, g_globalfifo.devno);
	class_destroy(g_globalfifo.class);
	unregister_chrdev_region(g_globalfifo.devno, 1);
    cdev_del(&g_globalfifo.cdev);
    printk(KERN_INFO"globalmem exit\n");
}

module_init(globalfifo_init);
module_exit(globalfifo_exit);

MODULE_AUTHOR("Ares");
MODULE_LICENSE("GPL");



