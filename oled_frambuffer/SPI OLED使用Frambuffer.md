## MMAP和IOREAMAP的区别

```c
virtual addr = ioremap(phy_adrr, size)   //将物理地址转换成虚拟地址
//得到的虚拟地址是给驱动程序用
```

```c
mmap 将内核空间的物理地址转换为用户空间的虚拟地址，给应用程序使用 
```



**应用程序关注Frambuffer的内容**：

```c
1、mmap得到的虚拟地址
2、LCD的x/y分辨率
3、bpp 像素大小
```



## 编写frambuffer驱动步骤

**内核frambuffer实现文件:**  fbmem.c

* 分配fb_info
* 设置fb_info
  * fb_var
  * fb_fix
* 注册fb_info
* 硬件操作



## frambuffer驱动框架分析









