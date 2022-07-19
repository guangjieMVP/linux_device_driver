KERNEL_DIR = /home/ares/work/ebf_linux_kernel-ebf_4.19.35_imx6ul

obj-m := ds18b20_drv.o

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) modules
	
.PHONY:clean
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(CURDIR) clean
