include/linux/spi/spi.h :

```C
使用 struct  spi_master 描述主机驱动
```

```c
使用 struct spi_driver描述设备驱动
    
struct spi_driver { 
    const struct spi_device_id *id_table; 
    int         (*probe)(struct spi_device *spi); 
    int         (*remove)(struct spi_device *spi); 
    void        (*shutdown)(struct spi_device *spi); 
    struct device_driver    driver; 
}; 

int spi_register_driver(struct spi_driver *sdrv) 
    
void spi_unregister_driver(struct spi_driver *sdrv) 
    

```



