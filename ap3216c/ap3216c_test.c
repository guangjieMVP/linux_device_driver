#include "stdio.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <stdint.h>

#define DEV_NAME   "/dev/ap3216c"

struct ap3216c_data {
    short int als;        /* 环境光亮度传感器数据 */
    short int ps;         /* 接近传感器数据 */
    short int ir;         /* 红外LED */
};

void sleep_ms(unsigned int ms)
{
    struct timeval delay;
	delay.tv_sec = 0;
	delay.tv_usec = ms * 1000; 
	select(0, NULL, NULL, NULL, &delay);
}

int main(int argc, char **argv)
{
    
    int fd;
    int ret;
  
    struct pollfd fds[1];
	
	/* 2. 打开文件 */
	fd = open(DEV_NAME, O_RDWR );   // | O_NONBLOCK

	if (fd < 0)
	{
		printf("can not open file %s, %d\n", DEV_NAME, fd);
		return -1;
	}

	struct ap3216c_data data;

	while (1)
	{
		if (read(fd, &data, sizeof(data)) == sizeof(data))
		{
			printf("als %d ps %d ir %d\r\n", data.als, data.ps, data.ir);
		}
		sleep_ms(1000);
	}
     
 
 
    printf("%s ok!\n", DEV_NAME);
    
    return 0;
}