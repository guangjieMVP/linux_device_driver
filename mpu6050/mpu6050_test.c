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

#define DEV_NAME   "/dev/mpu6050"

struct mpu6050_accel {
    short x;
    short y;
    short z;
};

struct mpu6050_gyro {
    short x;
    short y;
    short z;
};

struct mpu6050_data {
    struct mpu6050_accel accel;
    struct mpu6050_gyro gyro;
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
  
	/* 2. 打开文件 */
	fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);   // | O_NONBLOCK

	if (fd < 0)
	{
		printf("can not open file %s, %d\n", DEV_NAME, fd);
		return -1;
	}
     
	struct mpu6050_data   mpu6050_data;
	int size = sizeof(struct mpu6050_data);

	printf("read size %d\n", size);

	while (1)
	{
		if (read(fd, &mpu6050_data, size) == size)
		{
			printf("accel x %d y %d z %d\r\n", mpu6050_data.accel.x, mpu6050_data.accel.y, mpu6050_data.accel.z);
			printf("gyro x %d y %d z %d\r\n", mpu6050_data.gyro.x, mpu6050_data.gyro.y, mpu6050_data.gyro.z);
		}
		printf("running\r\n");
		sleep_ms(500);
	}
    
	close(fd);
    
    return 0;
}