

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>

#define DEV_NAME   "/dev/hc-sr04"

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
	fd = open(DEV_NAME, O_RDWR);   // | O_NONBLOCK

	if (fd < 0)
	{
		printf("can not open file %s, %d\n", DEV_NAME, fd);
		return -1;
	}

    int time_ns;
    while (1)
    {
        if ((ret = read(fd, &time_ns, 4)) == 4)
        {
            printf("time %d ns %d ms, distance %d mm %d cm\r\n", time_ns, time_ns/1000000, time_ns*340/2/1000000, time_ns*340/2/1000000/10);
        }
        else
        {
            printf("not get time, err %d\r\n", ret);
        }
        sleep_ms(500);
    }
}