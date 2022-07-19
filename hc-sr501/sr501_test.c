/*
 * @Author: your name
 * @Date: 2022-01-03 14:11:59
 * @LastEditTime: 2022-01-03 16:18:52
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \ebf_linux_kernel-ebf_4.19.35_imx6ulc:\Users\guangjie\DOCUME~1\MobaXterm\slash\RemoteFiles\67840_5_11\sr501_test.c
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>

#define DEV_NAME   "/dev/sr501"

int main(int argc, char **argv)
{
	int fd;
    int ret;
  
    struct pollfd fds[1];
	
	/* 2. 打开文件 */
	fd = open(DEV_NAME, O_RDWR);   // | O_NONBLOCK

	if (fd < 0)
	{
		printf("can not open file %s\n", DEV_NAME);
		return -1;
	}

    int state;
    while (1)
    {
        if ((ret = read(fd, &state, 4)) == 4)
            printf("state %d\r\n", state);
        else
            printf("not get stae %d\r\n", ret);
        // sleep(1);
    }

}