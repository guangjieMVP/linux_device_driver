

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>


int main(int argc, char **argv)
{
	int fd;
	int ret;
	char status = 0x00;
	int timeid = 0;

	struct pollfd fds[1];

	/* 1. 判断参数 */
	if (argc != 2)
	{
		printf("Usage: %s <dev> \n", argv[0]);
		return -1;
	}

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);

	if (fd < 0)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}

	fds[0].fd = fd;
	fds[0].events = POLLIN | POLLRDNORM;

	while (1)
	{
		ret = poll(fds, 1, 5000);
		if (ret == 1 && (fds[0].events & POLLIN))
		{
			read(fd, &status, 1);
			timeid++;
			printf("%d key press val = %d\r\n", timeid, status);
			//        status = 0x00;
		}
		if (ret == 0)
		{
			printf("poll timeout\r\n");
		}
	}

	close(fd);

	return 0;
}
