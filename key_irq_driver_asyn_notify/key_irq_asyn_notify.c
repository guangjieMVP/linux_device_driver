

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <signal.h>


/*
 * ./ledtest /dev/100ask_led0 on
 * ./ledtest /dev/100ask_led0 off
 */
 
static int fd;
static int count = 0;
 
void input_handler(int num)
{
  char data;
  int len;
  /*  读取并输出 STDIN_FILENO 上的输入  */
  len = read(fd, &data, 1);
  count = 0;
  printf("key is press val : %d\n", data);
}


int main(int argc, char **argv)
{
	
  int ret;
	char status = 0x00;
  int timeid = 0;
 
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
 
  int oflags;

/*  启动信号驱动机制  */
  signal(SIGIO, input_handler);
  
  fcntl(fd, F_SETOWN, getpid());
  oflags = fcntl(fd, F_GETFL);
  fcntl(fd, F_SETFL, oflags | FASYNC);
  
 
  while (1)
  {
      count++;
      printf("count = %d\r\n", count);
      sleep(2);
  }

	close(fd);
	
	return 0;
}


