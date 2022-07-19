
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

/*
 * ./ledtest /dev/100ask_led0 on
 * ./ledtest /dev/100ask_led0 off
 */
int main(int argc, char **argv)
{
	int fd;
	char status;
	
	/* 1. �жϲ��� */
	if (argc != 3) 
	{
		printf("Usage: %s <dev> <on | off>\n", argv[0]);
		return -1;
	}

	/* 2. ���ļ� */
	fd = open(argv[1], O_RDWR | O_NONBLOCK);   //�Է������÷�ʽ�� �� O_NONBLOCK�Ƿ������ñ����ϻ������������������
	if (fd == -1)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}

	/* 3. д�ļ� */
	if (0 == strcmp(argv[2], "on"))
	{
		status = 1;
		write(fd, &status, 1);
	}
	else
	{
		status = 0;
		write(fd, &status, 1);
	}

    sleep(15);
	
	close(fd);
	
	return 0;
}


