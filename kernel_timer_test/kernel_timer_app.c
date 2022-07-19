/*
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
*/
/*
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
*/
/*
  * @file:        xxx.c
  * @author:      guangjieMVP
  * @version:     v1.00.00
  * @date:        2020-xx-xx
  * @github:      https://github.com/guangjieMVP
  * @brief: 
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#define TIMER_MAGIC_NUM  0xFF
#define CLOSE_TIMER   (_IO(TIMER_MAGIC_NUM, 0))
#define OPEN_TIMER    (_IO(TIMER_MAGIC_NUM, 1))
#define MOD_TIMER     (_IO(TIMER_MAGIC_NUM, 2))

enum led_stat {
    LED_ON = 1,
    LED_OFF = !LED_ON,
};

/*
 * ./ledtest /dev/100ask_led0 on
 * ./ledtest /dev/100ask_led0 off
 */
int main(int argc, char **argv)
{
	int fd;
	char status = 0x00;
	
	/* 1. 判断参数 */
	if (argc != 2) 
	{
		printf("Usage: %s <dev> \n", argv[0]);
		return -1;
	}

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);

	if (fd == -1)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}

    status = LED_ON;
    write(fd, &status, 1);      //打开LED
    sleep(2);
    status = LED_OFF;
    write(fd, &status, 1);      //关闭LED
    sleep(2);


//    ioctl(fd, OPEN_TIMER);
    ioctl(fd, MOD_TIMER, 1000);
    printf("set timer 1000ms\r\n");
    sleep(6);

    ioctl(fd, MOD_TIMER, 500);
    printf("set timer 500ms\r\n");
    sleep(6);

    ioctl(fd, MOD_TIMER, 300);
    printf("set timer 300ms\r\n");

    sleep(5);

    ioctl(fd, CLOSE_TIMER);
    printf("close timer\r\n");

    
    status = LED_OFF;
    write(fd, &status, 1);      //关闭LED
  
	close(fd);
	
	return 0;
}


