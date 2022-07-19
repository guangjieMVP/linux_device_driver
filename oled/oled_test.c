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
#include <stdlib.h>

#include "oled_def.h"
#include "font.h"

#define DEV_NAME   "/dev/oled"

int oled_fd;

void sleep_ms(unsigned int ms)
{
    struct timeval delay;
	delay.tv_sec = 0;
	delay.tv_usec = ms * 1000; 
	select(0, NULL, NULL, NULL, &delay);
}


void oled_disp_char(int x, int y, unsigned char c)
{
	int i = 0;
	/* 得到字模 */
	const unsigned char *dots = oled_asc2_8x16[c - ' '];
	char pos[2];
#if 0
	/* 发给OLED */
	OLED_DIsp_Set_Pos(x, y);
	/* 发出8字节数据 */
	for (i = 0; i < 8; i++)
		oled_write_cmd_data(dots[i], OLED_DATA);
#endif
	pos[0] = x;
	pos[1] = y;

	ioctl(oled_fd, CMD_COMBINE(OLED_CMD_SET_XY, 2), &pos);
	ioctl(oled_fd, CMD_COMBINE(OLED_CMD_WRITE_DATAS, 8), dots);

#if 0
	OLED_DIsp_Set_Pos(x, y+1);
	/* 发出8字节数据 */
	for (i = 0; i < 8; i++)
		oled_write_cmd_data(dots[i+8], OLED_DATA);
#endif	
	pos[0] = x;
	pos[1] = y+1;
	ioctl(oled_fd, CMD_COMBINE(OLED_CMD_SET_XY, 2), pos);
	ioctl(oled_fd, CMD_COMBINE(OLED_CMD_WRITE_DATAS, 8), &dots[8]);

	// ioctl(fd, OLED_SET_XY, pos);
	// ioctl(fd, OLED_SET_DATAS | (8<<8), &dots[8]);
}

void oled_disp_string(uint8_t x, uint8_t y, char *str)
{
	uint8_t j = 0;

	while (str[j])
	{		
		oled_disp_char(x, y, str[j]);  /* 显示单个字符 */
		x += 8;
		if(x > 127)
		{
			x = 0;
			y += 2;
		}  
		j++;    /* 移动显示位置 */
	}
}

static void oled_test(void)
{
	oled_disp_string(0, 0, "Sad!");
	oled_disp_string(0, 2, "Bad!");
	oled_disp_string(0, 4, "Moonlight");
}

int main(int argc, char **argv)
{

    int ret;

	/* 2. 打开文件 */
	oled_fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);   // | O_NONBLOCK

	if (oled_fd < 0)
	{
		printf("can not open file %s, %d\n", DEV_NAME, oled_fd);
		return -1;
	}
    
	// short temp;
	// read(oled_fd, &temp, sizeof(temp));
	oled_test();

	sleep_ms(5000);
    
	close(oled_fd);
    
    return 0;
}