/*
 * @brief :  °´¼ü²âÊÔapp³ÌÐò 
 * @date :   2021-1-17
 * @version : v1.0.0
 * @Change Logs:   
 * @date         author         notes:  
 * 2021-1-17    guangjieMVP     first version
 */


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <signal.h>
#include <linux/input.h> 

static int fd;

static struct input_event inputevt; 

int main(int argc, char **argv)
{
    int ret;
 
	if (argc != 2) 
	{
		printf("Usage: %s <dev> \n", argv[0]);
		return -1;
	}

	fd = open(argv[1], O_RDWR);

	if (fd < 0)
	{
		printf("can not open file %s\n", argv[1]);
		return -1;
	}
 
    while (1)
    {
        ret = read(fd, &inputevt, sizeof(inputevt)); 
        if (ret > 0)
        {
            switch (inputevt.type)
            {
                case EV_KEY :
                    printf("key press %d %d\r\n", inputevt.code, inputevt.value);
                    break;
            }
        }
        memset(&inputevt, 0, sizeof(inputevt));
    }

	close(fd);
	
	return 0;
}


