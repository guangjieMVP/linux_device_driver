/***********************************************************************
 * @file:        xxx.c
 * @author:      guangjieMVP
 * @version:     v1.00.00
 * @date:        2020-xx-xx
 * @github:      https://github.com/guangjieMVP
 * @brief: 
***********************************************************************/
#ifndef _LED_OPR_H
#define _LED_OPR_H

struct led_operations {
	int num;
	int (*init) (int which); /* 初始化LED, which-哪个LED */       
    int (*deinit)(int which);   
	int (*ctrl) (int which, char status); /* 控制LED, which-哪个LED, status:1-�?0-�?*/
};


struct led_operations *get_board_led_opr(void);

#endif

