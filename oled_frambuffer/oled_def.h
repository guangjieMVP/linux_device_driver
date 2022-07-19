/*
 * @brief :   
 * @date :  2021-11-xx
 * @version : v1.0.0
 * @copyright(c) 2020 : OptoMedic company Co.,Ltd. All rights reserved
 * @Change Logs:   
 * @date         author         notes:  
 */
#ifndef _OLED_DEF_H_
#define _OLED_DEF_H_

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#define OLED_CMD_SET_XY                   0x01       /* 显示开关*/
#define OLED_CMD_WRITE_DATAS              0x02
#define OLED_CMD_SET_XY_WRITE_DATAS       0x03
#define OLED_CMD_DISP_ON_OFF              0x04

#define CMD_COMBINE(cmd, datasize)        (cmd | (datasize << 8))         /* 命令和数据大小组合 */


struct oled_disp_buffer {
    uint8_t x;
    uint8_t y;
    uint16_t len;
    uint8_t *buffer;
};
typedef struct oled_disp_buffer  oled_disp_buf_t;

#endif /* _OLED_DEF_H_ */