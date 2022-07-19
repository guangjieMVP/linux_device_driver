#ifndef _AP3216C_H_
#define _AP3216C_H_

#include <linux/i2c.h>

/* System Registers define */
#define AP3216C_REG_SYS_CONFIG         0x00          /* Control of basic functions */
#define AP3216C_REG_INT_STATUS         0x01          /* ALS and PS interrupt status output  */
#define AP3216C_REG_INT_CLEAR_MANNER   0x02          /* Auto/semi clear INT pin selector */

#define AP3216C_REG_IR_DATA_LOW        0x0A          /* Lower byte for IR ADC channel output */
#define AP3216C_REG_IR_DATA_HIGH       0x0B          /* Higher byte for IR ADC channel output  */
#define AP3216C_REG_ALS_DATA_LOW       0x0C          /* Lower byte for ALS ADC channel output  */
#define AP3216C_REG_ALS_DATA_HIGH      0x0D          /* Higher byte for ALS ADC channel output */
#define AP3216C_REG_PS_DATA_LOW        0x0E          /* Lower byte for PS ADC channel output */
#define AP3216C_REG_PS_DATA_HIGH       0x0F          /* Higher byte for PS ADC channel output */

/* ALS rAP3216C_egisters define */
#define AP3216C_REG_ALS_CONFIG               0x10          /* Control of gain, conversion time of persist for ALS */
#define AP3216C_REG_ALS_CALIBRATION          0X19          /* ALS window loss calibration  */
#define AP3216C_REG_ALS_LOW_THRESHOLD_LOW    0x1A         /* Lower byte of ALS low threshold */
#define AP3216C_REG_ALS_LOW_THRESHOLD_HIGH   0x1B         /* Higher byte of ALS low threshold */
#define AP3216C_REG_ALS_HIGH_THRESHOLD_LOW    0x1C        /* Lower byte of ALS high threshold */
#define AP3216C_REG_ALS_HIGH_THRESHOLD_HIGH   0x1D        /* Higher byte of ALS high threshold */

/* PS reAP3216C_gisters define */
#define AP3216C_REG_PS_CONFIG                 0x20        /* Control of gain, integrated time and persist for PS */
#define AP3216C_REG_PS_LED_DRIVER             0x21        /* Control of LED pulses number and driver current */
#define AP3216C_REG_PS_INT_FORM               0x22        /* Interrupt algorithms style select of PS */
#define AP3216C_REG_PS_MEAN_TIME              0x23        /* PS average time selector */
#define AP3216C_REG_PS_LED_WAITING_TIME       0x24        /* Control PS LED waiting time */
#define AP3216C_REG_PS_CALIBRATION_L          0x28        /* Offset value to eliminate cross talk */
#define AP3216C_REG_PS_CALIBRATION_H          0x29        /* Offset value to eliminate cross talk */
#define AP3216C_REG_PS_LOW_THRESHOLD_LOW      0x2A        /* Lower byte of PS low threshold */
#define AP3216C_REG_PS_LOW_THRESHOLD_HIGH     0x2B        /* Higher byte of PS low threshold */
#define AP3216C_REG_PS_HIGH_THRESHOLD_LOW     0x2C         /* Lower byte of PS high threshold */
#define AP3216C_REG_PS_HIGH_THRESHOLD_HIGH    0x2D         /* Higher byte of PS high threshold */

enum sys_config_value {
    AP3216C_SYS_CONF_POWER_DOWN = 0x000,
    AP3216C_SYS_CONF_ALS_ACTIVE = 0x001,
    AP3216C_SYS_CONF_PS_IR_ACTIVE = 0x002,
    AP3216C_SYS_CONF_ALS_PS_IR_ACTIVE = 0x03,
    AP3216C_SYS_CONF_SW_RESET = 0x04,       /* software reset */
    AP3216C_SYS_CONF_ALS_ONCE = 0x05,
    AP3216C_SYS_CONF_PS_IR_ONCE = 0x06,
    AP3216C_SYS_CONF_ALS_PS_IRONCE = 0x07,
};

#endif /* _AP3216C_H_ */