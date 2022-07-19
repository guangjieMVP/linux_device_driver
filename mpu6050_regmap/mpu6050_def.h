#ifndef _MPU6050_DEF_H_
#define _MPU6050_DEF_H_

typedef unsigned char   uint8_t;
typedef unsigned int    uint32_t;

/* register define */
#define MPU6050_SMPLRT_DIV                                  0x19     /* */
#define MPU6050_CONFIG                                      0x1A    
#define MPU6050_GYRO_CONFIG                                 0x1B
#define MPU6050_ACCEL_CONFIG                                0x1C
#define MPU6050_ACCEL_XOUT_H                                0x3B
#define MPU6050_ACCEL_XOUT_L                                0x3C
#define MPU6050_ACCEL_YOUT_H                                0x3D
#define MPU6050_ACCEL_YOUT_L                                0x3E
#define MPU6050_ACCEL_ZOUT_H                                0x3F
#define MPU6050_ACCEL_ZOUT_L                                0x40
#define MPU6050_TEMP_OUT_H                                  0x41
#define MPU6050_TEMP_OUT_L                                  0x42
#define MPU6050_GYRO_XOUT_H                                 0x43
#define MPU6050_GYRO_XOUT_L                                 0x44
#define MPU6050_GYRO_YOUT_H                                 0x45
#define MPU6050_GYRO_YOUT_L                                 0x46
#define MPU6050_GYRO_ZOUT_H                                 0x47
#define MPU6050_GYRO_ZOUT_L                                 0x48
#define MPU6050_PWR_MGMT_1                                  0x6B       /* power managemnt register */
#define MPU6050_WHO_AM_I                                    0x75       /* default value :  IIC addr 0x68 */

/* interrupt status register */
#define MPU6050_INT_STATUS                                  0x3A
#define MPU6050_INT_ENABLE                                  0x38
#define MPU6050_INT_PIN_CFG                                 0x37

#define I2C_RETRIES                                 0x0701
#define I2C_TIMEOUT                                 0x0702
#define I2C_SLAVE                                   0x0703             //IIC从器件的地址设置
#define I2C_BUS_MODE                                0x0780

#define MPU6050_SlaveAddress                                  0xD0
#define MPU6050_IIC_ADDR                                      0x68                  //MPU6050 IIC Address

struct mpu6050_accel {
    short x;
    short y;
    short z;
};

struct mpu6050_gyro {
    short x;
    short y;
    short z;
};

struct mpu6050_data {
    struct mpu6050_accel accel;
    struct mpu6050_gyro gyro;
};


#endif /* _MPU6050_DEF_H_ */