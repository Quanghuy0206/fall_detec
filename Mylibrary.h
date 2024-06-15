#ifndef MYLIBRARY_H
#define MYLIBRARY_H

#include "stm32f1xx_hal.h"  // Include HAL library
#include <stdint.h>

// MPU6050 related functions and definitions
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

extern int16_t Accel_X_RAW;
extern int16_t Accel_Y_RAW;
extern int16_t Accel_Z_RAW;
extern int16_t Gyro_X_RAW;
extern int16_t Gyro_Y_RAW;
extern int16_t Gyro_Z_RAW;
extern float Ax, Ay, Az;
extern float Gx, Gy, Gz;

// External I2C handler declaration
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;

// LCD related functions
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_init(void);
void lcd_send_string(char *str);


//MPU6050 functions
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
float AccelValue(float Ax, float Ay, float Az);

// GPIO Callback function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// Fall detection function
uint8_t fall(void);

// External variables for button states and device state
void LED_Fall_Detected(void);
extern volatile int sw1;
extern volatile int sw2;
extern uint8_t status;
extern uint8_t mode_on;

#endif // MYLIBRARY_H
