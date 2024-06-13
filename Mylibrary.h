#ifndef MYLIBRARY_H
#define MYLIBRARY_H

#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>

// Define constants and global variables
#define MPU6050_ADDR 0xD0
#define LCD_ADDR 0x27
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

extern volatile float a_x, a_y, a_z;
extern volatile float Threshold;
extern volatile uint8_t system_on;

// Function prototypes
void SysClkConf_72MHz(void);
void enter_sleep_mode(void);
float calculateAccelMagnitude(float ax, float ay, float az);
int checkFall(float ax, float ay, float az, float threshold);
void I2C_Init(void);
void MPU_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void MPU6050_Init(void);
void MPU6050_ReadAccelRaw(int16_t* ax, int16_t* ay, int16_t* az);
void LED_Init(void);
void MPU6050_ReadAccel(float* ax, float* ay, float* az);
void I2C_Write(uint8_t data);
void LCD_Write(uint8_t address, uint8_t *data, int size);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void EXTI_Config(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);

#endif // MYLIBRARY_H
