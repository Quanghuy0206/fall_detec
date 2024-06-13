#ifndef MYLIBRARY_H
#define MYLIBRARY_H

#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>

// Define constants and global variables
#define MPU6050_ADDR 0xD0
#define LCD_ADDR 0x27
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B 
#define ACCEL_CONFIG_REG 0x1C 
#define ACCEL_XOUT_H_REG 0x3B 
#define TEMP_OUT_H_REG 0x41 
#define GYRO_XOUT_H_REG 0x43 
#define PWR_MGMT_1_REG 0x6B 
#define WHO_AM_I_REG 0X75

extern volatile float Ax, Ay, Az;
extern volatile float LimitValue;
extern volatile uint8_t system_on;

// Function prototypes
void SysClkConf_72MHz(void);
void enter_sleep_mode(void);
float AccelValue(float Ax, float Ay, float Az)
void I2C_Init(void);
void MPU_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void LED_Init(void);
void I2C_Write(uint8_t data);
void LCD_Write(uint8_t address, uint8_t *data, int size);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void EXIT_Config(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);

#endif // MYLIBRARY_H
