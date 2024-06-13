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
extern volatile float Limit;
extern volatile uint8_t status;

// Khai bao ham
float AccelValue(float ax, float ay, float az);
void I2C_Init(void);
void MPU6050_Init(void);
void LED_Init(void);
void I2C_Write(uint16_t devAddress, uint8_t regAddress, uint8_t *data, uint16_t size);
void I2C_Read(uint16_t devAddress, uint8_t regAddress, uint8_t *data, uint16_t size);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void set_lcd(int row, int col);
void lcd_clear(void);
void SysClkConf_72MHz(void);
void EXIT_Config(void);
void delayMs(uint32_t ms);

#endif // MYLIBRARY_H
