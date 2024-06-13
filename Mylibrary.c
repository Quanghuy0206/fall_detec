#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>

// dia chi I2C cua MPU6050
#define MPU6050_ADDR 0xD0 //0x68<<1 

// dia chi cua  lcd
#define LCD_ADDR 0x27 // Ðia chi I2C cua LCD 

// thanh ghi cua MPU6050
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B 
#define ACCEL_CONFIG_REG 0x1C 
#define ACCEL_XOUT_H_REG 0x3B 
#define TEMP_OUT_H_REG 0x41 
#define GYRO_XOUT_H_REG 0x43 
#define PWR_MGMT_1_REG 0x6B 
#define WHO_AM_I_REG 0X75

//gia tri gia toc theo 3 phuong
int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;
volatile float Ax,Ay,Az;



volatile float Limit = 2.5; // GIA TRI DAT PHAT HIEN NGA

//bien trang thai cua he thong
volatile uint8_t system_on = 1;

//khai bao ham
void SysClkConf_72MHz(void) ;
void enter_sleep_mode(void);
float calculateAccelMagnitude(float ax, float ay, float az);
int checkFall(float ax, float ay, float az, float Limit);
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

//ham cau hinh xung he thong
void SysClkConf_72MHz(void) {
    //su dung hse
    RCC->CR |= RCC_CR_HSEON;
    while((RCC->CR & RCC_CR_HSERDY) == 0); // doi san sang

    //cau hinh PLL 
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // *9 = systemclock = 72MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // ADC prescale 6.
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB1 prescale 2.

    //chon nguon hse cho pll
    RCC->CFGR |= RCC_CFGR_PLLSRC; // PLLSRC HSE

    //bat pll
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0); // wait PLLRDY.

    //doi sysclock sang pll
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) == 0); //wait SWS.

    //tat hsi
    RCC->CR &= ~(RCC_CR_HSION); // off HSION
    while((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);
}

//ham delay
void delayMs(uint32_t ms){
	uint32_t i;
	for(i=0;i<ms;i++){
	SysTick->LOAD = 9000-1;
	SysTick->VAL = 0;
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE;
    }
}
//ham vao che do ngu
void enter_sleep_mode(void) {
    // cap xung cho pwr
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // xoa co pdds de vao Stop mode 
    PWR->CR &= ~PWR_CR_PDDS;

    // dat lpds de vao che do low-power deepsleep
    PWR->CR |= PWR_CR_LPDS;

    // dat che do sleepdeep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // xoa WFI/WFE de chuan bi ngu dung cac ham de hoan thanh cac lenh truoc
    __DSB();
    __ISB();

    // vao che do ngu
    __WFI();
}

//ham tinh toan do lon gia toc theo 3 phuong
float calculateAccelMagnitude(float ax, float ay, float az) {
    return sqrt(ax * ax + ay * ay + az * az);
}

//ham kiem tra nga
int checkFall(float ax, float ay, float az, float Limit) {
    float accelMag = calculateAccelMagnitude(ax, ay, az);
    return (accelMag > Limit) ? 1 : 0;
}

//cau hinh i2c
void I2C_Init(void) {
    // cap xung cho I2C va GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

		// cau hinh PB6 va PB7 cho I2C1 (scl va sda)
    // pull up
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1;
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
		GPIOB->ODR |= ((1<<6 | 1<<7));  
		
    // reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // cau hinh I2C1
    I2C1->CR2 |= 36; // tan so PCLK1
    I2C1->CCR = 180; // che do tieu chuan, 100kHz
    I2C1->TRISE = 37; //rise time toi da

    // bat I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

//ham ghi vao thanh ghi qua i2c
void MPU_WriteReg(uint8_t reg, uint8_t data) {
    // khoi dong i2c
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)); // doi den khi i2c duoc khoi dong

    // gui dia chi muc tieu(mpu6050) 
    I2C1->DR = MPU6050_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // gui dia chi cua thanh ghi muon ghi 
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // ghi du lieu vao thanh ghi reg
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data; // Send data
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // ket thuc i2c
    I2C1->CR1 |= I2C_CR1_STOP;
}

//ham doc tu thanh ghi qua i2c
uint8_t MPU_ReadReg(uint8_t reg) {
    uint8_t data;
    // bat dau i2c
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // gui dia chi cua mpu6050
    I2C1->DR = MPU6050_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // gui dia chi thanh ghi muon doc
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // khoi dong lai i2c de chuan bi cho viec doc
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // doc du lieu dia chi MPU6050_ADDR va set bit lsb = 1 de cho biet la muon nhan du lieu
    I2C1->DR = MPU6050_ADDR | 0x01;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; // xoa thanh ghi sr1, sr2 sau khi ghi hoac doc

    // xoa ack va gui tin hieu stop roi doc du lieu
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // cho tin hieu interrupt
    data = I2C1->DR;

    return data;
}

//ham cau hinh mpu
void MPU6050_Init(void) {
  uint8_t check, Data;

  // Đoc thanh ghi WHO_AM_I đe kiem tra thiet bi MPU6050
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
  if (check == 0x68)  // Kiểm tra mã ID của thiết bị MPU6050
  {
    // Đua thiet bi MPU6050 ra khoi che đo quan ly nang luc bang cach đat thanh ghi quan la 0
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

    // Cau hinh ty le du lieu 1kHz bang cach cai cau hinh thanh ghi SMPLRT_DIV
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

    // Cai cau hinh gia toc đo lên đo nhay ±2g trong thanh ghi ACCEL_CONFIG
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

    // Cai cau hinh tron xoay lên độ nhay ±250°/s trong thanh ghi GYRO_CONFIG
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
  }
}


void MPU6050_Read_Accel(void) {
  uint8_t Rec_Data[6];
  // Doc 6 BYTES data tu ACCEL_XOUT_H register
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

  // Tinh gia toc 3 truc
  Ax = Accel_X_RAW / 16384.0;
  Ay = Accel_Y_RAW / 16384.0;
  Az = Accel_Z_RAW / 16384.0;
}

void I2C_Write(uint8_t data){
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void LCD_Write(uint8_t address, uint8_t *data, int size){
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    uint8_t addr = (address << 1) ;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR1;
    (void) I2C1->SR2;
    for (uint8_t i = 0; i < size; i++) {
        I2C_Write(data[i]);  
    }
    I2C1->CR1 |= I2C_CR1_STOP;
}

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, 0x4E,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

// Hàm khoi tao LCD
void lcd_init (void)
{
	// 4 bit initialisation
	DelayMs(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	DelayMs(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	DelayMs(1);  // wait for >100us
	lcd_send_cmd (0x30);
	DelayMs(10);
	lcd_send_cmd (0x20);  // 4bit mode
	DelayMs(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	DelayMs(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	DelayMs(1);
	lcd_send_cmd (0x01);  // clear display
	DelayMs(1);
	DelayMs(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	DelayMs(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
// Ham thiet lap vi tri con tro
void set_lcd(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void EXTI_Config(void) {
    // cap xung cho afio và gpioa de su dung lam ngat cong tac
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		
    // pull up cho interrupt tu mpu6050 
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= 1<<0;

    // pullup cho cong tac tat mo he thong
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;
		GPIOA->ODR |= 1<<1;
		
    // cau hình afio và exti
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA; // bat 2 chan pa0 va pa1 voi nhiem vu ngat ngoai
    EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1; // xoa pending 
    
		//chon suon xuong
		EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1;

    // xoa suon len    
    EXTI->RTSR &= ~(EXTI_RTSR_TR0);
    EXTI->RTSR &= ~(EXTI_RTSR_TR1);

    // chon interrupt xoa event
    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;
    EXTI->EMR &= ~(EXTI_EMR_MR0);
    EXTI->EMR &= ~(EXTI_EMR_MR1);

    // dat muc do uu tien
		// ngat cua mpu6050
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
		//ngat cua nut tat bat he thong
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        system_on = !system_on; // dao nguoc che do
    }
}
void lcd_Writedata(int a){
		if(a==1){
			lcd_clear();
			lcd_set_cursor(1,4);
			lcd_send_string("nga");	
			delayMs(50);
		}
		else if(a==0){
			lcd_clear();
			lcd_set_cursor(1,4);
			lcd_send_string("binh thuong");	
			delayMs(50);
		}
}
void EXTI0_IRQHandler(void) {
		
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;
        if (system_on) {
            MPU6050_ReadAccel(&a_x, &a_y, &a_z);
            if (checkFall(a_x, a_y, a_z, Limit)) {						
		lcd_Writedata(1);
                GPIOC->ODR ^= (1 << 13);
		delayMs(100);
            } else {
		lcd_Writedata(0);
                GPIOC->BSRR |= (1 << 13); // tat den
            }
        }
    }
}
void LED_Init(void) {
	//led pc13 
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13;
    GPIOC->BSRR |= (1 << 13); // tat den
}
