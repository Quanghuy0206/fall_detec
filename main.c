#include "Mylibrary.h"
int main(void) {
    Systick_Init();
    I2C_Init();
    MPU6050_Init();
    delayMs(100);
    LED_Init();
    Switch_Init();
    lcd_init();
    set_lcd(0, 0);
    lcd_send_string("0");
    delayMs(50);
    while (1) {
        if (state == 0) {
            LED_Active_Reset();
            Systick_Shutdown();
        } else {
            Systick_Init();
            MPU6050_Read_Accel();
            lcd_set_cursor(0, 4);
            float Value = AccelValue(Ax,Ay,Az);
            if (Value > 2.5) {
               while (1) {
                   LED_Fall_Detected();
                   lcd_send_string("1");
               } 
            }
        }
    }
}
