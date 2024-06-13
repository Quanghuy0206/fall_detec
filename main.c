#include "Mylibrary.h"

int main(void) {
    // Call initialization functions
    SysClkConf_72MHz();
    LED_Init();
    I2C_Init();
    MPU6050_Init();
    EXTI_Config();
    lcd_init();
    lcd_set_cursor(0, 4);
    lcd_send_string("bat");
    delayMs(50);

    while (1) {
        // Main program loop
        if (!system_on) {
            lcd_clear();
            lcd_set_cursor(0, 4);
            lcd_send_string("tat");
            EXTI->IMR &= ~EXTI_IMR_MR0; // disable EXTI0 interrupt
            enter_sleep_mode();
            SysClkConf_72MHz(); // reconfigure system clock frequency
            EXTI->IMR |= EXTI_IMR_MR0; // enable EXTI0 interrupt
        }
    }
}
