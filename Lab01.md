```c
#include <xc.h>
#include<time.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF

int main(){
    //Init LCD and LEDs
    lcd_initialize();
    led_init();
    CLEARBIT(LED1_TRIS);
    CLEARBIT(LED2_TRIS);
    CLEARBIT(LED3_TRIS);
    CLEARBIT(LED4_TRIS);
    CLEARBIT(LED5_TRIS);
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    // Print Hello World
    lcd_printf("Group Members:\r* Name1\r* Name2\r* Name3");
    
    uint16_t i_counter = 0;
    
    while(1){
        lcd_locate(0, 7);
        
        lcd_printf("Counter = %u", i_counter); // u for print unsigned 
        
        LED5_PORT = i_counter & 1;
        Nop();
        LED4_PORT = i_counter >> 1 & 1;
        Nop();
        LED3_PORT = i_counter >> 2 & 1;
        Nop();
        LED2_PORT = i_counter >> 3 & 1;
        Nop();
        LED1_PORT = i_counter >> 4 & 1;
        __delay_ms(600);
        i_counter += 1;
    }
   
}

```
