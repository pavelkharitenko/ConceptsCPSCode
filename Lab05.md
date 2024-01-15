```c
#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * PWM code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

/*
 * touch screen code
 */

void init_AD1_for_ABS(){
        // init ADC's pins AN9 (for Y) and AN15 (for X), from Lab Manual 3.6
    
    // disalble ADC1
    AD1CON1.ADON = 0;
    // set to input
    TRISBbits.TRISB9 = 1; // AN9 is B9
    TRISBbits.TRISB15 = 1; // AN15 is B15
    // set to analogue input
    AD1PCFGLbits.PCFG9 = 0;
    AD1PCFGLbits.PCFG9 = 0;
    //Configure AD1CON1, set to 12bits operation mode
    SETBIT(AD1CON1bits.AD12B);
    // set output as integer
    AD1CON1bits.FORM = 0;
    // Set automatic conversion
    AD1CON1bits.SSRC = 0x7; 
    
    // Configure AD1CON2
    // Not using scanning sampling
    AD1CON2 = 0; 
    //Configure AD1CON3
    // Internal clock source
    CLEARBIT(AD1CON3bits.ADRC); 
     // Sample-to-conversion clock = 31Tad
    AD1CON3bits.SAMC = 0x1F;
    // Tad = 3Tcy (Time cycles)
    AD1CON3bits.ADCS = 0x2; 
    // Also leave AD1CON4 at its default value
    
    // Enable ADC1
    SETBIT(AD1CON1bits.ADON);
}

void init_touchscreen(){
    
    init_AD1_for_ABS();

    
    // Set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
    CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output
    Nop();
    
    // set to standby mode by setting E1 = 0, E2 = 0, E3 = 0
    CLEARBIT(PORTEbits.RE1);
    CLEARBIT(PORTEbits.RE2);
    CLEARBIT(PORTEbits.RE3);
    
    __delay_ms(10);
}

// 0 = X,   1 = Y
void change_read_dimension(uint16_t dim){
    
    // set X dim to be read
    if(dim == 0){
         CLEARBIT(PORTEbits.RE1);
         SETBIT(PORTEbits.RE2);
         SETBIT(PORTEbits.RE3);
    }
    
    // set Y dim to be read
    if(dim == 1){
        // set y+ to vdd (E2=0), y- to GND(E3=0), x+ to hi-z(an9), x- to hi-z
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
    }
    
    __delay_ms(10);
}

uint16_t read_ball_x(){
    // Read from AN15
    
    AD1CHS0bits.CH0SB = 0x014; 
    
    return 0;
}


/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: Lab 3 Group 2");
    
    // initialize touchscreen
    
    
    // initialize servos
    
    while(TRUE) {
        
    }
}

``` 
