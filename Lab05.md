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

uint16_t global_counter_tmr2;

/*
 * touch screen code
 */
void initialize_timer(){
    // Setup Timer 2
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    TMR2 = 0x00; 
    T2CONbits.TCKPS = 0b10; // select 1:64 prescaler
    CLEARBIT(IFS0bits.T2IF);
    CLEARBIT(IEC0bits.T2IE);
    // set timer period 20ms: 4000 = 20*10^-3 * 12.8MHz / 64
    PR2 = 4000;
    
    // Set Interrupt Priority
    IPC1bits.T2IP = 1;      // in dsPIC datasheet, timer2 priority can be set with the IPC1 register
    // Clear Interrupt Flags
    CLEARBIT(IFS0bits.T2IF);
    // Enable Interrupts
    IEC0bits.T2IE = 1;
    
    
    SETBIT(T2CONbits.TON);
}

void initialize_servos(uint8_t servo_number){
    if (servo_number == 0){ // x-servo --> OC8
        // Setup OC8
        CLEARBIT(TRISDbits.TRISD7);
        OC8R = 3700; // set initial duty cycle to 20-1.5ms
        OC8RS = 3700; // load OCRS: next PWM duty cycle
        OC8CON = 0x0006; // set OC8: PWM, no fault check, Timer 2
    }
    else if (servo_number == 1){ // y-servo --> OC7
        // Setup OC7
        CLEARBIT(TRISDbits.TRISD6);
        OC7R = 3700;
        OC7RS = 3700;
        OC7CON = 0x0006;
    }
}

void set_duty_cycle(uint8_t servo_number, double duty_cycle){
    double duty_period = duty_cycle * FCY * 1/64000;
    if (servo_number == 0){ // x-axis
        OC8RS = (uint16_t)(duty_period);
    } else if (servo_number == 1){ //y-axis
        OC7RS = (uint16_t)(duty_period);
    }
}


/*
 * main loop
 */
void initialize_touchscreen(){
    // set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1);
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);
    // AN15 --> x-coordinate
    // Disable ADC
    CLEARBIT(AD1CON1bits.ADON);
    //initialize PIN
    SETBIT(TRISBbits.TRISB15); // Set TRISB RB15 to input
    CLEARBIT(AD1PCFGLbits.PCFG15); // Set AD1 AN15 input pin as analog
    //AN9 --> y-coordinate
    //initialize PIN
    SETBIT(TRISBbits.TRISB9); // Set TRISB RB9 to input
    CLEARBIT(AD1PCFGLbits.PCFG9); // Set AD1 AN9 input pin as analog
    
    //Configure AD1CON1
    CLEARBIT(AD1CON1bits.AD12B); // Set 10b Operation Mode
    AD1CON1bits.FORM = 0; // Set integer output
    AD1CON1bits.SSRC = 0x7; // Set automatic conversion
    // Configure AD1CON2
    AD1CON2 = 0; // Not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); // Internal clock source
    AD1CON3bits.SAMC = 0x1F; // Sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; // Tad = 3Tcy (Time cycles)
    // Leave AD1CON4 at its default value
    // Enable ADC
    SETBIT(AD1CON1bits.ADON);
    
}

void change_dimension(uint8_t dimension){ // 0: x, 1: y

    if (dimension == 0){
        // set up the I/O pins E1, E2, E3 so that the touchscreen's x-coordinate pin connects to the ADC
        CLEARBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        SETBIT(PORTEbits.RE3);
    } else if (dimension == 1){
    // set up the I/O pins E1, E2, E3 so that the touchscreen's y-coordinate pin connects to the ADC
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
    }
    __delay_ms(10);
}

uint16_t read_ball_position_x(){
    // x-pin
    AD1CHS0bits.CH0SA = 0x00F; // Set ADC to Sample AN15 pin
    SETBIT(AD1CON1bits.SAMP); // Start to sample
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // MUST HAVE! Clear conversion done bit
    return ADC1BUF0;
}

uint16_t read_ball_position_y(){
    // y-pin
    AD1CHS0bits.CH0SA = 0x009; // Set ADC to Sample AN9 pin
    SETBIT(AD1CON1bits.SAMP); // Start to sample
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // MUST HAVE! Clear conversion done bit
    return ADC1BUF0;
}


void main_loop()
{
    
    
    // initialize touchscreen
    initialize_touchscreen();
    // initialize servos
    initialize_servos(0);
    initialize_servos(1);
    initialize_timer();
    
    
    
    
    /*while(TRUE){
        set_duty_cycle(0, 20-2.1);
        set_duty_cycle(1, 20-2.1);*/
    
    while(TRUE) {
        
        // print assignment information
        lcd_locate(0, 0);

        lcd_printf("Lab05: Touchscreen &\r\n");
        lcd_printf("       Servos");
        lcd_locate(0, 2);
        lcd_printf("Group: Group 2, lab session 3");
        lcd_locate(0, 5);
        change_dimension(0);
        lcd_printf("x: %u", read_ball_position_x());
        lcd_locate(0, 6);
        change_dimension(1);
        lcd_printf("y: %u", read_ball_position_y());
        
        
        if(global_counter_tmr2 >= 1000 ){
            global_counter_tmr2  = 0;
        }
        if (global_counter_tmr2 >=0 && global_counter_tmr2 <250){
            set_duty_cycle(0, 20-0.9); // bottom
            set_duty_cycle(1, 20-0.9); // left 
        }if(global_counter_tmr2 >= 250 && global_counter_tmr2 < 500){
            set_duty_cycle(0, 20-0.9); // bottom
            set_duty_cycle(1, 20-2.1); // right
        }if(global_counter_tmr2 >= 500 && global_counter_tmr2 < 750){
            set_duty_cycle(0, 20-2.1); // top 
            set_duty_cycle(1, 20-2.1); // right
        }if(global_counter_tmr2 >= 750){
            set_duty_cycle(0, 20-2.1);
            set_duty_cycle(1, 20-0.9);   
        }
        lcd_locate(0,7);
        lcd_printf("%u", global_counter_tmr2); 
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
    IFS0bits.T2IF = 0; // Clear the interrupt flag
    global_counter_tmr2++; // Increment a global counter
}
