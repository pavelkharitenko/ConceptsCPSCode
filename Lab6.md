```c

#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>
#include <stdbool.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * Parameters
 */
#define pi 3.1415926
#define radius 100
#define center_x 400
#define center_y 370
#define circle_period 5 // number of seconds to do a circle
#define omega 2*pi/circle_period // rad/s (v = 2*pi*r/T), w = v/r)



#define k_px 0.001
//#define k_dx 0.055
#define k_dx 0.02

#define k_py 0.001
#define k_dy 0.02

/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

/*
 * Global Variables
 */
// butterworth parameters
double butter_a = -0.67959929;
double butter_b1 = 0.16020035;
double butter_b2 = 0.16020035;

uint16_t global_counter_tmr3;
bool workload_status = false;
uint16_t missed_deadlines = 0;

volatile uint16_t setptX = center_x + radius;
volatile uint16_t setptY = center_y;

volatile uint16_t currentDim = 0;

volatile int16_t error_prevX;
volatile int16_t error_prevY;

volatile uint16_t raw_prevX;
volatile uint16_t output_prevX;
volatile uint16_t raw_X;
volatile uint16_t filt_X;

volatile uint16_t raw_Y;
volatile uint16_t filt_Y;
volatile uint16_t raw_prevY;
volatile uint16_t output_prevY;
        
/*
 * Timer Code
 */
void initialize_timer(){

    // Setup Timer 2 - 50Hz (works with servos)
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    TMR2 = 0; 
    T2CONbits.TCKPS = 0b10; // select 1:64 prescaler
    CLEARBIT(IFS0bits.T2IF);
    CLEARBIT(IEC0bits.T2IE);   
    PR2 = 4000; // set timer period: 1ms is 200 tps => 20ms is 4000 (because 12.8MHz / 64 = 200 000 tps is 1s)
    
    // Setup Timer 3 - 100Hz
    CLEARBIT(T3CONbits.TON);
    CLEARBIT(T3CONbits.TCS);
    CLEARBIT(T3CONbits.TGATE);
    TMR3 = 0; 
    T3CONbits.TCKPS = 0b10; // select 1:64 prescaler
    CLEARBIT(IFS0bits.T3IF);
    CLEARBIT(IEC0bits.T3IE);
    PR3 = 2000; // set timer period: 1ms is 200 tps => 20ms is 4000 (because 12.8MHz / 64 = 200 000 tps is 1s)

    // set timers priority
    IPC0bits.T2IP = 0x01;
    IPC2bits.T3IP = 0x01;
    
    
    // enable timers IR
    IEC0bits.T2IE = 1;
    IEC0bits.T3IE = 1;


    // enable timers    
    SETBIT(T2CONbits.TON);
    SETBIT(T3CONbits.TON); 
    
}

/*
 * Servo Code
 */
void initialize_servos(uint8_t servo_number){
    if (servo_number == 0){ // x-servo --> OC8
        // Setup OC8
        CLEARBIT(TRISDbits.TRISD7);
        OC8R = 3650; // set initial duty cycle to 1.75ms (based on TMR2 period))
        OC8RS = 3650; // load OCRS: next PWM duty cycle
        OC8CON = 0x0006; // set OC8: PWM, no fault check, TMR 2
    }
    else if (servo_number == 1){ // y-servo --> OC7
        // Setup OC7
        CLEARBIT(TRISDbits.TRISD6);
        OC7R = 3700; // set initial duty cycle to 20-1.5ms (based on TMR2 period))
        OC7RS = 3700;
        OC7CON = 0x0006;
    }
}

void set_duty_cycle(uint8_t servo_number, double duty_cycle){
    if (duty_cycle >= 2.1){
        duty_cycle = 2.1;
    }
    if (duty_cycle <= 0.9){
        duty_cycle = 0.9;
    }
    duty_cycle = 20 - duty_cycle;
    //lcd_printf("duty x: %f ", duty_control_x)
    double duty_period = duty_cycle * FCY * 1/64000;
    if (servo_number == 0){ // x-axis
        OC8RS = (uint16_t)(duty_period);
    } if (servo_number == 1){ //y-axis
        OC7RS = (uint16_t)(duty_period);
    }
}

/*
 * Touch screen code
 */
void initialize_touchscreen(){
    // set up the I/O pins E1, E2, E3 to be output pins
    CLEARBIT(TRISEbits.TRISE1); Nop();
    CLEARBIT(TRISEbits.TRISE2); Nop();
    CLEARBIT(TRISEbits.TRISE3); Nop();
    
    SETBIT(PORTEbits.RE1); Nop();
    SETBIT(PORTEbits.RE2); Nop();
    CLEARBIT(PORTEbits.RE3); Nop();
    
    // AN15 --> x-coordinate
    // Disable ADC
    CLEARBIT(AD1CON1bits.ADON); Nop();
    //initialize PIN
    SETBIT(TRISBbits.TRISB15); Nop();// Set TRISB RB15 to input
    CLEARBIT(AD1PCFGLbits.PCFG15); // Set AD1 AN15 input pin as analog
    //AN9 --> y-coordinate
    //initialize PIN
    SETBIT(TRISBbits.TRISB9); Nop(); // Set TRISB RB9 to input
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

void change_dimension(){ // 0: x, 1: y

    if (currentDim == 0){
        // set up the I/O pins E1, E2, E3 so that the touchscreen's x-coordinate pin connects to the ADC
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        AD1CHS0bits.CH0SA = 0x00F;
    } else {
    // set up the I/O pins E1, E2, E3 so that the touchscreen's y-coordinate pin connects to the ADC
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        AD1CHS0bits.CH0SA = 0x009;
    }
    currentDim ^= 1;
}

uint16_t read_position(){
    // x-pin
     // Set ADC to Sample AN15 pin
    SETBIT(AD1CON1bits.SAMP); // Start to sample
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // MUST HAVE! Clear conversion done bit
    return ADC1BUF0;
}

/*uint16_t read_position_y(){
    // y-pin
     // Set ADC to Sample AN9 pin
    SETBIT(AD1CON1bits.SAMP); // Start to sample
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // MUST HAVE! Clear conversion done bit
    return ADC1BUF0;
}*/ // these are the same... remove one only change dim is necessary to set output


/*
 * PD Controller
 */
void pd_controlx(uint16_t signal){
    int16_t error = center_x - signal; // x ranges from 70-725 (long side)
    int16_t d_error = (error - error_prevX);
    double duty_error = k_px*(error) + k_dx*(d_error);
    // map to duty cycle
    double x_control = 1.75 + duty_error;
    error_prevX = error;
    set_duty_cycle(0, x_control);
}

void pd_controly(uint16_t signal){
    int16_t error = center_y - signal; // y ranges from 90-630 (short side)
    int16_t d_error = (error - error_prevY);
    double duty_error = k_py*error + k_dy*d_error;   
    // map to duty cycle
    double y_control = 1.5 + duty_error;
    error_prevY = error;
    set_duty_cycle(1, y_control);
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */
uint16_t butter_filter(uint16_t raw, uint16_t raw_prev, uint16_t output_prev){
    return (uint16_t)(butter_b1*raw + butter_b2*raw_prev - butter_a*output_prev);
}


/*
 * main loop
 */
void main_loop()
{    
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: Lab 3 Group 2");
    lcd_locate(0, 2);
    
    // initialize touchscreen
    initialize_touchscreen();
    change_dimension(0);
    __delay_ms(10);
        
    // initialize servos
    initialize_servos(0);
    initialize_servos(1);    
    initialize_timer();
    
    while(TRUE) {
        // print missed deadlines every 5Hz (0.2s)
        if (global_counter_tmr3 == 20){
            lcd_locate(0,3);    
            lcd_printf("Deadlines Missed: %u ", missed_deadlines);
        }
        // alternate filter and control every 20ms (50Hz)
        if (global_counter_tmr3 % 2 == 0){
            //raw_X = read_position();
            filt_X = butter_filter(raw_X, raw_prevX, output_prevX);
            raw_prevX = raw_X;
            output_prevX = filt_X;
            filt_Y = butter_filter(raw_Y, raw_prevY, output_prevY);
            raw_prevY = raw_Y;
            output_prevY = filt_Y;
            pd_controlx(filt_X);
            pd_controly(filt_Y);
            //lcd_locate(0,4); 
            //lcd_printf("duty x: %f ", filt_x)
            workload_status = true;
            // advance setpoint, aim for one cycle in 5s. 
            setptX = (uint16_t)(center_x + radius * cos(omega*global_counter_tmr3));
            setptY = (uint16_t)(center_y + radius * sin(omega*global_counter_tmr3));
        }
    }
}

// timer3 interrupt - 100 Hz (check deadline, advance time, read either X or Y)
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ 
    IFS0bits.T3IF = 0; // Clear the interrupt flag 

    
    
    read_position();    // read current variable
    change_dimension(); // change variable
    

    global_counter_tmr3++; // Increment a global counter
    // check workload
    if (workload_status == false){
        missed_deadlines++;
    }
    if (global_counter_tmr3 == 500){
        global_counter_tmr3 = 0;
    }

}



```
