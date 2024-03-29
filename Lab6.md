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



// actual screen params // usual values
#define max_x 730 // 725
#define min_x 70 // 70
#define max_y 625 // 630
#define min_y 115 // 90

// reference circle
#define pi 3.1415926
#define radius 125
#define center_x ((max_x-min_x)/2.0)
#define center_y (max_y-min_y)/2.0
#define circle_period_in_10ms 900 // *10ms = 1 round around in ms

double omega =  2.0*pi/circle_period_in_10ms; // rad/s (v = 2*pi*r/T), w = v/r)


//#define Kp_x .0000001
//#define Kp_x 0.002
#define Kp_x 0.00051
#define Kd_x 0.0055
#define Kp_y 0.0006
#define Kd_y 0.005

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
double a = -0.67959929;
double b1 = 0.16020035;
double b2 = 0.16020035;

// current X Y variables
volatile double currXY[] = {0, 0};

// old X Y variables
volatile double minus1XY[] = {0, 0};

// current smoothed X Y variables
volatile double smoothedXY[] = {center_x, center_y};

// old error X Y
volatile double minus1errorXY[] = {0.0,0.0};

// global time in ms
volatile double global_ms = 0.0;

uint16_t global_counter_tmr3;

volatile uint16_t dim = 0;

volatile uint16_t nextDeadline = 2;         
volatile uint16_t n_deadlines_missed = 0;
        
/*
 * Timer Code
 */
void initialize_timer(){
    // Setup Timer 2
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
    IPC2bits.T3IP = 0x01;
    
    // enable timers IR
    IEC0bits.T3IE = 1;
    //IEC0bits.T2IE = 1;

    // enable timers    
    SETBIT(T3CONbits.TON);
    SETBIT(T2CONbits.TON); 
    
}

/*
 * Servo Code
 */
void initialize_servos(){
    // x-servo --> OC8
        // Setup OC8
        CLEARBIT(TRISDbits.TRISD7);
        OC8R = 3700; // set initial duty cycle to 1.75ms (based on TMR2 period))
        OC8RS = 3700; // load OCRS: next PWM duty cycle
        OC8CON = 0x0006; // set OC8: PWM, no fault check, TMR 2     
    
    // y-servo --> OC7
        // Setup OC7
        CLEARBIT(TRISDbits.TRISD6);
        OC7R = 3700; // set initial duty cycle to 20-1.5ms (based on TMR2 period))
        OC7RS = 3700;
        OC7CON = 0x0006;
    
}

void set_duty_cycle(uint8_t servo_number, double duty_cycle){
    
    // clip 
    if (duty_cycle >= 2.1){
        duty_cycle = 2.1;
    }
    if (duty_cycle <= 0.9){
        duty_cycle = 0.9;
    }
    
    
    duty_cycle = 20.0 - duty_cycle;
    
    //double duty_period = duty_cycle * FCY * 1/64000;
    
    double duty_period = duty_cycle * FCY * 1/64000;
    
    
    
    
    if (servo_number == 0){ // x-axis
        OC8RS = (uint16_t)(duty_period);
    } 
    if (servo_number == 1){ //y-axis
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

void change_dimension_ts(uint16_t currentDim){ // 0: x, 1: y

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
        //AD1CHS0bits.CH0SA = 0x009;
    }
    
    __delay_ms(10);
    
    
}

double read_position(uint16_t dim){
    if (dim == 0){
        AD1CHS0bits.CH0SA = 0x00F;
    } else{
        AD1CHS0bits.CH0SA = 0x009;
    }
    
    // Set ADC to Sample AN15 pin
    SETBIT(AD1CON1bits.SAMP); // Start to sample
    while(!AD1CON1bits.DONE); // Wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); // MUST HAVE! Clear conversion done bit
    

    uint16_t x = ADC1BUF0;
    double x2 = (double)x;
    return x2;
    
}

double clip_u_x(double u){
    if(u > 2.0){
        u = 2.0;
    }
    if(u < 1.0){
        u = 1.0;
    }
    return u;
}

double clip_u_y(double u){
    if(u > 2.0){
        u = 2.0;
    }
    if(u < 0.9){
        u = 0.9;
    }
    return u;
}

/*
 * PD Controller
 */
void pd_control_x(){
    double x_ref = center_x + radius * cos(omega*global_counter_tmr3) + 100;
    lcd_locate(0,4);
    lcd_printf("Ref X: %u ", (uint16_t) x_ref);
    
    
    //double x_ref = center_x;
    
    // compute X error
    double error_X = x_ref - currXY[0]; // x ranges from 70-725 (long side)
    
    
    
    double d_error_X = (error_X - minus1errorXY[0]);
    
    double pd_error_x = Kp_x * error_X + Kd_x * d_error_X;
    
    // map to duty cycle
    double u_x = (pd_error_x + 1.74);
    minus1errorXY[0] = error_X;
    lcd_locate(0,7);
    u_x = clip_u_x(u_x);
    lcd_printf("error x : %f    ", u_x);  
    set_duty_cycle(0, u_x);

}

void pd_control_y(){
    double y_ref  = center_y + radius * sin(omega*global_counter_tmr3);
    
    // compute Y error
    double error_Y = y_ref - currXY[1]; // y ranges from 90-630 (short side)
    double d_error_Y = (error_Y - minus1errorXY[1]);
    double pd_error_y = Kp_y * error_Y + Kd_y * d_error_Y;   
    
    // map to duty cycle
    double u_y = (1.57 + pd_error_y);
    minus1errorXY[1] = error_Y;
    lcd_locate(0,6);
    u_y = clip_u_y(u_y);
    lcd_printf("error y: %f    ", u_y);
    set_duty_cycle(1, u_y);
}

/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */
void apply_filter(){
    // equation is: curr_smoothed = b1*current + b2*minus1 + a*smoothed_prev
    
    smoothedXY[0] = b1*currXY[0] + b2*minus1XY[0] + a*smoothedXY[0];
    smoothedXY[1] = b1*currXY[1] + b2*minus1XY[1] + a*smoothedXY[1];
    
    minus1XY[0] = currXY[0];
    minus1XY[1] = currXY[1];
}


/*
 * main loop
 */
void main_loop()
{    
    // print assignment information
    lcd_clear();
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: Lab 3 Group 2");
    lcd_locate(0, 2);
    
    // initialize touchscreen
    initialize_touchscreen();
    
    change_dimension_ts(0);
    __delay_ms(20);
    currXY[0] = read_position(0);    
    
    
        
    // initialize servos
    initialize_servos();   
    // timer last
    initialize_timer();
    nextDeadline = (global_counter_tmr3 + 10)%  circle_period_in_10ms;
    
    while(TRUE) {
        
        //lcd_locate(0,4);
        //lcd_printf("Ref X: %u ", (uint16_t) (center_x + radius * cos(omega*global_counter_tmr3)));
        //lcd_printf("Ref X: %u ", (uint16_t) center_x);

        //lcd_locate(0,5);
        //lcd_printf("Ref Y: %u ", (uint16_t)(center_y + radius * sin(omega*global_counter_tmr3)));
        
        //lcd_locate(0,6);
        //lcd_printf("Gl. Ctr: %u    ", global_counter_tmr3);  
        
        //lcd_locate(0,3);
        //lcd_printf("smoothed x: %f    ", smoothedXY[0]);

        //lcd_locate(0,3);
        //lcd_printf("Meas. x: %f    ", currXY[0]);
        lcd_locate(0,3);
        lcd_printf("Deadl. missed: %u    ", n_deadlines_missed);

        dim = global_counter_tmr3 % 2;
        change_dimension_ts(dim); // change to next touchscreen reading axis
        if(dim == 0)
            currXY[0] = read_position(dim);    // read current variable
        if(dim == 1)
            currXY[1] = read_position(dim);
        
        if (global_counter_tmr3 > nextDeadline){
                n_deadlines_missed++;
        }
        
        // run control every time both positions were read
        if(dim == 1){
            
            nextDeadline = (global_counter_tmr3 + 10)% circle_period_in_10ms;
            
            // apply filter
            apply_filter();

            // track error
            pd_control_x();
            pd_control_y();
            
            
        }
        
        
        
    }
}

// timer3 interrupt - 100 Hz (check deadline, advance time, read either X or Y)
void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ 
    // runs every 10ms
    IFS0bits.T3IF = 0; // Clear the interrupt flag 

    
    
    // do time updates updates
    global_counter_tmr3++; // Increment a global counter
    
    // reset after T round around 
    if(global_counter_tmr3 == circle_period_in_10ms){
        global_counter_tmr3 = 0;
    }
    

}

```
