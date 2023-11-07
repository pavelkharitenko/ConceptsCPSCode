## lab02.c so far:

```c
#include "lab02.h"

#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

#define FCY_EXT 32768

uint16_t global_counter_tmr1;

uint16_t global_counter_tmr2;

//uint16_t seconds_passed = 0;

uint16_t minutes = 0;

void initialize_timer()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    
    // Disable the Timers
    T1CONbits.TON = 0;      // we need them not to start counting already
    T2CONbits.TON = 0;
    T3CONbits.TON = 0;
    
    
    // Set Prescaler
    T1CONbits.TCKPS = 0b11;     // (00 wwould have been 1:1 prescaler, timer follows exactly the oscillator)
    T2CONbits.TCKPS = 0b11;     // 11 is 1:256 ratio according to dsPIC datasheet, timer follows 
    T3CONbits.TCKPS = 0b00;
    
    // Set Clock Source
    T1CONbits.TCS = 1;      // Mode 1: external (slow) clock oscillator (32kHz), increment on the rising edge
    T2CONbits.TCS = 0;      // Mode 0: use internal clock with 12.8 Mhz. 
    T3CONbits.TCS = 0;
    
    // Set Gated Timer Mode -> don't use gating
    T1CONbits.TGATE = 0;
    T2CONbits.TGATE = 0;
    T3CONbits.TGATE = 0;
    
    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;

    // Load Timer Periods
    PR1 = 32767/256;        // timer 1 counts with speed of 32kHz, according to dsPIC datasheet, sum 32767 will appear once a second.
    PR2 = 100;        // timer 2 counts with speed of 12.8Mhz * 1/256 = 50000Hz. 50k is 1 sec, 50 is 1 ms, and 100 is 2 ms. 
    PR3 = 65535;
    
    // Reset Timer Values
    TMR1 = 0;
    TMR2 = 0;
    TMR3 = 0;

    // Set Interrupt Priority
    IPC0bits.T1IP = 1;
    IPC1bits.T2IP = 1;      // in dsPIC datasheet, timer2 priority can be set with the IPC1 register

    // Clear Interrupt Flags
    CLEARBIT(IFS0bits.T1IF);
    CLEARBIT(IFS0bits.T2IF);

    // Enable Interrupts
    IEC0bits.T1IE = 1;
    IEC0bits.T2IE = 1;
    
    // Enable the Timers
    T1CONbits.TON = 1; 
    T2CONbits.TON = 1; 
    T3CONbits.TON = 1; 

}

void timer_loop()
{
    // print assignment information
    lcd_printf("Lab02: Int & Timer");
    lcd_locate(0, 1);
    lcd_printf("Group: Group 2");
    
    
    uint16_t i = 0;
    while(TRUE)
    {
        i++;
        if(i==2000){
            i = 0;
            lcd_locate(0, 2);
            
            lcd_printf("%u:%u:%u", minutes, global_counter_tmr1, global_counter_tmr2 );
            //lcd_printf("seconds = %u",  global_counter_tmr1);
            //global_counter_tmr1 = 0;
            //global_counter_tmr2 = 0;
            TOGGLELED(LED3_PORT);
            
            lcd_locate(0, 3);
            lcd_printf("Timer3 content = %u",TMR3);
            
            
        }
    }
        
       
        
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every ??
    
    IFS0bits.T1IF = 0; // Clear the interrupt flag
    global_counter_tmr1++; // Increment a global counter
    if(global_counter_tmr1 % 60 == 0){
        global_counter_tmr1 = 0;
        minutes++;
    }
    TOGGLELED(LED1_PORT);
    
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
    
    IFS0bits.T2IF = 0; // Clear the interrupt flag
    global_counter_tmr2++; // Increment a global counter
    if (global_counter_tmr2 == 1000){
        global_counter_tmr2 = 0;
    }
    
    TOGGLELED(LED2_PORT);
    
    
}

//extern  int global_counter_tmr1;

//extern  int global_counter_tmr2;

```
