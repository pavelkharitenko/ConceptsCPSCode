lab03.c:
```c
#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    
    // set AN0-15 to digital/analog by configuring the AD1PCFGL register, set AN16-31 to digital/analog by configuring the AD1PCFGH register
    // AN10
    AD1PCFGLbits.PCFG10 = 1; // if this bit is set, it is conf. as a digital input (dsPIC datasheet 11.3)
    
    // AN11
    AD1PCFGLbits.PCFG11 = 1; // if this bit is set, it is conf. as a digital input (dsPIC datasheet 11.3)
    
    // AN13
    AD1PCFGLbits.PCFG13 = 1; // if this bit is set, it is conf. as a digital input (dsPIC datasheet 11.3)
    
    
    // set RD8, RB10, RB11, RB13 as output pins
    
    TRISBbits.TRISB10 = 0; // set the now digital pin in general as output
    TRISBbits.TRISB11 = 0; // similar
    TRISBbits.TRISB13 = 0; // similar
    
    TRISDbits.TRISD8 = 1;  // similar
    Nop();
    TRISDbits.TRISD8 = 0;  // similar
    
    // set default state: CS=??, SCK=??, SDI=??, LDAC=??
    
    DAC_CS_PORT = 1; // needs to be set to low during write operations
    DAC_SCK_PORT = 0;  // needs to be set to low, then after writing one bit, again to high, so probably needs to be set to low initially 
    DAC_LDAC_PORT = 1; // set to low->high later after all bits sent, to output the voltage, so probably needs to be set to high initially 
    DAC_SDI_PORT = 0;
     
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // configure timer
    
    // Disable the Timers
    T1CONbits.TON = 0;      // we need them not to start counting already
   
    // Set Prescaler
    T1CONbits.TCKPS = 0b00;     // (00 wwould have been 1:1 prescaler, timer follows exactly the oscillator)
    
    // Set Clock Source
    T1CONbits.TCS = 0;      // Mode 1: external (slow) clock oscillator (32kHz), increment on the rising edge
    
    // Set Gated Timer Mode -> don't use gating
    T1CONbits.TGATE = 0;
    
    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;

    // Load Timer Period (ISR triggered every ms)
    PR1 = (int)(FCY/1000UL);        // timer 1 counts with speed of 12.8MHz, so 128000 will appear after 1ms
    
    // Reset Timer Values
    TMR1 = 0;

    // Set Interrupt Priority
    IPC0bits.T1IP = 1;

    // Clear Interrupt Flags
    CLEARBIT(IFS0bits.T1IF);

    // Enable Interrupts
    IEC0bits.T1IE = 1;
    
    // Enable the Timers
    T1CONbits.TON = 1; 
    
}

// interrupt service routine?

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: Group 2");
    
    int mode = 1;
    while(TRUE)
    {
        // main loop code
        
      
        
        
        
    }
}


void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every 1ms
    
    static int global_counter = 1000;
    static int current_max_value = 1000;
    IFS0bits.T1IF = 0; // Clear the interrupt flag
    
    // set 1v to DAC as output if timer reached 1000ms (with 3.5V)
    if(global_counter ==  current_max_value && current_max_value == 1000){
        //TOGGLELED(LED2_PORT);
        //current_max_value = 500; // set max time for 500ms
        
        // send bits to DAC to configure 1V
        DAC_CS_PORT = 0; // needs to be set to low during write operations
        
        uint16_t message = 0b0011110000000000;
        double vltg = 2.5;
        //message |= 0x0FFF & (uint16_t)(vltg * 2000);
        
        int i = 15;
        for (i = 15; i>=0; i--){
            // set sck low before sending bit
            DAC_SCK_PORT = 0;
            Nop();
            
            // send respective bit (select first bit of message, and shift message to front every iteration by i)
            DAC_SDI_PORT =  (message >> i) & 1;
            LED2_PORT = (message >> i) & 1;
            // set sck high after sending bit
            DAC_SCK_PORT = 1;
            Nop();
            
        }
        
        // after finishing sending to SDI, set CS to High again
        DAC_CS_PORT = 1;
        DAC_SDI_PORT = 0; // according to exercise sheet
        Nop();
        
        DAC_LDAC_PORT = 0;
        Nop();
        DAC_LDAC_PORT = 1;
        Nop();
    }
    
   global_counter++; // Increment a global counter

    
}


```
