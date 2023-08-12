// File:   Source_main_PWM_Controller.c
// Author: Mike Suffolk

// MPLAB X IDE will select the correct PIC16F887.h file if "xc.h" is called.
#include <xc.h>

// Configure the PIC device (Important Parameters).
#pragma config FOSC = 0x04  // Internal Oscillator
#pragma config WDTE = 0x00  // Wathdog disabled
#pragma config PWRTE = 0x00 // Power up timer enabled
#pragma config MCLRE = 0x00 // MCLR internally tied to Vdd
#pragma config CP = 0x01    // No program code protection
#pragma config CPD = 0x01   // No data memory protection
#pragma config BOREN = 0x03 // Brown out reset enabled
#pragma config IESO = 0x00  // Internal / External switchover disabled
#pragma config FCMEN = 0x01 // Fail safe clock monitor enabled
#pragma config LVP = 0x00   // Low Voltage programming not enabled
#pragma config DEBUG = 0x01 // In-circuit debugger disabled
#pragma config BOR4V = 0x01 // Brown out reset at 4.0V
#pragma config WRT = 0x03   // Write protection off
#define _XTAL_FREQ 4000000  // Oscillator freq. Hz for __delay_ms() function

// Program variables & definitions.
unsigned int PWM_init_complete;
unsigned int DTY_CYC_SP;
unsigned int AI_Control1;
unsigned int AI_Control2;
unsigned int Delay_1;
unsigned int Delay_2;
unsigned int Delay_0Deg;
unsigned int Delay_15Deg;
unsigned int Delay_30Deg;
unsigned int Delay_45Deg;
unsigned int Delay_60Deg;
unsigned int Delay_75Deg;
unsigned int AI_Conversion_DN;
unsigned int Duty_Cycle_up_DN;
unsigned int Duty_Cycle_dw_DN;
unsigned int Motor_Stalled;

void PWM_init(void);
void main(void)

{ 
// First line of code delays the processor for 100ms
// Allows power supplies to switch on and stabilise.
__delay_ms(100);

// Call "PWM_init()" to initialise and configure PIC.
PWM_init();

// Perform an endless program loop.
while(PWM_init_complete = 1)
{

// When ADC conversion complete, move result into AI_Control1 register.
// ADRESL & ADRESH contain the analogue result as a 10-bit value (0d to 1023d)
// i.e 0V measured = 0d, 5V measured = 1023d
// POT output varies between 0V (ADRESL = 0d) and 1.087V (ADRESL = 222d).
if (ADCON0bits.nDONE == 0 && ADCON0bits.GO == 0 && AI_Conversion_DN == 0)
    {
        AI_Control1 = (ADRESL / 2);
        AI_Conversion_DN = 1;
    }   

// Increase the software setpoint by increments of one.
if ((AI_Control2 < AI_Control1) && Duty_Cycle_up_DN == 0)
    {
        AI_Control2 = AI_Control2 + 1;
        Duty_Cycle_up_DN = 1;
    }

// Decrease the software setpoint by increments of one.
if ((AI_Control2 > AI_Control1) && Duty_Cycle_dw_DN == 0)
    {
        AI_Control2 = AI_Control2 - 1;
        Duty_Cycle_dw_DN = 1;
    }   

// Restart the ADC conversion cycle & reset DONE status words.
if (Delay_1 == 20)
    {
        ADCON0bits.GO = 1;
        ADCON0bits.nDONE = 1;
        AI_Conversion_DN = 0;
        Duty_Cycle_up_DN = 0;
        Duty_Cycle_dw_DN = 0;

// When PWM setpoint is less than 4 counts, set Duty cycle to zero.
        if (AI_Control2 <= 4)
            DTY_CYC_SP = 0;
        
// When setpoint is greater than 8 map setpoint to the PWM control word.
        if ((AI_Control2 > 8) && (AI_Control2 <=100))
            DTY_CYC_SP = AI_Control2;
        
// When setpoint is greater than 105 counts, set Duty cycle to 116.
// PWM period = (PR2+1)*4*Tosc*TMR2PreScale = 124us...
// 116us Duty cycle averages a voltage of 22.5V at motor coils.
        if (AI_Control2 > 105)
            DTY_CYC_SP = 116;

// Write duty cycle setpoint to the PWM control registers.        
        CCPR1L  =   DTY_CYC_SP >> 2;          // Put MSB 8 Bits in CCPR1L
        CCP1CON &=  0xCF;                     // Make bit 4 and 5 zero
        CCP1CON |=  (0x30&(DTY_CYC_SP << 4)); // Assign last 2 LSBs to CCP1CON

// Reset the delay register to zero ready to increment again.
        Delay_1 = 0;
    }

// Increment the Delay value by one. If the processor speed is 4MHz
// Instruction cycle is Fosc/4 = 4Mhz/4 = 1MHz. One increment is
// T = 1/F = 1/1000000 [seconds] = 1uS.
// The data sheet suggests at 4MHz & Fosc/8 the delay period for the ADC
// shall be 2uS. So 20uS should be a large enough delay for the ADC conversion.
        Delay_1 = Delay_1 + 1;

// If RD7 (Motor Enable Switch) is OFF, then turn all MOSFET drivers OFF.
if (PORTDbits.RD7 == 0)
{   
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
    DTY_CYC_SP = 0;
    AI_Control1 = 0;
    AI_Control2 = 0;
    Delay_1 = 0;
    Delay_2 = 0;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;
    Motor_Stalled = 0;
}

// If motor is in fault state, turn power to coils OFF.
// If PWM Duty cycle is zero (motor not required to rotate) turn coils OFF.
// If hall sensors are not in a valid position, turn coils OFF.
if ((Motor_Stalled == 1) || (DTY_CYC_SP == 0) ||
   ((PORTEbits.RE0 == 0) && (PORTEbits.RE1 == 0) && (PORTEbits.RE2 == 0)) ||
   ((PORTEbits.RE0 == 1) && (PORTEbits.RE1 == 1) && (PORTEbits.RE2 == 1)))
{
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
}

//Motor 0 Degree Phase
// If Hall effect sensors reflect the required motor position (RE0 to RE2)
// and the motor enable switch is ON (RD7), then start increasing delay timer.
// The delay timer increases with instruction clock (1us intervals).
// While the delay timer is less than or equal to 1, turn all PWM Switches OFF,
// When the delay timer is greater than 1 (1us has elapsed), turn required
// PWM Switches ON, energising the appropriate motor coils.
// This 1us delay allows the MOSFETs to completely switch OFF before the next
// set of MOSFETs switch ON.
// Start a timer (TMR0) to detect if motor has stalled.
if  ((PORTEbits.RE0 == 1) && (PORTEbits.RE1 == 0) && (PORTEbits.RE2 == 0)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_0Deg = Delay_0Deg + 1;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;

    if (Delay_0Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
    
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    
    }
    if (Delay_0Deg > 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 1; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 1; // Coil A -
    
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

//Motor 15 Degree Phase
if  ((PORTEbits.RE0 == 1) && (PORTEbits.RE1 == 0) && (PORTEbits.RE2 == 1)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_15Deg = Delay_15Deg + 1;
    Delay_0Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;
    
    if (Delay_15Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    
    }
    if (Delay_15Deg > 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 1; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 1; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

//Motor 30 Degree Phase
if  ((PORTEbits.RE0 == 0) && (PORTEbits.RE1 == 0) && (PORTEbits.RE2 == 1)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_30Deg = Delay_30Deg + 1;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;
    
    if (Delay_30Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    
    }
    if (Delay_30Deg > 1)
    {
    PORTCbits.RC0 = 1; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 1; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

//Motor 45 Degree Phase
if  ((PORTEbits.RE0 == 0) && (PORTEbits.RE1 == 1) && (PORTEbits.RE2 == 1)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_45Deg = Delay_45Deg + 1;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;
    
    if (Delay_45Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    
    }
    if (Delay_45Deg > 1)
    {
    PORTCbits.RC0 = 1; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 1; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

//Motor 60 Degree Phase
if  ((PORTEbits.RE0 == 0) && (PORTEbits.RE1 == 1) && (PORTEbits.RE2 == 0)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_60Deg = Delay_60Deg + 1;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_75Deg = 0;
    
    if (Delay_60Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    
    }
    if (Delay_60Deg > 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 1; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 1; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -
        
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

//Motor 75 Degree Phase
if  ((PORTEbits.RE0 == 1) && (PORTEbits.RE1 == 1) && (PORTEbits.RE2 == 0)
        && (PORTDbits.RD7 == 1) && (Motor_Stalled == 0) && (DTY_CYC_SP > 0))
{
    Delay_75Deg = Delay_75Deg + 1;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    
    if (Delay_75Deg <= 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 0; // Coil C +
    PORTDbits.RD3 = 0; // Coil A -

    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 0;                   //Load a value of 0 into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
    Delay_2 = 0;
    }

    if (Delay_75Deg > 1)
    {
    PORTCbits.RC0 = 0; // Coil A +
    PORTCbits.RC3 = 0; // Coil B +
    PORTCbits.RC4 = 0; // Coil B -
    PORTCbits.RC5 = 0; // Coil C -
    PORTDbits.RD2 = 1; // Coil C +
    PORTDbits.RD3 = 1; // Coil A -
        
    if (INTCONbits.T0IF == 1) //TMR0 Interrupt occurs. (when TMR0 rolls over)
    {
        INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
        TMR0 = 0;                   //Load a value of 0 into the timer
        INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt
        Delay_2 = Delay_2 + 1;
    }
    }
}

// Ensure delay timers do not exceed a value of 10.
if (Delay_0Deg > 10)
    Delay_0Deg = 10;

if (Delay_15Deg > 10)
    Delay_15Deg = 10;

if (Delay_30Deg > 10)
    Delay_30Deg = 10;

if (Delay_45Deg > 10)
    Delay_45Deg = 10;

if (Delay_60Deg > 10)
    Delay_60Deg = 10;

if (Delay_75Deg > 10)
    Delay_75Deg = 10;

// Motor stalled. Delay_2 increments when TMR0 expires, every 0.0655 seconds.
if (Delay_2 >= 5)
    Motor_Stalled = 1;
    }
}

void PWM_init(void)
{
//  Set variables to zero initially
    PWM_init_complete = 0;
    DTY_CYC_SP = 0;
    AI_Control1 = 0;
    AI_Control2 = 0;
    Delay_1 = 0;
    Delay_2 = 0;
    AI_Conversion_DN = 0;
    Duty_Cycle_up_DN = 0;
    Duty_Cycle_dw_DN = 0;
    Delay_0Deg = 0;
    Delay_15Deg = 0;
    Delay_30Deg = 0;
    Delay_45Deg = 0;
    Delay_60Deg = 0;
    Delay_75Deg = 0;
    Motor_Stalled = 0;
    
//  Internal Oscillator Frequency Select Bits (4MHz)
    OSCCONbits.IRCF = 0b110;
    OSCCONbits.SCS = 1;
    OSCCONbits.HTS = 1;
    OSCCONbits.LTS = 1;
    OSCCONbits.OSTS = 0;
    OSCTUNEbits.TUN = 0b00000;
    
//  Disable AI channels which share the pins used as digital inputs or outputs
    ANSELHbits.ANS13 = 0;
    ANSELHbits.ANS12 = 0;
    ANSELHbits.ANS11 = 0;
    ANSELHbits.ANS10 = 0;
    ANSELHbits.ANS9 = 0;
    ANSELHbits.ANS8 = 1;    // Enable AN8, used as analogue input (POT)
    ANSELbits.ANS7 = 0;     // Disable AN7, used as digital input RE2
    ANSELbits.ANS6 = 0;     // Disable AN6, used as digital input RE1
    ANSELbits.ANS5 = 0;     // Disable AN5, used as digital input RE0
    ANSELbits.ANS4 = 0;
    ANSELbits.ANS3 = 0;
    ANSELbits.ANS2 = 0;
    ANSELbits.ANS1 = 0;
    ANSELbits.ANS0 = 0;

//  Setup ADC configuration
    ADCON0bits.ADCS0 = 1;
    ADCON0bits.ADCS1 = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.CHS = 0b1000;
    ADCON1bits.ADFM = 1;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO = 1;
    ADCON0bits.nDONE = 1;
     
//  Setup channels as digital outputs or inputs
    TRISAbits.TRISA7=0;
    TRISAbits.TRISA6=0;
    TRISAbits.TRISA5=0;
    TRISAbits.TRISA4=0;
    TRISAbits.TRISA3=0;
    TRISAbits.TRISA2=0;
    TRISAbits.TRISA1=0;
    TRISAbits.TRISA0=0;

    TRISBbits.TRISB7=0;
    TRISBbits.TRISB6=0;
    TRISBbits.TRISB5=0;
    TRISBbits.TRISB4=0;
    TRISBbits.TRISB3=0;
    TRISBbits.TRISB2=1; // RB2 is an input
    TRISBbits.TRISB1=0;
    TRISBbits.TRISB0=0;
 
    TRISCbits.TRISC7=0;    
    TRISCbits.TRISC6=0;    
    TRISCbits.TRISC5=0; // RC5 is an output
    TRISCbits.TRISC4=0; // RC4 is an output
    TRISCbits.TRISC3=0; // RC3 is an output
    TRISCbits.TRISC2=0; // PWM channel 1 and 2 configured as output 
    TRISCbits.TRISC1=0; // PWM channel 1 and 2 configured as output  
    TRISCbits.TRISC0=0; // RC0 is an output
    
    TRISDbits.TRISD7=1; // RD7 is an input (Enable Switch)    
    TRISDbits.TRISD6=0;
    TRISDbits.TRISD5=0;
    TRISDbits.TRISD4=0;
    TRISDbits.TRISD3=0; // RD3 is an output
    TRISDbits.TRISD2=0; // RD2 is an output
    TRISDbits.TRISD1=0;
    TRISDbits.TRISD0=0;    

    TRISEbits.TRISE3=0;
    TRISEbits.TRISE2=1; // RE2 is an input
    TRISEbits.TRISE1=1; // RE1 is an input
    TRISEbits.TRISE0=1; // RE0 is an input
    
//  Setup digital outputs as ON/OFF initially    
    PORTAbits.RA0 = 0;
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 0;
    PORTAbits.RA3 = 0;
    PORTAbits.RA4 = 0;
    PORTAbits.RA5 = 0;
    PORTAbits.RA6 = 0;
    PORTAbits.RA7 = 0;

    PORTBbits.RB0 = 0;
    PORTBbits.RB1 = 0;
    PORTBbits.RB2 = 0;
    PORTBbits.RB3 = 0;
    PORTBbits.RB4 = 0;
    PORTBbits.RB5 = 0;
    PORTBbits.RB6 = 0;
    PORTBbits.RB7 = 0;
    
    PORTCbits.RC0 = 0;
    PORTCbits.RC1 = 0;
    PORTCbits.RC2 = 0;
    PORTCbits.RC3 = 0;
    PORTCbits.RC4 = 0;
    PORTCbits.RC5 = 0;
    PORTCbits.RC6 = 0;
    PORTCbits.RC7 = 0;

    PORTDbits.RD0 = 0;
    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 0;
    PORTDbits.RD3 = 0;
    PORTDbits.RD4 = 0;
    PORTDbits.RD5 = 0;
    PORTDbits.RD6 = 0;
    PORTDbits.RD7 = 0;

    PORTEbits.RE0 = 0;
    PORTEbits.RE1 = 0;
    PORTEbits.RE2 = 0;
    PORTEbits.RE3 = 0;

//  Setup PWM mode
    CCPR1L = 0x00;          // Zero PWM duty cycle initially
    CCP1CONbits.DC1B0 = 0;  // Zero PWM duty cycle initially
    CCP1CONbits.DC1B1 = 0;  // Zero PWM duty cycle initially
    CCP1CONbits.CCP1M = 0b1100; // CCP1 and CCP2 are configured for PWM
    CCP2CONbits.CCP2M = 0b1100; // CCP1 and CCP2 are configured for PWM
    CCP1CONbits.P1M = 0b00;     // Single PWM output at P1A
    PR2 = 0x1e; // PWM Period = (PR2 + 1)*4*(1/Fosc)*(TMR2 Pre-scale)
    T2CONbits.T2CKPS = 0b01;    // Timer2 Pre-scale is 4
    T2CONbits.TOUTPS = 0b0000;  // Timer2 Post-scaler is 1:1
    TMR2 = 0x00;                // Zero Timer2
    T2CONbits.TMR2ON = 0b1;     // Timer2 is ON

// Timer 0 Setup //
    OPTION_REGbits.PSA = 0;     // Prescaler assigned to Timer 0
    OPTION_REGbits.PS = 0b111;  // Set the prescaler to 1:256
    OPTION_REGbits.T0CS = 0;    // Use the instruction clock (Fcy/4) for timer

// Set tag TRUE to inform program that initialisation of the PIC is complete.
    PWM_init_complete = 1;

// Delay processor for 100ms after initialising parameters
// this allows for all parameters to implement correctly.
    __delay_ms(100);
}