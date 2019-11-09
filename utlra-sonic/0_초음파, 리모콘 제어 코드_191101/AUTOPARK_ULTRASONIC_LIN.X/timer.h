#include "p33EV256GM106.h"

void delay_10ms(unsigned char num);
void Delayus(int);

volatile unsigned int f_tick, s_tick,Loop_timer,can_timer,linget_timer,accget_timer;

void timer_init()
{
    Loop_timer=0;
    s_tick = 0;
    f_tick = 0;                 // the timer ticks
    // Timer 1 to generate an interrupt every 250ms
    T1CONbits.TON = 0;          // Disable Timer1
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    PR1 = 39062;                // Load the period value (250ms/(256*25ns))
    IPC0bits.T1IP = 0x03;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Enable Timer1 interrupt

    //
    // Timer 2 to generate an interrupt every 10ms
    //
    T2CONbits.TON = 0;          // Disable Timer2
    T2CONbits.TCS = 0;          // Select internal instruction cycle clock
    T2CONbits.TGATE = 0;        // Disable Gated Timer mode
    T2CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    TMR2 = 0x00;                // Clear timer register
    PR2 = 1562/2;                 // Load the period value (5ms/(256*25ns))
    //PR2 = 1562;                 // Load the period value (10ms/(256*25ns))
    IPC1bits.T2IP = 0x02;       // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0;          // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1;          // Enable Timer2 interrupt

    T2CONbits.TON = 1;          // Start Timer2
    T1CONbits.TON = 1;          // Start Timer1
}
void delay_10ms(unsigned char num)
{
    f_tick = 0;                         //f_tick increments every 10ms
    while (f_tick < num);               // wait here until 'num' ticks occur
    f_tick = 0;
}

void Delayus(int delay)
{
    int i;
    for (i = 0; i < delay; i++)
    {
        __asm__ volatile ("repeat #39");
        __asm__ volatile ("nop");
    }
}


/* code for Timer1 ISR, called every 250ms*/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    s_tick++; // increment the 'slow tick'
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
}

/* code for Timer2 ISR, called every 10ms*/
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    f_tick++; // we increment the variable f_tick
    IFS0bits.T2IF = 0; //Clear Timer2 interrupt flag
    Loop_timer++;
    can_timer++;
    linget_timer++;
    accget_timer++;

}
