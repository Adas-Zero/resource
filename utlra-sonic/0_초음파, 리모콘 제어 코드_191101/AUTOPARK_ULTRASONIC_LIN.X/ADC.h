

#include "p33EV256GM106.h"
#define IN_H_ADC 6
#define IN_L_ADC 7 
#define OUT_H_ADC 12
#define OUT_L_ADC 13
void ADCInit(void) // will set 12bit, 4.96us/sample or 202KS/sec
{
    AD1CON1 = 0;            // POR: 10-bit @4ch mode, ADC disabled, manual sample
    AD1CON2 = 0;            // POR: AVdd/Avss for Vref, do not scan, IRQ every sample
    AD1CON3 = 0;            // POR: Use system clock, TAD = 1Tcyc, SAMC = 0TAD
    AD1CON4 = 0;            // POR: no DMA
    AD1CHS123 = 0;          // not used in 12bit mode, as only 1 S/H available
       
    AD1CON1bits.FORM = 0;   // integer data format (unsigned)
    AD1CON1bits.ASAM = 1;   // continuous automatic sampling enabled
    AD1CON3bits.ADCS = 8;   // 9 Tcy = 1TAD (so TAD = 9*25ns = 225ns = 4.44MHz)
    AD1CON3bits.SAMC = 8;   // set auto sample time as 8TAD = 1.8us
    AD1CON1bits.AD12B = 1;  // 12-bit conversion, 14TAD convert time
    AD1CON1bits.ADON = 1;   // enable converter
    // Turn on port RG8, which supplies +5V to pot
//    Delayus(150); // 150us is sufficient. Also allows pot voltage to stabilize, charges up anti-aliasing filter
    AD1CON1bits.SAMP = 1; // begin continuous sampling/conversion
    _TRISC0 = 1; 
    _TRISC1 = 1;
    _TRISE12 = 1;
    _TRISE13 = 1;
}

int ADCConvert(int channel)
{
    int AverageValue = 0;
    int i;
    for(i = 0; i < 4; i++)
    {
        AD1CHS0bits.CH0SA = channel;
        Delayus(100);
        _AD1IF = 0;
        AD1CON1bits.SAMP = 0;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        while (!_AD1IF);
        AverageValue = AverageValue + ADC1BUF0;
    }
    AverageValue = AverageValue >> 2;
    return AverageValue;
}
