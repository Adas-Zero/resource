// TODO Insert appropriate #include <>

#include "p33EV256GM106.h"


// TODO Insert C++ class definitions if appropriate
void InitSENT1_TX(void);
void InitSENT1_RX(void);
void init_hw(void);
void ADCInit(void);
int ADCConvert(int channel);
void InitMonitor(void);
void InitCAN(void);

void putU2(int);
void putsU2(char*);

void BuzzerInit();
void Buzzer1(void);
void Buzzer2(void);
void Buzzer3(void);
void Buzzer4(void);
void Buzzer5(void);
void LEDInit(void);
void SWinit(void);
int SWcheck(void);

#define         LED1        _LATC4
#define         TRISLED1    _TRISC4
#define         LED2        _LATC5
#define         TRISLED2    _TRISC5
#define         LED3        _LATC6
#define         TRISLED3    _TRISC6
#define         TRIS_POT    _TRISG6
#define         TRIS_TEMP   _TRISG7
#define         ANSEL_POT   _ANSG6
#define         ANSEL_TEMP  _ANSG7
#define         SW1         _RC7
#define         SW2         _RC8
#define         SW3         _RC9



#define         FCAN        40000000                // Fcyc = 1/2Fpll
#define         MONITOR_BAUD    115200                    // LIN2.1 Protocol has 19200 bfs

#define         UART_MONITOR  ((FCAN/MONITOR_BAUD)/16) - 1


#define         BAUD9600    ((FCAN/9600)/16) - 1
#define         BAUD19200   ((FCAN/19200)/16) - 1
#define         BAUD38400   ((FCAN/38400)/16) - 1   // this is what the demo UART serial baud rate is
#define         BAUD576000  ((FCAN/57600)/16) - 1   // selection of transmitter baud rate divisors
#define         ANSEL_RTS   _ANSE12
#define         ANSEL_CTS   _ANSE13
#define         TRIS_RTS    _TRISE12
#define         TRIS_MON    _TRISB4
#define         TRANSMIT 1
#define         RECEIVE 0
#define Relay_ON() _LATA12=1
#define Relay_OFF() _LATA12=0

#define CAN_NORMAL
//#define Santafe
//#define Morning
//#define Soul
//#define CAMRY

#ifdef CAMRY
#define Can_Velocity_ID 0X0B4
#define Can_Velocity_BYTE data[5]
#define Can_Velocity_BYTE2 data[6]
#define Can_Gear_fw 0x45 // D
#define Can_Gear_bw 0x47 // REAR
#endif


#ifdef CAN_NORMAL
#define Can_Velocity_ID 0X7E8
#define Can_Velocity_BYTE data[3]
#define Can_Gear_fw 0x45 // D
#define Can_Gear_bw 0x47 // REAR
#endif

#ifdef Soul  //-- ?????
#define Can_Velocity_ID 0x316
#define Can_Velocity_BYTE data[6]

#define Can_Gear_ID 0x111
#define Can_Gear_BYTE data[1]

#define Can_Gear_fw 0x45 // D
#define Can_Gear_bw 0x47 // REAR
#define Can_Gear_p 0x40  // PARKING
#define Can_Gear_n 0x46  // Nutral

#define Can_Brake_ID 0x329
#define Can_Brake_BYTE data[1]
#endif

#ifdef Morning  //-------------- ??
#define Can_Velocity_ID 0x316
#define Can_Velocity_BYTE data[6]

#define Can_Gear_ID 0x111
#define Can_Gear_BYTE data[1]

#define Can_Gear_fw 0x45
#define Can_Gear_bw 0x47
#define Can_Gear_p 0x40
#define Can_Gear_n 0x46

#define Can_Brake_ID 0x329
#define Can_Brake_BYTE data[1]
#endif

#ifdef Santafe
#define Can_Velocity_ID 0x368
#define Can_Velocity_BYTE data[1]

#define Can_Gear_ID 0x367
#define Can_Gear_BYTE data[4]
#define Can_Gear_fw 0x05
#define Can_Gear_bw 0x07
#define Can_Gear_p 0x00
#define Can_Gear_n 0x06

#define Can_Brake_ID 0x394
#define Can_Brake_BYTE data[5]
#define Can_Brake_on 0xc3
#define Can_Brake_off 0x83

#endif


void init_hw(void)
{
    // set up the LED and switch ports
    ANSEL_POT = 1;
    ANSEL_TEMP = 1;
    TRIS_POT = 1;
    TRIS_TEMP = 1;
    ANSELC = ANSELC & 0xFC3F;   // (re)set the 3 switch bits + CAN due to error in v1.20 header
    _TRISA12=0;
    
}

void BuzzerInit(void) // will set 12bit, 4.96us/sample or 202KS/sec
{
    //VOLUME 1,2,3,4,5 :RC9, RD6, RD5, RC7, RC6
    _TRISB14 = 0;                   //ON/OFF PWM_RB14
    _TRISC9 = 0;                    //VOLUME 1 RC9
    _TRISD6 = 0;                    //VOLUME 2 RD6
    _TRISD5 = 0;                    //VOLUME 3 RD5
    _TRISC7 = 0;                    //VOLUME 4 RC7
    _TRISC6 = 0;                    //VOLUME 5 RC6
    _LATB14=1;
}
void Buzzeroff(void)
{
        _LATC9=0;    _LATD6=0;    _LATD5=0;    _LATC7=0;    _LATC6=0;
}
void Buzzer1(void)
{
    _LATC9=1;    _LATD6=0;    _LATD5=0;    _LATC7=0;    _LATC6=0;
}
void Buzzer2(void)
{
    _LATC9=0;    _LATD6=1;    _LATD5=0;    _LATC7=0;    _LATC6=0;
}
void Buzzer3(void)
{
    _LATC9=0;    _LATD6=0;    _LATD5=1;    _LATC7=0;    _LATC6=0;
}
void Buzzer4(void)
{
    _LATC9=0;    _LATD6=0;    _LATD5=0;    _LATC7=1;    _LATC6=0;
}
void Buzzer5(void)
{
    _LATC9=0;    _LATD6=0;    _LATD5=0;    _LATC7=0;    _LATC6=1;
}
void LEDInit(void) // will set 12bit, 4.96us/sample or 202KS/sec
{
    //VOLUME 1,2,3,4,5 :RC9, RD6, RD5, RC7, RC6
    _TRISA10 = 0;                   //R_A10
    _TRISB13 = 0;                    //G_B13
    _TRISB12 = 0;                    //B_B12
}

#define LED_R_ON() _LATA10=1
#define LED_R_OFF() _LATA10=0
#define LED_G_ON() _LATB13=1
#define LED_G_OFF() _LATB13=0
#define LED_B_ON() _LATB12=1
#define LED_B_OFF() _LATB12=0
void SWinit(void)
{
    _TRISB11 = 1;  
}
int SWcheck(void)
{
    if(_RB11==1) return 0;
    else         return 1;
}


void InitMonitor(void)
{
    // map MONITOR_TX pin to port RB4, which is remappable RP36
    _RP36R = 0x03; // map UART2 TXD to pin RB4
    _TRISB4=0;
    // set up the UART for default baud, 1 start, 1 stop, no parity
    U2MODEbits.STSEL = 0;       // 1-Stop bit
    U2MODEbits.PDSEL = 0;       // No Parity, 8-Data bits
    U2MODEbits.ABAUD = 0;       // Auto-Baud disabled
    U2MODEbits.BRGH = 0;        // Standard-Speed mode
    U2BRG = UART_MONITOR;          // Baud Rate setting for 38400 (default)
    U2STAbits.UTXISEL0 = 0;     // Interrupt after TX buffer done
    U2STAbits.UTXISEL1 = 1;
    IEC1bits.U2TXIE = 1;        // Enable UART TX interrupt
    U2MODEbits.UARTEN = 1;      // Enable UART (this bit must be set *BEFORE* UTXEN)
    U2STAbits.UTXEN = 1;
}

void    htoa(int hh)
{
    int   temp;
    temp=hh;
    hh &= 0xf0;     hh >>= 4;
    if (hh >= 10)   hh += 7;    
    hh += '0';
    putU2(hh);

    hh=temp;
    hh &= 0x0f;
    if (hh >= 10)   hh += 7;  
    hh += '0';
    putU2(hh);
}

void dtoa(unsigned int i)
{
    putU2(i/100+'0');
    putU2((i%100)/10+'0');
    putU2(i%10+'0');    
}

void putsU2(char *s)
{
    while (*s)
    { // loop until *s =\0, end of string
        putU2(*s++);
    } // send the character and point to the next one
}

void putU2(int c)
{
    while (U2STAbits.UTXBF); // wait while Tx buffer full
    U2TXREG = c;
}


//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
    IFS0bits.DMA1IF = 0; // Clear the DMA1 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
{
    IFS1bits.DMA2IF = 0; // Clear the DMA2 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
{
    IFS2bits.DMA3IF = 0; // Clear the DMA3 Interrupt Flag;
}

void __attribute__((interrupt, auto_psv)) _DefaultInterrupt(void)
{
    while (1);
}

void __attribute__((interrupt, auto_psv)) _OscillatorFail(void)
{
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _MathError(void)
{
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _StackError(void)
{
    while (1);
}

void __attribute__((interrupt, no_auto_psv)) _AddressError(void)
{
    while (1);
}
