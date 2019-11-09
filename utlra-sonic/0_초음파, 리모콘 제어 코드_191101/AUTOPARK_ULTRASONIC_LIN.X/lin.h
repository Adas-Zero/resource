#include "p33EV256GM106.h"

#define         LIN1_CS      _LATB9
#define         LIN1_TXE     _LATC8
#define         ANSEL_LIN1_CS   _ANSB9
#define         ANSEL_LIN1_TXE  _ANSC8
#define         TRIS_LIN1_CS   _TRISB9
#define         TRIS_LIN1_TXE  _TRISC8
//#define         ANSEL_LIN   _ANSB9
//#define         TRISLINTXE  _TRISC8
//#define         TRISLINCS   _TRISB9

#define         LIN_ID1      0x1F                  // USS.1 LIN ID byte (without parity = 0x1F)
#define         LIN_ID2      0x5E                  // USS.2 LIN ID byte (without parity = 0x1E)
#define         LIN_ID3      0xDD                  // USS.3 LIN ID byte (without parity = 0x1D)
#define         LIN_ID4      0x9C                  // USS.4 LIN ID byte (without parity = 0x1C)
#define         LIN_ID_INIT  0xC1                  // init sensing LIN ID byte 
#define         LIN_ID_INIT_Head 0x0C
#define         LIN_ID_INIT_Rear 0x03

#define         LIN_BIT_STUFF 0x4                  // number of bit times to stuff 'idle' between bytes
#define         LIN_BAUD    19200                    // LIN2.1 Protocol has 19200 bfs
#define         LIN_BIT_TIME ((1000000/LIN_BAUD) + 0.5)   // 1 bit time calculation in us
#define         LIN_BRGVAL  BAUD19200  //#define         BAUD19200   ((FCAN/19200)/16) - 1
#define LIN_MESSAGE_SIZE  8         // message size of the received LIN demo message

void InitLIN_TX(void);
void InitLIN_RX(void);
void LIN_RXmode(void);
void LIN_TXmode(void);
void LIN_Serial(void);
void LIN_Parallel(void);
void Init_Sensing(unsigned char);
void LIN_Transmit(unsigned char LIN_ID);
void LIN_Filtering(unsigned char LIN_ID);
int Calc_Checksum(int data_byte,int checksum);

volatile int lin_index, lin_start;
volatile char LIN_RXBUF[LIN_MESSAGE_SIZE]; // buffer of the received LIN message
volatile char lin_rx;  // receive message flags
unsigned int LINDETECT[5];

void Init_LIN(void)
{
    lin_index = 0;
    lin_start = 0;
    int j;
    for (j = 0; j < 8; j++)
    {
        LIN_RXBUF[j] = 0;
    }
    // map LIN_RX pin to port RD8, which is remappable RPI72
    RPINR18 = 0B01111001;           // map LIN receiver to pin RG9 /RPI121
    _TRISG9 = 1;                    // digital input pin
    _ANSG9 = 0;                       // ?? AD/IO ?? ????

    _RP120R= 0X01;  // map LIN transmitter to pin RG8, hi byte
    _TRISG8 = 0;            // digital output pin
     _ANSG8 = 0;      
    // set up the UART for LIN_BRGVAL baud, 1 start, 1 stop, no parity
    U1MODEbits.STSEL = 0;           // 1-Stop bit
    U1MODEbits.PDSEL = 0;           // No Parity, 8-Data bits
    U1MODEbits.ABAUD = 0;           // Auto-Baud disabled
    U1MODEbits.BRGH = 0;            // Standard-Speed mode
    U1BRG = LIN_BRGVAL;             // Baud Rate setting

    U1STAbits.URXISEL = 0;          // Interrupt after one RX done
    IEC0bits.U1RXIE = 1;            // Enable UART1 RX interrupt
    IEC4bits.U1EIE = 1;             // Enable Error (Framing) Interrupt for BREAK
    U1MODEbits.UARTEN = 1;          // Enable UART1
  
    U1STAbits.UTXISEL0 = 1; // Interrupt after one TX done
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1TXIE = 0;    // Disable UART TX interrupt
    
    
    LIN1_CS=1;      //_LATB9
    LIN1_TXE=1;     //_LATC8
    ANSEL_LIN1_CS=1;   //_ANSB9
    ANSEL_LIN1_TXE=1;  //_ANSC8
    TRIS_LIN1_CS=0;   //_TRISB9
    TRIS_LIN1_TXE=0;  //_TRISC8
}

void LIN_TXmode(void)
{
    LIN1_TXE = 1;            // enable LIN transmitter
}

void LIN_RXmode(void)
{
    LIN1_TXE = 0;                    // disable LIN transmitter
}
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    while (U1STAbits.TRMT == 0); // wait for transmitter empty
    IFS0bits.U1TXIF = 0; // Clear TX1 Interrupt flag
}

void Init_Sensing(unsigned char LIN_ID)
{
    int data_byte,id_byte;
    while (U1STAbits.TRMT == 0);    // wait for transmitter empty
    while (U1STAbits.UTXBRK == 1);  // wait for HW to clear the previous BREAK
    U1STAbits.UTXEN = 1;            // Enable UART TX
    U1STAbits.UTXBRK = 1;           // set the BREAK bit
    U1TXREG = 0;                    // dummy write to trigger UART transmit
    Nop();                          // must wait 1 instruction cycle
    U1TXREG = 0x55;                 // AUTO-BAUD sync character per J2602 spec

    while (U1STAbits.TRMT == 0);                // wait for transmitter empty
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);      // wait for idle time stuff
    id_byte = LIN_ID_INIT;
    U1TXREG = id_byte;                       // transmit the protected ID byte
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);      // wait for idle time stuff

    int checksum = id_byte;                     // initial checksum value (limited to a byte value)

    switch(LIN_ID)
    {
        case LIN_ID1 :
            data_byte = 0x01;
            break;
        
        case LIN_ID2 :
            data_byte = 0x02;
            break;
            
        case LIN_ID3 :
            data_byte = 0x04;
            break;
            
        case LIN_ID4 :
            data_byte = 0x08;
            break;
    }
            
    checksum=Calc_Checksum(data_byte,checksum);
    while (U1STAbits.TRMT == 0);            // wait for transmitter empty
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);  // wait for idle time stuff
    U1TXREG = data_byte;                    // send it
     
    checksum = (~checksum) & 0xFF;          // invert, byte value
    while (U1STAbits.TRMT == 0);            // wait for transmitter empty
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);  // wait for idle time stuff
    U1TXREG = checksum;                     // send it
}

int Calc_Checksum(int data_byte,int checksum)
{
    checksum = checksum + data_byte;        // add next
    if (checksum > 0xFF)
    {
        checksum = (checksum & 0xFF) + 1;   // truncate and add carry bit
    }
    return checksum;
}

void LIN_Transmit(unsigned char LIN_ID)
{
    int id_byte;
    while (U1STAbits.TRMT == 0);    // wait for transmitter empty
    while (U1STAbits.UTXBRK == 1);  // wait for HW to clear the previous BREAK
    U1STAbits.UTXEN = 1;            // Enable UART TX
    U1STAbits.UTXBRK = 1;           // set the BREAK bit
    U1TXREG = 0;                    // dummy write to trigger UART transmit
    Nop();                          // must wait 1 instruction cycle
    U1TXREG = 0x55;                 // AUTO-BAUD sync character per J2602 spec
    while (U1STAbits.TRMT == 0);                // wait for transmitter empty
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);      // wait for idle time stuff
    //id_byte = (p1 << 7) | (p0 << 6) | LIN_ID1;   // stuff parity bits into proper places
    id_byte = LIN_ID;
    U1TXREG = id_byte;                          // transmit the protected ID byte
    Delayus(LIN_BIT_TIME * LIN_BIT_STUFF);      // wait for idle time stuff
}
int lin_count=0;
int LIN_buffer[30],LIN_buffer_num;
int lin_loop_count;
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    int datal;
    datal = U1RXREG;
    LIN_buffer[LIN_buffer_num++]=datal;
    IFS0bits.U1RXIF = 0;                // Clear RX1 Interrupt flag
}


void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void)
{
    //
    // a LIN 'BREAK' (13 consecutive '0's) will generate a Framing Error
    // ***NOTE*** This ISR MUST be at a higher priority than U1RX ISR in order
    // to test for framing error prior to testing for SYNC byte
    //

    if (U1STAbits.FERR == 1)
    {
        lin_start = 1;          // first message detection phase
    }
    IFS4bits.U1EIF = 0;         // Clear LIN Error Flag
}

