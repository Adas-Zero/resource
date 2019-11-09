#include "p33EV256GM106.h"
#include "PUA_Init.h"
#include "timer.h"
#include "lin.h"
#include "ADC.h"
#include "SPI.h"
#include "can.h"
#include "i2c.h"
#include "flash.h"
#include <math.h>

//  Macros for Configuration Fuse Registers 
_FOSCSEL(FNOSC_PRIPLL);
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT);
// Startup directly into XT + PLL
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystal

_FWDT(FWDTEN_OFF);      // Watchdog Timer Enabled/disabled by user software

_FICD(ICS_PGD2);        // PGD3 for external PK3/ICD3/RealIce, use PGD2 for PKOB
_FPOR(BOREN0_OFF);      // no brownout detect
_FDMT(DMTEN_DISABLE);   // no deadman timer  <<< *** New feature, important to DISABLE

/* CAN receive message structure in RAM */

// Prototype Declarations
void rxECAN(mID *message);

void oscConfig(void);
void clearIntrflags(void);
void ecan1WriteMessage(void);
void adc_out(void);
void Test_Mode(void);

void Transmit_Data(void);
void ftoa(float, char*);

void Receive_Data(unsigned char);
int ADC_IN_H(void);

volatile int channel, i,TempValue, AverageValue, PotValue;
volatile int p0, p1;
volatile int tickTime = 50;             // Tick time in us
volatile float peripheralClk = 39.77;   // in Mhz
volatile unsigned char Pot_Volts;

unsigned char mode,lin_error;
unsigned int ascii_lo, ascii_hi, hex_dig,sw_counter;
uint8_t pua_mode_on;

volatile int datal;
volatile int datah;

volatile unsigned int TOF;
volatile unsigned int Distance;
#define Morning
//---------------------------------
//#define CAMRY
//#define TPSHIGH 0x0BF0  //3.7V
//#define TPSLOW 0x0500   //1.6V

#ifdef Morning
#define TPSHIGH 0x0865 //2.59V
#define TPSLOW 0x017D  //0.46V
#define OUTHIGH 390// main 460mv
#define OUTLOW 193 // main 225mv
#endif

//#define TPSHIGH 0x0CF0 //4V
//#define TPSLOW 0x0250  //0.7V
//#define OUTHIGH 607 // 700mV
//#define OUTLOW 295 // 350mV
//#define OUTHIGH 1350//  1615mV
//#define OUTLOW 674 // 800mV

#define TPS1percent (TPSHIGH-TPSLOW)*0.01
#define FW_TPSCAL (TPSHIGH-TPSLOW)*0.5
#define BW_TPSCAL (TPSHIGH-TPSLOW)*0.3
#define FW_Velocity_high 30
#define BW_Velocity_high 50
#define OFFSETX 0
#define OFFSETY 0
#define OFFSETZ 0
#define ULTRA_RANGE 300
#define LOOP_CYCLE 3
#define CAN_TIMER_CYCLE 10

int TPS_ADD_UPHILL=0,TPS_ADD_DOWNHILL=0,LIN_ADD_TPS_F=0,LIN_ADD_TPS_B=0;

volatile bool RELAY_ON_PERCENT,RELAY_ON_Velocity,recently_cut,ad_read,can_read;
volatile int Can_Velocity,ADC_cal,ADC_get,TPS_cut_ADC,Car_status,Can_Gear,Can_Brake;

uint8_t acc_fb,acc_lr,acc_g,bufferi;
uint8_t get_acc;
float anglex=0.0, angley=0.0, anglez=0.0;
uint8_t C_ultra[5]={250,250,250,250,250};
int main(void)
{
    oscConfig(); // Configure Oscillator Clock Source
    clearIntrflags(); // Clear Interrupt Flags
    init_hw();     // Initialize hardware on the board
    InitMonitor();  // Initialize the monitor UART2 module
    timer_init();
    ADCInit();
    InitCAN();
    BuzzerInit();
    LEDInit();
    SWinit();
    I2C1_Initialize();
    Init_LIN();
    LIN_TXmode();
    SPI1_Initialize();
    DAC_set(0,OUTHIGH);
    delay_10ms(1);
    DAC_set(1,OUTLOW);

    Can_Velocity=0;
    //test
    _ANSA1 = 0;
    _TRISA1 = 1;
    _LATA1 = 0;
    while (1)
    {
        if(linget_timer>1)
        {
            linget_timer=0;
            LIN_GET();
        }
        if(Loop_timer>LOOP_CYCLE)
        {
            Loop_timer=0;
        }
        if(can_timer>21)
        {

            can_timer=0;
        }
    }
}

void clearIntrflags(void)
{
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
    IPC16bits.U1EIP = 6;        //service the LIN framing error before the RX
    IPC2bits.U1RXIP = 4;
    U2STAbits.UTXBF=0; // MONITOR TX FLAG CLEAR
}

void oscConfig(void)
{
    //  Configure Oscillator to operate the device at 80MHz/40MIPs
    // 	Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // 	Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    // To be safe, always load divisors before feedback
    CLKDIVbits.PLLPOST = 0;     // N1=2
    CLKDIVbits.PLLPRE = 0;      // N2=2
    PLLFBD = 38;                // M=(40-2), Fcyc = 40MHz for ECAN baud timer
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void)
{
    while (U2STAbits.TRMT == 0); // wait for transmitter empty
    IFS1bits.U2TXIF = 0; // Clear TX2 Interrupt flag
}
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0; // Clear RX2 Interrupt flag
}
/******************************************************************************
 * Function:        SENT1 Tx/Rx Interrupt
 *****************************************************************************/
void __attribute__((__interrupt__, __auto_psv__)) _SENT1Interrupt(void)
{
    /* Interrupt Service Routine code goes here */
    if (SENT1CON1bits.RCVEN == 1) // was a RX message?
    {
        // Read data from SENT registers
        datal = (SENT1DATL >> 4); // Format to 12 bit data
        datah = SENT1DATH; // switch data + pot

        sent_rx = 1; // a message was received
    };
    IFS11bits.SENT1IF = 0; // clear interrupt flag
}
/******************************************************************************
 * Function:        SENT1 error interrupt
 *****************************************************************************/
void __attribute__((__interrupt__, __auto_psv__)) _SENT1ERRInterrupt(void)
{
    // Sent Error handling code here

    IFS11bits.SENT1EIF = 0; // Clear interrupt flag.
    LED1 = 1;
    LED2 = 1;
    LED3 = 1;
    while (1); // sit here if error
}
void LIN_GET2()
{
    unsigned int ultra_buffer;
    switch(lin_loop_count++)
    {
        case 0:
            LIN_buffer_num=0;
            buffer_clear();
            Init_Sensing(0x0F);
            htoa(LIN_buffer[0]);
            htoa(LIN_buffer[1]);
            htoa(LIN_buffer[2]);
            htoa(LIN_buffer[3]);
            htoa(LIN_buffer[4]);
            htoa(LIN_buffer[5]);
            htoa(LIN_buffer[6]);
            htoa(LIN_buffer[7]);
            htoa(LIN_buffer[8]);
            htoa(LIN_buffer[9]);
            putU2(13);
            break;
        case 3:
            LIN_buffer_num=0;
            buffer_clear();
            LIN_Transmit(LIN_ID1);
            break;
        case 5:
            ultra_buffer=(((LIN_buffer[8]<<8)&0xff00)|(LIN_buffer[7]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0x000) 
            {   
                C_ultra[1]=ultra_buffer;
                C_ultra[0]|=0X01;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[1]=250;
                C_ultra[0]&=0b11111110;
            }
            else
            {
                C_ultra[1]=ultra_buffer;
                C_ultra[0]&=0b11111110;                
            }
            LIN_buffer[5]=0;
            LIN_buffer[4]=0;
            LIN_buffer_num=0;
            buffer_clear();
            LIN_Transmit(LIN_ID2);
            //Init_Sensing(LIN_ID2);
            break;
        case 7: 
//            putU2('@');
//            htoa(LIN_buffer[0]);
//            htoa(LIN_buffer[1]);
//            htoa(LIN_buffer[2]);
//            htoa(LIN_buffer[3]);
//            htoa(LIN_buffer[4]);
//            htoa(LIN_buffer[5]);
//            htoa(LIN_buffer[6]);
//            htoa(LIN_buffer[7]);
//            htoa(LIN_buffer[8]);
//            htoa(LIN_buffer[9]);
//            putU2(13);
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0x000) 
            {
                C_ultra[2]=ultra_buffer;
                C_ultra[0]|=0X02;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[2]=250;
                C_ultra[0]&=0b11111101;
            }
            else
            {
                C_ultra[2]=ultra_buffer;
                C_ultra[0]&=0b11111101;                
            }
            
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            Init_Sensing(LIN_ID3);
            break;
        case 14:
            LIN_Transmit(LIN_ID3);
            break;
        case 15: 
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0) 
            {
                C_ultra[3]=ultra_buffer;
                C_ultra[0]|=0X04;
            }
            else if(C_ultra[3]>250)
            {
                C_ultra[3]=250;
                C_ultra[0]&=0b11111011;
            }
            else
            {
                C_ultra[3]=ultra_buffer;
                C_ultra[0]&=0b11111011;
            }
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            Init_Sensing(LIN_ID4);  
            break;
        case 19: 
            LIN_Transmit(LIN_ID4);
            break;
        case 20:
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0)
            {
                C_ultra[4]=ultra_buffer;
                C_ultra[0]|=0X08;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[4]=250;
                C_ultra[0]&=0b11110111;
            }
            else
            {
                C_ultra[4]=ultra_buffer;
                C_ultra[0]&=0b11110111;
            }
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            lin_loop_count=0;
            break;
    }
}
void LOGGER(void)
{
    putU2(0x0d);
    putsU2(" Relay_on acc:");
    htoa(get_acc);
    putsU2(" aps:");
    htoa(ADC_cal>>8);
    htoa(ADC_cal&0xff);
    if(Can_Gear==Can_Gear_fw)
    {
        putsU2(" FA_cal:");
        htoa((TPS_ADD_UPHILL>>8)&0xff);
        htoa((TPS_ADD_UPHILL)&0xff);
        putsU2(" lin_cal:");
        htoa((LIN_ADD_TPS_F>>8)&0xff);
        htoa((LIN_ADD_TPS_F)&0xff);
    }
    else
    {
        putsU2(" BA_cal:");
        htoa((TPS_ADD_DOWNHILL>>8)&0xff);
        htoa((TPS_ADD_DOWNHILL)&0xff);
        putsU2(" lin_cal:");
        htoa((LIN_ADD_TPS_B>>8)&0xff);
        htoa((LIN_ADD_TPS_B)&0xff);                    
    }
    putsU2(" LIN: ");
    dtoa(LINDETECT[0]); putU2(' ');
    dtoa(LINDETECT[1]); putU2(' ');
    dtoa(LINDETECT[2]); putU2(' ');
    dtoa(LINDETECT[3]); putU2(0x0d);
}
void LIN_GET()
{
    unsigned int ultra_buffer;
    switch(lin_loop_count++)
    {
        case 0:
            Init_Sensing(LIN_ID1);
            break;
        case 4:
            LIN_Transmit(LIN_ID1);
            break;
        case 5: 
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0x000) 
            {   
                C_ultra[1]=ultra_buffer;
                C_ultra[0]|=0X01;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[1]=250;
                C_ultra[0]&=0b11111110;
            }
            else
            {
                C_ultra[1]=ultra_buffer;
                C_ultra[0]&=0b11111110;                
            }
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            Init_Sensing(LIN_ID2);
            break;
        case 9:
            LIN_Transmit(LIN_ID2);
            break;
        case 10: 
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0x000) 
            {
                C_ultra[2]=ultra_buffer;
                C_ultra[0]|=0X02;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[2]=250;
                C_ultra[0]&=0b11111101;
            }
            else
            {
                C_ultra[2]=ultra_buffer;
                C_ultra[0]&=0b11111101;                
            }
            
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            Init_Sensing(LIN_ID3);
            break;
        case 14:
            LIN_Transmit(LIN_ID3);
            break;
        case 15: 
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0) 
            {
                C_ultra[3]=ultra_buffer;
                C_ultra[0]|=0X04;
            }
            else if(C_ultra[3]>250)
            {
                C_ultra[3]=250;
                C_ultra[0]&=0b11111011;
            }
            else
            {
                C_ultra[3]=ultra_buffer;
                C_ultra[0]&=0b11111011;
            }
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            Init_Sensing(LIN_ID4);  
            break;
        case 19: 
            LIN_Transmit(LIN_ID4);
            break;
        case 20:
            ultra_buffer=(((LIN_buffer[9]<<8)&0xff00)|(LIN_buffer[8]&0x00ff))/58; //Temperature calibration required 
            if(ultra_buffer==0)
            {
                C_ultra[4]=ultra_buffer;
                C_ultra[0]|=0X08;
            }
            else if(ultra_buffer>250)
            {
                C_ultra[4]=250;
                C_ultra[0]&=0b11110111;
            }
            else
            {
                C_ultra[4]=ultra_buffer;
                C_ultra[0]&=0b11110111;
            }
            LIN_buffer[9]=0;
            LIN_buffer[8]=0;
            LIN_buffer_num=0;
            lin_loop_count=1;
            CAN_ID = 0xC8;
            CAN_TX_DATA[0] = C_ultra[0];
            CAN_TX_DATA[1] = C_ultra[1];
            CAN_TX_DATA[2] = C_ultra[2];
            CAN_TX_DATA[3] = C_ultra[3];
            CAN_TX_DATA[4] = C_ultra[4];
            CAN_TX_DATA[5] = 0;
            CAN_TX_DATA[6] = 0;
            CAN_TX_DATA[7] = 0;
            CAN_Transmit();
            Init_Sensing(LIN_ID1);
            break;
    }
}
void buffer_clear(void)
{
    LIN_buffer[0]=0;
    LIN_buffer[1]=0;
    LIN_buffer[2]=0;
    LIN_buffer[3]=0;
    LIN_buffer[4]=0;
    LIN_buffer[5]=0;
    LIN_buffer[6]=0;
    LIN_buffer[7]=0;
    LIN_buffer[8]=0;
    LIN_buffer[9]=0;
}