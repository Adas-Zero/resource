void InitCAN(void);
void CAN_RData(void);
void CAN_Transmit(void);
void clearRxFlags(unsigned char buffer_number);

/* ECAN message type identifiers */
#define CAN_MSG_DATA 0x01
#define CAN_MSG_RTR 0x02
#define CAN_FRAME_EXT 0x03
#define CAN_FRAME_STD 0x04
#define CAN_BUF_FULL 0x05
#define CAN_BUF_EMPTY 0x06
#define NUM_OF_ECAN_BUFFERS 32
#define MSG_SID 0x123              // the arbitrary CAN SID of the transmitted message

#define NUM_DIGITS  5               // floating point digits to print
#define STRING_BUFFER_SIZE  64      // arbitrary length message buffer

/* CAN filter and mask defines */
/* Macro used to write filter/mask ID to Register CiRXMxSID and
CiRXFxSID. For example to setup the filter to accept a value of
0x123, the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
write the register space to accept message with ID 0x123
USE FOR STANDARD MESSAGES ONLY */
#define CAN_FILTERMASK2REG_SID(x) ((x & 0x07FF)<< 5)
/* the Macro will set the "MIDE" bit in CiRXMxSID */
#define CAN_SETMIDE(sid) (sid | 0x0008)
/* the macro will set the EXIDE bit in the CiRXFxSID to
accept extended messages only */
#define CAN_FILTERXTD(sid) (sid | 0x0008)
/* the macro will clear the EXIDE bit in the CiRXFxSID to
accept standard messages only */
#define CAN_FILTERSTD(sid) (sid & 0xFFF7)


volatile unsigned int ecan1MsgBuf[NUM_OF_ECAN_BUFFERS][8]
__attribute__((aligned(NUM_OF_ECAN_BUFFERS * 16)));
typedef struct{
	/* keep track of the buffer status */
	unsigned char buffer_status;
	/* RTR message or data message */
	unsigned char message_type;
	/* frame type extended or standard */
	unsigned char frame_type;
	/* buffer being used to send and receive messages */
	unsigned char buffer;
	/* 29 bit id max of 0x1FFF FFFF
	*  11 bit id max of 0x7FF */
	unsigned long id;
	unsigned int data[8];
	unsigned char data_length;
}mID;
volatile int CAN_ID = 0XC81;
volatile int CAN_TX_DATA[8] = {0x02,0x01,0x0D,0,0,0,0,0};
volatile int CAN_RX_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char Buf_result[NUM_DIGITS + 2];        // digits + '.' and allow for '-'
char *pBuf;                             // buffer for ASCII result of a float
char s[STRING_BUFFER_SIZE];             // s[] holds a string to transmit

mID canRxMessage;
volatile char can_rx, sent_rx;  // receive message flags

void InitCAN(void)
{
    _TRISG9 = 0;
    _LATG9 = 0;
    _TRISF1 = 0;
    _TRISF0 = 1;

    //
    // remap the CAN module to the proper pins on the board
    //
    RPINR26 = 0x60;         // connect CAN RX to RPI96
    RPOR9 = 0x000E;         // connect CAN TX to RP97

    C1CTRL1bits.REQOP = 4;

    while (C1CTRL1bits.OPMODE != 4);
    C1CTRL1bits.WIN = 0;

    /* Set up the CAN module for 250kbps speed with 10 Tq per bit. */

    C1CFG1 = 0x83;          // BRP = 8 SJW = 2 Tq
    C1CFG2 = 0x2D2;
    C1FCTRL = 0xC01F;       // No FIFO, 32 Buffers

    //
    // set up the CAN DMA0 for the Transmit Buffer
    //
    DMA0CONbits.SIZE = 0x0;
    DMA0CONbits.DIR = 0x1;
    DMA0CONbits.AMODE = 0x2;
    DMA0CONbits.MODE = 0x0;
    DMA0REQ = 70;
    DMA0CNT = 7;
    DMA0PAD = (volatile unsigned int)&C1TXD;
    DMA0STAL = (unsigned int)&ecan1MsgBuf;
    DMA0STAH = (unsigned int)&ecan1MsgBuf;

    C1TR01CONbits.TXEN0 = 0x1;          // Buffer 0 is the Transmit Buffer
    C1TR01CONbits.TX0PRI = 0x3;         // transmit buffer priority

    DMA0CONbits.CHEN = 0x1;

    /* initialise the DMA channel 2 for ECAN Rx */
    /* setup channel 2 for peripheral indirect addressing mode
    normal operation, word operation and select as Rx to peripheral */
    DMA2CON = 0x0020;
    /* setup the address of the peripheral ECAN1 (C1RXD) */
	DMA2PAD = (volatile unsigned int)&C1RXD;
 	/* Set the data block transfer size of 8 */
 	DMA2CNT = 7;
 	/* automatic DMA Rx initiation by DMA request */
	DMA2REQ = 0x0022;
	/* start adddress offset value */
	DMA2STAL=(unsigned int)(&ecan1MsgBuf);
    DMA2STAH=(unsigned int)(&ecan1MsgBuf);
	/* enable the channel */
	DMA2CONbits.CHEN=1;

	/* 4 CAN Messages to be buffered in DMA RAM */
	C1FCTRLbits.DMABS=0b000;

    /* Filter configuration */
	/* enable window to access the filter configuration registers */
	C1CTRL1bits.WIN = 0b1;
	/* select acceptance mask 0 filter 0 buffer 1 */
	C1FMSKSEL1bits.F0MSK = 0;

    /* setup the mask to check every bit of the standard message, the macro when called as */
    /* CAN_FILTERMASK2REG_SID(0x7FF) will write the register C1RXM0SID to include every bit in */
    /* filter comparison */
    C1RXM0SID=CAN_FILTERMASK2REG_SID(0x7FF);
    C1RXM1SID=CAN_FILTERMASK2REG_SID(0x7FF);
    C1RXM2SID=CAN_FILTERMASK2REG_SID(0x7FF);
    

	/* configure accpetence filter 0
	setup the filter to accept a standard id of 0x123,
	the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
	write the register C1RXF0SID to accept only standard id of 0x123
	*/
	C1RXF0SID = CAN_FILTERMASK2REG_SID(Can_Velocity_ID); // VELOCITY
	C1RXF1SID = CAN_FILTERMASK2REG_SID(0x123); // VELOCITY
    C1RXF2SID = CAN_FILTERMASK2REG_SID(0x123); // VELOCITY
    C1RXF3SID = CAN_FILTERMASK2REG_SID(0x123); // VELOCITY
        //    C1RXF1SID = CAN_FILTERMASK2REG_SID(0x7E8); //MSG_SID = 0x123
//    C1RXF2SID = CAN_FILTERMASK2REG_SID(Can_Brake_ID); // BRAKE
//    C1RXF3SID = CAN_FILTERMASK2REG_SID(Can_Gear_ID); // GEAR 
    
    //C1RXF4SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF5SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF6SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF7SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF8SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF9SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF10SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF11SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF12SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF13SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
    //C1RXF14SID = CAN_FILTERMASK2REG_SID(0xB1); //MSG_SID = 0x123
	/* set filter to check for standard ID and accept standard id only */
	C1RXM0SID = CAN_SETMIDE(C1RXM0SID);
    C1RXM1SID = CAN_SETMIDE(C1RXM1SID);
    C1RXM2SID = CAN_SETMIDE(C1RXM2SID);
    
	C1RXF0SID = CAN_FILTERSTD(C1RXF0SID);
    C1RXF1SID = CAN_FILTERSTD(C1RXF1SID);
    //C1RXF2SID = CAN_FILTERSTD(C1RXF2SID);
    //C1RXF3SID = CAN_FILTERSTD(C1RXF3SID);
    //C1RXF4SID = CAN_FILTERSTD(C1RXF4SID);
    //C1RXF5SID = CAN_FILTERSTD(C1RXF5SID);
    //C1RXF6SID = CAN_FILTERSTD(C1RXF6SID);
    //C1RXF7SID = CAN_FILTERSTD(C1RXF7SID);
    //C1RXF8SID = CAN_FILTERSTD(C1RXF8SID);
    //C1RXF9SID = CAN_FILTERSTD(C1RXF9SID);
    //C1RXF10SID = CAN_FILTERSTD(C1RXF10SID);
    //C1RXF11SID = CAN_FILTERSTD(C1RXF11SID);
    //C1RXF12SID = CAN_FILTERSTD(C1RXF12SID);
    //C1RXF13SID = CAN_FILTERSTD(C1RXF13SID);
    //C1RXF14SID = CAN_FILTERSTD(C1RXF14SID);
    //C1RXF15SID = CAN_FILTERSTD(C1RXF15SID);
	/* acceptance filter to use buffer 1 for incoming messages */
	C1BUFPNT1bits.F0BP = 0b0001;
    C1BUFPNT1bits.F1BP = 0b0001;
    C1BUFPNT1bits.F2BP = 0b0001;
    C1BUFPNT1bits.F3BP = 0b0001;

	/* enable filter 0 */
	C1FEN1bits.FLTEN0 = 1;
    C1FEN1bits.FLTEN1 = 1;
    /* clear window bit to access ECAN control registers */
	C1CTRL1bits.WIN = 0;

    /* ECAN1, Buffer 1 is a Receive Buffer */
	C1TR01CONbits.TXEN1 = 0;

    /* clear the buffer and overflow flags */
	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;

    // Place the ECAN module in Normal mode.
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);

    //
    // CAN RX interrupt enable - 'double arm' since 2-level nested interrupt
    //
    C1INTEbits.RBIE = 1;
    IEC2bits.C1IE = 1;
}


void CAN_RData(void)
{
    int CAN_RX_ID = 0;
    CAN_RX_ID = canRxMessage.id;
    CAN_RX_DATA[0] = canRxMessage.data[0];
    CAN_RX_DATA[1] = canRxMessage.data[1];
    CAN_RX_DATA[2] = canRxMessage.data[2];
    CAN_RX_DATA[3] = canRxMessage.data[3];
    CAN_RX_DATA[4] = canRxMessage.data[4];
    CAN_RX_DATA[5] = canRxMessage.data[5];
    CAN_RX_DATA[6] = canRxMessage.data[6];
    CAN_RX_DATA[7] = canRxMessage.data[7];
    
    if(canRxMessage.id == 0xa1)
    {
        CAN_ID = 0x1a;
        CAN_TX_DATA[0] = LIN_RXBUF[1];
        CAN_TX_DATA[1] = LIN_RXBUF[2];
        CAN_TX_DATA[2] = LIN_RXBUF[3];
        CAN_TX_DATA[3] = LIN_RXBUF[4];
        CAN_TX_DATA[4] = LIN_RXBUF[5];
        CAN_TX_DATA[5] = LIN_RXBUF[6];
        CAN_TX_DATA[6] = LIN_RXBUF[7];
        CAN_TX_DATA[7] = LIN_RXBUF[8];
        CAN_Transmit();
    }
}

void CAN_Transmit(void)
{
    ecan1MsgBuf[0][0] = CAN_ID << 2;

    ecan1MsgBuf[0][1] = 0x0000;
    /* CiTRBnDLC = 0b0000 0000 xxx0 1111
    EID<17:6> = 0b000000
    RTR = 0b0
    RB1 = 0b0
    RB0 = 0b0
    DLC = 6 */
    ecan1MsgBuf[0][2] = 0x0008;

    // Write message 6 data bytes as follows:
    //
    // POTH POTL TEMPH TEMPL 0000 S3S2S1
    //
    ecan1MsgBuf[0][3] = (CAN_TX_DATA[1] << 8) | (CAN_TX_DATA[0]);
    ecan1MsgBuf[0][4] = (CAN_TX_DATA[3] << 8) | (CAN_TX_DATA[2]);
    ecan1MsgBuf[0][5] = (CAN_TX_DATA[5] << 8) | (CAN_TX_DATA[4]);
    ecan1MsgBuf[0][6] = (CAN_TX_DATA[7] << 8) | (CAN_TX_DATA[6]);

    Nop();
    Nop();
    Nop();
    /* Request message buffer 0 transmission */
    C1TR01CONbits.TXREQ0 = 0x1;
    /* The following shows an example of how the TXREQ bit can be polled to check if transmission
    is complete. */
    Nop();
    Nop();
    Nop();
    while (C1TR01CONbits.TXREQ0 == 1);
    // Message was placed successfully on the bus, return
}

void rxECAN(mID *message)
{
	unsigned int ide=0;
	unsigned int rtr=0;
	unsigned long id=0;

	/*
	Standard Message Format:
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits

	Remote Transmission Request Bit for standard frames
	SRR->	"0"	 Normal Message
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames
	SRR->	should always be set to "1" as per CAN specification

	Extended  Identifier Bit
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier

	Remote Transmission Request Bit for extended frames
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames
	*/

	/* read word 0 to see the message type */
	ide=ecan1MsgBuf[message->buffer][0] & 0x0001;

	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan1MsgBuf[message->buffer][0] & 0x1FFC) >> 2;
		message->frame_type=CAN_FRAME_STD;
		rtr=ecan1MsgBuf[message->buffer][0] & 0x0002;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1MsgBuf[message->buffer][0] & 0x1FFC;
		message->id=id << 16;
		id=ecan1MsgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan1MsgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;
		message->frame_type=CAN_FRAME_EXT;
		rtr=ecan1MsgBuf[message->buffer][2] & 0x0200;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(rtr==1)
	{
		message->message_type=CAN_MSG_RTR;
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan1MsgBuf[message->buffer][3];
		message->data[1]=(unsigned char)((ecan1MsgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan1MsgBuf[message->buffer][4];
		message->data[3]=(unsigned char)((ecan1MsgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan1MsgBuf[message->buffer][5];
		message->data[5]=(unsigned char)((ecan1MsgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan1MsgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan1MsgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=(unsigned char)(ecan1MsgBuf[message->buffer][2] & 0x000F);
	}
	clearRxFlags(message->buffer);
}

void clearRxFlags(unsigned char buffer_number)
{
	if((C1RXFUL1bits.RXFUL1) && (buffer_number==1))
		/* clear flag */
		C1RXFUL1bits.RXFUL1=0;
	/* check to see if buffer 2 is full */
	else if((C1RXFUL1bits.RXFUL2) && (buffer_number==2))
		/* clear flag */
		C1RXFUL1bits.RXFUL2=0;
	/* check to see if buffer 3 is full */
	else if((C1RXFUL1bits.RXFUL3) && (buffer_number==3))
		/* clear flag */
		C1RXFUL1bits.RXFUL3=0;
	else;

}

void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
{
    IFS2bits.C1IF = 0; // clear interrupt flag
    if (C1INTFbits.TBIF)
    {
        C1INTFbits.TBIF = 0;
    }
    if (C1INTFbits.RBIF)
    {
        if(C1RXFUL1bits.RXFUL1)
        {
        /* set the buffer full flag and the buffer received flag */
        canRxMessage.buffer_status = CAN_BUF_FULL;
        canRxMessage.buffer = 1;
        can_rx = 1;
        }
        C1INTFbits.RBIF = 0;
    }
}
void ftoa(float f, char *buf)
{
    int pos, ix, dp, num;
    pos = 0;
    ix = 0;
    dp = 0;
    num = 0;

    if (f < 0)
    {
        buf[pos++] = '-';
        f = -f;
    }
    dp = 0;
    while (f >= 10.0)
    {
        f = f / 10.0;
        dp++;
    }
    for (ix = 1; ix < (NUM_DIGITS + 1); ix++)
    {
        num = (int)f;
        f = f - num;
        buf[pos++] = '0' + num;
        if (dp == 0) buf[pos++] = '.';
        f = f * 10.0;
        dp--;
    }
}