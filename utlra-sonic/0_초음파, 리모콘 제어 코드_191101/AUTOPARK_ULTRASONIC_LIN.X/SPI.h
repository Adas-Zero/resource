/**
  SPI1 Generated Driver API Header File

  Company:
    Microchip Technology Inc.

  File Name:
    spi1.h

  @Summary
    This is the generated header file for the SPI1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for SPI1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.145.0
        Device            :  dsPIC33EV256GM106
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36b
        MPLAB             :  MPLAB X v5.25
*/

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#ifndef _SPI1_H
#define _SPI1_H

/**
 Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/**
 Section: Data Type Definitions
*/
        
/**
  SPI1_DUMMY_DATA 

  @Summary
    Dummy data to be sent. 

  @Description
    Dummy data to be sent, when no input buffer is specified in the buffer APIs.
 */
#define SPI1_DUMMY_DATA 0x0
		
/**
  SPI1_FIFO_FILL_LIMIT

  @Summary
    FIFO fill limit for data transmission. 

  @Description
    The amount of data to be filled in the FIFO during transmission. The maximum limit allowed is 8.
 */
#define SPI1_FIFO_FILL_LIMIT 0x8

//Check to make sure that the FIFO limit does not exceed the maximum allowed limit of 8
#if (SPI1_FIFO_FILL_LIMIT > 8)

    #define SPI1_FIFO_FILL_LIMIT 8

#endif

/**
  SPI1 Status Enumeration

  @Summary
    Defines the status enumeration for SPI1.

  @Description
    This defines the status enumeration for SPI1.
 */
typedef enum {
    SPI1_SHIFT_REGISTER_EMPTY  = 1 << 7,
    SPI1_RECEIVE_OVERFLOW = 1 << 6,
    SPI1_RECEIVE_FIFO_EMPTY = 1 << 5,
    SPI1_TRANSMIT_BUFFER_FULL = 1 << 1,
    SPI1_RECEIVE_BUFFER_FULL = 1 << 0
}SPI1_STATUS;

/**
  SPI1 Mode Enumeration

  @Summary
    Defines the mode of operation for SPI1.

  @Description
    This defines the mode of operation for SPI1.
 */
typedef enum {
    SPI1_DRIVER_TRANSFER_MODE_8BIT   = 0,
    SPI1_DRIVER_TRANSFER_MODE_16BIT  = 1,
    SPI1_DRIVER_TRANSFER_MODE_32BIT  = 2,
} SPI1_TRANSFER_MODE;

/**
 Section: Interface Routines
*/

/**
  @Summary
    Initializes the SPI instance : 1

  @Description
    This routine initializes the spi1 driver instance for : 1
    index, making it ready for clients to open and use it.

    This routine must be called before any other SPI1 routine is called.
    This routine should only be called once during system initialization.
 
  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t   myWriteBuffer[MY_BUFFER_SIZE];
    uint16_t   myReadBuffer[MY_BUFFER_SIZE];
    uint16_t writeData;
    uint16_t readData;
    SPI1_STATUS status;
    unsigned int    total;
    SPI1_Initialize;

    total = 0;
    numberOfBytesFactor = 2;
    do
    {
        total  = SPI1_Exchange16bitBuffer( &myWriteBuffer[total], (MY_BUFFER_SIZE - total)*numberOfBytesFactor, &myReadBuffer[total]);

        // Do something else...

    } while( total < MY_BUFFER_SIZE );

    readData = SPI1_Exchange16bit( writeData);

    status = SPI1_StatusGet();

    </code>

*/

void SPI1_Initialize (void);

/**
  @Summary
    Exchanges one word of data from SPI1

  @Description
    This routine exchanges one word of data from SPI1.
    This is a blocking routine.

  @Preconditions
    The SPI1_Initialize routine must have been called for the specified
    SPI1 driver instance.
    The SPI transfer mode should be selected as 16bit mode in the initialization. 
    Do not select 8 bit mode, only the lower byte of the data will sent or received if selected. 

  @Returns
    Data read from SPI1

  @Param
    data         - Data to be written onto SPI1.

  @Example 
    Refer to SPI1_Initialize() for an example	
 
*/

uint16_t SPI1_Exchange16bit( uint16_t data );

/**
  @Summary
    Exchanges data from a buffer of size one word from SPI1

  @Description
    This routine exchanges data from a buffer of size one word from the SPI1.
    This is a blocking routine.

  @Preconditions
    The SPI1_Initialize routine must have been called for the specified
    SPI1 driver instance.
    The SPI transfer mode should be selected as 16bit mode in the initialization. 
    Do not select 8 bit mode, only the lower byte of the data will sent or received if selected. 

  @Returns
    Number of 16bit data written/read.

  @Param
    dataTransmitted         - Buffer of data to be written onto SPI1.
 
  @Param
    byteCount         - Number of bytes to be exchanged.
 
  @Param
    dataTransmitted         - Buffer of data to be read from SPI1.

  @Example 
    Refer to SPI1_Initialize() for an example	
 
*/

uint16_t SPI1_Exchange16bitBuffer(uint16_t *dataTransmitted, uint16_t byteCount, uint16_t *dataReceived);


/**
  @Summary
    Returns the value of the status register of SPI instance : 1

  @Description
    This routine returns the value of the status register of SPI1 driver instance : 1

  @Preconditions
    None.

  @Returns
    Returns the value of the status register.

  @Param
    None.

  @Example 
    Refer to SPI1_Initialize() for an example	
 
*/

SPI1_STATUS SPI1_StatusGet(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif //_SPI1_H
    
/*******************************************************************************
 End of File
*/

    
/**
  SPI1 Generated Driver API Source File

  Company:
    Microchip Technology Inc.

  File Name:
    spi1.c

  @Summary
    This is the generated source file for the SPI1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for SPI1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.145.0
        Device            :  dsPIC33EV256GM106
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36b
        MPLAB             :  MPLAB X v5.25
*/

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

/**
 Section: File specific functions
*/

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void);
void SPI1_Exchange( uint8_t *pTransmitData, uint8_t *pReceiveData );
uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData);

/**
 Section: Driver Interface Function Definitions
*/
#define MY_BUFFER_SIZE 1
#define SS_ON()  _LATC5=0
#define SS_OFF() _LATC5=1


void SPI1_Initialize (void)
{
    // MSTEN Master; DISSDO disabled; PPRE 64:1; SPRE 8:1; MODE16 enabled; SMP Middle; DISSCK disabled; CKP Idle:Low, Active:High; CKE Idle to Active; SSEN disabled; 
    SPI1CON1 = 0x420;
    // SPIFSD disabled; SPIBEN enabled; FRMPOL disabled; FRMDLY disabled; FRMEN disabled; 
    SPI1CON2 = 0x01;
    // SISEL SPI_INT_SPIRBF; SPIROV disabled; SPIEN enabled; SPISIDL disabled; 
    SPI1STAT = 0x800C;
    _TRISC5 = 0;
    _TRISA4 = 0;
    _TRISC3 = 0;
    _ANSA4 = 0;
    _ANSC3 = 0;
}

void SPI1_Exchange( uint8_t *pTransmitData, uint8_t *pReceiveData )
{

    while( SPI1STATbits.SPITBF == true )
    {

    }

    SPI1BUF = *((uint16_t*)pTransmitData);

    while ( SPI1STATbits.SRXMPT == true);

    *((uint16_t*)pReceiveData) = SPI1BUF;

}

uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData)
{

    uint16_t dataSentCount = 0;
    uint16_t dataReceivedCount = 0;
    uint16_t dummyDataReceived = 0;
    uint16_t dummyDataTransmit = SPI1_DUMMY_DATA;

    uint8_t  *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;

    addressIncrement = 2;
    byteCount >>= 1;


    // set the pointers and increment delta 
    // for transmit and receive operations
    if (pTransmitData == NULL)
    {
        sendAddressIncrement = 0;
        pSend = (uint8_t*)&dummyDataTransmit;
    }
    else
    {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*)pTransmitData;
    }
        
    if (pReceiveData == NULL)
    {
       receiveAddressIncrement = 0;
       pReceived = (uint8_t*)&dummyDataReceived;
    }
    else
    {
       receiveAddressIncrement = addressIncrement;        
       pReceived = (uint8_t*)pReceiveData;
    }


    while( SPI1STATbits.SPITBF == true )
    {

    }

    while (dataSentCount < byteCount)
    {
        if ( SPI1STATbits.SPITBF != true )
        {

            SPI1BUF = *((uint16_t*)pSend);

            pSend += sendAddressIncrement;
            dataSentCount++;
        }

        if (SPI1STATbits.SRXMPT == false)
        {
            *((uint16_t*)pReceived) = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }

    }
    while (dataReceivedCount < byteCount)
    {
        if (SPI1STATbits.SRXMPT == false)
        {
            *((uint16_t*)pReceived) = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }

    return dataSentCount;
}

uint16_t SPI1_Exchange16bit( uint16_t data )
{
    uint16_t receiveData;

    SPI1_Exchange((uint8_t*)&data, (uint8_t*)&receiveData);

    return (receiveData);
}

uint16_t SPI1_Exchange16bitBuffer(uint16_t *dataTransmitted, uint16_t byteCount, uint16_t *dataReceived)
{
    return (SPI1_ExchangeBuffer((uint8_t*)dataTransmitted, byteCount, (uint8_t*)dataReceived));
}


/**

    The module's transfer mode affects the operation
    of the exchange functions. The table below shows
    the effect on data sent or received:
    |=======================================================================|
    | Transfer Mode  |     Exchange Function      |        Comments         |
    |=======================================================================|
    |                | SPIx_Exchange8bitBuffer()  |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange8bit()        |                         |
    |     8 bits     |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() | Do not use. Only the    |
    |                |----------------------------| lower byte of the 16-bit|
    |                | SPIx_Exchange16bit()       | data will be sent or    |
    |                |                            | received.               |
    |----------------|----------------------------|-------------------------|
    |                | SPIx_Exchange8bitBuffer()  | Do not use. Additional  |
    |                |----------------------------| data byte will be       |
    |                | SPIx_Exchange8bit()        | inserted for each       |
    |                |                            | 8-bit data.             |
    |     16 bits    |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange16bit()       |                         |
    |----------------|----------------------------|-------------------------|
*/
inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void)
{
	if (SPI1CON1bits.MODE16 == 0)
        return SPI1_DRIVER_TRANSFER_MODE_8BIT;
    else
        return SPI1_DRIVER_TRANSFER_MODE_16BIT;
}


SPI1_STATUS SPI1_StatusGet()
{
    return(SPI1STAT);
}
void DAC_set(int channel,uint16_t vout)
{
    uint16_t   myWriteBuffer[MY_BUFFER_SIZE];
    uint16_t   myReadBuffer[MY_BUFFER_SIZE];
    unsigned int total=0;
    
    if     (channel==1) {myWriteBuffer[0]= 0b1010000000000000 | (vout&0xfff);}
    else if(channel==0) {myWriteBuffer[0]= 0b0010000000000000 | (vout&0xfff);}
    SS_ON();
    do
    {
        total  = SPI1_Exchange16bitBuffer( &myWriteBuffer[total],(MY_BUFFER_SIZE - total)*2, &myReadBuffer[total]);
    } while( total < MY_BUFFER_SIZE );
    SS_OFF();
}

/**
 End of File
*/
