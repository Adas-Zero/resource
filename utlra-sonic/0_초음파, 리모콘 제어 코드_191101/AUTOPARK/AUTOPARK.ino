

#include <MsTimer2.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

#define CAN_BAUD (CAN_500KBPS)
#define CAN_INIT_RETRY_DELAY 50
#define CAN_STANDARD 0
#define CAN_LENGTH 8
/////////////////////////////////////////////////////////////////////////////////////
#define UP_SELECT 4
#define DOWN_SELECT 5
#define LEFT_SELECT A4
#define RIGHT_SELECT A5
#define ENCODER_A A0
#define ENCODER_B A1
#define ENCODER_Z A2
#define ENCODER_Z1 A3
#define MCP2515_INT 2
unsigned char can_txdata[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char len = 0;
unsigned char can_rxdata[8];
unsigned char can_rxdata_id=0;
unsigned char input1=0;
unsigned char input2=0;
unsigned char input3=0;
unsigned char input4=0;

bool b_can_rx_flag=0;
bool b_can_tx_flag=0;
int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN); // Set CS pin
/////////////////////////////////////


char cTemp; // complete command
String sCommand = "";

void Serial_command_func() 
{
  sCommand = "";
  while(Serial.available())
  {
    cTemp = Serial.read();
    sCommand.concat(cTemp);
  }
  if(sCommand != "")
  {
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_can()
{
  while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    delay( CAN_INIT_RETRY_DELAY );
    Serial.println("CAN - init fail");
    Serial.println(" Init CAN - again");
    delay(10);
  }
  Serial.println("CAN BUS Shield init ok!");
  attachInterrupt(digitalPinToInterrupt(2), MCP2515, FALLING); // start interrupt
  CAN.init_Mask(0, 0, 0x3FF);                         // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3FF);
  CAN.init_Filt(0, 0, 0xCB);                          // there are 6 filter in mcp2515
  CAN.init_Filt(1, 0, 0x55);                          // there are 6 filter in mcp2515
  CAN.init_Filt(2, 0, 0x64);                          // there are 6 filter in mcp2515
  CAN.init_Filt(3, 0, 0x90);                          // there are 6 filter in mcp2515  
  CAN.init_Filt(4, 0, 0x1a);                          // there are 6 filter in mcp2515
  CAN.init_Filt(5, 0, 0x91);                          // there are 6 filter in mcp2515
  Serial.println("CAN filter - init ok!");
}

void init_serial( void )
{
  Serial.begin( 115200 );
  while (!Serial);
  Serial.println( "init_serial: ok" );
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
}

void init_devices_io( void )
{
  pinMode( UP_SELECT, OUTPUT );
  pinMode( DOWN_SELECT, OUTPUT );
  pinMode( LEFT_SELECT, OUTPUT );
  pinMode( RIGHT_SELECT, OUTPUT );
  pinMode( ENCODER_A, INPUT );
  pinMode( ENCODER_B, INPUT );
  pinMode( ENCODER_Z, INPUT );
  pinMode( ENCODER_Z1, INPUT );
  pinMode( MCP2515_INT, INPUT );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
void mcp_can_recieve() {
    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, can_rxdata);    // read data,  len: data length, buf: data buf
        can_rxdata_id = CAN.getCanId();
        input1=can_rxdata[0];
        input2=can_rxdata[1];
        input3=can_rxdata[2];
        input4=can_rxdata[3];
        b_can_rx_flag=1;
        Serial.println("can_rx");
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mcp_can_tx(char ID_SELECT) {
    b_can_tx_flag=0;
    can_txdata[0]= 1;
    can_txdata[1]= 2;
    can_txdata[2]= 3;
    can_txdata[3]= 4;
    can_txdata[4]= 5;
    can_txdata[5]= 6;
    can_txdata[6]= 7;
    can_txdata[7]= 8;
    CAN.sendMsgBuf(ID_SELECT, CAN_STANDARD, CAN_LENGTH, can_txdata );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool b_mcp2515_int=0;
void MCP2515(void) 
{
  b_mcp2515_int = 1;
}

bool b_timer1flag=0,flag=0;
void timer1()
{  
  b_timer1flag=1;
  if(input1>0) {input1--;}
  else {input1=0;}
  if(input2>0) {input2--; }
  else {input2=0;}
  if(input3>0) {input3--;}
  else {input3=0;}
  if(input4>0) {input4--;}
  else {input4=0;}
}
void setup() {
  // put your setup code here, to run once:
  init_devices_io();
  init_serial();
  init_can();
  MsTimer2::set(10, timer1); // 10ms period
  MsTimer2::start();
}
void loop() {
  if(b_mcp2515_int)
  {
    b_mcp2515_int = 0;
    mcp_can_recieve();
    CAN.Receiveflagclear(); //mcp2515_setRegister(MCP_CANINTF,0x00); //fix mcp_can.cpp&.h
  }
//  Serial_command_func();
  if(b_timer1flag)
  {
    b_timer1flag=0;
    Serial.print(input1,HEX);
    Serial.print(input2,HEX);
    Serial.print(input3,HEX);
    Serial.print(input4,HEX);
    Serial.println("");
    
    if(input1>0)    {digitalWrite( UP_SELECT, 1 );}
    else {digitalWrite( UP_SELECT, 0 );}
    if(input2>0)    {digitalWrite( DOWN_SELECT, 1 );}
    else {digitalWrite( DOWN_SELECT, 0 );    }
    if(input3>0)    {digitalWrite( LEFT_SELECT, 1 );}
    else {digitalWrite( LEFT_SELECT, 0 );   }
    if(input4>0)    {digitalWrite( RIGHT_SELECT, 1 );}
    else {digitalWrite( RIGHT_SELECT, 0 );    }
  }
  b_can_tx_flag=1;
  if(b_can_tx_flag>10)
  {
    b_can_tx_flag=0;
    mcp_can_tx(0xA1);
  }
  pin_read();
}

void pin_read()
{
  input1=0;  input2=0;  input3=0;  input4=0;
  input1=input1|digitalRead(ENCODER_A);
  input2=input2|digitalRead(ENCODER_B)<<1;
  input3=input3|digitalRead(ENCODER_Z)<<2;
  input4=input4|digitalRead(ENCODER_Z1)<<3;
}

