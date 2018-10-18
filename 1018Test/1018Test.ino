 /*
 *Copyright: Copyright (c) 2018
 *Created on 2018-9-28  
 *Author:Zihao Wang
 *Version 1.0 
 *Title: motion_control_server
 *description: This code file is modified base on wiki-dfrobot. aimed to build the message transmission for ROS.
 *AN3461 is useful preference to understand how to calculate roll and pitch. When using ADXL345, pay attention
 *not to aligned with its x-axis pointing vertically upwards or downwards (Instability).
 *
 *
 *Reference: @Bob Chen - E32-TTL-100
 */
 
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "E32-TTL-100.h"

/*
 need series a 4.7k Ohm resistor between .
 UNO/NANO(5V mode)                E32-TTL-100
    *--------*                      *------*
    | D7     | <------------------> | M0   |
    | D8     | <------------------> | M1   |
    | A0     | <------------------> | AUX  |
    | D10(Rx)| <---> 4.7k Ohm <---> | Tx   |
    | D11(Tx)| <---> 4.7k Ohm <---> | Rx   |
    *--------*                      *------*
*/
#define DEVICE (0x53)      //ADXL345 device address
#define DEVICE2 (0x1D)
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)
#define TO_SEND 46

#define M0_PIN  7
#define M1_PIN  8
#define AUX_PIN A0
#define SOFT_RX 10
#define SOFT_TX 11

#define JOINT1_LOW -40
#define JOINT1_HIGH 40
#define JOINT2_LOW 0
#define JOINT2_HIGH 50
#define JOINT3_LOW 0
#define JOINT3_HIGH 60
#define GRIBBER_LOW 30
#define GRIBBER_HIGH 139

#define ROLL_LOW -60
#define ROLL_HIGH 60
#define PITCH_LOW -60
#define PITCH_HIGH 60
#define ROLL2_LOW -60
#define ROLL2_HIGH 60
#define PITCH2_LOW -60
#define PITCH2_HIGH 60

const char ST_SIG = 0xff;
const char CHANNEL = 0x17;
const char STARTSIGN = 0xa5;
const char ENDSIGN = 0x5a;
const char NEWLINESIG = 0x0a;

const float STATE = 6;
const float AXIS = 0;

float RX;
float RY;
float RZ;
float RH;
const float ISGRAB = 0;
const float STARTVE = 0;
float ENDVE;
const float MAXVE = 0;

byte buff[TO_READ] ;        //6 bytes buffer for saving data read from the device
char str[512];              //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int x, y, z;                //three axis acceleration data
float roll = 0.00, pitch = 0.00;   //Roll & Pitch are the angles which rotate by the axis X and y 
float roll2 = 0.00, pitch2 = 0.00;
//in the sequence of R(x-y-z).
unsigned char sendBuff[TO_SEND] = {0};

SoftwareSerial softSerial(SOFT_RX, SOFT_TX);  // RX, TX

union floatData{
  float num;
  unsigned char data[4]; 
}command_data1,command_data2,command_data3,command_data4,command_data5,command_data6,command_data7,command_data8,command_data9,command_data10;   // double to bytes union =_=


//=== AUX ===========================================+
bool AUX_HL;
bool ReadAUX() {
  int val = analogRead(AUX_PIN);

  if(val<50)
  {
    AUX_HL = LOW;
  }else {
    AUX_HL = HIGH;
  }

  return AUX_HL;
}

//return default status
RET_STATUS WaitAUX_H()
{
  RET_STATUS STATUS = RET_SUCCESS;

  uint8_t cnt = 0;
  uint8_t data_buf[100], data_len;

  while((ReadAUX()==LOW) && (cnt++<TIME_OUT_CNT))
  {
    Serial.print(".");
    delay(100);
  }

  if(cnt==0) {}
  else if(cnt>=TIME_OUT_CNT) {
    STATUS = RET_TIMEOUT;
    Serial.println(" TimeOut");
  }
  else {
    Serial.println("waiting~");
  }

  return STATUS;
}
//=== AUX ===========================================-
//=== Mode Select ===================================+
bool chkModeSame(MODE_TYPE mode) {
  static MODE_TYPE pre_mode = MODE_INIT;

  if(pre_mode == mode)
  {
    //Serial.print("SwitchMode: (no need to switch) ");  Serial.println(mode, HEX);
    return true;
  }
  else {
    Serial.print("SwitchMode: from ");  Serial.print(pre_mode, HEX);  Serial.print(" to ");  Serial.println(mode, HEX);
    pre_mode = mode;
    return false;
  }
}

void SwitchMode(MODE_TYPE mode)
{
  if(!chkModeSame(mode))
  {
    WaitAUX_H();

    switch (mode) {
      case MODE_0_NORMAL:
        // Mode 0 | normal operation
        digitalWrite(M0_PIN, LOW);
        digitalWrite(M1_PIN, LOW);
        break;
      case MODE_1_WAKE_UP:
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M1_PIN, LOW);
        break;
      case MODE_2_POWER_SAVIN:
        digitalWrite(M0_PIN, LOW);
        digitalWrite(M1_PIN, HIGH);
        break;
      case MODE_3_SLEEP:
        // Mode 3 | Setting operation
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M1_PIN, HIGH);
        break;
      default:
        return ;
    }

    WaitAUX_H();
    delay(10);
  }
}
//=== Mode Select ===================================-
//=== Basic cmd =====================================+
void cleanUARTBuf() {
  bool IsNull = true;

  while (softSerial.available()) {
    IsNull = false;

    softSerial.read();
  }
}

void triple_cmd(SLEEP_MODE_CMD_TYPE Tcmd) {
  uint8_t CMD[3] = {Tcmd, Tcmd, Tcmd};
  softSerial.write(CMD, 3);
  delay(50);  //need ti check
}

RET_STATUS Module_info(uint8_t* pReadbuf, uint8_t buf_len) {
  RET_STATUS STATUS = RET_SUCCESS;
  uint8_t Readcnt, idx;

  Readcnt = softSerial.available();
  //Serial.print("softSerial.available(): ");  Serial.print(Readcnt);  Serial.println(" bytes.");
  if (Readcnt == buf_len)
  {
    for(idx=0;idx<buf_len;idx++)
    {
      *(pReadbuf+idx) = softSerial.read();
      Serial.print(" 0x");
      Serial.print(0xFF & *(pReadbuf+idx), HEX);    // print as an ASCII-encoded hexadecimal
    } Serial.println("");
  }
  else {
    STATUS = RET_DATA_SIZE_NOT_MATCH;
    Serial.print("  RET_DATA_SIZE_NOT_MATCH - Readcnt: ");  Serial.println(Readcnt);
    cleanUARTBuf();
  }

  return STATUS;
}
//=== Basic cmd =====================================-
//=== Sleep mode cmd ================================+
RET_STATUS Write_CFG_PDS(struct CFGstruct* pCFG)
{
  softSerial.write((uint8_t *)pCFG, 6);

  WaitAUX_H();
  delay(1200);  //need ti check

  return RET_SUCCESS;
}

RET_STATUS Read_CFG(struct CFGstruct* pCFG)
{
  RET_STATUS STATUS = RET_SUCCESS;

  //1. read UART buffer.
  cleanUARTBuf();

  //2. send CMD
  triple_cmd(R_CFG);

  //3. Receive configure
  STATUS = Module_info((uint8_t *)pCFG, sizeof(CFGstruct));
  if(STATUS == RET_SUCCESS)
  {
  Serial.print("  HEAD:     ");  Serial.println(pCFG->HEAD, HEX);
  Serial.print("  ADDH:     ");  Serial.println(pCFG->ADDH, HEX);
  Serial.print("  ADDL:     ");  Serial.println(pCFG->ADDL, HEX);

  Serial.print("  CHAN:     ");  Serial.println(pCFG->CHAN, HEX);
  }

  return STATUS;
}

RET_STATUS Read_module_version(struct MVerstruct* MVer)
{
  RET_STATUS STATUS = RET_SUCCESS;

  //1. read UART buffer.
  cleanUARTBuf();

  //2. send CMD
  triple_cmd(R_MODULE_VERSION);

  //3. Receive configure
  STATUS = Module_info((uint8_t *)MVer, sizeof(MVerstruct));
  if(STATUS == RET_SUCCESS)
  {
    Serial.print("  HEAD:     0x");  Serial.println(MVer->HEAD, HEX);
    Serial.print("  Model:    0x");  Serial.println(MVer->Model, HEX);
    Serial.print("  Version:  0x");  Serial.println(MVer->Version, HEX);
    Serial.print("  features: 0x");  Serial.println(MVer->features, HEX);
  }

  return RET_SUCCESS;
}

void Reset_module()
{
  triple_cmd(W_RESET_MODULE);

  WaitAUX_H();
  delay(1000);
}

RET_STATUS SleepModeCmd(uint8_t CMD, void* pBuff)
{
  RET_STATUS STATUS = RET_SUCCESS;

  Serial.print("SleepModeCmd: 0x");  Serial.println(CMD, HEX);
  WaitAUX_H();

  SwitchMode(MODE_3_SLEEP);

  switch (CMD)
  {
    case W_CFG_PWR_DWN_SAVE:
      STATUS = Write_CFG_PDS((struct CFGstruct* )pBuff);
      break;
    case R_CFG:
      STATUS = Read_CFG((struct CFGstruct* )pBuff);
      break;
    case W_CFG_PWR_DWN_LOSE:

      break;
    case R_MODULE_VERSION:
      Read_module_version((struct MVerstruct* )pBuff);
      break;
    case W_RESET_MODULE:
      Reset_module();
      break;

    default:
      return RET_INVALID_PARAM;
  }

  WaitAUX_H();
  return STATUS;
}
//=== Sleep mode cmd ================================-

RET_STATUS SettingModule(struct CFGstruct *pCFG)
{
  RET_STATUS STATUS = RET_SUCCESS;

#ifdef Device_A
  pCFG->ADDH = DEVICE_A_ADDR_H;
  pCFG->ADDL = DEVICE_A_ADDR_L;
#else
  pCFG->ADDH = DEVICE_B_ADDR_H;
  pCFG->ADDL = DEVICE_B_ADDR_L;
#endif

  pCFG->OPTION_bits.trsm_mode =TRSM_FP_MODE;
  pCFG->OPTION_bits.tsmt_pwr = TSMT_PWR_10DB;

  STATUS = SleepModeCmd(W_CFG_PWR_DWN_SAVE, (void* )pCFG);

  SleepModeCmd(W_RESET_MODULE, NULL);

  STATUS = SleepModeCmd(R_CFG, (void* )pCFG);

  return STATUS;
}


RET_STATUS SendMsg() {
  RET_STATUS STATUS = RET_SUCCESS;

  SwitchMode(MODE_0_NORMAL);

//  if(ReadAUX()!=HIGH)
//  {
//    return RET_NOT_IMPLEMENT;
//  }
//  delay(10);
//  if(ReadAUX()!=HIGH)
//  {
//    return RET_NOT_IMPLEMENT;
//  }
//
// 
  //Send format : ADDH ADDL CHAN DATA_0 DATA_1 DATA_2 ...
  // Broadcast mode
//    uint8_t SendBuf[6] = { 0xFF, 0xFF, 0x17, 0x00, 0x00, 0x3F}; //for A
//  softSerial.write(SendBuf, 6);
//  uint8_t SendBuf2[6] = { 0xFF, 0xFF, 0x17, 0x00, 0x00, 0x3F}; //for B
//  softSerial.write(SendBuf2, 6);
  softSerial.write(sendBuff, TO_SEND);

  return STATUS;
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//calculate the Roll&Pitch
void RP_calculate(int opcode){
  double x_Buff = float(x);
  double y_Buff = float(y);
  double z_Buff = float(z);
  if(opcode == 1) {
    roll = atan2(y_Buff , z_Buff) * 57.3;
    pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
  }
  else if(opcode == 2) {
    roll2 = atan2(y_Buff , z_Buff) * 57.3;
    pitch2 = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
  }
  
}
void mapFunc() {
  roll = map(roll, ROLL_LOW, ROLL_HIGH, JOINT1_LOW, JOINT1_HIGH);
  pitch = map(pitch, PITCH_LOW, PITCH_HIGH, JOINT3_LOW, JOINT3_HIGH);
  roll2 = map(roll2, ROLL2_LOW, ROLL2_HIGH, GRIBBER_LOW, GRIBBER_HIGH);
  pitch2 = map(pitch2, PITCH2_LOW, PITCH2_HIGH, JOINT2_LOW, JOINT2_HIGH);
}
// transform pitch/roll to ROS communication form, then send.
void motionDataSend() {
//  roll
  mapFunc();
  RX = roll;
  RY = pitch2; 
  RZ = pitch; 
  RH = roll2; 
  
  command_data1.num = STATE;
  command_data2.num = AXIS;
  command_data3.num = RX;
  command_data4.num = RY;
  command_data5.num = RZ;
  command_data6.num = RH;
  command_data7.num = ISGRAB;
  command_data8.num = STARTVE;
  command_data9.num = ENDVE;
  command_data10.num = MAXVE;

  sendBuff[0] = ST_SIG;
  sendBuff[1] = ST_SIG;
  sendBuff[2] = CHANNEL;
  sendBuff[3] = STARTSIGN;
  sendBuff[44] = ENDSIGN;
  sendBuff[45] = NEWLINESIG; // new line sig
  for(int i = 0;i < 4;i++) {
    sendBuff[i + 4] = command_data1.data[i];
    sendBuff[i + 8] = command_data2.data[i];
    sendBuff[i + 12] = command_data3.data[i];
    sendBuff[i + 16] = command_data4.data[i];
    sendBuff[i + 20] = command_data5.data[i];
    sendBuff[i + 24] = command_data6.data[i];
    sendBuff[i + 28] = command_data7.data[i];
    sendBuff[i + 32] = command_data8.data[i];
    sendBuff[i + 36] = command_data9.data[i];
    sendBuff[i + 40] = command_data10.data[i];
  }
  //debug
  Serial.print("\nroll:");
  Serial.println(roll);
  Serial.print("pitch:");
  Serial.println(pitch);

  Serial.print("roll2:");
  Serial.println(roll2);
  Serial.print("pitch2:");
  Serial.println(pitch2);
  for(int i = 0;i < TO_SEND;i++) Serial.print(sendBuff[i],HEX);
  //sendMsg
//  if(SendMsg() == RET_SUCCESS) Serial.println("Msg Transmited successful");
  
}



void setup() {
  RET_STATUS STATUS = RET_SUCCESS;
  struct CFGstruct CFG;
  struct MVerstruct MVer;

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);

  Wire.begin();
  softSerial.begin(9600);
  Serial.begin(9600);
    
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);

//  writeTo(DEVICE2, 0x2D,0);
//  writeTo(DEVICE2, 0x2D,16);
//  writeTo(DEVICE2, 0x2D,8);
  
#ifdef Device_A
  Serial.println("[10-A] ");
#else
  Serial.println("[10-B] ");
#endif

  STATUS = SleepModeCmd(R_CFG, (void* )&CFG);
  STATUS = SettingModule(&CFG);

  STATUS = SleepModeCmd(R_MODULE_VERSION, (void* )&MVer);

  // switch Mode 0 to normal operation
  SwitchMode(MODE_0_NORMAL);

  //self-check initialization.
  WaitAUX_H();
  delay(10);
  
  if(STATUS == RET_SUCCESS)
    Serial.println("Setup init OK!!");
}


void loop() {
  readFrom(DEVICE, regAddress, TO_READ, buff);
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  RP_calculate(1);
  
  readFrom(DEVICE2, regAddress, TO_READ, buff);
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  RP_calculate(2);

 
//  if(-60.0 <= roll && roll <= 60.0 && 20.0 <= pitch && pitch <= 60 ) motionDataSend();
  motionDataSend();
  delay(800);
}
