/*
 *Copyright: Copyright (c) 2018
 *Created on 2018-9-28  
 *Author:Zihao Wang
 *Version 1.0 
 *Title: motion_control_server
 *description: This code file is modified base on wiki-dfrobot. aimed to build the message transmission for ROS.
 *AN3461 is useful preference to understand how to calculate roll and pitch. When using ADXL345, pay attention
 *not to aligned with its x-axis pointing vertically upwards or downwards (Instability).
 */

#include <Wire.h>
#include <String.h>

#define DEVICE (0x53)      //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)
#define TO_SEND 42
 
const char STARTSIGN = 0xa5;
const char ENDSIGN = 0x5a;

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
byte buff2[TO_READ] ;        //6 bytes buffer for saving data read from the device
char str[512];              //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int x, y, z;                //three axis acceleration data
int x2, y2, z2;
float roll = 0.00, pitch = 0.00;   //Roll & Pitch are the angles which rotate by the axis X and y 
float roll2 = 0.00, pitch2 = 0.00;
//in the sequence of R(x-y-z).

unsigned char sendBuff[TO_SEND] = {0};

// typedef to change double to bytes for our further ROS information transmission.
union floatData{
  float num;
  unsigned char data[4]; 
}command_data1,command_data2,command_data3,command_data4,command_data5,command_data6,command_data7,command_data8,command_data9,command_data10;   // double to bytes union =_=

void setup() {
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
//  pinMode(4,OUTPUT);
//  pinMode(2,OUTPUT);

  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);

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
void RP_calculate(){
  double x_Buff = float(x);
  double y_Buff = float(y);
  double z_Buff = float(z);

  double x_Buff2 = float(x2);
  double y_Buff2 = float(y2);
  double z_Buff2 = float(z2);
  
  roll = atan2(y_Buff , z_Buff) * 57.3;
  pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;

  roll2 = atan2(y_Buff2 , z_Buff2) * 57.3;
  pitch2 = atan2((- x_Buff2) , sqrt(y_Buff2 * y_Buff2 + z_Buff2 * z_Buff2)) * 57.3;
  
  Serial.print("roll:");
  Serial.print(roll);
  Serial.print("pitch:");
  Serial.println(pitch);

  Serial.print("roll2:");
  Serial.print(roll2);
  Serial.print("pitch2:");
  Serial.println(pitch2);
  
}

// transform pitch/roll to ROS communication form, then send.
void motion_write() {
  RX = roll;
  RY = 0; // not finish
  RZ = pitch;
  RH = 0; // not finish
  
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

  sendBuff[0] = STARTSIGN;
  sendBuff[41] = ENDSIGN;
  
  for(int i = 0;i < 4;i++) {
    sendBuff[i + 1] = command_data1.data[i];
    sendBuff[i + 5] = command_data2.data[i];
    sendBuff[i + 9] = command_data3.data[i];
    sendBuff[i + 13] = command_data4.data[i];
    sendBuff[i + 17] = command_data5.data[i];
    sendBuff[i + 21] = command_data6.data[i];
    sendBuff[i + 25] = command_data7.data[i];
    sendBuff[i + 29] = command_data8.data[i];
    sendBuff[i + 33] = command_data9.data[i];
    sendBuff[i + 37] = command_data10.data[i];
  }
}
  
void loop() {
 
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345                                    //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!

                             //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];

  
  readFrom(DEVICE, regAddress, TO_READ, buff2); 

  
  x2 = (((int)buff2[1]) << 8) | buff2[0];     
  y2 = (((int)buff2[3])<< 8) | buff2[2];
  z2 = (((int)buff2[5]) << 8) | buff2[4];
      
  //we send the x y z values as a string to the serial port
  //  Serial.print("The acceleration info of x, y, z are:");
  //  sprintf(str, "%d %d %d", x, y, z);  
//  Serial.print(str);
//  Serial.write(10);
  //Roll & Pitch calculate
  RP_calculate();
  motion_write();
//  Serial.print("Roll:"); Serial.println( roll ); 
//  Serial.print("Pitch:"); Serial.println( pitch );
//  Serial.println("");
  //It appears that delay is needed in order not to clog the port
  delay(500);
}
