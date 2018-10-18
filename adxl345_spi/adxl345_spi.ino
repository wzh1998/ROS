//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

//Assign the Chip Select signal to pin 52.
int CS=52;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int16_t x,y,z;
byte buff;
void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  SPI.setClockDivider(42);  
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  delay(100);
  digitalWrite(CS, LOW);
  SPI.transfer(DATA_FORMAT);
  SPI.transfer(0x01);
  digitalWrite(CS, HIGH);
  delay(50);   
  digitalWrite(CS, LOW);
  SPI.transfer(POWER_CTL);
  SPI.transfer(0x08);
  digitalWrite(CS, HIGH);
  delay(50);   
}

void loop(){
  digitalWrite(CS, LOW);
  SPI.transfer(DATAX0);
  values[0] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);  
//  
  digitalWrite(CS, LOW);
  SPI.transfer(DATAX1);
  values[1] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);    
//  
  digitalWrite(CS, LOW);
  SPI.transfer(DATAY0);
  values[2] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);  
//  
  digitalWrite(CS, LOW);
  SPI.transfer(DATAY1);
  values[3] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);  
//
  digitalWrite(CS, LOW);
  SPI.transfer(DATAZ0);
  values[4] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);  
//
  digitalWrite(CS, LOW);
  SPI.transfer(DATAZ1);
  values[5] = SPI.transfer(0x00);
  digitalWrite(CS, HIGH);
  delay(50);   
//  
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = (((int)values[1]) <<8) | values[0];
  //The Y value is stored in values[2] and values[3].
  y = (((int)values[3]) <<8) | values[2];
  //The Z value is stored in values[4] and values[5].
  z = (((int)values[5]) <<8) | values[4]; 
  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);   
  delay(1000); 
}
