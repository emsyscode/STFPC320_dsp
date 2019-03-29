/*
The code is not clean and has some functions that were used for diagnostic purposes only.
As I usually say, the code can be improved, but I made it as simple as possible so that 
anyone can make changes and visualize their results.
We do not use external libraries in order to be totally owners of the expected result 
when calling a function.
I have not had any concern with the timers, so the code should be regarded as a starting 
point for the interface with the board under observation.
*/

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
//sign & c are initialized inside inline asm code
//register uint8_t sign, c;
#pragma GCC diagnostic pop
//#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include <Arduino.h>
#include "SoftwareI2C.h" // This library is only to run with find slaves on BUS scl&sca.
SoftwareI2C softwarei2c;  
#define sda 7
#define scl 8
#define addressWR  0x52
#define addressRD  0x53
// add this to the top of your sketch
#define NOP __asm__ __volatile__ ("nop\n\t")
// and then use it in code as follows
// NOP; // delay 62.5ns on a 16MHz AtMega
//To read the configuration data from STFPC320, 0x71 command. After this, the STFPC320
//will output a maximum of 14 bytes with the configuration data.
//Up to a maximum of 14-bytes are sent from MSB to LSB as configuration data. 
//Read RTC register command: 0x73
//IR data read command (0x62) on STFPC320
//#define RTC_I2C_ADDRESS 0x68  // This is address of I2C to module of RTC arduino.
#define I2C_wrAddr 0x52  // This is address of clock on board with STFPC320
#define I2C_rdAddr 0x53  // This is address of clock on board with STFPC320
#define I2C_rdRTC 0x73  // Read of values on RTC, 01-0F
#define I2C_rdCONF 0x71 // Read data of active configuration of STFPC320
// Note: (Write: 0x52H and Read: 0x53H) Board with STFPC320 !!!!
int flag = 0;
int ack;
char value=0;  // Note: This need be only char, becaus support the value of "-1" to compare the decrement of "value" variable!!!
unsigned char positionArray=0;
unsigned char arrAdjusteTime[8]={0x05, 0x03, 0x09, 0x05, 0x03, 0x02, 0x00, 0x00};
unsigned char arrTime[22];
unsigned char arrRC[4];
unsigned char arrKeys[8];
unsigned char arrNumbers[10] = {0x3F, 0x06, 0x5B, 0x4F,0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F}; // This is a table to the 7 segments digits
unsigned char arrGrid[6] = {0xD0, 0xD3, 0xD9, 0xDC}; // The address of grids is: 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100
//#define lcd_data_pin 0xA0//p2 port
//#define output 0x80//p0 port
//#define rs P3_0
//#define rw P3_1
//#define en P3_6
void nop(void);
unsigned char charIRed=0;
unsigned char reead, write2, readed;
unsigned char rd,wrt,wrt2,i,j;
unsigned int temp=0;
unsigned int clkHigh750nSec=750; // must be greater than 600 nano Sec //  to debug the timers of SDC & SCL // 1.3 uSecs until start new communication at least!
unsigned int sclLow1300nSec=1300;  // minimo 1.3uSeconds
unsigned int sdaLow110nSec=110; // time between data I'm let the time of uP communication set
unsigned int sdaHigh110nSec=110;
unsigned char counter=0;
unsigned char ssDU=30;
unsigned char mmDU=30;
unsigned char hhDU=12;
unsigned char WD=01;
unsigned char digitA=0;
unsigned char digitB=0;
 int flagSetAlarm=0;
 int flagSetTime=0;
 int flagLed=0;
 int flagLed3=0;
 int flagLed2=0;
 int flagLed1=0;
 int flagConf=0; // Flag to active the set the display of time or the numbers!
 int flagWriteSS =0;
 int flagWriteMM =0;
 int flagWriteHH =0;
 int flagDE25=0;  // Led red near of connector BE1
/******************* RTC *************************/
/****** convertion of DEC to BCD *****************/ 
unsigned char dec2bcd(unsigned char val){
  return( (val/10*16) + (val%10) );
}
/*************************************************/
//
/****** convertion of BCD to DEC *****************/
// Convert binary coded decimal to normal decimal numbers
unsigned char bcd2dec(unsigned char val){
  return( (val/16*10) + (val%16) );
}
/*************************************************/
void adjustRTC(void){
// set the initial time here:
// RTC secds, minu, hrs, day, date, month, year
// setRTCtime(30,35,20,01,03,02,19);  // setting of initial values of clock
}
/*****************************************************************************/
void tst(void)
{
int a=0;
int x = 0;    // initialize all bits as 0
x = x | (1<<10);   // this sets the 10th bit (counting from the LSB, with a 0-based index) to 1.
a = x & (1<<22);   // this gets the value of the 22nd bit
x = x ^ (1<<12);   // this flips the 12th bit
}
void noOperand(){
  asm volatile (
    "sbi %0, %1 \n" //pinMode(13, OUTPUT);
    : : "I" (_SFR_IO_ADDR(DDRB)), "I" (DDB5)
  );
}
void nop(unsigned int multiple){
 // and then use it in code as follows
 for (int i =0; i< multiple; i++){
  NOP; // delay 62.5ns on a 16MHz AtMega
 }
}
//*********************myDelay****************************/
void myDelay(unsigned int count)
{
  int i,j;
  for(i=0;i<count;i++)
    for(j=0;j<1275;j++);
}
//******************** ZONE of I2C communicaations *********************//
//*********************START****************************/
void start(){    //start condition
  // Time need waite until start new communication. ( 1.3 uSec) but the delay cant be done here,I'm do it on stp
   // The start condition is: Downing SDA while SCL is HIGH
   // The status of SCL is HIGH because the Stop let it in this value!
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  digitalWrite(sda,LOW);
  nop(10);
}
//*********************STOP****************************/
void stp(){     //stp condition
  digitalWrite(sda, LOW);
  nop(10);
  digitalWrite(scl, HIGH);  // The SCL stay every time on upper value, this mean must finish allways upper!
  nop(10);
  digitalWrite(sda, HIGH); // The stop condition is: Rise SDA while SCL is HIGH
  nop(10);
  delayMicroseconds(2);  // time of free bus before a new start communication(1.3 uSec)
}
//*********************Slave AKNOWLEDGE****************************/
void readAcknowledge(){    //acknowledge condition
  digitalWrite(scl, LOW);
  nop(10);
  pinMode(sda, INPUT);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(20);
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  pinMode(sda, OUTPUT);
  nop(10);
  
}
//*********************Master AKNOWLEDGE****************************/
void sendAcknowledge(){    //acknowledge condition
  // This is used when the Master is reading and need send a ACK by each byte received drom Slave!
  // SDA must be as a output pin to force bus sda go down!
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, LOW); // Here is the master wich send a ack to the slave!!!
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
}
void noAcknowledge(){    //acknowledge condition
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  ack=digitalRead(PD7);         //reading acknowledge
  digitalWrite(scl, LOW);
  nop(10);
}
//******************** END ZONE of I2C communicaations *********************//

//******************* ZONE of communication to the Slavesent mode ****************************//
void send_byte(unsigned char Bits){  //send byte serially
// On the send Byte, the SDA must be High to allow the 
//receiver send a low pulse in sda(like a pull-up status)
unsigned char data = 170; //value to transmit, binary 10101010
unsigned char mask = 1; //our bitmask
data=Bits;
digitalWrite(scl, LOW);   // Must stay low value after 8 bits
nop(10);
pinMode(sda, OUTPUT);
// The start let the SCL in low value
          for (mask = 10000000; mask>0; mask >>= 1) { //iterate through bit mask
                  if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(sda, HIGH);
                    nop(10);
                    //Serial.print("1");
                  }
                  else{ //if bitwise and resolves to false
                    digitalWrite(sda, LOW);
                    nop(10);
                    //Serial.print("0");
                  }
                  
            //Note: The change of data must occurr while the SCL is LOW, only after the pulse of SCL take place!
            digitalWrite(scl, HIGH);  // Generate a pulse to validation of data on bus.
            nop(10);
            digitalWrite(scl, LOW);   // Must stay low value after 8 bits
            nop(10);
          }
        pinMode(sda, INPUT);
}
//
unsigned char read_byte(){     //reading from EEPROM serially
 unsigned int i;
 int val = 0;      // variable to store the read value
 reead=0;
 digitalWrite(scl, LOW);
 pinMode(sda, INPUT);
 nop(30);
        for(i=0;i<8;i++){
          reead=reead<<1;
          digitalWrite(scl, HIGH);
          nop(50);
          val = digitalRead(PD7);
            if(val == 1){
            reead++;}
       digitalWrite(scl, LOW);
       nop(50);   
        }
 pinMode(sda, OUTPUT);
 nop(30);
  //Serial.print(" reead: ");  // This print lines do a delay to big... use it only to debug!
  //Serial.println(reead, BIN);// Only to debug
  return reead;       //Returns 8 bit data here
}
//
void readBIT(void){
  String variable = "PORTD";
  if ( variable == "PORTD" )
  {
    Serial.println(bitRead(PORTD,3));
  }
}
void save(){   //save in EEPROM
  start();
  send_byte(0xA0);            //device address
  readAcknowledge();
  send_byte(0x00);            //word address
  readAcknowledge();
  send_byte(5);               //send data
  readAcknowledge();
  send_byte(65);
  readAcknowledge();
  stp();
  if(ack=0)
  {
    digitalWrite(13, HIGH);
    myDelay(100);
    digitalWrite(13, LOW);
    myDelay(100);
  }
  else
  digitalWrite(13, HIGH);
  readAcknowledge();
}
//**********************************************************/
void Read(unsigned char *second,
          unsigned char *minute,
          unsigned char *hour,
          unsigned char *dayOfWeek,
          unsigned char *dayOfMonth,
          unsigned char *month,
          unsigned char *year){
  start();
  send_byte(I2C_rdRTC);// send_byte(I2C_rdAddr); //I2C_rdRTC
  readAcknowledge();
  send_byte(0x01);  // The first Byte on STFPC is reserved!
  readAcknowledge();
  start();
  send_byte(I2C_rdRTC);// send_byte(I2C_rdAddr);           //device address
  readAcknowledge();
  *second=bcd2dec(read_byte());
  readAcknowledge();
  *minute=bcd2dec(read_byte());
  readAcknowledge();
  *hour=bcd2dec(read_byte());
  readAcknowledge();
  *dayOfWeek=bcd2dec(read_byte());
  readAcknowledge();
  *dayOfMonth=bcd2dec(read_byte());
  readAcknowledge();
  *month=bcd2dec(read_byte());
  readAcknowledge();
  *year=bcd2dec(read_byte());
  noAcknowledge();
  stp();
  //
}
void adjustTime(){
  send_byte(0b00111111);   // number 0
  readAcknowledge();
  send_byte(0b00000110);   // Number 1
  readAcknowledge();
  send_byte(0b01011011);   // number 2
  readAcknowledge();
  send_byte(0b01001111);   // number 3
  readAcknowledge();
  send_byte(0b01100110);   // number 4
  readAcknowledge();
  send_byte(0b01101101);   // Number 5
  readAcknowledge();
  send_byte(0b01111101);   // number 6
  readAcknowledge();
  send_byte(0b00000111);   // number 7
  readAcknowledge();
  send_byte(0b01111111);   // number 8
  readAcknowledge();
  send_byte(0b01101111);   // number 9
  readAcknowledge();
}
/**************************** Test of the 4 displays count from 0-9 each *********************************/
void tstDisplay(void){
 unsigned char nGrid=0;
 unsigned char nNumber=0;
 
  for ( unsigned char i=0; i< 16; i=i+3){ // Note: Each grid have 3 bytes to address it(better say 2&1/2) The grid of 2 points is active also(1 more)!
    nGrid= (0xD0 | i);
    for (unsigned char s=0; s< 10; s++){
       start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(nGrid);   //  
            readAcknowledge();
            send_byte(arrNumbers[s]);  // 
            readAcknowledge();
       stp();
            delay(100);
    }
  } 
}
void writeSS(void){
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010000);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01101101);   // Number 5
            readAcknowledge();
         stp();
         start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010011);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01101101);   // Number 5
            readAcknowledge();
         stp();
}
void writeMM(void){
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010000);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00110111);   // M
            readAcknowledge();
         stp();
         start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010011);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00110111);   // M
            readAcknowledge();
         stp();
}
void writeHH(void){
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010000);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01110110);   // H
            readAcknowledge();
         stp();
         start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010011);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01110110);   // H
            readAcknowledge();
         stp();
}
void swapLedDE25(void){
  if (flagDE25==0){
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010110);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b11111111);  // 
            readAcknowledge();
            send_byte(0b11111111);  //
            readAcknowledge();
            
          stp();
          flagDE25=1;
  }
  else{
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010110);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00000000);  // 
            readAcknowledge();
            send_byte(0b00000000);  //
            readAcknowledge();
            
          stp();
          flagDE25=0;
  }
}
void write1234(void){            
//This function let display on normal ON and write 1234 to display.
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010000);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00111111);  // number 0
            readAcknowledge();
            stp();
            delay(500);
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010011);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00000110);   // Number 1
            readAcknowledge();
            stp();
            delay(500);
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010110);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b00000000);  // First byte of 3 digit
            readAcknowledge();
            stp();
            delay(500);
            
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010111);
            readAcknowledge();
            send_byte(0b00000100);  // Second byte of 3 digit(I'm want active the two point in display, but not work! Check it!
            readAcknowledge();
            stp();
            delay(500);
            
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11011001);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01011011);   // number 2
            readAcknowledge();
            stp();
            delay(500);
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11011100);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(0b01001111);   // number 3
            readAcknowledge();
            stp();
            delay(500);
}
void led1on(){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001110);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
            stp();
            delay(300); 
}
void led2on(){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001101);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
            stp();
            delay(300); 
}
void led3on(){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001011);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
            stp();
            delay(300);
}
void ledsOff(){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001111);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
            stp();
}
void led3Swap(void){
  if (flagLed3==0){
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001011);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
           stp();
           flagLed3=1;
  }   
  else{
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001111);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
           stp();
           flagLed3=0;
  }
}
void led2Swap(void){
  if (flagLed2==0){
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001101);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
           stp();
           flagLed2=1;
  }   
  else{
          start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00001111);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
           stp();
           flagLed2=0;
  }
}
void led1Swap(void){
      if (flagLed1 == 0){
       start();
                send_byte(addressWR);  // slave with write active 
                readAcknowledge();
                send_byte(0b01000011);  //Data Setting Command word fix to write LED
                readAcknowledge();
                send_byte(0b00001110);  //xxxx4321 x don't care, led brigth when is set to "0"
                readAcknowledge();
              stp();
              flagLed1=1;
      }
      else{       
              start();
                send_byte(addressWR);  // slave with write active 
                readAcknowledge();
                send_byte(0b01000011);  //Data Setting Command word fix to write LED
                readAcknowledge();
                send_byte(0b00001111);  //xxxx4321 x don't care, led brigth when is set to "0"
                readAcknowledge();
              stp();
              flagLed1=0;
      }
}
/**********************  Control LED swithc ON/OFF  ***************************/
void OnOffLed(){
          //Command to control status LED
          //Data is written to the LED port by a write command, starting from the most significant bit of the port. 
          //When a bit of this port is set to 0, the corresponding LED lights up; when the bit is set to a 1, the LED turns off. 
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000011);  //Data Setting Command word fix to write LED
            readAcknowledge();
            send_byte(0b00000000);  //xxxx4321 x don't care, led brigth when is set to "0"
            noAcknowledge();
            stp();
            delay(100);
}
void initSTFPC320(){
  // Note: On the STFPC320 the action of write use allways the "acknowledge".
  // On the read the last write is follwed by the "noAcknowledge".
/*
After the proper power-up sequence, an example for configuration of the STFPC320 is given below: 
1. Configuring display & RC 
  0x09 = RTC display & enables the guard timer to issue STBY 
  0xAA = raw format, RC-5 protocol, 10s for initial guard time value 
  0x0A = digit 1 will show the Hour MSB, 11 digits/17 segments display 
2. Configure RTC segments 
  0xA4 = display control command with RTC segments configuration 
  0x10, 0x32, 0x54, 0x76, 0xAB, 0xEE = 6 bytes used to map the
  segment locations for RTC 
  0x20 = format of RTC (separator a part of same digit, HH: MM: SS 
  format with no AM/PM) 
3. Configure front panel keys as wake-up keys (hotkeys) 
  0xA2 = hotkey setting  command for front panel keys. Subsequent 3 bytes
  are used to configure the desired front panel keys to be wake-up keys as 
  described in Section Section : Thus,  if the a segment is located on 
  segment 1, the bits b3-b0 are “0000”. If b segment
  is located on segment 2, the bits b3-b0 are “0001” and so on. on page 39. 
4. Configure RC keys as wake-up keys (hotkeys) 
  0xA1 = hotkey setting command for  RC keys. For one hot key configuration
  from one RC device address, 2 bytes are sent (1st byte is RC address and 2
  byte is hotkey command as described in Section 7.3 on page 38.  
*/
    start();
    send_byte(0x09);
    readAcknowledge();
    send_byte(0xAA);
    readAcknowledge();
    send_byte(0x0A);
    readAcknowledge();
    send_byte(0xA4); readAcknowledge(); 
    send_byte(0x10); readAcknowledge();
    send_byte(0x32); readAcknowledge();
    send_byte(0x54); readAcknowledge();
    send_byte(0x76); readAcknowledge();
    send_byte(0xAB); readAcknowledge();
    send_byte(0xEE); readAcknowledge();
    send_byte(0x20);
    readAcknowledge();
    send_byte(0xA2);
    readAcknowledge();
    send_byte(0xA1);
    noAcknowledge();
    stp();
}
void confDSP(void){
// Here we do the configuration of format HH:MM:SS or HH:MM and segments position
    start();
    send_byte(addressWR);  // slave with write active 
    readAcknowledge();
    send_byte(0xA4); readAcknowledge();  // Start configuration of DSP segments 
    send_byte(0x10); readAcknowledge();
    send_byte(0x32); readAcknowledge();
    send_byte(0x54); readAcknowledge();
    send_byte(0x76); readAcknowledge();
    send_byte(0xAB); readAcknowledge();
    send_byte(0xEE); readAcknowledge();
    send_byte(0x40); noAcknowledge(); // Byte 7 to set the DSP //Important the bit 6 at value 1 define a digit between hh&mm and mm&ss. Bit 5 to 1 is HH:MM:SS, 0x20 set 12H with AM/PM
    stp();
}
void setTimer(void){
  /*************************  Set of Sec, Mins, Hours, Day ...**********************************/
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000000);   //  The bit B2 is 1 to fix address and 0 to auto-increment address
            noAcknowledge();
            stp();
            
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11000001);   // 0x01 address register of RTC 
            readAcknowledge();
            send_byte(0b00000000);  // Seconds 
            readAcknowledge();
            send_byte(0b01011001);  // min 59
            readAcknowledge();
            send_byte(0b00100011);  // hrs 23
            readAcknowledge();
            send_byte(0b11110000);  // 4 bits on left is from RS3-RS0 set of pulse to external, Set day 
            noAcknowledge();
            stp();
}
 /*************************  Set of Sec, Mins, Hors, Day ...**********************************/
void setAlarms(void){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000000);   //  The bit B2 is 1 to fix address and 0 to auto-increment address
            noAcknowledge();
            stp();
            
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11001100);   // 0x0C address register of RTC 
            readAcknowledge();
            send_byte(0b00000000);  // Hrs
            readAcknowledge();
            send_byte(0b01000000);  // min 59
            readAcknowledge();
            send_byte(0b00100011);  // hrs 23
            readAcknowledge();
            send_byte(0b11110000);  // 
            noAcknowledge();
            stp();
}
void readFlags(void){
  int t=0;
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11001100);  // 0x04 address register of RTC 
            readAcknowledge();
            start();
            send_byte(addressRD);  // 
            readAcknowledge();
                  for( t=0; t< 4; t++){
                   arrTime[t]=read_byte();
                  noAcknowledge();
                  }
            stp();
            delay(1);
                for (int n=0; n< 4; n++){
                Serial.println(arrTime[n],HEX);
                } 
}
 /************************ configuration to get frequency on IN_SQRT ******************/
void confHz(void){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11000100);  // 0x04 address register of RTC 
            readAcknowledge();
            send_byte(0b11110000);  // RS3,RS2,RS1,RS0 as deffine of freq ... 1111xxxx=1Hz, 0001xxxx=32768.
            readAcknowledge();
            stp();
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11001010);  // 0x0A address register of RTC
            readAcknowledge();
            send_byte(0b11100000);  // AFE, SQWE, 1, 10M, ...
            readAcknowledge();
            stp();
}
void confTwoPoints(void){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11010110);  // 0x16 memory position 16(Third digit ":" points)
            readAcknowledge();
            send_byte(0b00000000);  // 
            readAcknowledge();
            send_byte(0b00000100);  // 
            readAcknowledge();
            stp();
}
void confDisplayClock(void){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b00001001);  // Configuration mode setting command ((00 0 1 to normal display ... b0 enable timer guard stby) see page 33
            readAcknowledge();
            send_byte(0b00101000);  // First byte format after configuration byte  (RC5 =00101000;Sony=01011000; and 8 secds of guard to stby)...
            readAcknowledge();
            send_byte(0b00001011);  // Second byte format after configuration type (First digit on RTC=0000 "default") 12 digits 16 segments
            readAcknowledge();
            stp();
}
void confDisplayNormal(void){
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b00010001);  // Configuration mode setting command ((00 0 1 to normal display ... b0 enable timer guard stby) see page 33
            readAcknowledge();
            send_byte(0b00101000);  // First byte format after configuration byte  (RC5 =00101000;Sony=01011000; and 8 secds of guard to stby)...
            readAcknowledge();
            send_byte(0b00001011);  // Second byte format after configuration type (First digit on RTC=0000 "default") 12 digits 16 segments
            readAcknowledge();
            stp();
}
void activeKeys(void){
  // This is the configuration to allow Read Keys...
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            //send_byte(0b10001000);  //B5=0 and B3=1 display ON and scan keys OFF
            //readAcknowledge();
            send_byte(0b10001101); // B5=0 and B3=0 display off and scan keys on, B2-B0 set pulse display 1/16 to 14/16 (also dimmer diode DE25)
            readAcknowledge();
            send_byte(0b10000000); // B5=0 and B3=1 display OFF and scan keys ON
            noAcknowledge();
        stp();
         //
        readKeys();
        //
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b10001000);  //B5=0 and B3=1 display ON and scan keys OFF
            readAcknowledge();
            send_byte(0b10001101); // B5=0 and B3=0 display off and scan keys on, B2-B0 set pulse display 1/16 to 14/16 (also dimmer diode DE25)
            readAcknowledge();
            //send_byte(0b10000000); // B5=0 and B3=1 display OFF and scan keys ON
            //noAcknowledge();
        stp(); 
}
 /**************************Conf DSP and Keyboard  ************************************/
void confDisplayAndKey(void){
 // This is the configuration to allow write on DSP... 
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            //send_byte(0b10001000);  //B5=0 and B3=1 display ON and scan keys OFF
            //readAcknowledge();
            send_byte(0b10001101); // B5=0 and B3=0 display off and scan keys on, B2-B0 set pulse display 1/16 to 14/16 (also dimmer diode DE25)
            readAcknowledge();
            send_byte(0b10000000); // B5=0 and B3=1 display OFF and scan keys ON
            noAcknowledge();
            stp(); 
            
            start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b01000000);   //  The bit B2 is 1 to fix address and 0 to auto-increment address
            readAcknowledge();
            stp();
}
/**************************** Set time to the STFPC320 **********************************/
void setTime(char ss, char mm, char hh, char weekDay){
  // Congigure te time and day to the module DS3231 
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b01000000);   //  The bit B2 is 1 to fix address and 0 to auto-increment address
      readAcknowledge();
    start();
    //Note: The write mode finish use allways the Acknowledge sended by the SLAVE!
      send_byte(addressWR);// Address of slave RTC(On the datasheet they refer 0x68 and on the scan BUS, slave give-me as address 0x57 & 0x68!!!
      readAcknowledge();
      send_byte(0b11000001); // // Address 0x01 is the position of memory of RTC to keep seconds at STFPC320 // Pay attention to bits weight 7&6!!!
      readAcknowledge();
      send_byte(ss);//Seconds 00-59 
      readAcknowledge();
      send_byte(mm);//Minuts 00-59 
      readAcknowledge();
      send_byte(hh);//Hours & mode 12/24(AM/PM) 00-23 
      readAcknowledge();
      send_byte(weekDay);//Day
      readAcknowledge();     
    stp();
}
/*********************************** Read time from the STFPC320 *********************************/
void readTime() {
 //
start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b01000000);   //  The bit B2 is 1 to fix address and 0 to auto-increment address
      readAcknowledge();
    stp();
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11000001);   //  Define address of RTC memory, start on 0x01
      readAcknowledge();
    stp();
    
    start();
     send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0x73);  // This command define the increment mode, also bits 5&4 to 1's and bits 1&0 to read RTC mode
      readAcknowledge();
      start();
      send_byte(addressRD);  // slave with write active and command read mode!
      readAcknowledge();
        // If I use a read command without RAM address, then finish the last read with a noAcknwoledge!!! ONLY ON THIS READ FORMAT!!!
            for(int s=0; s<15; s++){  // start read from position of RTC as 0x01. (On the DS3231, start from 0x00)
               arrTime[s]=read_byte();
                if (s==14){
                noAcknowledge();
                }
                else{
                sendAcknowledge();
                }
            }
    stp();
        for (int n=0; n< 15; n++){
          Serial.print(arrTime[n],HEX); Serial.print(", ");
        }
        Serial.println();
 }
/************************** Show time on the console of arduino ***********************/
void showTime(void){
  Serial.print(arrTime[2],HEX);Serial.print(":"); // Each byte represent two digits, 0x59 means 59 seconds for example. 
  Serial.print(arrTime[1],HEX);Serial.print(":"); // I'm don't implement protections of over digits to avoid situations like: 0x78 scds. 
  Serial.print(arrTime[0],HEX); 
  Serial.print(" Week Day:");Serial.print((arrTime[3] & 0x0F),HEX);
  Serial.println();
}
/******************************  Read RC  ***************************/
void readRC() {
 // read Remote Command (RC) based on infra red receiver!!!
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0x62);  // command to define read RC operation of infra red.
      readAcknowledge();
    start();
      send_byte(addressRD);  // slave with write active and command read mode!
      readAcknowledge();
        // If I use a read command without RAM address, then finish the last read with a noAcknwoledge!!! ONLY ON THIS READ FORMAT!!!
            for(int s=0; s<3; s++){  // start read RC means 3 Bytes!
               arrRC[s]=read_byte();
                if (s==2){
                noAcknowledge();
                }
                else{
                sendAcknowledge();
                }
            }
    stp();
        for (int n=0; n< 3; n++){
          Serial.print(arrRC[n],HEX); Serial.print(", ");
        }
        Serial.println();
        // Here is a sample of read a RC from the Infra red receiver... 
        if ((arrRC[1] == 0x17) and (arrRC[0] != charIRed)){  // value 4 or 6 as information you see change if presse same button, this belongs to the protocol.
        led3Swap();
        charIRed=arrRC[0];
        }
        else
        if((arrRC[1] == 0x27) and (arrRC[0] != charIRed)){   // value 4 or 6 as information you presse same button!
        led2Swap();
        charIRed=arrRC[0];
        }
 }
/****************************** Read Keys ***************************/
void readKeys() {
 //
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0x60);  // command to define read keys operation
      readAcknowledge();
    start();
      send_byte(addressRD);  // slave with write active and command read mode!
      readAcknowledge();
        // If I use a read command without RAM address, then finish the last read with a noAcknwoledge!!! ONLY ON THIS READ FORMAT!!!
            for(int s=0; s<4; s++){  // start read keys (2*12=24) means 3 Bytes!
               arrKeys[s]=read_byte();
                if (s==3){
                noAcknowledge();
                }
                else{
                sendAcknowledge();
                }
            }
    stp();
        for (int n=0; n< 4; n++){
          Serial.print(arrKeys[n],HEX); Serial.print(", "); // This print lines, are only to debug and confirm the read key values, Please comment it after debug!
        }
        Serial.println();
        actionKeys();
 }
void writeDigits(char digit3,char digit4){
  //This function do the update each  time you select seconds, minuts or hours keys
        start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11011001);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(digit3);   //
            readAcknowledge();
         stp();
         start();
            send_byte(addressWR);  // slave with write active 
            readAcknowledge();
            send_byte(0b11011100);   //  Use 1º 0b11010000, 2º 0b11010011, 3º 0b11011001, 4º 0b11011100, take att to the 0b11010110 is under 2 points in the middle!!!
            readAcknowledge();
            send_byte(digit4);   // 
            readAcknowledge();
         stp();
}
void updateSS(void){
  writeSS(); 
          digitA=arrAdjusteTime[0];
          digitB=arrAdjusteTime[1];
          writeDigits(arrNumbers[digitA], arrNumbers[digitB]);
}
void updateMM(void){
  writeMM();
          digitA=arrAdjusteTime[2];
          digitB=arrAdjusteTime[3];
          writeDigits(arrNumbers[digitA], arrNumbers[digitB]);
}
void updateHH(void){
   writeHH();  
          digitA=arrAdjusteTime[4];
          digitB=arrAdjusteTime[5];
          writeDigits(arrNumbers[digitA], arrNumbers[digitB]);
}
 /*************************** Actions started by press of Key *******************************/
 void actionKeys(void){
  // The keys go from "1" to "13" with name as "SExx"
  // I've added the 5 buttons is not present on the PCB(SE3,SE5, SE7,SE9 & SE13)
  unsigned char key =0;
  unsigned char var =0;
  
      if (arrKeys[2] >0){
        key=arrKeys[2];   // Byte of weight 2
        switch (key){
          case 0x01: {  //SE1: read rc
            swapLedDE25(); // Only to confirm I'm reach this point! Note: This LED only change if is not in clock mode( press button SE14 )
            break;
          }
          case 0x02: {  //SE2: Active the Seconds adjuste
            positionArray=0x00;  // Move the point of arrAdjusteTime to the first position(0:1)!!!
            flagWriteSS=1;
            flagWriteMM=0;
            flagWriteHH=0;
          updateSS();
          break;}
          case 0x08: { // SE4: Active the Minutes adjuste
            positionArray=0x02; //Move the point of arrAdjusteTime to the second position(2:3)!!!
            flagWriteSS=0;
            flagWriteMM=1;
            flagWriteHH=0;
          updateMM();
          break;}
          case 0x20: { // SE6: Active the Hours adjuste
            positionArray=0x04; //Move the point of arrAdjusteTime to the third position(4:5)!!!
            flagWriteSS=0;
            flagWriteMM=0;
            flagWriteHH=1;
         updateHH();
          break;}
          case 0x80: {
          swapLedDE25();
          break;}
          case 0x10: {
            value=arrAdjusteTime[positionArray];
            value=value+1;
            if (value >= 0x0A){
              value=0;
            }
            arrAdjusteTime[positionArray]=value;
            Serial.println(value, HEX);
            break;
          }
          case 0x04: {  // >> btn
        if (positionArray <= 0){
          positionArray=5;
        }
        else{
          positionArray--;
        }
        Serial.println(positionArray, HEX);
        break;
      }
      case 0x40: { // << btn
        if (positionArray >= 5){
          positionArray=0;
        }
        else{
          positionArray++;
        }
        Serial.println(positionArray, HEX);
        break;
      }
     }
  }
    Serial.print(key, HEX); Serial.print(" - ");
    //Note: The array variable contents start from the left to right, the key sequence is from right to left.
    if (arrKeys[1] >0){
    key=arrKeys[1];    // Byte of weight 1
    Serial.println(key, HEX);
    switch (key){ 
      case 0x02: { // BTN SE10 set flagSetTime to implement protections if necessary!!!
        if (flagSetTime==0){
          led2Swap(); 
          flagSetTime=1;
        }
        else{
          led2Swap();
          flagSetTime=0;
        }
        break;
      }
      case 0x08: { // BTN SE12 show the contents of array "arrAdjusteTime"
        for(int i=0; i<9; i++){
         Serial.println(arrAdjusteTime[i]);
        }
        break;
      }
      case 0x20: {  //BTN SE14 swap between clock and display mode!
        if (flagConf==0){
          flagConf=1;
           confDisplayClock();
        }
        else{
          confDisplayNormal();
          flagConf=0;
        }
        break;
      }
      case 0x80: led3Swap(); break;
      case 0x01: {  //BTN SE9 btn down
            value=arrAdjusteTime[positionArray];
            value=value-1;
            if (value <= -1){
              value=9;
            }
            arrAdjusteTime[positionArray]=value;
            Serial.println(value, HEX);
            break;
          }
      case 0x10: {  //BTN SE13  set and send to adjuste clock.
        ssDU=arrAdjusteTime[0]; ssDU = ((ssDU << 4) | arrAdjusteTime[1]);
        mmDU=arrAdjusteTime[2]; mmDU = ((mmDU << 4) | arrAdjusteTime[3]);
        hhDU=arrAdjusteTime[4]; hhDU = ((hhDU << 4) | arrAdjusteTime[5]);
        setTime(ssDU, mmDU, hhDU, 0xF2);
        flagWriteSS=0;
        flagWriteMM=0;
        flagWriteHH=0;
        // Active the flag and put display as clock mode after set the hour's
        flagConf=1;
        confDisplayClock();
        break;
      } 
    } 
  }
 if (flagWriteSS==1){
  updateSS();
 }
 if(flagWriteMM==1){
  updateMM();
 }
 if (flagWriteHH==1){
  updateHH();
 }
  
  //(char ss, char mm, char hh, char weekDay) // this is wich the set time waite as input
  //(0x51, 0x59, 0x23, 0xF7) // this is the values to set ss, mm, hh and weekday
  // Variables used:  ssDU=30; mmDU=30; hhDU=12; WD=01;
  //arrAdjusteTime[8] // array to keep the values to set time!
 }
/*************** SCAN BUS (Find slaves connecte to the bus SCL & SCA ******************/
void scanBUS(void){
  Serial.begin(9600);
  softwarei2c.begin(7, 8);       // sda, scl is the pins I'm use to communicate with BUS
    
  Serial.println("begin to scan...");
    for(unsigned char i=1; i<=127; i++)
    {
        if(softwarei2c.beginTransmission(i))
        {
            Serial.print("0x");
            Serial.println(i, HEX);
            //while(1);
        }
        softwarei2c.endTransmission();
    }
    Serial.println("find nothing more");
    //while(1);
}
/***************** Set of configuration initial to the STFPC320 **********************/
void begConf(void){
      //initSTFPC320();
      
      confDisplayAndKey();  // This function is very important to see the clock or display format
      delay(100);
      confDisplayNormal();
      delay(100);
      write1234();
      delay(100);
      confTwoPoints();
      delay(1000);
      tstDisplay();
      confDisplayClock();
      //setTimer();
      confDSP();
      delay(100);
      confHz();
      delay(100);
      led1on();
      led2on();
      led3on();
      delay(1000);
      ledsOff();
      
      //OnOffLed();
      //readFlags();
}
/******************************** Setup of Arduino  *****************************************/
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
pinMode(sda, OUTPUT);
pinMode(scl, OUTPUT); 
pinMode(13, OUTPUT);
// I need set the config before the Interrupt take place to avoid wrong configuration!
// As I don't set the CLI or SEI, stay with default, only set it with OCIE1A trigger compare.
//Make a delay to avoid wrong configuration of STFPC320 during power up
//Depending of flags status, maybe is necessary do a second reset to the clock be active!
delayMicroseconds(200);
  begConf();
  scanBUS();
  setTime(0x51, 0x59, 0x23, 0xF7);  // Note: The day of week have also the RS0-RS3 to set pulse external (F to 1Hz) see table.
  tstDisplay();
   
  TCCR1A = 0;
  TCCR1B = 0;// This initialisations is very important, to have sure the trigger take place!!!
  TCNT1  = 0;
  // Use 62499 to generate a cycle of 1 sex 2 X 0.5Secs (16MHz / (2*256*(1+62449) = 0.5
  OCR1A = 62499;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10));    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt 
}
void loop() {
 // put your main code here, to run repeatedly:
         
 while(1);
  
  //0x09 = RTC display & enables the guard timer to issue STBY 
  //0xAA = raw format, RC-5 protocol, 10s for initial guard time value 
  //0x0A = digit 1 will show the Hour MSB, 11 digits/17 segments display 
  
//Read as an example:   0b01100001  “address setting command” 
//Address setting command with RTC memory address of 0x01 making a OR with 0xC1(RTC start first position as 0xC1)
//Read RTC register command: 0x73
//Subsequently STFPC320 will output the data byte from RTC memory location 0x01.
// Write sample:
//As an example, address setting command with normal display memory address of 0x10: 0xC2 subsequently
//the host can write the data bytes starting from memory location 0x10
//The address range from 00h-0Fh represents the RTC register map. For writing data to RTC or Normal display registers, 
//address command is sent followed by the RTC or normal display data.
//10h-3Fh represents the normal display memory map. On power application, the default address location is 0x10.
}
// Routine called each seconds by compare trigger. )I don't have protection to key press, please implement it to be more accuracy!
void ISR_rotine(void){
  digitalWrite(13, !digitalRead(13));  // blink led as second indicator...
     //counter++;
     readTime(); // Function to read the time from the STFPC320!
     showTime(); // Function to present the readed time on the console of arduino.
     led1Swap();
     readKeys();
     readRC(); // Read the 3 bytes from RC(remote command IR) // Note: I'm not worry about the delay to read it, if you need implement it on the loop!
}
ISR(TIMER1_COMPA_vect)   {    //This is the interrupt request
     // https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
     // this timer is to avoid read many times the RTC... read one time by second... avoid load at uC
     //Note: If I let here three lines of command, the compiler give to me a error of library???
     ISR_rotine();
} 



Reply
