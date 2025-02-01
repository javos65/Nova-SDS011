

/*****************************************************************************
* | File      	:	SDS011.ino
* | Function    :	Particle sensor interface over Uart
* | Info        :   JV 2025
*----------------
* |	This version:   V1.0
* | Date        :   2025-01-16
* | Info        :   Basic version
*
*  https://github.com/javos65/Nova-SDS011
*
******************************************************************************/
int incomingByte = 0; // for incoming serial data

struct MessageData {         
uint8_t start;       // 0xAA
uint8_t command;   // Command number - 0XC0
uint16_t pm25;     // DATA1+DATA2 PM2.5 low + high
uint16_t pm10;     // DATA3+DATA4 PM10 low + high
uint8_t id1;       // DATA5 ID1
uint8_t id2;       // DATA6 ID2
uint8_t checksum;  // checksum = DATA1+DATA2+DATA3 ... + DATA6
uint8_t end;       // 0XAB
uint8_t marker;    // marker - not used
};

uint8_t MESSAGE_SETWORK[19]  = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x60, 0x08, 0xAB}; //byte 15+16 = ID
uint8_t MESSAGE_SETSLEEP[19] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x60, 0x08, 0xAB}; //byte 15+16 = ID
uint8_t MESSAGE_SETID[19]    = {0xAA, 0xB4, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x01, 0x01, 0xA1, 0x60, 0xA7, 0xAB}; //byte 13+14 = new ID, 15+16 = ID
uint8_t PREFERED_ID=0x55; // ID1 + ID2 same value
MessageData G_Message;
//SensorData G_Message;

void setup() {
  Serial.begin(9600); //initialize serial communication at a 9600 baud rate
  Serial1.begin(9600); //initialize serial communication at a 9600 baud rate

if (!init_sensor_SDS011()){
    Serial.println("SDS011 failed init");
    }
else {
    Serial.print("SDS011 init, ID"); Serial.print(G_Message.id1,HEX); Serial.println(G_Message.id2,HEX);
    }

}

void loop() {
  // send data only when you receive data:
check_message();
print_message();
sleep_sensor();
Serial.print("SDS011 in sleep for 2 minutes.");delay_seconds(120,5);
wakeup_sensor();
Serial.print("SDS011 starting, wait 30 seconds.");delay_seconds(30,2);
}

void delay_seconds(uint8_t seconds, uint8_t gap)
{
  Serial.print(".");
  for(uint8_t t=0;t<seconds;++t){
    if (t%gap==0) Serial.print("."); // print every 5 seconds a point
    delay(1000);
  }
  Serial.println(",");
}

void print_message()
{
Serial.print("Received SDS011 data :");
Serial.print("CO:0x");Serial.print(G_Message.command,HEX);
Serial.print(", PM2.5: ");Serial.print(G_Message.pm25/10);Serial.print(".");Serial.print(G_Message.pm25%10);
Serial.print(", PM10: ");Serial.print(G_Message.pm10/10);Serial.print(".");Serial.print(G_Message.pm10%10);
Serial.print(", ID:0x");Serial.print(G_Message.id1,HEX);Serial.print(G_Message.id2,HEX);
Serial.print(", CHK:0x");Serial.print(G_Message.checksum,HEX);
Serial.println(".\n");
}



uint8_t send_message(uint8_t *m, uint8_t l)
{
 for(uint8_t t=0;t<l;++t)
  Serial1.write( (uint8_t) *(m+t) );
  return(1);
}

uint8_t init_sensor_SDS011()
{
 uint8_t r=check_message();r=check_message();  // Read First data, read twice
 if(r!=0) 
 {
  MESSAGE_SETID[13] = PREFERED_ID;MESSAGE_SETID[14] = PREFERED_ID;      // new ID
  MESSAGE_SETID[15] = G_Message.id1;MESSAGE_SETID[16] = G_Message.id2;  // actual ID
  MESSAGE_SETID[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETID[17] += MESSAGE_SETID[t] ;} // calc CRC
  send_message(MESSAGE_SETID,19);
 }
 return(r);
}

uint8_t sleep_sensor()
{
  MESSAGE_SETSLEEP[15] = PREFERED_ID;MESSAGE_SETSLEEP[16] = PREFERED_ID;      // new ID
  MESSAGE_SETSLEEP[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETSLEEP[17] += MESSAGE_SETSLEEP[t] ;} // calc CRC
  send_message(MESSAGE_SETSLEEP,19);
 return(1);
}

uint8_t wakeup_sensor()
{
  MESSAGE_SETWORK[15] = PREFERED_ID;MESSAGE_SETWORK[16] = PREFERED_ID;      // new ID
  MESSAGE_SETWORK[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETWORK[17] += MESSAGE_SETWORK[t] ;} // calc CRC
  send_message(MESSAGE_SETWORK,19);
 return(1);
}

uint8_t check_message()
{
  uint8_t t=1,s=0;
  char c;
  Serial1.flush();
  while(t!=0){
    t++;
    if (Serial1.available()){
      c= Serial1.read();
      if (c==0xAA) { //start message
          s=read_message();
          break;
          }
      }
    if(t>50) {t=0;break;} // stop after 50 tries x 100ms = 5sec
    delay(100);
    }
  if(t==0) return(0);
  else return(s);
}

uint8_t read_message()
{
  uint8_t t=1,s=0;
  char c;
  uint8_t *px = (uint8_t*) &G_Message;
  *(px+s) = 0xAA;
  while(t!=0)
  {
      t++;
      if (Serial1.available()){
      c= Serial1.read(); //Serial.print(c,HEX);Serial.print(".");
      s++;*(px+s) = c;
      if (c==0xAB) {break;} // end of message; break
      }
      if(t>50 || s>9) {t=0;break;} // stop after 50 tries x 100ms = 5sec   or if more than 9
      delay(100);  
  }
  if(t==0) return(0);
  else return(s);  
}