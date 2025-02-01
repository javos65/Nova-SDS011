/*
  This example shows how to interact with NiNa internal memory partition
  APIs are modeled on SerialFlash library (not on SD) to speedup operations and avoid buffers.

  Read / Write / Erase 2x long int Sensor value 
  2022 - Jay fOx
*/

#include <WiFiNINA.h>
#define DATAFILE "/fs/datafile"           // Name your File -same in your
#define DEBUG_X 1                         // Debug mode for serial momitor, leave it and no Seriall is spammed

long int m1,m2;
    
void setup() {

Serial.begin(115200); while (!Serial);
  // check for the presence of the shield:
if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("NINA WiFi shield not present");
// don't continue:
    while (true);
  }

m1= 1;m2=2;
/*Test Flash*/ 
Serial.println("Nina Flash file test :");
Read_Data(&m1,&m2);
Serial.print("Read Flash: ");Serial.print(m1);;Serial.print(" [long int] + " ); Serial.print(m2);Serial.print(" [epoch int]\n" ); 
delay(2000);

m1= 2345;m2=2;
Serial.print("To Save to Flash : ");Serial.print(m1);Serial.print(" [long int] + " );Serial.print(m2);Serial.print(" [epoch int]\n" ); 
Write_Data(m1,m2);
Read_Data(&m1,&m2);
Serial.print("Read Flash: ");Serial.print(m1);;Serial.print(" [long int] + " ); Serial.print(m2);;Serial.print(" [epoch int]\n" ); 
//Erase_Credentials();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("." );
  delay(1000);
}



/* Read 2x long int from file*/
int Read_Data(long int * data1,long int * data2)
{
int c=0;
byte b;
long int d1=0,d2=0;
  WiFiStorageFile file = WiFiStorage.open(DATAFILE);
  if (file) {
    file.seek(0);
    if (file.available()) {  // read file buffer into memory, max size is 2 long int (2x4 bytes)
      c += file.read(&b, 1);  d1+= ((long int) b);
      c += file.read(&b, 1);  d1+= ((long int) b)*0x100;
      c += file.read(&b, 1);  d1+= ((long int) b)*0x10000;
      c += file.read(&b, 1);  d1+= ((long int) b)*0x1000000;     
      c += file.read(&b, 1);  d2+= ((long int) b);
      c += file.read(&b, 1);  d2+= ((long int) b)*0x100;
      c += file.read(&b, 1);  d2+= ((long int) b)*0x10000;
      c += file.read(&b, 1);  d2+= ((long int) b)*0x1000000;  
    }
    if (c==8) {
#if DEBUG_X      
        Serial.print("* Read Data : ");Serial.print(c); Serial.println(" bytes");
#endif      
      *data1=d1;*data2=d2;
      }
      else c=0; // bytes dont match !
   file.close(); return(c);
 }
 else {
#if DEBUG_X
   Serial.println("* Cant read Meter Data.");
#endif    
  file.close();return(0);
 }

}

/* Write 2x long int to file*/
int Write_Data(long int data1,long int data2)
{
  int c=0;
  byte b;
  WiFiStorageFile file = WiFiStorage.open(DATAFILE);
  if (file) {
    file.erase();     // erase content before writing
    }
  b= (byte) (data1&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data1>>8)&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data1>>16)&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data1>>24)&0x000000FF);  c+=file.write(&b, 1);    
  b= (byte) (data2&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data2>>8)&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data2>>16)&0x000000FF);  c+=file.write(&b, 1);
  b= (byte) ((data2>>24)&0x000000FF);  c+=file.write(&b, 1); 
  if(c==8) {
#if DEBUG_X
        Serial.print("* Written Data : ");Serial.print(c); Serial.println(" bytes");
#endif
   file.close(); return(c);
 }
 else {
#if DEBUG_X
   Serial.println("* Cant write Meter Data.");
#endif  
  file.close(); return(0);
 }

}

/* Check datafile */
int Check_File()
{
  WiFiStorageFile file = WiFiStorage.open(DATAFILE);
  if (file) {
#if DEBUG_X
 Serial.println("* Found datafile.");
#endif  
  file.close(); return(1);
 }
 else {
  #if DEBUG_X
 Serial.println("* Could not find datafile.");
#endif  
  file.close(); return(0);
 }
}
