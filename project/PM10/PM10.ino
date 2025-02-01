/*
Demo sensor with MQTT posting and web interface
Program Flow :

Setup environment
Main Loop 
{
Check sensor data
Send data to MQTT
Check wifi, ntp and reset conditions
Service Webserver for X minutes
}

Sensor is Dust Sensor PM10 / PM2.5 and Humidity Sensor
Lifetime AVGvalue is kept in Flash, and read at power-up

All Global Variables start with G_<name>

(C) 2025 Jay Fox
*************************************************************/

#include <WiFiNINA.h>       // https://github.com/arduino-libraries/WiFiNINA    : Wifi Libs for uBlox ESP32 wifi module
#include <PubSubClient.h>   // https://github.com/knolleary/pubsubclient   : MQTT client server
#include <EasyWiFi_nano.h>  // https://github.com/javos65/EasyWifi-for-Nano   : Easy wifi setuo with Flash support, see WifiStorage.ino
#include <RTCZero.h>        // https://github.com/arduino-libraries/RTCZero   : SAMD21 lib for Realtime Clock
#include <WDTZero.h>        // https://github.com/javos65/WDTZero   : SAMD21 lib for Watchdog Timer
#include <NTPClient.h>      // https://github.com/arduino-libraries/NTPClient   : NTP timing client Lib
#include <Wire.h>

#define VERSION "1.2.0"  // Software verson nr
#define DEBUG_X 1       // Debug info for messages over serial
#define DEBUG_XX 1      // Debug info for led blinking 
#define LED_BOARD LED_BUILTIN   // Wifi1010 io D6 / Nano IOT 33 io D13
#define SERVER_PORT 80
#define MAX_MISSED_DATA 2000          // MAX data missed from Client/Web HTTP reply before time-out (accept short messages only)
#define DATAFILE "/fs/datafile"       // Data file in flash with Sensor data - see WifiStorageData.ino example to read and write flash
#define MQTTTOPIC "nodered/sensor/PM/state"
#define MQTTPORT 1883
#define MQTTSERVER IPAddress(192,168,200,20)
#define SERVERIMAGE "<img src=\"http://192.168.200.6/web_images/LocaLSensor.png\">"

WDTZero MyWatchDoggy; // Define WDT  
EasyWiFi MyEasyWiFi;  // Define Wifi
WiFiServer G_Myserver(SERVER_PORT); // Define Local WebServer
WiFiUDP ntpUDP;  
NTPClient timeClient(ntpUDP); // Define NTP
WiFiClient pubsubclient; // Define MQTT client
PubSubClient G_Mqttclient(pubsubclient);
String G_P1payload="{\"Device\":\"Dust\",\"Name\":\"DustSensor\",\"epochtime\":1608102499483,\"LifetimeF\":782450 ,\"PM25\":12.5,\"PM10\":7.5,\"PMAvg\":0}";  
String G_P1topic= MQTTTOPIC;
int G_Mqttport=MQTTPORT;
IPAddress G_Mqttip=MQTTSERVER,G_Myip;
RTCZero G_rtc;
long int G_Flash; // Global Value for Flash write-counter
long unsigned int G_Rtcvalue ;// Global RTC value for time keeping               

/*********** PM Sensor SDS011  DATA  **********/
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

#define PMPEAK 250          // max PM value seen as valid
#define PMMAXMES 10         // max message size SDS011 message
#define PMSLEEPTIME (60*10) // Sleep time in second => < 10 Minutes
#define PMSTARTTIME 30      // 30 second start time before measurements
uint8_t MESSAGE_SETWORK[19]  = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x60, 0x08, 0xAB}; //byte 15+16 = ID
uint8_t MESSAGE_SETSLEEP[19] = {0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x60, 0x08, 0xAB}; //byte 15+16 = ID
uint8_t MESSAGE_SETID[19]    = {0xAA, 0xB4, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x01, 0x01, 0xA1, 0x60, 0xA7, 0xAB}; //byte 13+14 = new ID, 15+16 = ID
float GROWFACTOR25[13]    = {0.722, 0.749, 0.779, 0.814, 0.855, 0.903, 0.963, 1.039, 1.14, 1.284, 1.519, 2.026, 3.950}; // Growfactor Humidity PM2.5 40%-100%
float GROWFACTOR10[13]    = {0.52, 0.55, 0.585, 0.627, 0.677, 0.738, 0.816, 0.918, 1.062, 1.280, 1.666, 2.614, 7.442}; // Growfactor Humidity PM10 40%-100%

uint8_t PREFERED_ID=0x55; // ID1 + ID2 same value
MessageData G_Message;
float G_PM25=10.0,G_PM10=10.0,G_LifetimePM=10; // Global variable used
float G_Latitude = 51.574379792111884, G_Longitude = 5.316302761598326;
/*********** SHT4x Sensor Data  **********/
#define SHT4X_ACCHIGH    0xFD
#define SHT4X_ACCMEDIUM  0xF6
#define SHT4X_ACCLOW     0xE0
#define SHT4X_ID         0x89
#define SHT4X_HEAT110MW  0x2F

#define SHT4X_HDURATION   10
#define SHT4X_MDURATION    4
#define SHT4X_LDURATION    2
#define SHT4X_IDDURATION   2
#define SHT4X_HEATDURATION   1010

#define I2CADDRESS      0x44
#define DATALENGTH        6

float G_Temperature=0;
float G_Humidity=0;
uint16_t G_Sleeptime = PMSLEEPTIME;


void setup() {
/*********** Board LED  SETUP  **********/
pinMode(LED_BOARD, OUTPUT);
#if DEBUG_XX
  ledblink(1,500); 
#endif
/*********** Serial SETUP  **********/
#if DEBUG_X
int t=10;  //Initialize serial and wait for port to open, max 10 second waiting
  Serial.begin(115200);
  while (!Serial) {
    ; delay(1000);
    if ( (t--)== 0 ) break;
  } 
#endif 
/*********** RTC SETUP  **********/
G_rtc.begin();
G_rtc.disableAlarm();        
/*********** EASY WIFI SETUP  **********/
if (WiFi.status() == WL_NO_SHIELD) {   // check for the presence of the shield:
#if DEBUG_X
    Serial.println("! WiFi chip not present - halted to reset");
#endif
    AlarmShutdown();
  }
delay(1000);
#if DEBUG_X
   Serial.println("* WiFi starting...");
#endif  
MyEasyWiFi.seed(11);     // set seed for encoded storage  -see also WifiStorage.ino
MyEasyWiFi.start();     // Start Wifi login 
if ( WiFi.status() != WL_CONNECTED){
#if DEBUG_X
   Serial.println("! WiFi cant connect- halted to reset.");
#endif
    AlarmShutdown();
    }

//*********** Server SETUP  **********/
G_Myserver.begin();
#if DEBUG_X
   Serial.print("* Webserver starting : ");   Serial.println(G_Myserver.status());
#endif  

//*********** NTP time client preparation  **********/
timeClient.begin();delay(500);
timeClient.setUpdateInterval(3600000); // update hourly
if(!timeClient.isTimeSet()) timeClient.update();
G_rtc.setEpoch(timeClient.getEpochTime());
#if DEBUG_X
  Serial.print("* Time set Epoch ");Serial.print(G_rtc.getEpoch());Serial.println("");
#endif 
//*********** Mqtt preparation  **********/
G_Mqttclient.setServer(G_Mqttip, G_Mqttport);
G_Mqttclient.setBufferSize(128);
//*********** WATCHDOG SETUP  **********/
MyWatchDoggy.attachShutdown(myshutdown);
MyWatchDoggy.setup(WDT_HARDCYCLE16S);  // initialize WDT-hardcounter refesh cycle on 16sec interval
//***********  PM Sensor setup  - UART **********/ 
 Serial1.begin(9600); //initialize serial communication at a 9600 baud rate
 delay(100);
if (init_sensor_SDS011()==0){
#if DEBUG_X   
    Serial.println("! SDS011 failed init");
 #endif  
    }
else {
#if DEBUG_X   
    Serial.print("* SDS011 sensor found, ID="); Serial.print(G_Message.id1,HEX); Serial.println(G_Message.id2,HEX);
 #endif      
    }
  /*********** ISHT4x Sensor SETUP I2C  **********/
uint8_t data[6];
boolean st;
Wire.end();
Wire.begin();
if( readsensor_H(SHT4X_ID,data,SHT4X_IDDURATION) ){
      Serial.print("* SHT4x sensor found, ID=");Serial.print(data[0],HEX);Serial.print(data[1],HEX);Serial.print(data[3],HEX);Serial.println(data[4],HEX);
  }
else{
#if DEBUG_X   
    Serial.println("! No Humidity sensor found");
 #endif    
}
ReadSH4x();

//***********  data setup from Flash **********/ 
long vv,tt;
if( Read_Data(&vv,&tt) == 8 ){
   G_LifetimePM=(float) vv;G_Flash=tt; 
#if DEBUG_X
    Serial.print("* Read Sensor Flash data: ");Serial.print(vv);Serial.print(" \ Flashtick: ");Serial.print(tt);;Serial.print("\n" ); 
#endif  
    }
else
    {
    G_LifetimePM= 0;G_Flash=0;
#if DEBUG_X
    Serial.print("* No Flash data, set your actual data via webhost interface ");
#endif  
  }

#if DEBUG_XX
  ledblink(2,500);    // blink 2 times slow : ready
#endif
}


void loop() {

//test();
MyWatchDoggy.clear();

wakeup_sensor();
// Check Web server for 30 sec
G_Rtcvalue =  G_rtc.getEpoch() ; // read time
while ( (G_rtc.getEpoch()-G_Rtcvalue)< PMSTARTTIME  ) // Servercheck for 30 seconds
{
  MyWatchDoggy.clear();  
  if(CheckServerClients()==1) Mqtt_Publishstructure(); 
  MyWatchDoggy.clear();  
  delay(500);
}

read_sensor();     // Read and make sensor data 
#if DEBUG_XX
  ledblink(3,250);    // blink 3 times quick : Sensor Read
#endif

print_message();
sleep_sensor();

MyWatchDoggy.clear();  
Mqtt_makemessage();
Mqtt_Publishstructure();

// Check Wifi connection
MyWatchDoggy.clear();  
if ( WiFi.status() != WL_CONNECTED) MyEasyWiFi.start();     // Start Wifi login again, if this takes more than WDT (16 seconds), system resets

// Check Web server for 2 minutes
G_Rtcvalue =  G_rtc.getEpoch() ; // read time
while ( (G_rtc.getEpoch()-G_Rtcvalue)< G_Sleeptime  ) // Servercheck for 15 minutes 
  {
  MyWatchDoggy.clear();  
  if(CheckServerClients()==1) Mqtt_Publishstructure(); 
  MyWatchDoggy.clear();  
  delay(500);
  // check daily time to reset at one minute to midnight
  if( (G_rtc.getHours()==23) && (G_rtc.getMinutes()>=58)) {
    Serial.println("! Daily reset scheduled.");
    AlarmShutdown(); 
    }

  }
}



void test()
{
  wakeup_sensor();
  while(1)
  {
  check_message();      // Read and make sensor data G_Message
  print_message();
  MyWatchDoggy.clear();
  delay(250);
  }
}


// create message string into Gobal G_P1payload
void Mqtt_makemessage()
{
  G_Rtcvalue =  G_rtc.getEpoch() ; // read time
  G_P1payload="{\"Device\":\"DST01\",\"Name\":\"Dust Sensor\",\"epochtime\":"; G_P1payload.concat(G_Rtcvalue); 
  G_P1payload+= ",\"LifetimePM\":";   G_P1payload.concat(G_LifetimePM);  
  G_P1payload+= ",\"PM25\":";   G_P1payload.concat(G_PM25);  
  G_P1payload+= ",\"PM10\":";   G_P1payload.concat(G_PM10);  
  G_P1payload+= ",\"P_Unit\":\"ug/m3\"";    
  G_P1payload+= ",\"HUM\":";   G_P1payload.concat(G_Humidity*100);  
  G_P1payload+= ",\"H_Unit\":\"\%\"";    
  G_P1payload+= ",\"TMP\":";   G_P1payload.concat(G_Temperature);  
  G_P1payload+= ",\"T_Unit\":\"celcius\"";    
  G_P1payload+= ",\"LAT\":";   G_P1payload.concat(G_Latitude);  
  G_P1payload+= ",\"LON\":";   G_P1payload.concat(G_Longitude);  
  G_P1payload+= ",\"Flashwrites\":";   G_P1payload.concat(G_Flash);  
  G_P1payload += ",\"valid\": 1,\"message\":\"ok\"}"; 
}

// Publish Message over MQTT, keep pubsub-client Global - otherwiasse running out of avaialbel sockets... PubSub bug ??
// meeasage is in 
void Mqtt_Publishstructure()
{
  uint16_t i,z,l; 
  long timer;
  boolean P1_mqqtsend=false;
  boolean P1_mqqterror=false;
  char *topic; topic=&(G_P1topic[0]); 
  char *payload; payload=&(G_P1payload[0]);
  l=G_P1payload.length();
  
  timer = G_rtc.getEpoch();
MyWatchDoggy.clear();
  while (!P1_mqqtsend && !P1_mqqterror) 
  {
    // Attempt to connect
    if (G_Mqttclient.connect("JSONArduino09") ) {
#if DEBUG_X
      Serial.print("* Sending message: "); Serial.println(G_P1payload);       
      Serial.print("* Mqtt connected..");
#endif        
      // Once connected, publish an announcement...
MyWatchDoggy.clear();      
      z=G_Mqttclient.beginPublish(topic,l,true); 
      for(i=0;i<=l;++i) 
      {
        G_Mqttclient.print(payload[i]); 
MyWatchDoggy.clear();
      }
      z=G_Mqttclient.endPublish();
MyWatchDoggy.clear();

#if DEBUG_X    
      Serial.print(".. published payload, Client status: ");Serial.print(G_Mqttclient.state());Serial.print(" Publisch return:");Serial.println(z);
#endif         
      break;
    } 
    else {
      if (G_rtc.getEpoch()-timer > 10000) {P1_mqqterror=true;break;} // check max 10 seconds
      delay(500);
    } // end if- connected
  }  
if(P1_mqqterror==true)   
    { 
#if DEBUG_X    
      Serial.print("* Connection to mqttserver failed, Client status");Serial.println(G_Mqttclient.state());
#endif      
    }  
}

void myshutdown()
{
#if DEBUG_X   
Serial.print("\n! We gonna shut down !\n");
#endif  
}

void AlarmShutdown()
{
    MyWatchDoggy.setup(WDT_HARDCYCLE8S); // set WDT to 8 sec  
    while (true)     // don't continue if no Sensor found blink 8 sec, then reset
      {
      digitalWrite(LED_BOARD, HIGH);delay(500);
      digitalWrite(LED_BOARD, LOW);delay(500);      
      }
}

void ledblink(int loops,int times)
{
for(int t=0;t<loops;++t)
  {
  digitalWrite(LED_BOARD, HIGH); delay(times/2); 
  digitalWrite(LED_BOARD, LOW); delay(times/2); 
  }
}

/********* WEBSERVER ROUTINES *************/

// Check server if client is there, and service requests //
uint8_t CheckServerClients() {

  // Local Varaibles
  String currentLine = "";  // date line input client
  String s1,s2;
  float f;
  int t=0, d = 0,v=0; // client data counter and loop counter, t is local loop counter
  byte hr,mn,sc; // hour and minute read characters
  long int h1,h2,h3,h4; // input read words
  char st[7] = {0,0,0,0,0,0,0};
  uint8_t exitcommand=0;

  G_Myip = WiFi.localIP();
  WiFiClient client = G_Myserver.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
//#if DEBUG_X
//    Serial.println("*New client*");           // print a message out the serial port
//#endif
    currentLine = "";                       // make a String to hold incoming data from the client
    d = 0;
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
//#if DEBUG_X
//        Serial.write(c);                     // print it out the serial monitor - debug only
//#endif
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
          SendHTMLHeader1(client);     
            client.print( "<p style=\"font-family:Candara; color:GhostWhite\"></font>");
            client.print( "<font size=6>Particle Sensor</font>&nbsp&nbsp&nbsp&nbsp<font size=3>Version ");client.print(VERSION);client.println("</font><br>");
            client.print( "<font size=2>");client.print(WiFi.SSID());client.println(" / ");client.print(G_Myip);client.println("<br>");
            client.print("MQTT Server ");client.print(G_Mqttip);client.println(" / ");client.print(MQTTTOPIC);client.println("</font></p>");            
            client.print("<p style=\"font-family:Helvetica; color:#334455\"><font size=2>-------------------------------------------<br>");
            client.print("Longterm PMavg : ");client.print(G_LifetimePM);client.print(" ug/m3.<br>"); 
            client.print("PM2.5 : ");client.print(G_PM25);client.print(" ug/m3.<br>");     
            client.print("PM10  : ");client.print(G_PM10);;client.print(" ug/m3.<br>"); 
            client.print("H% : ");client.print(G_Humidity*100);;client.print(" %<br>");    
            client.print("T  : ");client.print(G_Temperature);;client.print(" C.<br>");                      
            client.print("Flash written : ");client.print(G_Flash);client.print(" times<br>");   
            client.print("Post interval : ");client.print(G_Sleeptime/60);client.print(" minutes <br>");            
            client.print("Time Client : ");client.print(timeClient.getFormattedTime());client.print(" <br>");
            client.print("<br>-------------------------------------------<br>");
            client.print("Click <a href=\"/H\">help</a> for Help.<br><br>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
#if DEBUG_X
            Serial.println("* Web: Html Home-page send");
#endif
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("GET /H")) {
          MyWatchDoggy.clear();
          SendHTMLHeader1(client);     
        client.print( "<p style=\"font-family:Candara; color:GhostWhite\"> </font> ");
          client.print( "<font size=6>Particle Sensor</font>&nbsp&nbsp&nbsp&nbsp<font size=4>Help Menu</font>");
          client.print("<p style=\"font-family:Helvetica; color:#334455\"><font size=2>-------------------------------------------<br>");

          client.print("Use HTML-command   http:\\\\"); client.print(G_Myip); client.print("/[cmd]<br><br>");
          client.print("[cmd] = JSON - Request JSON data<br>");
          client.print("[cmd] = R xyz - ReSet and save Lifetime Data<br>");
          client.print("[cmd] = S - Save Lifetime Data<br>");
          client.print("[cmd] = T - Force NTP Time<br>");
          client.print("[cmd] = P - Publish Mqtt data<br>");
          client.print("[cmd] = I xyz - Set Post interval in minutes<br>");
          client.print("[cmd] = H - This Help<br>");
          client.print("<br>");
          client.print("Click <a href=\"/\">here</a> to return to menu.<br><br>");
          client.println();
#if DEBUG_X
          Serial.println("* Web: Help Send");
#endif
          break;
        }

        if (currentLine.endsWith("GET /JSON")) {
        MyWatchDoggy.clear();
        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-type:text/html"));
        client.println();
        Mqtt_makemessage();client.print(G_P1payload);
        client.println();
#if DEBUG_X
      Serial.println("* Web: HTTP JSON send");
#endif
        break;
        } 

        if (currentLine.endsWith("GET /R")) {
          ReadPointSeparatedValues(client,&h1,&h2,&h3,&h4);
          G_LifetimePM = (float) h1;G_Flash++;
          Write_Data( (long int) h1,G_Flash);
          SendHTMLHeader1(client);     
          client.print("New data set to : "); client.print(h1); client.print("<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
#if DEBUG_X
          Serial.println("*Data set and saved, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /I")) {
          ReadPointSeparatedValues(client,&h1,&h2,&h3,&h4);
          G_Sleeptime = h1*60; // minutes to seconds
          if (G_Sleeptime > 60*60 ) G_Sleeptime=60*60;
          SendHTMLHeader1(client);     
          client.print("Sleeptime set to : "); client.print(G_Sleeptime/60); client.print("minutes<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
#if DEBUG_X
          Serial.println("*Data set and saved, break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /P")) {
          exitcommand=1; // publish data
          SendHTMLHeader1(client);     
          client.print("Meter data published.<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
#if DEBUG_X
          Serial.println("* Data Published,  break out*");
#endif
          break;
        }


        if (currentLine.endsWith("GET /S")) {
          G_Flash++;Write_Data((long int) G_LifetimePM,G_Flash);
          SendHTMLHeader1(client);     
          client.print("Meter data saved : "); client.print(G_LifetimePM); client.print("<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
#if DEBUG_X
          Serial.println("* Meter Saved,  break out*");
#endif
          break;
        }

        if (currentLine.endsWith("GET /T")) {
        timeClient.forceUpdate();
        G_rtc.setEpoch(timeClient.getEpochTime());

          SendHTMLHeader1(client);     
          client.print("NTP time epoch : "); client.print(timeClient.getFormattedTime() ); client.print("<br>"); 
          client.print("<br>Click <a href=\"/\">here</a> to return to menu.<br>");
          client.print("<meta http-equiv=\"refresh\" content=\"5;url=/\" />");
          client.println();
#if DEBUG_X
          Serial.println("* NTP time set,  break out*");
#endif
          break;
        }
        
      }
      else {
        d++;
        if (d > MAX_MISSED_DATA) { // defined in arduino_secrets.h
#if DEBUG_X
          Serial.println("* Client missed-data time-out");
#endif
          break;   // time-out to prevent to ever waiting for misssed non-send data newclient //
        }
      }
    }
    // close the connection:
    client.stop();
    
//#if DEBUG_X
//    Serial.println("*Client disonnected*");
//#endif
  } // end ifloop (client)
  else
  {
    client.stop();
  } 

return(exitcommand);  
}

// read 4 'point'-separated integer values from open client, format GET [command]wwww.xxxx.yyyy.zzzz -> read as from first x-value after GET comand.
// maps wwww-> h1 xxxx -> h2 yyyy -> h3 zzzz -> h4, separator can be :  [.] [/] [:] [,], usable for time, dates, ipadress etc etc.
void ReadPointSeparatedValues(WiFiClient client,long int *h1,long int *h2, long int *h3,long int *h4)
{
byte hr,t=1;
uint16_t h;
int signh=1;
*h1=0;*h2=0;*h3=0,*h4=0;
hr=(byte)client.read();                                               // readfirst character
while( (hr!=32) && (hr!=13) )                                         // Read till space (end of command) or CR character (end of line)
      { 
      if( hr =='.' || hr ==':' || hr==',' || hr=='/') { *h4=*h3;*h3=*h2;*h2=(*h1)*signh;*h1=0;signh=1;++t; }                     // at separator: new int valueto read: shift values to next pushed variable
      if( hr =='-') { signh=-1; }  
      if (hr>=48 && hr<=57) { *h1= (*h1)*10 + (hr-48); }              // process only ascii numbers 0-9
      hr=(byte)client.read(); //Serial.print(hr);
      }    
 if  (t ==1)  *h1=(*h1)*signh;                                 // 1 ints read: value in h1 = ok, adapt sign
 else if  (t ==2)  {h=*h1;*h1=*h2;*h2=h;}                        // 2 ints read : swap value h1 and h2, h3 is kept 0, h4 is kept 0
 else if  (t ==3)  {h=*h1;*h1=*h3;*h3=h;}                        // 3 ints read : swap order 3->1, (2=2), 1->3
 else if  (t ==4)  {h=*h1;*h1=*h4;*h4=h;h=*h2;*h2=*h3;*h3=h;}    // 4 ints read : swap order : 4->1, 1->4, 2->3 3->2
 else {*h1=0;*h2=0;*h3=0;*h4=0; }                                // 5 or more ints read : failure, return 0's
}

// send HTML Header with Mata Code
void SendHTMLHeader1(WiFiClient client){
     client.println(F("HTTP/1.1 200 OK")); 
     client.println(F("Content-type:text/html"));
     client.println();    
     client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">")); // metaview    
     client.print(SERVERIMAGE);      
     client.println(F("<p style=\"font-family:Candara;\">"));
     client.print(F("<body style=\"background-color:#"));
     client.print("A");client.print(random(1,14),HEX);
     client.print("A");client.print(random(1,14),HEX);
     client.print("A");client.print(random(1,14),HEX);
     client.println("\">"); 
}


/* Read 2x long int from flash file*/
int Read_Data(long int * data1,long int * data2)
{
int c=0;
byte b;
long int d1=0,d2=0;
  WiFiStorageFile file = WiFiStorage.open(DATAFILE);
  if (file) {
    file.seek(0);
    if (file.available()) {  // read file buffer into memory, max size is one long int (4 bytes)
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
        Serial.print("* Read Data : ");Serial.print(c); Serial.print("bytes , Data : ");Serial.println(d1);
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

/* Write 2x long int to flash file*/
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
        Serial.print("* Written Data : ");Serial.print(c); Serial.print("bytes , Data : ");Serial.println(data1);
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


/* Check data flashfile */
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



void print_message()
{
Serial.print("Last SDS011 message :");
Serial.print("CO:0x");Serial.print(G_Message.command,HEX);
Serial.print(", PM2.5: ");Serial.print(G_Message.pm25/10);Serial.print(".");Serial.print(G_Message.pm25%10);
Serial.print(", PM10: ");Serial.print(G_Message.pm10/10);Serial.print(".");Serial.print(G_Message.pm10%10);
Serial.print(", ID:0x");Serial.print(G_Message.id1,HEX);Serial.print(G_Message.id2,HEX);
Serial.print(", CHK:0x");Serial.print(G_Message.checksum,HEX);
Serial.println(".\n");
}

void print_message_short()
{
Serial.print("Last SDS011 message :");
Serial.print(", PM2.5: ");Serial.print(G_Message.pm25/10);Serial.print(".");Serial.print(G_Message.pm25%10);
Serial.print(", PM10: ");Serial.print(G_Message.pm10/10);Serial.print(".");Serial.print(G_Message.pm10%10);
Serial.println(".\n");
}

uint8_t send_message(uint8_t *m, uint8_t l)
{
 for(uint8_t t=0;t<l;++t)
  Serial1.write( (uint8_t) *(m+t) );
  return(1);
}

// init sensor, set ID
uint8_t init_sensor_SDS011()
{
 uint8_t r=0;
wakeup_sensor();
 r=check_message();  
 if(r==PMMAXMES) 
 {
  MESSAGE_SETID[13] = PREFERED_ID;MESSAGE_SETID[14] = PREFERED_ID;      // new ID
  MESSAGE_SETID[15] = G_Message.id1;MESSAGE_SETID[16] = G_Message.id2;  // actual ID
  MESSAGE_SETID[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETID[17] += MESSAGE_SETID[t] ;} // calc CRC
  send_message(MESSAGE_SETID,19);
  return(1);
 }
 else return(0);
}

uint8_t sleep_sensor()
{
  MESSAGE_SETSLEEP[15] = 0xFF;MESSAGE_SETSLEEP[16] = 0xFF;      // 0xFF = all sensor
  MESSAGE_SETSLEEP[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETSLEEP[17] += MESSAGE_SETSLEEP[t] ;} // calc CRC
  send_message(MESSAGE_SETSLEEP,19);
 return(1);
}

uint8_t wakeup_sensor()
{
  MESSAGE_SETWORK[15] = 0xFF;MESSAGE_SETWORK[16] = 0xFF;      // 0xFF = all sensor
  MESSAGE_SETWORK[17]=0; // CRC reset
  for(uint8_t t=2;t<17;++t) {MESSAGE_SETWORK[17] += MESSAGE_SETWORK[t] ;} // calc CRC
  send_message(MESSAGE_SETWORK,19);
 return(1);
}

// read sensor : average 5 messages and calculate floatingpoint PM2.5 and PM10 + Hum correction
uint8_t read_sensor()
{
float pm25avg=0,pm10avg=0;  
uint8_t t,c=0;
int8_t i=0;
for(t=0;t<5;++t) // read 5 values
  {
    if(check_message()==PMMAXMES) // only read valid messages: size = 10
    {
      c++;
      pm25avg+= (float) G_Message.pm25; // add results
      pm10avg+= (float) G_Message.pm10; // add results
      //print_message_short();
    }
    MyWatchDoggy.clear();
  }
if(c!=0){
  ReadSH4x();
  i=(int8_t) (  (12*(G_Humidity-0.4)/0.6) + 0.5    ); // calculate index
  if(i>12) i=12;
  if (i<0) i=0;
     //Serial.print("index=");Serial.println(i);
  pm25avg = (pm25avg/c)/10;
  pm10avg = (pm10avg/c)/10;
     //Serial.print("raw25=");Serial.println(pm25avg);
     //Serial.print("raw10=");Serial.println(pm10avg);
  if (pm25avg<PMPEAK ) {G_PM25=pm25avg/GROWFACTOR25[(uint8_t) i];} // calc average, avoid peaks,correct Humidity
  if (pm10avg<PMPEAK ) {G_PM10=pm10avg/GROWFACTOR10[(uint8_t) i];} // calc average, avoid peaks, correct Humidity
  G_LifetimePM= (0.9*G_LifetimePM)+(0.05*(G_PM25+G_PM10));
      //Serial.print("PM25=");Serial.println(G_PM25);
      //Serial.print("PM10=");Serial.println(G_PM10);
  return(1);
  }
  else return(0);
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
    MyWatchDoggy.clear();
    if(t>40) {t=0;break;} // stop after 40 tries x 50ms = 2sec
    delay(50);
    }
  if(t==0) return(0);
  else return(s);
}

uint8_t read_message()
{
  uint8_t t=1,s=0;
  char c;
  uint8_t *px = (uint8_t*) &G_Message;
  *(px+s) = 0xAA; s=1;
  while(t!=0)
  {
      t++;
      if (Serial1.available()){
      c= Serial1.read();    //Serial.print(c,HEX);Serial.print(".");
      *(px+s)=c;s++;        // write value, then increase
      if (c==0xAB) {break;} // if end of message; break
      }
      MyWatchDoggy.clear();
      if(t>40 || s>PMMAXMES) {t=0;break;} // stop after 40 tries x 50ms = 2sec   or more than 10 characters
      delay(50);  
  }
  if(t==0) return(0);
  else return(s);  
}

// Read sensor, Calculate %h (0.0...1.0) and C (-45...80)
boolean ReadSH4x()
{
boolean st;
uint8_t data[6]; 
uint16_t val; 
if (G_Humidity >0.75){  st = readsensor_H(SHT4X_HEAT110MW,data,SHT4X_HEATDURATION);} // use heater above 75% humidity
else {st = readsensor_H(SHT4X_ACCMEDIUM,data,SHT4X_MDURATION);}
if(st){
  val = ((uint16_t) (data[0] << 8) + data[1]);
  G_Temperature = -45 + 175 * ((float) val / 0xFFFF);
  val = ((uint16_t) (data[3] << 8) + data[4]);
  G_Humidity = ( -6 + 125 * ((float) val / 0xFFFF) )/100;
  if (G_Humidity<0) G_Humidity=0;
  if (G_Humidity>1) G_Humidity=1;  
  return(true);
}
else return(false);
}

/*********I2C Read and Write routines *************/
boolean readsensor_H(uint8_t Command, uint8_t *data, uint8_t duration)
{
  Wire.beginTransmission(I2CADDRESS);
  if (Wire.write(Command) != 1) {Serial.println("! write command[0] failed\n");return false;}
  if (Wire.write(0x00) != 1) {Serial.println("! write command[1] failed\n");return false;}
  Wire.endTransmission(); // returns a code 3: Data notacknowledged ? - ignored
  delay(duration);
  Wire.requestFrom(I2CADDRESS, DATALENGTH);

  // check if the same number of bytes are received that are requested.
  if (Wire.available() != DATALENGTH) {Serial.println("! Datalength not available\n");return false;}

  for (int i = 0; i < DATALENGTH; ++i) {
    data[i] = Wire.read();
  }
  Wire.endTransmission();
  return true;
}




