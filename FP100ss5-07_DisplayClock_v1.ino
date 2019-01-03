/*
Board: -> Generic ESP8266 Module
Flash Mode: -> DIO
Flash Frequency: -> 80 MHz
CPU Frequency: -> 160 MHz
Flash Size: -> 2M (1M SPIFFS)
Debug Port: -> Disabled
Debug Level: -> None
Reset Method: -> nodemcu
Upload Speed: -> 921600
*/
#define VERSION "Version 1.00"

#define countof(a) (sizeof(a) / sizeof(a[0]))

#include <Wire.h>
#include <EEPROM.h>
#include <pgmspace.h>
#include <Ticker.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>
//https://github.com/tzapu/WiFiManager

// DS1307(RTC)
#include "RtcDS1307.h"
RtcDS1307<TwoWire> Rtc(Wire);
int timedata;
int clockdata[14];
bool RTCInterruptFlag = false;
Ticker rtc_setting_delay; // Ticker Setting

void ICACHE_RAM_ATTR RTCInterrupt() {
  //Serial.println("RTC Interrupt...");
  RTCInterruptFlag = true;
}

// PCFPCF8574(I/O-Extension)
#include "pcf8574_esp.h"
PCF857x pcf8574(0x20, false);
bool PCFInterruptFlag = false;

void ICACHE_RAM_ATTR PCFInterrupt() {
  Serial.println("PFC Interrupt...");
  PCFInterruptFlag = true;
}

// STTS751(Temperature)
#include "STTS751.h"
STTS751 stts;

// ESP8266 Pin-Config
#define CLOCK     13//SH_CP(11)/ESP8266(13)
#define LATCH     15//ST_CP(12)/ESP8266(15)
#define DATA      12//DS(14)/ESP8266(12)
#define SEG       16//283(A0)/ESP8266(16)

#define PFC_INT   4
#define RTC_INT   5

#define LUX       A0

// PCF857x Pin-Config
#define LED_1   1
#define LED_2   2
#define LED_3   0
#define SW_1    4
#define SW_2    6
#define SW_3    5

// ElectroDisplay Setting
#define SIGNAL_DELAY 350  // delayMicroseconds
#define FLIP_TIME_B 750   // delayMicroseconds
#define FLIP_TIME_W 500   // delayMicroseconds

#define SEG_DELAY   200
#define SEG_DELAY_NIGHTMODE 750

// Wifi Setting
//　Local intialization. Once its business is done, there is no need to keep it around
WiFiManager wifiManager;

WiFiUDP udp;

// mDNS-HostName
#define MDNS_NAME    "7Seg_clock" // MDNS_NAME + ".local"

// WebServer
#define LOCAL_PORT_WEB 80
#define LOCAL_PORT_UDP 2390

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
// NTP Servers&TimeZones
static const char* ntpServerName = "ntp.nict.jp";
//static const char* ntpServerName = "ntp.jst.mfeed.ad.jp";
//static const char* ntpServerName = "ntp1.jst.mfeed.ad.jp";
//static const char* ntpServerName = "ntp2.jst.mfeed.ad.jp";
//static const char* ntpServerName = "ntp3.jst.mfeed.ad.jp";

const int timeZone = +9;     // Japan Time

// TCP server at port 80 will respond to HTTP requests
WiFiServer server(LOCAL_PORT_WEB);

// 7Segment-Control
uint8_t BinaryString[3];

static const uint8_t numbertable[] = {
  0b11111100, /* 0 */
  0b01100000, /* 1 */
  0b11011010, /* 2 */
  0b11110010, /* 3 */
  0b01100110, /* 4 */
  0b10110110, /* 5 */
  0b10111110, /* 6 */
  0b11100000, /* 7 */
  0b11111110, /* 8 */
  0b11110110, /* 9 */
  0b11101110, /* A */
  0b00111110, /* b */
  0b10011100, /* C */
  0b01111010, /* d */
  0b10011110, /* E */
  0b10001110, /* F */

  0b00000000, // 10 (blank)
  0b00000001, // 11 (DP)
  0b00000010, // 12 (-)
  
  0b11101110, // 13 A
  0b00111110, // 14 B
  0b00011010, // 15 C
  0b01111010, // 16 D
  0b10011110, // 17 E
  0b10001110, // 18 F
  0b10111100, // 19 G
  0b00101110, // 20 H
  0b00100000, // 21 I
  0b01111000, // 22 J
  0b10101110, // 23 K
  0b00011100, // 24 L
  0b11101100, // 25 M
  0b00101010, // 26 N
  0b00111010, // 27 O
  0b11001110, // 28 P
  0b11100110, // 29 Q
  0b00001010, // 30 R
  0b00110110, // 31 S
  0b00011110, // 32 T
  0b00111000, // 33 U
  0b01111100, // 34 V
  0b01111110, // 35 W
  0b01101110, // 36 X
  0b01110110, // 37 Y
  0b11011000  // 38 Z
};

// Variable
const int VOLT = 3.3;
const int ANALOG_MAX = 1024;
const int RES = 10000;
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
bool DisplayFlag = true, mt = true, send_ntp_flag = false, TempDispFlag = false;
long ntp_response_delay,beginWait;

// Setup Section
void setup() {
  // Serial Setup
  Serial.begin(115200);

  // Pin Setup
  pinMode(DATA, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(SEG, OUTPUT);
  
  pinMode(PFC_INT, INPUT_PULLUP);
  pinMode(RTC_INT, INPUT);

  // Disable ALL-Pins
 for (uint8_t m = 0; m < 2; m++) {
    shiftOut(DATA, CLOCK, MSBFIRST, 0);
  }
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);
  digitalWrite(SEG, LOW);

  // SystemInfomation
  Serial.println("1inch ElectromagneticDisplay-Clock Starting up...");
  Serial.println();
  
  Serial.print( F("Heap: ") ); Serial.println(system_get_free_heap_size());
  Serial.print( F("Boot Vers: ") ); Serial.println(system_get_boot_version());
  Serial.print( F("CPU: ") ); Serial.println(system_get_cpu_freq());
  Serial.print( F("SDK: ") ); Serial.println(system_get_sdk_version());
  Serial.print( F("Chip ID: ") ); Serial.println(system_get_chip_id());
  Serial.print( F("Flash ID: ") ); Serial.println(spi_flash_get_id());
  Serial.print( F("Flash Size: ") ); Serial.println(ESP.getFlashChipRealSize());
  Serial.print( F("Vcc: ") ); Serial.println(ESP.getVcc());
  Serial.println();
  
  Serial.print( F("FirmwareVersion: ") );Serial.println(VERSION);
  Serial.print( F("DATE & Time: ") );Serial.print(__DATE__);Serial.println(__TIME__);
  Serial.println();

  // EEPROM
  EEPROM.begin(512);

  // 7Segment StartAnimation
  Sig7seg_clear_all();
  delay(1000);

  for (uint8_t dig=0; dig<4; dig++){
    Sig7seg_setSegments(0b10000000,dig);
    delay(200);
    Sig7seg_setSegments(0b11000000,dig);
    delay(200);
    Sig7seg_setSegments(0b11100000,dig);
    delay(200);
    Sig7seg_setSegments(0b11110000,dig);
    delay(200);
    Sig7seg_setSegments(0b11111000,dig);
    delay(200);
    Sig7seg_setSegments(0b11111100,dig);     
    delay(200); 
  }
  delay(2500);

  //I2C Setup
  byte error, address;
  int nDevices = 0;

  Wire.begin(2,14);
  Wire.setClock(400000L);
  
  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  
  //DS1307
  Rtc.Begin();
  Serial.println("RTC Powerup...");

  // if you are using ESP-01 then uncomment the line below to reset the pins to
  // the available pins for SDA, SCL
  // Wire.begin(0, 2); // due to limited pins, use pin 0 and 2 for SDA, SCL
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  if (!Rtc.IsDateTimeValid()) 
  {
      // Common Cuases:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");

      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      Rtc.SetDateTime(compiled);
  }

  if (!Rtc.GetIsRunning())
  {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) 
  {
      Serial.println("RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
  }
  else if (now > compiled) 
  {
      Serial.println("RTC is newer than compile time. (this is expected)");
  }
  else if (now == compiled) 
  {
      Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.SetSquareWavePin(DS1307SquareWaveOut_1Hz); 
  attachInterrupt(digitalPinToInterrupt(RTC_INT), RTCInterrupt, FALLING);
  
  //SSTS751
  stts.setup();
  Serial.println("Temperature Powerup...");
  
  // PCF8574
  pcf8574.begin();
  Serial.println("I/O-Extention Powerup...\n");
  
  // Most ready-made PCF8574-modules seem to lack an internal pullup-resistor, so you have to use the ESP8266-internal one.
  pcf8574.write(LED_1,LOW);//LED1
  pcf8574.write(LED_2,HIGH);//LED2
  pcf8574.write(LED_3,HIGH);//LED3
  pcf8574.write(SW_1,HIGH);//SW1
  pcf8574.write(SW_2,HIGH);//SW2
  pcf8574.write(SW_3,HIGH);//SW3
  
  pcf8574.resetInterruptPin();
  attachInterrupt(digitalPinToInterrupt(PFC_INT), PCFInterrupt, FALLING);

  // wifi
  wifiManager.setConfigPortalTimeout(150);
  wifiManager.autoConnect();
  pcf8574.write(LED_2,LOW);//LED2
  Serial.print("WiFi network started");
  delay(500);

  while (WiFi.status() != WL_CONNECTED)
  {
    pcf8574.write(LED_2,!pcf8574.read(LED_2));

    delay(500);
    Serial.print(".");
    
    if(micros()>1000*360)break;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    
    Serial.println("WiFi-Connecting...");
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp8266.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if (!MDNS.begin("esp8288")) {
      Serial.println("Error setting up MDNS responder!");
      while(1) { 
        delay(1000);
      }
    }
    Serial.println("mDNS responder started");

    // Start TCP (HTTP) server
    server.begin();
    Serial.println("TCP server started");
    
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", LOCAL_PORT_WEB);

    // Start UDP Service
    udp.begin(LOCAL_PORT_UDP);
    Serial.print("UDP server started(Local port): ");
    Serial.println(udp.localPort());

    pcf8574.write(LED_3,LOW);//LED3
  }

  // Initialize END
  delay(2000);
  pcf8574.write(LED_1,HIGH);//LED1
  pcf8574.write(LED_2,HIGH);//LED2
  pcf8574.write(LED_3,HIGH);//LED3

  // TimeDisplay
  timedisplay();
}

// Main Loop Section
void loop() {

  //////////////////////////
  // Recive Packet interrupt
  //////////////////////////
  while (millis() - beginWait < 850) { // Packet TimeOutLoop
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      // Serial.print("packet received, length=");
      // Serial.println(size);
      getNtppacket(ntp_response_delay);
      break;
    }
  }
  if(millis() - beginWait > 850 & send_ntp_flag){
    Serial.println("No NTP Response...");
    send_ntp_flag = false;
  }

  //////////////////////////
  // RTC interrupt
  //////////////////////////
  if(RTCInterruptFlag){
    int reading = analogRead(LUX);
    float voltage = ((long)reading * VOLT * 1000) / ANALOG_MAX;
    float microamp = (voltage * RES) / 1000;
    float lx = microamp / (290 / 100);
    
    float temperature = stts.readTemperature();

    // Debug Print
    // Serial.print(lx);
    // Serial.print(" lx / ");
    // Serial.print(temperature);
    // Serial.println("℃");
    
    // DS1307
    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    
    RtcDateTime dtime(Rtc.GetDateTime());

    // Debug Print
    //printDateTime(Rtc.GetDateTime());
    // Serial.print(dtime.Year());
    // Serial.print("/");
    // Serial.print(dtime.Month());
    // Serial.print("/");
    // Serial.print(dtime.Day());
    // Serial.print(" ");
    // Serial.print(dtime.Hour());
    // Serial.print(":");
    // Serial.print(dtime.Minute());
    // Serial.print(":");
    // Serial.println(dtime.Second());
    
    // HH:MM ClockDisplay
    if(!TempDispFlag & dtime.Second() == 0 & lx > 100 & DisplayFlag){
      
      // TimeDisplay
      timedisplay();

      Serial.println("ClockTime Diaplay");
      Serial.print(dtime.Year());
      Serial.print("/");
      Serial.print(dtime.Month());
      Serial.print("/");
      Serial.print(dtime.Day());
      Serial.print(" ");
      Serial.print(dtime.Hour());
      Serial.print(":");
      Serial.print(dtime.Minute());
      Serial.print(":");
      Serial.println(dtime.Second());

    }else if(TempDispFlag & lx > 100 & DisplayFlag){
      Sig7seg_setSegments(numbertable[(int)temperature/10],0);
      delay(SEG_DELAY);
      Sig7seg_setSegments(numbertable[(int)temperature%10],1);
      delay(SEG_DELAY);
      Sig7seg_setSegments(numbertable[(int)(temperature*10)-(int)temperature*10],2);
      delay(SEG_DELAY);
      Sig7seg_setSegments(numbertable[12],3);
      delay(SEG_DELAY);
    }else if(lx < 100){
      nightmode();
      Serial.println("NightMode");
    }

    // Send NTP-Packet
    if(dtime.Minute() % 15 == 0 & dtime.Second() == 30){
      IPAddress ntpServerIP;
      
      while (udp.parsePacket() > 0) ; // discard any previously received packets
      
      // Send NTP Packet
      WiFi.hostByName(ntpServerName, ntpServerIP);
      sendNTPpacket(ntpServerIP);

      // DelayCalculation
      ntp_response_delay = micros(); // Tc1
      beginWait = millis();
      send_ntp_flag = true;
    }
    
    // RTC FlagSet
    RTCInterruptFlag=false;
  }

  //////////////////////////
  // PFC interrupt(SW&LED)
  //////////////////////////
  if(PCFInterruptFlag){
    // DO NOTE: When you write LOW to a pin on a PCF8574 it becomes an OUTPUT.
    // It wouldn't generate an interrupt if you were to connect a button to it that pulls it HIGH when you press the button.
    // Any pin you wish to use as input must be written HIGH and be pulled LOW to generate an interrupt.
    
    Serial.println("Got an interrupt: ");
    if(pcf8574.read(SW_1)==LOW) {
      mt = !mt;

      // TimeDisplay
      timedisplay();

      // Debug Print
      //pcf8574.write(LED_1,!pcf8574.read(LED_1));
      // Serial.println("SW1 is LOW!");
    }
    if(pcf8574.read(SW_2)==LOW) {
      
      // FlagChange
      DisplayFlag = !DisplayFlag;

      if(DisplayFlag){
        // TimeDisplay
        timedisplay();
      }else{
        Sig7seg_setDash(0);
        delay(SEG_DELAY);
        Sig7seg_setDash(1);
        delay(SEG_DELAY);
        Sig7seg_setDash(2);
        delay(SEG_DELAY);
        Sig7seg_setDash(3);
        delay(SEG_DELAY);
      }
      
      // Debug Print
      //pcf8574.write(LED_2,!pcf8574.read(LED_2));
      //Serial.println("SW2 is LOW!");
    }
    if(pcf8574.read(SW_3)==LOW) {
      
      // FlagChange
      TempDispFlag= !TempDispFlag;

      // TimeDisplay
      delay(10);
      if(!TempDispFlag)timedisplay();

      // Debug Print
      //pcf8574.write(LED_3,!pcf8574.read(LED_3));
      //Serial.println("SW3 is LOW!");
    }

    // PFC FlagSet
    PCFInterruptFlag=false;
  }
  
  if (!Rtc.IsDateTimeValid()) 
  {
      // Common Cuases:
      //    1) the battery on the device is low or even missing and the power line was disconnected
      Serial.println("RTC lost confidence in the DateTime!");
  }
}
void timedisplay(){
  RtcDateTime dtime(Rtc.GetDateTime());

  uint8_t dhour = dtime.Hour();

  if(!mt){dhour = dhour % 12;}
  
  Sig7seg_setSegments(numbertable[dhour/10],0);
  delay(SEG_DELAY);
  Sig7seg_setSegments(numbertable[dhour%10],1);
  delay(SEG_DELAY);
  Sig7seg_setSegments(numbertable[dtime.Minute()/10],2);
  delay(SEG_DELAY);
  Sig7seg_setSegments(numbertable[dtime.Minute()%10],3);
  delay(SEG_DELAY);
}
void nightmode(){
  Sig7seg_setDash(0);
  delay(SEG_DELAY_NIGHTMODE);
  Sig7seg_setDash(1);
  delay(SEG_DELAY_NIGHTMODE);
  Sig7seg_setDash(2);
  delay(SEG_DELAY_NIGHTMODE);
  Sig7seg_setDash(3);
  delay(SEG_DELAY_NIGHTMODE);
}

// ===============================
// ** DS1307 Function Section
// ===============================
void printDateTime(const RtcDateTime& dt)
{
    clockdata[0] = dt.Year()/1000;
    clockdata[1] = dt.Year()/100%10;
    clockdata[2] = dt.Year()%100/10;
    clockdata[3] = dt.Year()%100%10;

    clockdata[4] = dt.Month()/10;
    clockdata[5] = dt.Month()%10;
    
    clockdata[6] = dt.Day()/10;
    clockdata[7] = dt.Day()%10;
    
    clockdata[8] = dt.Hour()/10;
    clockdata[9] = dt.Hour()%10;
    
    clockdata[10] = dt.Minute()/10;
    clockdata[11] = dt.Minute()%10;
    
    clockdata[12] = dt.Second()/10;
    clockdata[13] = dt.Second()%10;
    
  char datestring[20];    
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%04u/%02u/%02u %02u:%02u:%02u"),
            dt.Year(),
            dt.Month(),
            dt.Day(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}
/*-------- NTP code ----------*/
#define SECS_PER_MIN  ((time_t)(60UL))
#define SECS_PER_HOUR ((time_t)(3600UL))
#define SECS_PER_DAY  ((time_t)(SECS_PER_HOUR * 24UL))
#define DAYS_PER_WEEK ((time_t)(7UL))
#define SECS_PER_WEEK ((time_t)(SECS_PER_DAY * DAYS_PER_WEEK))
#define SECS_PER_YEAR ((time_t)(SECS_PER_WEEK * 52UL))
#define SECS_YR_2000  ((time_t)(946684800UL)) // the time at the start of y2k
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
#define YEAR_OFFSET 1970

typedef struct  { 
  uint8_t Second; 
  uint8_t Minute; 
  uint8_t Hour; 
  uint8_t Wday;   // day of week, sunday is day 1
  uint8_t Day;
  uint8_t Month; 
  uint8_t Year;   // offset from 1970; 
}   timeElements_t;
static timeElements_t tm;
static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0

// NTP Time Delay Calculation Use
//long ntp_response_delay,code_delay,beginWait;
long code_delay;
int8_t offset_sec;
uint32_t Ts1, Ts2;
float Ts1p, Ts2p;

// NTP TimeDelayCalculation
// *Originate Time Stamp:Tc1
// *Respons Time Stamp  :Tc2
// *Receive Time Stamp  :Ts1
// *Transmit Time Stamp :Ts2
//
// SendDelayTime : (Tc2-Tc1)-((Ts2-Ts1)+(Ts2p-Ts1p))
// 1-Way SendDelayTime : ((Tc2-Tc1)-((Ts2-Ts1)+(Ts2p-Ts1p)))/2
// Client TimeDelay : ((Ts2-Ts1)+(Ts2p-Ts1p))/2-(Tc2-Tc1)/2

time_t getNtppacket(long ntp_response_delay)
{

//  while (millis() - beginWait < 1500) { // Packet TimeOutLoop
//    int size = udp.parsePacket();
//    if (size >= NTP_PACKET_SIZE) {
      
  // NTP DelayCalculation
  ntp_response_delay = micros() - ntp_response_delay; // Tc2-Tc1
  code_delay = micros();
  
  Serial.println("Receive NTP Response");
  Serial.print("Tc2-Tc1 :");
  Serial.println(ntp_response_delay);
  
  udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
  
  unsigned long highWord ,lowWord;
  
  highWord = word(packetBuffer[32], packetBuffer[33]);
  lowWord = word(packetBuffer[34], packetBuffer[35]);
  unsigned long Ts1 = highWord << 16 | lowWord;
  
  highWord = word(packetBuffer[40], packetBuffer[41]);
  lowWord = word(packetBuffer[42], packetBuffer[43]);
  unsigned long Ts2 = highWord << 16 | lowWord;

  highWord = word(packetBuffer[36], packetBuffer[37]);
  lowWord = word(packetBuffer[38], packetBuffer[39]);
  uint32_t Ts1p = (highWord << 16 | lowWord)/ 4294967296.0 * 1000 * 1000;
  
  highWord = word(packetBuffer[44], packetBuffer[45]);
  lowWord = word(packetBuffer[46], packetBuffer[47]);
  uint32_t Ts2p = (highWord << 16 | lowWord)/ 4294967296.0 * 1000 * 1000;

  ntp_response_delay = (ntp_response_delay-(((Ts2-Ts1)*1000*1000)+(Ts2p-Ts1p)))/2; // microseconds(Ts2 - Ts1)
  int rtc_start_delay = 1*1000*1000-(ntp_response_delay+Ts2p);

  
  Serial.print("Ts1 :");
  Serial.println(Ts1);
  Serial.print("Ts2 :");
  Serial.println(Ts2);
  Serial.print("Ts1p:");
  Serial.println(Ts1p);
  Serial.print("Ts2p:");
  Serial.println(Ts2p);
  Serial.print("NTP ResponseTimeDelay:");
  Serial.println(ntp_response_delay);

  Serial.println("");
  
  Serial.print("Setting RTC Time:");
  Serial.println(Ts2+1);
  Serial.print("NTP SetTimeDelay(micro):");
  Serial.println(rtc_start_delay);
    
  // // PacketDump
  // for(int i=0;i<48;i++){
  //   Serial.print(packetBuffer[i]);
  //   Serial.print("\t");
  // }

  // now convert NTP time into everyday time:
  Serial.print("Unix time :");
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const unsigned long seventyYears = 2208988800UL;
  // subtract seventy years:
  unsigned long epoch = Ts2 - seventyYears + timeZone * SECS_PER_HOUR+1;
  // print Unix time:
  Serial.println(epoch);
  
  tm.Second = epoch % 60;
  epoch /= 60; // now it is minutes

  tm.Minute = epoch % 60;
  epoch /= 60; // now it is hours

  tm.Hour = epoch % 24;
  epoch /= 24; // now it is days
  
  tm.Wday = ((epoch + 4) % 7) + 1;  // Sunday is day 1 
  
  int year = 0;  
  unsigned long days = 0;
  while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= epoch) {
    year++;
  }
  tm.Year = year; // year is offset from 1970 
  
  days -= LEAP_YEAR(year) ? 366 : 365;
  epoch  -= days; // now it is days in this year, starting at 0
  
  days=0;
  int month=0;
  int8_t monthLength=0;
  for (month=0; month<12; month++) {
    if (month==1) { // february
      if (LEAP_YEAR(year)) {
        monthLength=29;
      } else {
        monthLength=28;
      }
    } else {
      monthLength = monthDays[month];
    }
    
    if (epoch >= monthLength) {
      epoch -= monthLength;
    } else {
        break;
    }
  }
  tm.Month = month + 1;  // jan is month 1  
  tm.Day = epoch + 1;     // day of month
  
  Serial.print("The UTC time is : "); 
  Serial.print(tm.Year + YEAR_OFFSET);
  Serial.print('/');
  Serial.print(tm.Month);
  Serial.print('/');
  Serial.print(tm.Day);
  Serial.print(' ');
  Serial.print(tm.Hour);
  Serial.print(':');
  Serial.print(tm.Minute);
  Serial.print(':');
  Serial.println(tm.Second);

  Rtc.SetIsRunning(false);

  RtcDateTime compiled = RtcDateTime(tm.Year + YEAR_OFFSET,tm.Month,tm.Day,tm.Hour,tm.Minute,tm.Second);//RtcDateTime(2005,12,10,3,15,00);
  Rtc.SetDateTime(compiled);
  code_delay = micros() - code_delay;
  
  rtc_setting_delay.once_ms((rtc_start_delay-code_delay)/1000, rtc_restart);
  Serial.print("Code Delay :");
  Serial.println(code_delay);

  send_ntp_flag = false;
}


// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  Serial.print("Transmit NTP Request: ");
  Serial.println(address);
  
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void rtc_restart(){
  Rtc.SetIsRunning(true);
  Serial.println("Setting NTP Time");
}

// ===============================
// ** 7Seg-1inch Electromagnetic Display Function Section
// ===============================

void Sig7seg_clear_all(void) {
  for (uint8_t l = 0; l < 2; l++) {
    for (uint8_t m = 0; m < 4; m++) {
      Sig7seg_setSegments(0b00000000,m);
      delay(25);
    }
    delay(50);
  }
}
void Sig7seg_setDash(uint8_t dig){
  Sig7seg_setSegments(0b00000010,dig);
}
void Sig7seg_write(int8_t d,uint8_t dig) {
  
  if (d > 15) return;
  if (d < 0) {
      Sig7seg_setDash(dig);
      return;
  }
  Sig7seg_setSegments(numbertable[d],dig);
}
void ChangeFunc(uint8_t DecimalNumber)
{
  int i,j;
  
  for(i = 0, j = 3 - 1; j >= 0; i++, j--)
  {
    if((DecimalNumber >> i) & 1){
      BinaryString[j] = 1;
    }else{
      BinaryString[j] = 0;
    }
  }
}

void Sig7seg_setSegments(uint8_t bits,uint8_t dig){

  uint8_t bits_1,bits_2;
  
  ChangeFunc(dig*2);

  for(uint8_t i = 0; i <2; i++){
    if(BinaryString[i] == 0){
      bitWrite(bits_1, i, 0);
    }else{
      bitWrite(bits_1, i, 1);
    }
  }
  
//    if(BinaryString[2] == 0){
//      digitalWrite(SEG, LOW);
//    }else{
//      digitalWrite(SEG, HIGH);
//    }
  
  for(uint8_t i = 0; i < 3; i++){
    if(bitRead(bits,i+5) == 1){
      bitWrite(bits_1, -(i*2)+6, 1);
      bitWrite(bits_1, -(i*2)+7, 0);
    }else{
      bitWrite(bits_1, -(i*2)+6, 0);
      bitWrite(bits_1, -(i*2)+7, 0);
    }
  }
  
  for(uint8_t i = 0; i < 4; i++){
    if(bitRead(bits,i+1) == 1){
      bitWrite(bits_2, -(i*2)+6, 1);
      bitWrite(bits_2, -(i*2)+7, 0);
    }else{
      bitWrite(bits_2, -(i*2)+6, 0);
      bitWrite(bits_2, -(i*2)+7, 0);
    }
  }
  
  uint8_t rbits_1,rbits_2;
  
  for(uint8_t i = 0; i <2; i++){
    if(BinaryString[i] == 0){
      bitWrite(rbits_1, i, 0);
    }else{
      bitWrite(rbits_1, i, 1);
    }
  } 
  for(uint8_t i = 0; i < 3; i++){
    if(bitRead(bits,i+5) == 1){
      bitWrite(rbits_1, -(i*2)+6, 0);
      bitWrite(rbits_1, -(i*2)+7, 0);
    }else{
      bitWrite(rbits_1, -(i*2)+6, 0);
      bitWrite(rbits_1, -(i*2)+7, 1);
    }
  }
  
  for(uint8_t i = 0; i < 4; i++){
    if(bitRead(bits,i+1) == 1){
      bitWrite(rbits_2, -(i*2)+6, 0);
      bitWrite(rbits_2, -(i*2)+7, 0);
    }else{
      bitWrite(rbits_2, -(i*2)+6, 0);
      bitWrite(rbits_2, -(i*2)+7, 1);
    }
  }
  
  noInterrupts();
  shiftOut(DATA, CLOCK, MSBFIRST, rbits_2);
  shiftOut(DATA, CLOCK, MSBFIRST, rbits_1);
  //interrupts();
  delayMicroseconds(SIGNAL_DELAY);
  
  digitalWrite(LATCH, HIGH);
  digitalWrite(SEG, LOW);
  delayMicroseconds(FLIP_TIME_B);
  digitalWrite(LATCH, LOW);
  interrupts();

  noInterrupts();
  shiftOut(DATA, CLOCK, MSBFIRST, bits_2);
  shiftOut(DATA, CLOCK, MSBFIRST, bits_1);
  //interrupts();
  delayMicroseconds(SIGNAL_DELAY);
  
  digitalWrite(LATCH, HIGH);
  digitalWrite(SEG, HIGH);
  delayMicroseconds(FLIP_TIME_W);
  digitalWrite(LATCH, LOW);
  interrupts();

//  noInterrupts();
//  for (uint8_t m = 0; m < 2; m++) {
//    shiftOut(DATA, CLOCK, MSBFIRST, 0b00000000);
//  }
//  interrupts();
//  delayMicroseconds(SIGNAL_DELAY);
//  
//  digitalWrite(LATCH, HIGH);
//  digitalWrite(LATCH, LOW);
}
