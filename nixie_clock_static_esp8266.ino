#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <Time.h>
#include <TimeLib.h>

#define latchGPIO 12
#define clockGPIO 14
#define dataGPIO 16
#define anodeGPIO 13

String serial_buffer;
signed long target_poison_time = 0;
signed long update_time = 0;

char ssid[] = "Piston WiFi";
char pass[] = "voidvoidvoid";
boolean isNegotiated = false;

unsigned int localPort = 2390;
IPAddress timeServerIP;
const char* ntpServerName = "ru.pool.ntp.org";
const int timeZone = 2;     // Central European Time
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;

RTC_DS1307 rtc;

DateTime getTimeNow() { 
  DateTime now = rtc.now();
  return now;
}

void rtcTimeSetSerial() { 
  boolean txfinish = false;
  int date[6];
  for(int i = 0; i < 6; i++) date[i] = 0;
  while(txfinish != true) {
    String timeset_buffer = "";
    if(Serial.available()) timeset_buffer = Serial.readString();
    if(timeset_buffer != "") {
      int commaIndex = timeset_buffer.indexOf(',');
      int secondCommaIndex = 0;
      date[0] = timeset_buffer.substring(0, commaIndex).toInt();
      int date_i = 0;
      while(secondCommaIndex != -1) {
        secondCommaIndex = timeset_buffer.indexOf(',', commaIndex + 1);
        if(date_i < 6) date[++date_i] = timeset_buffer.substring(commaIndex + 1, secondCommaIndex).toInt();
        commaIndex = secondCommaIndex;
      }
    }
    if(date[5] > 0) txfinish = true;
  }
  rtc.adjust(DateTime(date[0], date[1], date[2], date[3], date[4], date[5]));
}

void disableTubes() {
  //analogWrite(anodeGPIO, 0);
  digitalWrite(anodeGPIO, LOW);
  digitalWrite(latchGPIO, LOW);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  digitalWrite(latchGPIO, HIGH);
}

void enableTubes() {
  digitalWrite(latchGPIO, LOW);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, 0b00000000);
  digitalWrite(latchGPIO, HIGH);
  digitalWrite(anodeGPIO, HIGH);
  //analogWrite(anodeGPIO, brightness);
}

boolean checkRunAntiPoison(signed long &target_time) { // check if it`s time to run anti cathode poisoning procedure, if so, return true and set counter to current UNIX time + 20 minutes
   DateTime currentTime = rtc.now();
   if(currentTime.unixtime() > target_time) {
    target_time = currentTime.unixtime() + 1200;
    return true;
   }
   else return false;
}

void preventCathodePoison() {
  if(isNegotiated) update_rtc_by_ntp();
  for(int i = 0; i < 10; i++) {
    byte shiftByte = i; 
    shiftByte = shiftByte << 4;
    shiftByte += i;
    digitalWrite(latchGPIO, LOW); 
    shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);
    shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);
    shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);
    digitalWrite(latchGPIO, HIGH);
    delay(300);
  }
}

void printTimeTubes(DateTime currentTime) {
  
  int currentHours = (int)currentTime.hour();
  int currentMinutes = (int)currentTime.minute();
  int currentSeconds = (int)currentTime.second();
  
  byte shiftByte = 0; 
  shiftByte = currentSeconds / 10 % 10; //seconds
  shiftByte = shiftByte << 4;
  shiftByte += currentSeconds % 10;
  digitalWrite(latchGPIO, LOW); 
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);

  shiftByte = 0; 
  shiftByte = currentMinutes / 10 % 10; //seconds
  shiftByte = shiftByte << 4;
  shiftByte += currentMinutes % 10;
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);

  shiftByte = 0; 
  shiftByte = currentHours / 10 % 10; //seconds
  shiftByte = shiftByte << 4;
  shiftByte += currentHours % 10;
  shiftOut(dataGPIO, clockGPIO, LSBFIRST, shiftByte);
  digitalWrite(latchGPIO, HIGH); 
}

void sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
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

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

void update_rtc_by_ntp() {
  time_t t = 0;
  while(t == 0) t = getNtpTime();
  tmElements_t tm;
  breakTime(t, tm);
  rtc.adjust(DateTime(tm.Year, tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second));
}

void setup() {
  pinMode(latchGPIO, OUTPUT);
  pinMode(clockGPIO, OUTPUT);
  pinMode(dataGPIO, OUTPUT);
  pinMode(anodeGPIO, OUTPUT);

  Serial.begin(115200);
  if (! rtc.begin()) { // Init RTC
    Serial.println("Couldn't find RTC");
    while (1);
  }

  WiFi.begin(ssid, pass);
  Serial.print("Connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if(millis() - start > 15000) break;
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if(!(WiFi.localIP().toString().equals("0.0.0.0"))) isNegotiated = true;
  if(isNegotiated) {
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP().toString());
    Serial.println("Starting UDP");
    udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
  }
  else Serial.println("Unable to connect");
  
  randomSeed(analogRead(0)); // for anti cathode poisoning procedure
  enableTubes();
}

void loop() {
  if(Serial.available()) {
    if(Serial.readString() == "timeset") rtcTimeSetSerial();
  }
  if(checkRunAntiPoison(target_poison_time)) preventCathodePoison();
  printTimeTubes(getTimeNow());
  delay(1000);
}
