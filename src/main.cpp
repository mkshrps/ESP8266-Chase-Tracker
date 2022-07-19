/*******************************************
 * LoRa Receiver with LCD support
 * Mike Sharps 15/7/2019
 * R  1.0.0
 * */

#define _MAIN_
#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
//#include <U8x8lib.h>
#include <LiquidCrystal_I2C.h>
#include "main.h"
#include "ESP8266WiFi.h"
#include <ESP8266WiFiMulti.h>
#include <habhub.h>
#include "calc_crc.h"
#include "mjswifi.h"
#include <LoRa_LIB.h>
#include "telemetry.h"

#define DEBUG 0
#define WIFI_ON 1

void onReceive(int packetSize);
#define ESP8266_R1

#ifdef ESP8266_R1
#define SS 15
#define RST -1      // not used
//#define DIO 0   // this works
#define DIO 16
//#define MISO    D12
//#define MOSI    D11
//#define SPICLK  D13

#endif
/*
#define SDA  2
#define SCL  14
#define DIO 4 //D2  // DIO 4
#define RST -1
#define SS  15 //D8  // GPIO 15
*/
#define _EP32_
//#define RST_PIN -1
#define LED_WARN            5
#define LED_OK              6
#define A0_MULTIPLIER      4.9
// define for lcd ascii library
#define I2C_ADDRESS 0x3C
#define NUM_PAGES   5
#define START_PAGE  0
#define HAB_UPDATE_TIMEOUT 15000

int habhubStatus = 0;
int habhubTimer = 20;
int oldHabTimer = 0;
int buttonLongDelay = 2000;
int buttonShortDelay = 500;
int buttonStart = 0;
int buttonEnd = 0;

void onReceive(int packetSize);

// Wifi Creds

ESP8266WiFiMulti WiFiMulti;
WiFiClient espClient;

static const uint32_t GPSBaud = 9600;
const char *ListenerID = "MJSCHASE";
//const char *habID = "LYCEUM";

char rxBuffer[256];
char lastValidString[250];

int printTimer = 200;
//int rssi = 0;
int snr = 0;

byte to_node = 1;
byte from_node = 10;
byte ID = 10;
byte flags = 0;

// object button
struct buttonT{
int buttonPin = 2;
int lastButtonState;
unsigned int lastDebounceTime;
unsigned int debounceDelay = 100;
bool buttonState ;
bool buttonPressed = false; 
} button1;

int currentPage = START_PAGE;

int timer  = 0; //millis(); // get current timer
int onesec = 0;
int elapsed = 0;
int newtimer = timer+10;
char disp_str[20];
int flightCount=0;
bool rxDone = false;
int startTimer, oldTimer;
// OLED display device 
//U8X8_SSD1306_128X64_NONAME_SW_I2C lcd(/* clock=*/ 5, /* data=*/ 4);

LiquidCrystal_I2C lcd(0x27,20,4);
//I2C display declaration
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

// The TinyGPS++ object
TinyGPSPlus gps;

static const int RXPin = 0, TXPin = 02;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
bool localGPS_valid = false, remoteGPS_valid = false;

// current lora frequency
long currentFrq = 434500000;  
//long currentFrq = 433650000; // calling mode frq  

char pbuff[100];
int listnerCount = 0;

long stepFrq(long frq);
int n=0;
int payloadLength = 0;    // set >0 for implicit
int loraMode = 1;         // this is whtat most people seem to use
//int loraMode = 5;         // calling mode


////  Setup   /////

void setup()
{
  
  Serial.begin(74880);
  // ss.begin(GPSBaud);


  Serial.println("Initialising Display");
  //delay(12000);  
  // start the wifi connection
  
  Wire.begin();//Change to Wire.begin() for non ESP.

  lcd.begin(20,4);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("HAB Track");
  lcd.setCursor(0,1);
  lcd.print("Connecting to WIFI");
  lcd.setCursor(0,2);
  lcd.print("\nTracker 434.5Mhz: ");

    //WiFiMulti.addAP("SmartCityControl", "38289743");   // add Wi-Fi networks you want to connect to
    WiFiMulti.addAP("*****", "*******");
    WiFiMulti.addAP("*****", "*******");   // add Wi-Fi networks you want to connect to
    

//  wifiok = connectToWifi(espClient,ssid,password);
  //wifiok = connectToWifiMulti(WiFiMulti);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("HAB Track");
  lcd.setCursor(0,1);
  if(WIFI_ON){
    lcd.print("Conn to WIFI:");
    lcd.setCursor(0,2);
    lcd.print(WiFi.SSID());
  }
  else{
    lcd.print("Disabled WIFI ");
  
  }
  
  delay(3000);
  
  //display_init();
  SPI.begin();    // default esp8266 SPI pins 
  LoRa.setPins(SS,-1,DIO);

// Lora SETUP //
  
  if(!LoRa.begin(currentFrq)){
    Serial.println("Lora not detected");
  }

  // set the LoRa parameters to standard PITS mode
  setPitsMode(loraMode); // test on SSDV mode
  
  //LoRa.setPreambleLength(preambleLength);
  //LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.setTxPower(10,PA_OUTPUT_PA_BOOST_PIN);
  
  Serial.println("LoRa in receive mode");

  ss.begin(9600);

  pinMode(button1.buttonPin,INPUT_PULLUP);
  GPS.isValid = false;
  remote_data.isValid = false;

  startTimer = oldTimer = millis();

  habhubTimer = oldHabTimer = startTimer;

    
}


long stepFrq(long frq){

  frq += 1000;
  
  LoRa.setFrequency(frq);
  Serial.print("New Frq = ");
  Serial.println(frq);
  return frq;

}


void setPitsMode(int mode){
    int sf,cr;
    long bw;
    payloadLength = 0;
  
  switch(mode){
    
    case 0:
      sf=11;
      bw=20.8E3;
      cr = 8;
      break;

    case 1:
      sf=6;
      bw=20.8E3;
      cr = 5;
      payloadLength = 255;
      break;

    case 2:
      sf=7;
      bw = 62.5E3;
      cr = 8;
      break;
    
    case 3:
      sf=7;
      bw = 20.8E3;
      cr = 8;
      break;

    case 5:
    sf = 11;
    bw=41.7E3;
    cr=8;
    break;

    default:
      sf=7;
      bw = 62.5E3;
      cr = 8;
      break;
    }

    LoRa.setSpreadingFactor(sf);
    LoRa.setSignalBandwidth(bw);
    LoRa.setCodingRate4(cr);
    
    Serial.print("SF = " );
    Serial.print(sf);
    Serial.print(" BW = " );
    Serial.print(bw);
    Serial.print(" CR = " );
    Serial.println(cr);

   //return payloadLength;

}


void setCustomLoRaMode(int sf,int cr,float bw){

    LoRa.setSpreadingFactor(sf);
    LoRa.setSignalBandwidth(bw);
    LoRa.setCodingRate4(cr);
    //(sf==6) payloadLength = ? 255 : 0  ;

    Serial.print("SF = " );
    Serial.print(sf);
    Serial.print(" BW = " );
    Serial.print(bw);
    Serial.print(" CR = " );
    Serial.println(cr);

}

void  readPacketsLoop(){
  //Serial.println("read loop");
  int packetSize = LoRa.parsePacket(payloadLength); // for implicit just set anything > 0
  if (packetSize) {
    onReceive(packetSize);
    
    long freqErr = LoRa.packetFrequencyError();
    Serial.println(freqErr);
    if(abs(freqErr) > 500){
      Serial.println("FRQ was: ");
      Serial.println(currentFrq);
      Serial.println("FRQ Error: ");
      Serial.println(freqErr);

      if(freqErr > 0){
        currentFrq -= abs(freqErr);
      }
      else{
        currentFrq += abs(freqErr);
      }

      LoRa.setFrequency(currentFrq);
      Serial.println("Correcting FRQ to: ");
      Serial.println(currentFrq);
    }
    // flag that we got a valid message 
    remote_data.active = true;
    remote_data.lastPacketAt = millis();
  }
  else{
    // check if it is a long time since we got a message
    if((millis() - remote_data.lastPacketAt) >= 20000){
      remote_data.active=false;
      remote_data.rssi = 0;
    }
  }
}

void printSatData(){
  Serial.println("location valid");
  Serial.println("altitude valid");
  snprintf(pbuff,100,"local Lng: %2.6f, Local Lat:%2.6f",GPS.Longitude,GPS.Latitude);
  Serial.println(pbuff);

  Serial.print("Distance (km) to Target: ");
  Serial.println(remote_data.distancem);
  Serial.print("Course to Target ");
  Serial.println(remote_data.courseTo);
  Serial.print("Human directions: ");
  Serial.println(gps.cardinal(remote_data.courseTo));

  Serial.print("satellites = ");
  Serial.println(gps.satellites.value());
  Serial.print("fixes = ");
  Serial.println(gps.sentencesWithFix());
  Serial.print("failed chk = ");
  Serial.println(gps.failedChecksum());
}


void loop()
{

  startTimer = millis();
  habhubTimer = startTimer;

  if((startTimer - oldTimer) > 100 ){
    oldTimer = startTimer;
    int currentRSSI = LoRa.currentRssi();
    //Serial.print(currentRSSI);
    remote_data.currentRssi = currentRSSI;
    if(n++ >= 10){
      onesec_events();
      n=0;
    }
    /*
    if(n++ >= 10){
      currentFrq = stepFrq(currentFrq);
      n=0;
    }
    */
  }

  // read the state of the switch into a local variable:
    int reading = digitalRead(button1.buttonPin);
    // If the switch changed, due to noise or pressing:
    if (reading != button1.lastButtonState) {
      // reset the debouncing timer
      button1.lastDebounceTime = millis();
    }

    if ((millis() - button1.lastDebounceTime) > button1.debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != button1.buttonState) {
        
        button1.buttonState = reading;

        // only toggle the LED if the new button state is HIGH
        if (button1.buttonState == LOW) {
          button1.buttonPressed = true;
          buttonStart = millis();
          buttonEnd = buttonStart;
        }
        // button just got released
        else{
          if(millis() - buttonStart > buttonLongDelay){
              // do long delay task
              if(currentPage == 4){
                if(loraMode == 1){
                  loraMode =3;
                  currentFrq = 434450E3;

                  LoRa.setFrequency(currentFrq);
                } 
                else{
                  loraMode = 1;
                  currentFrq = 434500E3;
                  LoRa.setFrequency(currentFrq);
                }
                setPitsMode(loraMode);
              }

        }
        else{
          // short delay so just change page  
          if(++currentPage > NUM_PAGES) currentPage = START_PAGE;
          displayPage(currentPage);
          }
              
        }
      }
    }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    button1.lastButtonState = reading;

    delay(10);

    readPacketsLoop();

    while (ss.available() > 0){
      char dat = ss.read();
      gps.encode(dat);
if(DEBUG == 1){
      Serial.print(dat);
}
    }
    //  Serial.println("\n decoded GPS");
      // load up the LoRa struct with data
      if(gps.location.isUpdated()){
        if (gps.location.isValid()){
          GPS.Longitude = (float) gps.location.lng();
          GPS.Latitude = (float) gps.location.lat();
        }

        if (gps.altitude.isValid()){
          GPS.Altitude = (long) gps.altitude.meters();
        }

        // if gps data is ready
        if (gps.location.isValid() && gps.altitude.isValid()){
          GPS.isValid = true;
          //Serial.println("values valid");
        }
        GPS.Satellites = (unsigned int) gps.satellites.value();
        GPS.failedCS = (unsigned int) gps.failedChecksum();
        if(gps.time.isValid() && gps.time.isUpdated() ){
        GPS.Hours = (uint8_t) gps.time.hour();
        GPS.Minutes = (uint8_t) gps.time.minute();
        GPS.Seconds = (uint8_t) gps.time.second();
        }

        if(GPS.isValid && remote_data.isValid){
          // get course and distance if we have a remote tracker
          remote_data.courseTo =gps.courseTo(GPS.Latitude,GPS.Longitude,remote_data.latitude,remote_data.longitude);
          remote_data.distancem = gps.distanceBetween(GPS.Latitude,GPS.Longitude,remote_data.latitude,remote_data.longitude);
          //remote_data.cardinalCourseTo = gps.cardinal(remote_data.courseTo);
        }
    }


    if(--printTimer <= 0){
      if(GPS.isValid){
        //printSatData();
      }
      else{
        Serial.println("No local GPS Data received");
      }
      printTimer = 200;
    }

      // Process message
      if(rxDone==true){
    
        // if message is not telemetry
        if(rxBuffer[0]=='$' && rxBuffer[1]=='?'){

          int remoteID = (int) rxBuffer[1];
          
          Serial.println("Remote Station " + String(remoteID) + " Active");
         }

        // data ok
        else {
          Serial.println(rxBuffer);
          
          char *tmpBuff = (char *) malloc(sizeof(rxBuffer));
          strcpy(tmpBuff,rxBuffer);
          // extract the telem values
          getTelemetryData(tmpBuff);
          
          free((void *)tmpBuff);
          if(remote_data.isValid){
            strcpy(lastValidString,rxBuffer);

          }
          // BuildSentence(txBuffer,rxBuffer,sizeof(txBuffer),remote_data.callSign);
          
          //rssi = LoRa.packetRssi();
          snr = LoRa.packetSnr();

          //remote_data.isValid = true;
        }
    rxDone = false;
    delay(50);
    }

    else{
      
      if(!remote_data.active){
        //remote_data.rssi = 0;
      }

    }

    // time to send to send to habhub
    if((habhubTimer - oldHabTimer) > HAB_UPDATE_TIMEOUT ){
      oldHabTimer = habhubTimer;
      // only send the last valid telemetry string if it's received anything
      if(strlen(lastValidString)){
        Serial.println(lastValidString);  
        habhubStatus = uploadTelemetryPacket( lastValidString , remote_data.flightCount , (char *)ListenerID);
      }
      else{
        habhubStatus = -2; // no valid data to upload yet
      }
      if(listnerCount++ > 2){
        listnerCount = 0;
        // update Listener
        if(GPS.isValid){
          uploadListenerPacket((char *)ListenerID,time(NULL),GPS.Latitude,GPS.Longitude,"Whip");
        }
      }
    }
}

void onReceive(int packetSize)
{
  if(packetSize <4){
    //Serial.println("****Invalid packet****");
    return;
  }
  
  // received a packet
  Serial.println("");
  Serial.print("Received packet ");

  memset(rxBuffer,0,sizeof(rxBuffer));

  // read packet
  int i;
  for (i = 0; i < packetSize; i++)
  {
    rxBuffer[i] = (char)LoRa.read(); 
    //Serial.print(rxBuffer[i]);
  }
  rxBuffer[i] = 0;

  // print RSSI of packet
  Serial.print(" RSSI ");
  remote_data.rssi = LoRa.packetRssi(); 
  Serial.println(remote_data.rssi);
  rxDone = true;
}

void onesec_events(){
  displayPage(currentPage);
  
}

void fivesec_events(){
  Serial.println("five second");
  return;
}

void tensec_events(){
  Serial.println("ten second");
  return;
}


/* *******************
 *  DISPLAY ROUTINES
 *  
 */

void displayPage(int page){
  switch(page) {
    case  0:
      display_direction_screen();
    break;
    case  1:
      display_hab();
    break;
    case  2:
      display_gps();
    break;
    case  3:
      display_signal_page();    
    break;
    case  4:
      display_frequency_page();    
    break;
    
  }
}
void display_init(){
   return;
}

void  display_frequency_page(void){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Curr Frq ");
      lcd.print(currentFrq);

      lcd.setCursor(0,1);
      lcd.print("RSSI ");
      lcd.print(remote_data.rssi);
      lcd.setCursor(0 ,2);
      lcd.print("Frq Error ");
      long freqErr = LoRa.packetFrequencyError();
      lcd.print(freqErr);
      lcd.setCursor(0 ,3);
      lcd.print("Lora Mode ");
      lcd.print(loraMode);
      

}

void  display_signal_page(void){
      lcd.clear();
      lcd.setCursor(0,0);
      if(remote_data.active){
        lcd.print("Receiving Signal");
      }
      else{
        lcd.print("*** No Signal ***");
      
      }

      lcd.setCursor(0,1);
      lcd.print("RSSI ");
      lcd.print(remote_data.rssi);
      lcd.setCursor(10 ,1);
      lcd.print("CRSS ");
      lcd.print(remote_data.currentRssi);
      lcd.setCursor(0,2);
      lcd.print("SNR ");
      lcd.print(snr);
      long freqErr = LoRa.packetFrequencyError();
      lcd.setCursor(8,2);
      lcd.print("Ferr ");
      lcd.print(freqErr);
      
      lcd.setCursor(0,3);
      lcd.print("HABHUB ");
      if(habhubStatus > 0){
        lcd.print("OK >> ");
        lcd.print(habhubStatus);
      }
      else{
        lcd.print("ERROR");
      }

}    

void display_gps(){
      double val;
      char dstr[20];

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("You Are Here");

      val = GPS.Latitude;
      dtostrf(val,4,6,dstr);
      lcd.setCursor(0,1);
      lcd.print("Lat: ");
      lcd.print(dstr);

      val = GPS.Longitude;
      dtostrf(val,4,6,dstr);
      lcd.setCursor(0,2);
      lcd.print("Lng: ");
      lcd.print(dstr);

      lcd.setCursor(0,3);
      lcd.print("Sats: ");
      lcd.print(GPS.Satellites);

      val = GPS.failedCS;
      lcd.setCursor(9,3);
      lcd.print("CRC ERR:");
      lcd.print(GPS.failedCS);

}
      

void display_hab(){
      
      double val;
      char dstr[20];
      //Serial.println("Display results on OLED");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("HAB is Here");
      if(remote_data.isValid){
        lcd.print(" *");
      }
      else
      {
        lcd.print("x");
      }
      
      val = remote_data.longitude;
      dtostrf(val,4,6,dstr);
      //Serial.println(dstr);
      lcd.setCursor(0,1);
      lcd.print("Lng: ");
      lcd.print(dstr);
      
      val = remote_data.latitude;
      dtostrf(val,4,6,dstr);
      lcd.setCursor(0,2);
      lcd.print("Lat: ");
      lcd.print(dstr);
    
      val = remote_data.alt;
      dtostrf(val,2,0,dstr);
      lcd.setCursor(0,3);
      lcd.print("Alt: ");
      lcd.print(dstr);

      lcd.setCursor(10,3);
      lcd.print("T:");
      lcd.print(remote_data.time);
      
      //itoa(remote_data.flightCount,dstr,10);
      //lcd.println(dstr);
   
   return;
}

void display_temp(){
   return;
}

void display_direction_screen(void)
{
  double  DirectionToHAB;
  int Clock;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("This Way:");
    if(GPS.isValid && remote_data.isValid){
    // Have both positions so we can calculate distance and direction
    
    int courseDeg = remote_data.courseTo;
    DirectionToHAB = floor(courseDeg / 30);
    if(DirectionToHAB == 0)
    {
      Clock = 12;
    } 
    else{
      Clock = DirectionToHAB;
    }

    lcd.setCursor(0,1);
    lcd.print("Dir:  ");
    lcd.print(Clock);
    lcd.print(" o'clock");
    
    lcd.setCursor(0,2);
    lcd.print("Dist: ");
    lcd.print(remote_data.distancem);
    lcd.print("m");

    lcd.setCursor(0,3);
    lcd.print("Dir:  ");
    lcd.print(courseDeg);
    lcd.print("Deg > ");

    lcd.print(gps.cardinal(remote_data.courseTo));
    
  }
  else
  {
    lcd.setCursor(0,1);
    lcd.print("No direction ...");
    if (GPS.Satellites <= 3)
    {
      lcd.setCursor(0,2);
      lcd.print("No Local GPS");
      Serial.println("No Local Gps\n");
    }
    else{
      lcd.setCursor(0,2);
      lcd.print("Local GPS OK");
      Serial.println("Local Gps OK");
    }    

    if (remote_data.lastPacketAt == 0)
    {
      lcd.setCursor(0,3);
      lcd.print("No Remote GPS");
      Serial.println("No Remote GPS");
    }
  }
}

