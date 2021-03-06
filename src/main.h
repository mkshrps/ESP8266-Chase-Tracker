/*
 * miniLoraTrackerGPS.h
 *
 *  Created on: 4 Apr 2019
 *      Author: mikes
 */

#ifdef _MAIN_

struct TGPS
{
  time_t  lastPacketAt;
  uint32_t time; 
  uint8_t Hours, Minutes, Seconds;
  unsigned long SecondsInDay;         // Time in seconds since midnight
  double Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  unsigned int failedCS;
  int Speed;
  int Direction;
  byte FixType;
  bool isValid;
} GPS;

struct remoteT
{
  char callSign[12];
  long  lastPacketAt;
  double longitude;
  double latitude;
  float alt;
  double  courseTo;
  double  distancem;
  char  cardinalCourseTo[20];
  float temperature_c;
  float humidity;
  bool  active;
  int satellites;
  int hours;
  int minutes;
  int seconds;
  int flightCount;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  bool isValid;
  int rssi;
  int currentRssi;
  char time[12];
  int speed;
  int heading;
} remote_data;

#else

extern struct TGPS
{
  time_t  lastPacketAt;
  uint32_t  time;
  uint8_t Hours, Minutes, Seconds;
  unsigned long SecondsInDay;         // Time in seconds since midnight
  double Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  unsigned int failedCS;
  int Speed;
  int Direction;
  byte FixType;
  } GPS;

extern int SentenceCounter;
extern unsigned char packet[];                 // used to send arbitrary LoRa Messages for diagnostics and gen info

extern struct remoteT
{
  char callSign[12];
  long  lastPacketAt;
  double longitude;
  double latitude;
  float alt;
  double  courseTo;
  double  distancem;
  char  cardinalCourseTo[20];
  float temperature_c;
  float humidity;
  bool  active;
  int satellites;
  int hours;
  int minutes;
  int seconds;
  int flightCount;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
  bool isValid;
  int rssi;
  int currentRssi;
  char time[12];
  int speed;
  int heading;
} remote_data;

#endif

void display_init(void);
void display_gps(void);
void display_direction_screen(void);
void display_frequency_page(void);
void display_signal_page(); 
void displayPage(int page);
void onesec_events(void);
void display_hab();
void setPitsMode(int mode);
void setCustomLoRaMode(int sf,int cr,float bw);







