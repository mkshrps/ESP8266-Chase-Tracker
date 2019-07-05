#ifndef _WIFI_H_
#define _WIFI_H_

#ifdef _WIFI_

#else
bool connectToWifi( const char* ssid, const char* password);
bool connectToWifiMulti( ESP8266WiFiMulti &WiFiMulti);


#endif


#endif
