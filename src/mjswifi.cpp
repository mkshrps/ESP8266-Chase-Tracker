#define _WIFI_
#include <ESP8266WiFi.h>
#include <ESP8266WiFIMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include "mjswifi.h"
// We start by connecting to a WiFi network


bool connectToWifi(const char* ssid, const char* password){
    
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int tryCount = 0;
    while ((WiFi.status() != WL_CONNECTED) && tryCount++<30) {
        delay(500);
        Serial.print(".");
    }
    if(tryCount >= 30){
        return false;
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
}

bool connectToWifiMulti(ESP8266WiFiMulti &WiFiMulti){
    
    Serial.println();
    Serial.print("Connecting to ");

    // see if the user wants to add a wifi password etc
    int tryCount = 0;
    while(WiFiMulti.run() != WL_CONNECTED && tryCount < 30) {
        tryCount++;
        delay(500);
        Serial.print(".");
    }
    if(tryCount >= 30){
        Serial.println("WiFI Connect FAILED");
        Serial.println("WiFI Connect FAILED");
        Serial.println("WiFI Connect FAILED");

        return false;
    }
    Serial.println("WiFI Connect OK OK OK");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
}