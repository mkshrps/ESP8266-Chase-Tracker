
#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <base64.h>
#include "xbase64.h"
#include "sha256.h"


// #include "habhub.h"


void hash_to_hex( unsigned char *hash, char *line )
{
    int idx;

    for ( idx = 0; idx < 32; idx++ )
    {
        sprintf( &( line[idx * 2] ), "%02x", hash[idx] );
    }
    line[64] = '\0';
}

int SendHabPayload(char *payload,char *docID){
    int httpResponseCode=0;

    if(WiFi.status()== WL_CONNECTED){
    
        HTTPClient http;
        
        WiFiClient wifiClient;

        const char * url_base = "http://habitat.habhub.org/habitat/_design/payload_telemetry/_update/add_listener/%s";
//        const char * url_base_test = "http://192.168.1.6:1880/habitat";
        

        char url[strlen(url_base)+100];

//   trap any errors here use snprintf
// use this when testing with node red
//        sprintf( url, url_base_test, docID);
        Serial.println(payload);

        sprintf( url, url_base, docID);
        Serial.println(url);
        Serial.print("Send packet to habhub - Response=");

        http.begin(wifiClient,url);
        http.addHeader("Accept","application/json");            
        http.addHeader("Content-Type","application/json");            
        http.addHeader("charsets","utf-8");            

        httpResponseCode = http.PUT(payload);   
        
        if(httpResponseCode>0){
        
            String response = http.getString(); 
        
            //Serial.print(httpResponseCode);
            Serial.println(response);          

        }
        else{
        
            Serial.print("Error on sending PUT Request: ");
            Serial.println(httpResponseCode);     
        }
        http.end();       
    }
    
    else{
        Serial.println("Error in WiFi connection");
    }
    return httpResponseCode;
}


// Send telemetry to habhub or local map plotter
int uploadTelemetryPacket( char * Sentence , int packetNumber, char * callSign)
{

        SHA256_CTX ctx;
        unsigned char hash[32];
        char doc_id[70];
        char json[250]; 
        char now[32];
        time_t rawtime;
        struct tm *tm;
               
        // Get formatted timestamp
        time( &rawtime );
        tm = gmtime( &rawtime );
        tm->tm_year = 2019;
        tm->tm_mon =  7;
        tm->tm_mday = 19;
        strftime( now, sizeof( now ), "%Y-%0m-%0dT%H:%M:%SZ", tm );
        int len = strlen(Sentence); 
        
        /*
        Serial.print("Sentence Length = ");
        Serial.print(len);
        Serial.print("--");
        Serial.println(Sentence[len-2]);        
        
        //Sentence[len] = '\n';
        //Sentence[len + 1] = '\0';
        
        Serial.print("HABHUB Payload = ");
        Serial.println(Sentence);

        // Grab current telemetry string and append a linefeed
        
        // Convert sentence to base64 for transmission to the web server
        /*
        xbase64_encode( Sentence, strlen( Sentence ), &base64_length,
                       base64_data );
        base64_data[base64_length] = '\0';

        // Take SHA256 hash of the base64 version and express as hex.  This will be the document ID
        sha256_init( &ctx );
        sha256_update( &ctx, base64_data, base64_length );
        sha256_final( &ctx, hash );
        hash_to_hex( hash, doc_id );
*/      

        String base64_data = base64::encode(Sentence,false);
        Serial.println(base64_data.c_str());
        sha256_init( &ctx );
        sha256_update( &ctx, (char *) base64_data.c_str(), base64_data.length());
        sha256_final( &ctx, hash );
        // create doc_id as hex
        hash_to_hex( hash, doc_id );

        // Serial.println(doc_id);
        char counter[10];
        sprintf( counter, "%d", packetNumber);

        // Create json with the base64 data in hex, the tracker callsign and the current timestamp
        sprintf( json,
                 "{\"data\": {\"_raw\": \"%s\"},\"receivers\": {\"%s\": {\"time_created\": \"%s\",\"time_uploaded\": \"%s\"}}}",
                 base64_data.c_str(), callSign, now, now );

        int response = SendHabPayload(json,doc_id);
        return response;

//        sprintf( url, "http://habitat.habhub.org/habitat/_design/payload_telemetry/_update/add_listener/%s", doc_id);
//        sprintf( url, "http://192.168.1.6:1880/habitat/");

        // send to the node red http endpoint url 
//        sprintf( url, "http://192.168.1.6:1880/habitat/");

        // Set the headers
        //headers = NULL;
        //headers = curl_slist_append(headers, "Accept: application/json");
        //headers = curl_slist_append(headers, "Content-Type: application/json");
        //headers = curl_slist_append(headers, "charsets: utf-8" );

        // PUT to http://habitat.habhub.org/habitat/_design/payload_telemetry/_update/add_listener/<doc_id> with content-type application/json

        
		        // send to url here
        //Serial.println(json);

}

int uploadListenerPacket(char *callsign, time_t gps_time, float gps_lat, float gps_lon, const char *antenna)
{
       int httpResponseCode=0;
       char JsonData[200];
       char payload[300];
       
       WiFiClient wifiClient;
       HTTPClient http;   
       // const char * strTime = "2019-05-04T13:00:00";
       
       if(WiFi.status()== WL_CONNECTED){
    
       sprintf( JsonData, "{\"latitude\": %f, \"longitude\": %f, \"chase\": %s}",
                     gps_lat, gps_lon, "true" );
        
        sprintf( payload, "callsign=%s&time=%d&data=%s", callsign,
                     (int) time(NULL), JsonData );
        Serial.print("Listener Payload: ");
        Serial.println(payload);
        
        
        //http.addHeader("Accept","application/json");            
        //http.addHeader("Content-Type","application/json");            
        http.begin(wifiClient,"http://habitat.habhub.org/transition/listener_telemetry");
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        http.addHeader("charsets","utf-8");            
        
       int  httpResponseCode = http.POST(payload);   
        
        if(httpResponseCode>0){
        
            String response = http.getString(); 
        
            Serial.println(httpResponseCode);
            Serial.println(response);          
        
        }
        else{
        
            Serial.print("Error on sending PUT Request: ");
            Serial.println(httpResponseCode); 
        }
        http.end();       
    }
    
    else{
        Serial.println("Error in WiFi connection");
    }

       if(WiFi.status()== WL_CONNECTED){
    
        // Now specify the POST data
        sprintf( JsonData, "{\"radio\": \"%s\", \"antenna\": \"%s\"}",
                    "LoRa RFM98W", antenna );
        sprintf( payload, "callsign=%s&time=%d&data=%s", callsign, (int) time(NULL), JsonData );

        Serial.print("Listener Info Payload: ");
        Serial.println(payload);
        //http.addHeader("Accept","application/json");            
        //http.addHeader("Content-Type","application/json");            
        
        //http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        //http.addHeader("charsets","utf-8");            


        http.begin(wifiClient,"http://habitat.habhub.org/transition/listener_information");
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        http.addHeader("charsets","utf-8");            

//        http.begin("192.168.1.6:1880/habitat");
        
       int  httpResponseCode = http.POST(payload);   
        
        if(httpResponseCode>0){
        
            String response = http.getString(); 
        
            Serial.println(httpResponseCode);
            Serial.println(response);          
        
        }
        else{
        
            Serial.print("Error on sending PUT Request: ");
            Serial.println(httpResponseCode);     
        }
        http.end();       
    }
    
    else{
        Serial.println("Error in WiFi connection");
    }

return httpResponseCode;
}



/*  typical json string 
{"data": {"_raw": "JCRNSlMwMSwtMi41MDY2MTgsNTMuMjI2ODg3LDQ0LDEyLDE3LDQzLDQ3KjIxMDAKCg=="},"receivers": {"MJS01": {"time_created": "1970-01-01T00:05:01Z","time_uploaded": "1970-01-01T00:05:01Z"}}}
*/