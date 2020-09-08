#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "MHI-AC-Ctrl-core.h"

// Stored network credentials
const char* filename = "/credentials.txt";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String getTemperature() 
{
  return String("1");
}
  
String getHumidity() 
{
  return String("2");
}

String getPressure() 
{
  return String("3");
}

// Replaces placeholder with LED state value
String processor(const String& var)
{
  Serial.println(var);
  if(var == "STATE"){
    // do nothing for STATE yet....
  }
  else if (var == "TEMPERATURE"){
    return getTemperature();
  }
  else if (var == "HUMIDITY"){
    return getHumidity();
  }
  else if (var == "PRESSURE"){
    return getPressure();
  }  
}
 
void setup()
{
  boolean apmode = false;
  String credentials, ssid, passphrase;
  char * pch;
  String apssid = "ACWiFi " + String(ESP.getChipId());
  // Serial port for debugging purposes
  Serial.begin(115200);

  // Initialize SPIFFS
  if(!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Retrieve stored station SSID and passphrase from SPIFFS
  File f = SPIFFS.open(filename, "r");
  if(!f)
  {
    apmode = true;
    Serial.println("Failed to open credentials.txt from SPIFFS");
  }
  else
  {
    Serial.println("Reading credentials.txt from SPIFFS");
    for(int i = 0; i < f.size(); i++)
    {
      credentials += (char)f.read();
    }
    f.close();
    // Parse ssid and passphrase from credentials string
    int a = credentials.indexOf('\n');
    ssid = credentials.substring(0, a-1);
    passphrase = credentials.substring(a+1, credentials.length() - 1);
  }
  
  // Try Station WiFi stored credentials, and if they fail to connect after 30 seconds, start up in access point mode
  if(apmode)
  {
    Serial.println("AP Mode Starting, no STATION credentials found");
    WiFi.softAP(apssid); // start open access point
  }
  else
  {
    Serial.println("STATION Mode Starting");
    WiFi.begin(ssid, passphrase);
    for(int i = 0; i < 30; i++)
    {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(1000);      
    }
    if (WiFi.status() != WL_CONNECTED) 
    {
      Serial.println("AP Mode Starting, failed to connect to STATION");
      WiFi.softAP(apssid); // start open access point 
    }
  }
    
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  //server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request){
  //  digitalWrite(ledPin, HIGH);    
  //  request->send(SPIFFS, "/index.html", String(), false, processor);
  //});
  
  // Route to set GPIO to LOW
  //server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request){
  //  digitalWrite(ledPin, LOW);    
  //  request->send(SPIFFS, "/index.html", String(), false, processor);
  //});

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getTemperature().c_str());
  });
  
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getHumidity().c_str());
  });
  
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getPressure().c_str());
  });

  // Start server
  server.begin();
}
 
void loop()
{
  
}

void(* resetFunc) (void) = 0; // declare reset function at address 0, call resetFunc() to reset the system.
