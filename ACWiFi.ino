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

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

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
  WiFi.mode(WIFI_OFF); // set initial wifi mode
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("********************************");
  Serial.println("*     AC WiFi Adaptor v1.0     *");
  Serial.println("*           start-up           *");
  Serial.println("********************************");
  // Initialize SPIFFS
  Serial.println("Mounting SPIFFS.");
  if(!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS.");
    Serial.println("Unable to continue booting.");
    return;
  }
  else
  {
    Serial.println("SPIFFS mounted successfully.");
    // Retrieve stored station SSID and passphrase from SPIFFS
    File f = SPIFFS.open(filename, "r");
    if(!f)
    {
      apmode = true;
      Serial.println("Failed to open credentials.txt from SPIFFS.");
    }
    else
    {
      Serial.println("Reading credentials.txt from SPIFFS.");
      for(int i = 0; i < f.size(); i++)
      {
        credentials += (char)f.read();
      }
      f.close();
      // Parse ssid and passphrase from credentials string
      int a = credentials.indexOf('\n');
      ssid = credentials.substring(0, a-1);
      passphrase = credentials.substring(a+1, credentials.length());
      Serial.println("STATION WiFi credentials loaded from SPIFFS.");
      Serial.println("SSID: " + ssid);
      Serial.println("PASSPHRASE: " + passphrase);
    }
  }  
  
  // Try Station WiFi stored credentials, and if they fail to connect after 30 seconds, start up in access point mode
  if(apmode)
  {
    Serial.println("AP Mode Starting, no STATION credentials found.");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(apssid); // start open access point
  }
  else
  {
    WiFi.mode(WIFI_STA);
    Serial.println("STATION Mode Starting.");
    WiFi.begin(ssid, passphrase);
    for(int i = 0; i < 30; i++) // 30 second timeout
    {
      if (WiFi.status() == WL_CONNECTED) break;
      Serial.println(String(i)+"s....");
      delay(1000);      
    }
    if (WiFi.status() != WL_CONNECTED) 
    {
      WiFi.disconnect(true); // disconnect from access point and turn off STATION mode.
      Serial.println("AP Mode Starting, failed to connect to STATION.");
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(local_IP, gateway, subnet);
      WiFi.softAP(apssid); // enable AP mode
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
