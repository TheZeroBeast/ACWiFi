#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "MHI-AC-Ctrl-core.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Stored network credentials
const char* filename = "/credentials.txt";

boolean firstrun = true;

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

#define ONE_WIRE_BUS 4          // D2, PIN for connecting temperature sensor DS18x20 DQ pin
#define TEMP_MEASURE_PERIOD 0.5  // period in seconds for temperature measurement with the external DS18x20 temperature sensor

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer;     // arrays to hold device address

void setup_ds18x20() 
{
  sensors.begin();
  Serial.printf_P(PSTR("Found %i DS18xxx family devices.\n"), sensors.getDS18Count());
  if (!sensors.getAddress(insideThermometer, 0))
    Serial.println(F("Unable to find address for Device 0"));
  else
    Serial.printf_P(PSTR("Device 0 Address: 0x%02x\n"), insideThermometer);
  sensors.setResolution(insideThermometer, 9); // set the resolution to 9 bit
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); // Send the command to get temperatures
}

String getDs18x20Temperature() 
{
  String temperature;
  static unsigned long DS1820Millis = millis();  
  if ((millis() - DS1820Millis > TEMP_MEASURE_PERIOD * 1000)||(firstrun)) 
  {
    int16_t tempR = sensors.getTemp(insideThermometer);
    char strtmp[10];
    dtostrf(sensors.rawToCelsius(tempR), 0, 1, strtmp);
    temperature = strtmp;
    DS1820Millis = millis();
    sensors.requestTemperatures();
    firstrun = false;
  }
  return temperature;
}

String getTemperature() 
{
  return getDs18x20Temperature();
  //return String("1");
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
    for(int i = 0; i < 10; i++) // 30 second timeout
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
  
  Serial.println("Initializing DS18B20 temperature sensor.");
  setup_ds18x20();
  
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
