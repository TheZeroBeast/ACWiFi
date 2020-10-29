#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <stdio.h>
#include <string.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <WebSerial.h>
#include "html.h"
#include <EEPROM.h>
#include <SPISlave.h>

boolean firstruntemp = true;
float temptrim = 6.2;
int espledflashcounter = 0;
boolean spidatareceived = false;
uint32_t savedInts;
uint8_t* inputBuffer;
String tworawpackets;
int rawpacketcounter = 0;

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

#define ONE_WIRE_BUS 4          // D2, PIN for connecting temperature sensor DS18x20 DQ pin
#define TEMP_MEASURE_PERIOD 0.5  // period in seconds for temperature measurement with the external DS18x20 temperature sensor
#define SCK_PIN 14

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer;     // arrays to hold device address

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++)
  {
    d += char(data[i]);
  }
  WebSerial.println(d);
}

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
  if ((millis() - DS1820Millis > TEMP_MEASURE_PERIOD * 1000)||(firstruntemp)) 
  {
    temperature = String(float(sensors.getTempC(insideThermometer)) - temptrim);
    DS1820Millis = millis();
    sensors.requestTemperatures();
    firstruntemp = false;
  }
  return temperature;
}

String getTemperature() 
{
  return getDs18x20Temperature();
}

String getCurrentTempSetting()
{
  return String(float(25.0));
}
  
// Replaces placeholder with LED state value
String processor(const String& var)
{
  //Serial.println(var); // turn on for debugging purposes
  if(var == "STATE"){
    // do nothing for STATE yet....
  }
  else if (var == "TEMPERATURE"){
    return getTemperature();
  }
}

void UpdateWiFiCreds(String ssid, String pass)
{
  // Erase EEPROM locations
  for(int e = 0; e < 96; e++)
    {
      EEPROM.write(e,0);
    }
  EEPROM.commit();
  
  // Write ssid
  for(int a = 0; a < 32; a++) // 32 char read from memory into ssid string
    {
      EEPROM.write(a, ssid[a]);
      Serial.println("Writing :" + ssid[a]);
    }
  
  // Write passphrase
  for(int b = 32; b < 96; b++) // 64 char read from memory into passphrase string
    {
      EEPROM.write(b, pass[b-32]);
      Serial.println(pass[b-32]);
    }
  EEPROM.end(); 
}

void setup()
{
  pinMode(16, OUTPUT); // set pin 16 as output for level shifter enable control
  digitalWrite(16, 1); // turn on level shifter
  EEPROM.begin(96); // 32 bytes for SSID and 64 bytes for PASSPHRASE
  pinMode(LED_BUILTIN, OUTPUT); 
  pinMode(SCK_PIN, INPUT);
  boolean apmode = false;
  String ssid, passphrase;
  char * pch;
  String apssid = "ACWiFi " + String(ESP.getChipId());
  // Serial port for debugging purposes
  WiFi.mode(WIFI_OFF); // set initial wifi mode
  Serial.begin(115200);
  Serial.println("********************************");
  Serial.println("*     AC WiFi Adaptor v1.0     *");
  Serial.println("*           start-up           *");
  Serial.println("********************************");
  Serial.println(" ");
  // Load stored variables for ssid and passphrase
  Serial.println("Loading stored WiFi credentials from memory.");
  char ssidchar;
  for(int a = 0; a < 32; a++) // 32 char read from memory into ssid string
    {
      ssid += char(EEPROM.read(a));
    }
  char passphrasechar;
  for(int b = 32; b < 96; b++) // 64 char read from memory into passphrase string
    {
      passphrase += char(EEPROM.read(b));
    }  
  Serial.println("SSID: " + ssid);
  Serial.println("PASSPHRASE: " + passphrase);    
  Serial.println("STATION Mode Starting.");
  WiFi.mode(WIFI_STA);
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
  Serial.println("Initializing OTA support.");
  ArduinoOTA.setPassword("esp8266");
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");  
  Serial.println("Initializing DS18B20 temperature sensor.");
  setup_ds18x20();

// Route for api status request
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String JSON;
    StaticJsonDocument<1000> jsonBuffer;
    //load test data
    jsonBuffer["field1"] = "Test Field 1";
    jsonBuffer["field2"] = "Test Field 2";
    jsonBuffer["field3"] = "Test Field 3";
    jsonBuffer["field4"] = "Test Field 4";
    jsonBuffer["field5"] = "Test Field 5";
    jsonBuffer["field6"] = "Test Field 6";
    jsonBuffer["field7"] = "Test Field 7";
    jsonBuffer["field8"] = "Test Field 8";
    jsonBuffer["field9"] = "Test Field 9";
    jsonBuffer["field10"] = "Test Field 10";
    serializeJson(jsonBuffer, JSON);
    request->send(200, PSTR("text/html"), JSON);
  });

// Route for setup web page
  String setuppage = SETUP_page;
  server.on("/setup.html", HTTP_GET, [setuppage](AsyncWebServerRequest *request){
    request->send(200, "text/html", setuppage);
  });
  
  // Route for root / web page
  String indexpage = INDEX_page;
  server.on("/", HTTP_GET, [indexpage](AsyncWebServerRequest *request){
    request->send(200, "text/html", indexpage);
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getTemperature().c_str());
  });

  server.on("/currenttempsetting", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getCurrentTempSetting().c_str());
  });

  server.on("/updatewifi", HTTP_GET, [](AsyncWebServerRequest *request){
    String newssid;
    String newpass;
    if (request->hasParam("ssid"))
    {
      newssid = request->getParam("ssid")->value();
      newpass = request->getParam("pass")->value();
      UpdateWiFiCreds(newssid, newpass);
      request->send_P(200, "text/plain", "WiFi Settings Updated OK, restarting system in 5 seconds.");
      Serial.println("*******************************");
      Serial.println("*        Updating WiFi        *");
      Serial.println("*         Credentials         *");
      Serial.println("*******************************");
      Serial.println("New SSID: " + newssid);
      Serial.println("New PASS: " + newpass);
      Serial.println("*******************************");
      Serial.println("    Restarting system NOW!     ");
      Serial.println("*******************************");
      ESP.restart();
    }
    else
      request->send_P(200, "text/plain", "Nothing to update");
  });

  Serial.println("Starting webserial.");
  // Start webserial - accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);

  // Start web server
  Serial.println("Starting web server.");
  server.begin();  

  Serial.println("Initializing SPI.");
  SPISlave.begin(); 
  int startMillis = millis();
  int SCKMillis = millis();
  while (millis() - SCKMillis < 5)
  {
    if (!digitalRead(SCK_PIN)) SCKMillis = millis();
  }
  SPISlave.onData([](uint8_t * data, size_t len) {
    (void) len;
    savedInts = noInterrupts();  //disable interrupts
    inputBuffer = data;
    spidatareceived = true;
    });  
  //SPISlave.begin();  
}

void loop()
{  
  ArduinoOTA.handle();
  //if (espledflashcounter == 10)
  //{
  //  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //  espledflashcounter = 0;
  //}
  //espledflashcounter++;
  if (spidatareceived)
  {
    for (int i = 0; i < 32; i++) tworawpackets += String(((char *)inputBuffer)[i], HEX);
    rawpacketcounter++;
    if (rawpacketcounter == 2)
    {
      Serial.println(tworawpackets);
      tworawpackets = "";
      rawpacketcounter = 0;
    }
    spidatareceived = false;
    xt_wsr_ps(savedInts); // enable interrupts    
  }  
}
