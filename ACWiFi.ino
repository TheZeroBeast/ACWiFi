#include <ESP8266WiFi.h>
#include <stdio.h>
#include <string.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

#include "MHI-AC-Ctrl.h"

const PROGMEM char* kResponseOk = "true";
const PROGMEM char* kResponseFail = "false";

#define D_CMND_MHI_POWER "mhi"
#define D_CMND_MHI_MODE "Mode"
#define D_CMND_MHI_TSETPOINT "Tsetpoint"
#define D_CMND_MHI_FAN "Fan"
#define D_CMND_MHI_VANES "Vanes"
#define D_CMND_MHI_ERROPDATA "ErrOpData"

#define MESSZ 1040

const char kMhiCommands[] PROGMEM = "|" D_CMND_MHI_POWER "|" D_CMND_MHI_MODE "|" D_CMND_MHI_TSETPOINT "|" D_CMND_MHI_FAN "|" D_CMND_MHI_VANES "|" D_CMND_MHI_ERROPDATA;

const uint16_t TOPSZ = 151;

boolean firstruntemp = true;
float temptrim = 6.2;
uint8_t inputCount = 0;
String tworawpackets = "";
uint8_t* inputCountPnt = &inputCount;
String* tworawpacketsPnt = &tworawpackets;
unsigned long espledtimestamp;
unsigned long spibuffertimer;

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

#define ONE_WIRE_BUS 4          // D2, PIN for connecting temperature sensor DS18x20 DQ pin
#define TEMP_MEASURE_PERIOD 0.5  // period in seconds for temperature measurement with the external DS18x20 temperature sensor
#define SCK_PIN 14

OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress insideThermometer;     // arrays to hold device address

class MhiEventHandler : public CallbackInterface 
{
public:
    bool cbiMhiEventHandlerFunction(const char* key, const char* value) {
      //Response_P(PSTR("%s"), value);
      //MqttPublishPrefixTopic_P(TELE, key);
    }
};

MhiEventHandler mhiEventHandler;
MHIAcCtrl mhiAc = MHIAcCtrl(&mhiEventHandler);

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
  delay(2000);
  pinMode(16, OUTPUT); // set pin 16 as output for level shifter enable control
  digitalWrite(16, 1); // turn on level shifter
  EEPROM.begin(96); // 32 bytes for SSID and 64 bytes for PASSPHRASE
  pinMode(LED_BUILTIN, OUTPUT); 
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
  Serial.printf_P(PSTR("CPU frequency[Hz]=%lu\n"), F_CPU);
  Serial.println("Loading stored WiFi credentials from memory.");
  char ssidchar;
  for(int a = 0; a < 32; a++) // 32 char read from memory into ssid string
    ssid += char(EEPROM.read(a));
char passphrasechar;
  for(int b = 32; b < 96; b++) // 64 char read from memory into passphrase string
    passphrase += char(EEPROM.read(b));
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
  Serial.println("MHI SPI Initializing...");
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  Serial.println("MHI loop started...");
  mhiAc.loop();
  espledtimestamp = millis();
  spibuffertimer = millis();
}

void loop()
{
  unsigned long now = millis();
  ArduinoOTA.handle();
  if (now - espledtimestamp > 1000)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    espledtimestamp = now;
  }
}
