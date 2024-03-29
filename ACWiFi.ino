#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <ir_MitsubishiHeavy.h>
#include <IRutils.h>

// IO pin definitions
#define SCK_PIN  14
#define MOSI_PIN 13
#define MISO_PIN 12

// set devicename, used for ArduinoOTA
String devicename = "ACWiFi-office";

// set pin used for IR recv signal input
const uint16_t kRecvPin = 5; // GPIO5 = D1 on Wemos D1/R1 mini

// set pin used for IR send signal output
const uint16_t kIrLedPin  = 4; // GPIO4 = D2 on Wemos D1/R1 mini

// IRremote config and init
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;  // Milli-Seconds
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);
IRsend irsend(kIrLedPin, true, false);
IRMitsubishiHeavy152Ac ac(kIrLedPin, true, false);
decode_results results;

// MQTT configuration and variables
const char* mqtt_server = "192.168.1.199";
const char* mqtt_username = "mqtt";
const char* mqtt_password = "mqtt";
const char* mqtt_discovery_topic =              "homeassistant/climate/office/config";
const char* mqtt_mode_command_topic =           "homeassistant/climate/office/mode";
const char* mqtt_swing_mode_command_topic =     "homeassistant/climate/office/swing";
const char* mqtt_fan_mode_command_topic =       "homeassistant/climate/office/fan";
const char* mqtt_temperature_command_topic =    "homeassistant/climate/office/temperature";
const char* mqtt_current_state_topic =          "homeassistant/climate/office/state";
char mqttbuffer[60];
WiFiClient espClient;
PubSubClient client(espClient);
String currentmode = "";
String currentswing = "";
String currentfan = "";
String currenttargettemp = "";
String currentroomtemp = "";
bool   current3D = false;
bool   new3D = false;
bool   currentFanAuto = false;
bool   newFanAuto = false;

int starttime = 0;
int starttimediscovery = 0;

// WiFi credentials
const char* wifissid = "McKWiFi24GHz";
const char* wifipassword = "AlfieZephyr";

byte frame_no = 1; // Counter for how many times a frame variation has been sent (max. = 48)

bool checksumError = false;
bool newPayloadReceived = false;

byte newMode     = 0;
byte newVanes    = 0;
byte newFanspeed = 0;
byte newSetpoint = 0;

const byte mosi_frame_sig[3] = {0x6d, 0x80, 0x04}; // SPI frame start signature: first 3 bytes in a full SPI data frame. Used to sync to MHI SPI data in SPI_sync() routine. Might be different on other unit types!

// Bitfield:             1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
byte miso_frame[20] = {0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00};

byte mosi_frame[20]; // Array to collect a single frame of SPI data received from the MHI unit

byte mosi_bitfield4_10[7]; // Array containing bitfields 4-10 from rx_SPIframe, which holds current MHI mode, vanes, fans speed, ambient temperature and setpoint

//MODE bitmasks                            Bitfield #4
const byte modeMask[8][2]      { //     CLEAR   |    SET
                                   { 0b00100010, 0b00000000 },  //0 = Unchanged (only clear 'write' bits)
                                   { 0b00100011, 0b00000010 },  //1 = OFF
                                   { 0b00111111, 0b00110011 },  //2 = HEAT
                                   { 0b00111111, 0b00101011 },  //3 = COOL
                                   { 0b00111111, 0b00100011 },  //4 = AUTO
                                   { 0b00111111, 0b00100111 },  //5 = DRY
                                   { 0b00111111, 0b00101111 },  //6 = FAN
                                   { 0b00100011, 0b00000011 }}; //7 = ON (using last mode)

//VANES bitmasks                          Bitfield #4             Bitfield #5
const byte vanesMask[6][4]     { //     CLEAR   |    SET        CLEAR   |    SET
                                   { 0b10000000, 0b00000000, 0b10000000, 0b00000000 },  //0 = Unchanged (only clear 'write' bits)
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10000000 },  //1 = 1 (up)
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10010000 },  //2 = 2
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10100000 },  //3 = 3
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10110000 },  //4 = 4 (down)
                                   { 0b11000000, 0b11000000, 0b10000000, 0b00000000 }}; //5 = swing

//FANSPEED bitmasks                        Bitfield #5             Bitfield #10
const byte fanspeedMask[5][4]  { //     CLEAR   |    SET        CLEAR   |    SET
                                   { 0b00001000, 0b00000000, 0b11011000, 0b00000000 },  //0 = Unchanged (only clear 'write' bits)
                                   { 0b00001111, 0b00001000, 0b11011000, 0b00000000 },  //1 = Speed 1
                                   { 0b00001111, 0b00001001, 0b11011000, 0b00000000 },  //2 = Speed 2
                                   { 0b00001111, 0b00001010, 0b11011000, 0b00000000 },  //3 = Speed 3
                                   { 0b00001111, 0b00001010, 0b11011000, 0b00010001 }}; //4 = Speed 4

// Handle recieved MQTT message, just print it
void callback(char* topic, byte* payload, unsigned int length) 
{
  DynamicJsonDocument jsonBuffer(1024);
  String messageReceived="";
  for (int i = 0; i < length; i++) messageReceived+=((char)payload[i]);  
  Serial.println("Received message:");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(messageReceived);
  if (strcmp(topic, mqtt_mode_command_topic) == 0)
  {
     if( messageReceived == "off")           {newMode = 1; Serial.println("Turn Off.");}
     else if( messageReceived == "heat")     {newMode = 2; new3D = true; Serial.println("Turn On - Heat.");}
     else if( messageReceived == "cool")     {newMode = 3; new3D = true; Serial.println("Turn On - Cool.");}
     else if( messageReceived == "auto")     {newMode = 4; Serial.println("Turn On - Auto.");} 
     else if( messageReceived == "dry")      {newMode = 5; Serial.println("Turn On - Dry.");}
     else if( messageReceived == "fan_only") {newMode = 6; Serial.println("Turn On - Fan.");}
     if ( messageReceived != "off" && currentFanAuto) newFanAuto = true;
  }
  else if (strcmp(topic, mqtt_swing_mode_command_topic) == 0)
  {
     current3D = false;
     if ( messageReceived == "Lowest" )      {newVanes = 1; Serial.println("Vanes set to Lowest.");}
     else if( messageReceived == "Low" )     {newVanes = 2; Serial.println("Vanes set to Low.");}
     else if( messageReceived == "Middle" )  {newVanes = 3; Serial.println("Vanes set to Middle.");}
     else if( messageReceived == "High" )    {newVanes = 4; Serial.println("Vanes set to High.");} 
     else if( messageReceived == "Swing" )   {newVanes = 5; Serial.println("Vanes set to Swing.");}
     else if( messageReceived == "3DAuto" )  {new3D = true; Serial.println("Vanes set to 3DAuto.");}
  }
  else if (strcmp(topic, mqtt_fan_mode_command_topic) == 0)
  {
    currentFanAuto = false;
     if ( messageReceived == "Low" )         {newFanspeed = 1; Serial.println("Fan speed set to Low.");}
     else if( messageReceived == "Med" )     {newFanspeed = 2; Serial.println("Fan speed set to Med.");}
     else if( messageReceived == "High" )    {newFanspeed = 3; Serial.println("Fan speed set to High.");}
     else if( messageReceived == "Max" )     {newFanspeed = 4; Serial.println("Fan speed set to Auto.");}
     else if( messageReceived == "Auto" )    {newFanAuto = true; Serial.println("Fan speed set to Auto.");}
  }
  else if (strcmp(topic, mqtt_temperature_command_topic) == 0)
  {
     newSetpoint = messageReceived.toFloat();
     Serial.println("Target temperature set to " + String(newSetpoint));
  }
}

// Reconnect to MQTT broker
void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ACWiFi-office", mqtt_username, mqtt_password))
    {
      Serial.println("connected to MQTT broker!");
      
      client.setBufferSize(2048);
      // suscribe to MQTT topics
      Serial.print("Subscribe to homeassistant/.. topics. Status=");
      if ( client.subscribe(mqtt_mode_command_topic) &&
           client.subscribe(mqtt_swing_mode_command_topic) &&
           client.subscribe(mqtt_fan_mode_command_topic) &&
           client.subscribe(mqtt_temperature_command_topic))
         Serial.println("OK"); else Serial.println("FAILED");
    }
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Trying again during next program loop.");
      return;
    }
  }
}

void update_checksum()
{
  uint16_t sum = 0;
  for (int bf = 0; bf < 18; bf++) sum += miso_frame[bf];                                                          //Calculate checksum by summing bitfield 1 - 18 of SPI frame
  miso_frame[18] = highByte(sum);                                                                                 //Calculate MSB and LSB of checksum and write to byte 19 and 20
  miso_frame[19] = lowByte(sum);
}

bool verify_checksum()
{
  uint16_t sum = 0;
  for (int bf = 0; bf < 18; bf++) sum += mosi_frame[bf];
  return (mosi_frame[18] == highByte(sum) && mosi_frame[19] == lowByte(sum));                             //Calculate MSB and LSB of checksum and compare with byte 19 and 20. Returns true if checksum is correct.
}

void exchange_payloads()
{
  bool sync = false;
  switch (frame_no)
  {
    case 0:
      bitClear(miso_frame[17], 2);                                                                        //Clear clock bit 3 in bitfield 18-> update checksum and resend for 24 cycles
      frame_no++;
      break;
    
    case 1:                                                                                               //<FRAME 24> Current frame variation has been sent 24 times -> clear clock bit in bit field 18 for the next 24 frames
      bitSet(miso_frame[17], 2);                                                                          //Clear clock bit 3 in bitfield 18-> update checksum and resend for 24 cycles
      if (verify_checksum())                                                                              //Verify checksum
        memcpy(&mosi_bitfield4_10, &mosi_frame[3], 7);                                                    //Get bitfields 4-10 from the last MHI SPI frame to use for the upcoming tx_SPIframe update
      frame_no = 0;
      
      //******************* CONSTRUCTION OF UPDATED BIT FIELDS *******************
      //Set 'state change' bits and 'write' bits if MQTT update received from ESP
      //otherwise only clear 'write' bits using masks from the xxxMask[0][] arrays
      //Bitfields 4, 5, 6, 10 are based on the last received MHI values (frame 47)
      miso_frame[3]  =  mosi_bitfield4_10[0] & ~modeMask[newMode][0];                                     //Clear mode bits (bitfield 4)
      miso_frame[3] |=  modeMask[newMode][1];                                                             //Set mode bits

      miso_frame[3] &= ~vanesMask[newVanes][0];                                                           //Clear vanes bits (bitfield 4)
      miso_frame[3] |=  vanesMask[newVanes][1];                                                           //Set vanes bits

      miso_frame[4]  =  mosi_bitfield4_10[1] & ~vanesMask[newVanes][2];                                   //Clear vanes bits (bitfield 5)
      miso_frame[4] |=  vanesMask[newVanes][3];                                                           //Set vanes bits

      miso_frame[4] &= ~fanspeedMask[newFanspeed][0];                                                     //Clear fanspeed bits (bitfield 5)
      miso_frame[4] |=  fanspeedMask[newFanspeed][1];                                                     //Set fanspeed bits

      bitWrite(mosi_bitfield4_10[6], 0, bitRead(mosi_bitfield4_10[6], 6));                                //Copy bit 7 from rx_SPIframe[9] to bit 1 as the status bits for fan speed 4 appear to be swapped (!?) between MISO and MOSI
      miso_frame[9] &=  ~0b00111111;                                                                      //Clear bits 1-6 and keep bits 7-8

      miso_frame[9] |=  (mosi_bitfield4_10[6] & ~fanspeedMask[newFanspeed][2]);
      miso_frame[9] |=  fanspeedMask[newFanspeed][3];                                                     //Set fanspeed bits

      //Construct setpoint bitfield (#6) from last MHI value or MQTT update
      if (newSetpoint == 0) miso_frame[5] = mosi_bitfield4_10[2] & ~0b10000000;                           //Copy last received MHI setpoint and clear the write bit
      else miso_frame[5] = (newSetpoint << 1) | 0b10000000;                                               //MQTT updated setpoint in degrees Celsius -> shift 1 bit left and set write bit (#8)

      //Reset all state changes
      newMode = 0;
      newVanes = 0;
      newFanspeed = 0;
      newSetpoint = 0;
      break;
  }
  update_checksum();                                                                                       //<FRAME 47> Collect the most recent bit fields 4-10 for constructing an updated tx_SPIframe after the upcoming frame (48)

  int SCKMillis = millis();               // time of last SCK low level
  while (millis() - SCKMillis < 5) { // wait for 5ms stable high signal to detect a frame start
    if (!digitalRead(SCK_PIN)) SCKMillis = millis();
  }

  // read/write MOSI/MISO frame
  for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { // read and write a data packet of 20 bytes
    byte MOSI_byte = 0;
    byte bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) { // read and write 1 byte
      while (digitalRead(SCK_PIN)) // wait for falling edge
      {
        if (millis() - SCKMillis > 100) return;
      } 
      if ((miso_frame[byte_cnt] & bit_mask) > 0)
        digitalWrite(MISO_PIN, 1);
      else
        digitalWrite(MISO_PIN, 0);
      while (!digitalRead(SCK_PIN)) {} // wait for rising edge
      if (digitalRead(MOSI_PIN))
        MOSI_byte += bit_mask;
      bit_mask = bit_mask << 1;
    }

    if (mosi_frame[byte_cnt] != MOSI_byte) {
      newPayloadReceived = true;
      mosi_frame[byte_cnt] = MOSI_byte;
    }

    if(verify_checksum()) checksumError = false;
    else checksumError = true;
  }
}

void sendDiscovery()
{
  char mqttdiscoverybuffer[2000];
  sprintf(mqttdiscoverybuffer, "{\"name\":\"office\", "
                                "\"mode_command_topic\":\"%s\", "
                                "\"mode_state_topic\":\"%s\", "
                                "\"mode_state_template\":\"{{ value_json.mode }}\", "
                                "\"swing_mode_command_topic\":\"%s\", "
                                "\"swing_mode_state_topic\":\"%s\", "
                                "\"swing_mode_state_template\":\"{{ value_json.swing_mode }}\", "
                                "\"fan_mode_command_topic\":\"%s\", "
                                "\"fan_mode_state_topic\":\"%s\", "
                                "\"fan_mode_state_template\":\"{{ value_json.fan_mode }}\", "
                                "\"temperature_command_topic\":\"%s\", "
                                "\"temperature_state_topic\":\"%s\", "
                                "\"temperature_state_template\":\"{{ value_json.target_temp }}\", "
                                "\"current_temperature_topic\":\"%s\", "
                                "\"current_temperature_template\":\"{{ value_json.current_temp }}\", "
                                "\"min_temp\":\"16\", "
                                "\"max_temp\":\"30\", "
                                "\"temp_step\":\"1.0\", "
                                "\"modes\":[\"off\", \"heat\", \"cool\", \"auto\", \"dry\", \"fan_only\"], "
                                "\"swing_modes\":[\"Lowest\", \"Low\", \"Middle\", \"High\", \"Swing\", \"3DAuto\"], "
                                "\"fan_modes\":[ \"Auto\", \"Low\", \"Med\", \"High\", \"Max\"], "
                                "\"unique_id\":\"office\" }",
    mqtt_mode_command_topic, mqtt_current_state_topic, mqtt_swing_mode_command_topic,
    mqtt_current_state_topic, mqtt_fan_mode_command_topic, mqtt_current_state_topic,
    mqtt_temperature_command_topic, mqtt_current_state_topic, mqtt_current_state_topic);
  client.publish(mqtt_discovery_topic, mqttdiscoverybuffer);
}

void sendState()
{
  if (!(mosi_frame[3] & 0b00000001))
    currentmode = "off";
  else if (((modeMask[0][0]^modeMask[2][0])&modeMask[2][1]) ==
          (((modeMask[0][0]^modeMask[2][0])&modeMask[2][1]) & mosi_frame[3]))
    currentmode = "heat";
  else if (((modeMask[0][0]^modeMask[6][0])&modeMask[6][1]) ==
          (((modeMask[0][0]^modeMask[6][0])&modeMask[6][1]) & mosi_frame[3]))
    currentmode = "fan_only";
  else if (((modeMask[0][0]^modeMask[5][0])&modeMask[5][1]) ==
          (((modeMask[0][0]^modeMask[5][0])&modeMask[5][1]) & mosi_frame[3]))
    currentmode = "dry";
  else if (((modeMask[0][0]^modeMask[4][0])&modeMask[4][1]) ==
          (((modeMask[0][0]^modeMask[4][0])&modeMask[4][1]) & mosi_frame[3]) &&
          (mosi_frame[3] & 0b00001000) != 0b00001000)
    currentmode = "auto";
  else if (((modeMask[0][0]^modeMask[3][0])&modeMask[3][1]) ==
          (((modeMask[0][0]^modeMask[3][0])&modeMask[3][1]) & mosi_frame[3]))
    currentmode = "cool";

  if (current3D)
    currentswing = "3DAuto";
  else if (0b01000000 & mosi_frame[3])
    currentswing = "Swing";
  else if ((0b00100000 & mosi_frame[4]) && (0b00010000 & mosi_frame[4]))
    currentswing = "High";
  else if (0b00100000 & mosi_frame[4])
    currentswing = "Middle";
  else if (0b00010000 & mosi_frame[4])
    currentswing = "Low";
  else if (!(0b00110000 & mosi_frame[4]))
    currentswing = "Lowest";
  if (currentFanAuto)
    currentfan = "Auto";
  else if (!(mosi_frame[4] & 0b00000111))
    currentfan = "Low";
  else if (mosi_frame[4] & 0b00000001)
    currentfan = "Med";
  else if ((mosi_frame[4] & 0b00000010) && !(mosi_frame[9] & 0b01000000))
    currentfan = "High";
  else if ((mosi_frame[4] & 0b00000010) && (mosi_frame[9] & 0b01000000))
    currentfan = "Max";

  int tempsetpoint = (mosi_frame[5] & 0x7F) /2; // bit masked so MSB ignored as we only need mosiframe[5](6:0)
  currenttargettemp = String(tempsetpoint).c_str();

  float roomtemp = (mosi_frame[6] - 61) / 4;
  currentroomtemp = String(roomtemp).c_str();

  // Not using errorcode below as unsure how to implement it in Home Assistant Climate integration?
  //int errorcode = mosi_frame[7];

  // build state payload and publish to state topic
  char mqttstatebuffer[250];
  sprintf(mqttstatebuffer, "{\"mode\":\"%s\", \"swing_mode\":\"%s\", \"fan_mode\":\"%s\", \"target_temp\":\"%s\", \"current_temp\":\"%s\""
  " }", currentmode.c_str(), currentswing.c_str(), currentfan.c_str(), currenttargettemp.c_str(), currentroomtemp.c_str());
  client.publish(mqtt_current_state_topic, mqttstatebuffer);
}

void sendPendingIRCmd()
{
  if ((new3D || newFanAuto) && currentmode != "off")
  {
    irrecv.disableIRIn();
    if (currentmode != "off") ac.setPower(true);
    else ac.setPower(false);
    if( currentmode == "heat")          ac.setMode(kMitsubishiHeavyHeat);
    else if( currentmode == "cool")     ac.setMode(kMitsubishiHeavyCool);
    else if( currentmode == "auto")     ac.setMode(kMitsubishiHeavyAuto); 
    else if( currentmode == "dry")      ac.setMode(kMitsubishiHeavyDry);
    else if( currentmode == "fan_only") ac.setMode(kMitsubishiHeavyFan);
    ac.setTemp(currenttargettemp.toInt());
    if (new3D)
    {
      new3D = false;
      current3D = true;
      ac.set3D(true);
    }
    else
    {
      if (currentswing == "Swing")
        ac.setSwingVertical(kMitsubishiHeavy152SwingVAuto);
      else if (currentswing == "High")
        ac.setSwingVertical(kMitsubishiHeavy152SwingVHigh);
      else if (currentswing == "Middle")
        ac.setSwingVertical(kMitsubishiHeavy152SwingVMiddle);
      else if (currentswing == "Low")
        ac.setSwingVertical(kMitsubishiHeavy152SwingVLow);
      else if (currentswing == "Lowest")
        ac.setSwingVertical(kMitsubishiHeavy152SwingVLowest);
    }
    if (newFanAuto)
    {
      newFanAuto = false;
      currentFanAuto = true;
      ac.setFan(kMitsubishiHeavy152FanAuto);
    }
    else if (currentfan == "Low")
      ac.setFan(kMitsubishiHeavy152FanLow);
    else if (currentfan == "Med")
      ac.setFan(kMitsubishiHeavy152FanMed);
    else if (currentfan == "High")
      ac.setFan(kMitsubishiHeavy152FanHigh);
    else if (currentfan == "Max")
      ac.setFan(kMitsubishiHeavy152FanMax);
    ac.send();
    irrecv.enableIRIn(true);
  }
}

String toBin(byte toBin)
{
  String temp = "";
  byte bitMask = 0b10000000;
  for (uint8_t i = 0; i < 8; i++) {
    if (toBin & bitMask) temp += "1";
    else temp += "0";
    bitMask = bitMask >> 1;
  } return temp;
}

void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifissid, wifipassword);
  Serial.print("Connecting to WiFi ..");
  int timeout = 30; // 30 second WiFi timeout
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (timeout == 0) return;
    timeout--;
  }
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected to SSID:" + String(wifissid));
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  digitalWrite(LED_BUILTIN, HIGH);
  bool setSleepMode(WIFI_MODEM_SLEEP); // set modem sleep mode
}

void initOTA() 
{
  if (WiFi.status() == WL_CONNECTED)
  {
    ArduinoOTA.setHostname(devicename.c_str());
  
    ArduinoOTA.onStart([]() {
      digitalWrite(LED_BUILTIN, LOW);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    });
    ArduinoOTA.onEnd([]() {
      digitalWrite(LED_BUILTIN, HIGH);
    });
    ArduinoOTA.onError([](ota_error_t error) {
      digitalWrite(LED_BUILTIN, HIGH);
    });
    ArduinoOTA.begin();
  }
}

void wifiReconnect()
{
  initOTA();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup() 
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  initWiFi();
  wifiReconnect();
  irrecv.enableIRIn(true); // start IR receiver
  irsend.begin();       // Start up the IR sender.
  sendDiscovery();
  starttime = millis();
  starttimediscovery = millis();
}

void loop() 
{
  exchange_payloads();
  sendPendingIRCmd();
  if (WiFi.status() == WL_CONNECTED)
  {
    ArduinoOTA.handle();
    if (!client.connected()) reconnect(); // Reconnect to MQTT broker if required
    client.loop(); // MQTT client loop
    if (irrecv.decode(&results))
    {
      uint16_t *raw_array = resultToRawArray(&results);
      // Find out how many elements are in the array.
      uint16_t length = getCorrectedRawLength(&results);
      // Send it out via the IR LED circuit.
      irsend.sendRaw(raw_array, length, 0);
      // Resume capturing IR messages. It was not restarted until after we sent
      // the message so we didn't capture our own message.
      irrecv.resume();
      // Deallocate the memory allocated by resultToRawArray().
      delete [] raw_array;
    }
    if (millis() - starttimediscovery > 30000) { sendDiscovery(); starttimediscovery = millis(); }
    if ((newPayloadReceived && millis() - starttime > 1000 && !checksumError) || (millis() - starttimediscovery > 30000 && !checksumError))
    {
      sendState();
      // Debug message
      //Serial.println(mqttbuffer);
      Serial.println("     Byte03   Byte04   Byte05   Byte06   Byte07   Byte08   Byte09   Byte10   Byte11   Byte12   Byte13   Byte14   Byte15   Byte16   Byte17 ");
      Serial.print("IN :");
      for (uint8_t i = 3; i < 18; i++)
        Serial.printf("%s,", toBin(mosi_frame[i]).c_str());
      Serial.println();
      Serial.print("OUT:");
      for (uint8_t i = 3; i < 18; i++)
        Serial.printf("%s,", toBin(miso_frame[i]).c_str());
      Serial.println();
      newPayloadReceived = false;
      starttime = millis();
    }
  }
  else wifiReconnect();
  yield();
}
