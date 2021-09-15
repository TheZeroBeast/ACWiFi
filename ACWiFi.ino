#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// IO pin definitions
#define SCK_PIN  14
#define MOSI_PIN 13
#define MISO_PIN 12

// set devicename, used for ArduinoOTA
String devicename = "ACWiFi-" + String(ESP.getChipId());

// set pin used for IR recv signal input
const uint16_t kRecvPin = 5; // GPIO5 = D1 on Wemos D1/R1 mini

// future note - GPIO4 = D2 on Wemos D1/R1 mini is IR signal out to MHI unit

// IRremote config and init
IRrecv irrecv(kRecvPin);

decode_results results;

// MQTT configuration and variables
const char* mqtt_server = "192.168.1.200";
const char* mqtt_username = "";
const char* mqtt_password = "";
const char* mqtt_domoticz_topic_in =           "domoticz/in";
const char* mqtt_domoticz_topic_out_mode =     "domoticz/out/163";
const char* mqtt_domoticz_topic_out_vane =     "domoticz/out/164";
const char* mqtt_domoticz_topic_out_fanspeed = "domoticz/out/165";
const char* mqtt_domoticz_topic_out_setpoint = "domoticz/out/167";
char mqttbuffer[60];
WiFiClient espClient;
PubSubClient client(espClient);

int starttime = 0;

// Domoticz IDX List - update the IDX values to suit your Domoticz configuration
const int idxroomtemp =         162;
const int idxmodeselector =     163;
const int idxvaneselector =     164;
const int idxfanspeedselector = 165;
const int idxtempsetpoint =     167;
const int idxerrorcode =        168;
const int idxoutdoortemp =      169;
const int idxirremotedata =     170;

// WiFi credentials
const char* wifissid = "Go Away";
const char* wifipassword = "away1234";

byte frame_no = 1; // Counter for how many times a frame variation has been sent (max. = 48)

bool checksumError = false;
bool newPayloadReceived = false;

byte newMode     = 0;
byte newVanes    = 0;
byte newFanspeed = 0;
byte newSetpoint = 0;
bool updatedMode     = false;
bool updatedVanes    = false;
bool updatedFanspeed = false;
bool updatedSetpoint = false;

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
  
  DeserializationError error = deserializeJson(jsonBuffer, messageReceived);
  if (error)
  {
     Serial.println("parsing Domoticz/out JSON Received Message failed");
     Serial.print(F("deserializeJson() failed with code "));
     Serial.println(error.c_str());
     return;
  }
  if (strcmp(topic, mqtt_domoticz_topic_out_mode) == 0) 
  {
     const char* command = jsonBuffer["svalue1"];
     if( strcmp(command, "0") == 0 )       {newMode = 1; Serial.println("Turn Off.");}
     else if( strcmp(command, "20") == 0 ) {newMode = 2; Serial.println("Turn On - Heat.");}
     else if( strcmp(command, "30") == 0 ) {newMode = 3; Serial.println("Turn On - Cool.");}
     else if( strcmp(command, "40") == 0 ) {newMode = 4; Serial.println("Turn On - Auto.");} 
     else if( strcmp(command, "50") == 0 ) {newMode = 5; Serial.println("Turn On - Dry.");}
     else if( strcmp(command, "60") == 0 ) {newMode = 6; Serial.println("Turn On - Fan.");}
     else if( strcmp(command, "10") == 0 ) {newMode = 7; Serial.println("Turn On - Last Mode.");}
  }
  else if (strcmp(topic, mqtt_domoticz_topic_out_vane) == 0)
  {
     const char* command = jsonBuffer["svalue1"];
     if( strcmp(command, "0") == 0 )      {newVanes = 1; Serial.println("Vanes set to 1.");}
     else if( strcmp(command, "10") == 0 ) {newVanes = 2; Serial.println("Vanes set to 2.");}
     else if( strcmp(command, "20") == 0 ) {newVanes = 3; Serial.println("Vanes set to 3.");}
     else if( strcmp(command, "30") == 0 ) {newVanes = 4; Serial.println("Vanes set to 4.");} 
     else if( strcmp(command, "40") == 0 ) {newVanes = 5; Serial.println("Vanes set to swing.");}
  }
  else if (strcmp(topic, mqtt_domoticz_topic_out_fanspeed) == 0)
  {
     const char* command = jsonBuffer["svalue1"];
     if( strcmp(command, "0") == 0 ) {newFanspeed = 1; Serial.println("Fan speed set to 1.");}
     else if( strcmp(command, "10") == 0 ) {newFanspeed = 2; Serial.println("Fan speed set to 2.");}
     else if( strcmp(command, "20") == 0 ) {newFanspeed = 3; Serial.println("Fan speed set to 3.");}
     else if( strcmp(command, "30") == 0 ) {newFanspeed = 4; Serial.println("Fan speed set to 4.");} 
  }
  else if (strcmp(topic, mqtt_domoticz_topic_out_setpoint) == 0)
  {
     float command = jsonBuffer["svalue1"];
     newSetpoint = command;
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
    if (client.connect("ACWiFi")) 
    {
      Serial.println("connected to MQTT broker!");
      
      client.setBufferSize(2048);
      // suscribe to MQTT topics
      Serial.print("Subscribe to domoticz/out/.. topics. Status=");
      if ( client.subscribe(mqtt_domoticz_topic_out_mode) &&
           client.subscribe(mqtt_domoticz_topic_out_vane) &&
           client.subscribe(mqtt_domoticz_topic_out_fanspeed) &&
           client.subscribe(mqtt_domoticz_topic_out_setpoint))
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
      if (verify_checksum())                                                                               //Verify checksum
      {
        memcpy(&mosi_bitfield4_10, &mosi_frame[3], 7);                                                    //Get bitfields 4-10 from the last MHI SPI frame to use for the upcoming tx_SPIframe update
        checksumError = false;
      }
      else checksumError = true;                                                                                             //<FRAME 48> Current frame variation has been send 48 times -> construct next frame variant using most recent bit fields 4-10 collected in frame 47
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
      if (byte_cnt == 3)
      {
        if ((mosi_frame[byte_cnt] & 0b00011101) != (MOSI_byte & 0b00011101)) updatedMode = true;
        if ((mosi_frame[byte_cnt] & 0b01000000) != (MOSI_byte & 0b01000000)) updatedVanes = true;
      }
      else if (byte_cnt == 4)
      {
        if ((mosi_frame[byte_cnt] & 0b00110000) != (MOSI_byte & 0b00110000)) updatedVanes = true;
        if ((mosi_frame[byte_cnt] & 0b00000111) != (MOSI_byte & 0b00000111)) updatedFanspeed = true;
      }
      else if (byte_cnt == 5 && mosi_frame[byte_cnt] != MOSI_byte) updatedSetpoint = true;
      else if (byte_cnt == 9 && (mosi_frame[byte_cnt] & 0b01000000) != (MOSI_byte & 0b01000000)) updatedFanspeed = true;
      newPayloadReceived = true;
      mosi_frame[byte_cnt] = MOSI_byte;
    }
  }
  if (updatedMode) domoticzModeUpdate();
  if (updatedVanes) domoticzVanesUpdate();
  if (updatedFanspeed) domoticzFanspeedUpdate();
  if (updatedSetpoint) domoticzSetpointUpdate();
}

void domoticzModeUpdate()
{
  updatedMode = false;
  if (!(mosi_frame[3] & 0b00000001))
  {
    Serial.println("Power Off.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Off\" }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (((modeMask[0][0]^modeMask[2][0])&modeMask[2][1]) ==
          (((modeMask[0][0]^modeMask[2][0])&modeMask[2][1]) & mosi_frame[3]))
  {
    Serial.println("Power On - Heat.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 20 }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (((modeMask[0][0]^modeMask[6][0])&modeMask[6][1]) ==
          (((modeMask[0][0]^modeMask[6][0])&modeMask[6][1]) & mosi_frame[3]))
  {
    Serial.println("Power On - Fan.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 60 }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (((modeMask[0][0]^modeMask[5][0])&modeMask[5][1]) ==
          (((modeMask[0][0]^modeMask[5][0])&modeMask[5][1]) & mosi_frame[3]))
  {
    Serial.println("Power On - Dry.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 50 }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (((modeMask[0][0]^modeMask[4][0])&modeMask[4][1]) ==
          (((modeMask[0][0]^modeMask[4][0])&modeMask[4][1]) & mosi_frame[3]) &&
          (mosi_frame[3] & 0b00001000) != 0b00001000)
  {
    Serial.println("Power On - Auto.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 40 }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (((modeMask[0][0]^modeMask[3][0])&modeMask[3][1]) ==
          (((modeMask[0][0]^modeMask[3][0])&modeMask[3][1]) & mosi_frame[3]))
  {
    Serial.println("Power On - Cool.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 30 }", idxmodeselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
}

void domoticzVanesUpdate()
{
                                 /*{ 0b10000000, 0b00000000, 0b10000000, 0b00000000 },  //0 = Unchanged (only clear 'write' bits)
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10000000 },  //1 = 1 (up)
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10010000 },  //2 = 2
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10100000 },  //3 = 3
                                   { 0b11000000, 0b10000000, 0b10110000, 0b10110000 },  //4 = 4 (down)
                                   { 0b11000000, 0b11000000, 0b10000000, 0b00000000 }}; //5 = swing*/
  updatedVanes = false;
  if (0b01000000 & mosi_frame[3])
  {
      Serial.println("Vanes Swing.");
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 40 }", idxvaneselector);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if ((0b00100000 & mosi_frame[4]) && (0b00010000 & mosi_frame[4]))
  {
      Serial.println("Vanes Pos.4 (DOWN).");
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 30 }", idxvaneselector);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (0b00100000 & mosi_frame[4])
  {
      Serial.println("Vanes Pos.3.");
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 20 }", idxvaneselector);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (0b00010000 & mosi_frame[4])
  {
      Serial.println("Vanes Pos.2.");
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 10 }", idxvaneselector);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (!(0b00110000 & mosi_frame[4]))
  {
      Serial.println("Vanes Pos.1 (UP).");
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Off\" }", idxvaneselector);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
}

void domoticzFanspeedUpdate()
{
  updatedFanspeed = false;
  if (!(mosi_frame[4] & 0b00000111))
  {
    Serial.println("Fan Speed is 1.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Off\" }", idxfanspeedselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if (mosi_frame[4] & 0b00000001)
  {
    Serial.println("Fan Speed is 2.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 10 }", idxfanspeedselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if ((mosi_frame[4] & 0b00000010) && !(mosi_frame[9] & 0b01000000))
  {
    Serial.println("Fan Speed is 3.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 20 }", idxfanspeedselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
  else if ((mosi_frame[4] & 0b00000010) && (mosi_frame[9] & 0b01000000))
  {
    Serial.println("Fan Speed is 4.");
    // create mqtt string for errorcode
    sprintf(mqttbuffer, "{\"command\": \"switchlight\", \"idx\": %d, \"switchcmd\": \"Set Level\", \"level\": 30 }", idxfanspeedselector);
    // send data to the MQTT topic
    client.publish(mqtt_domoticz_topic_in, mqttbuffer);
  }
}

void domoticzSetpointUpdate()
{
  updatedSetpoint = false;
  int tempsetpoint = (mosi_frame[5] & 0x7F) /2; // bit masked so MSB ignored as we only need mosiframe[5](6:0)
  Serial.printf("Setpoint Thermostat updated to :%d", tempsetpoint);
  Serial.println();
  // create mqtt string for tempsetpoint
  sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%d\" }", idxtempsetpoint, tempsetpoint);
  // send data to the MQTT topic
  client.publish(mqtt_domoticz_topic_in, mqttbuffer);
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
    if (timeout == 0) return;
    timeout--;
  }
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
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

void setup() 
{
  Serial.begin(115200);
  pinMode(16, OUTPUT); // turn on level shifter
  digitalWrite(16, 1); // turn on level shifter
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  initWiFi();
  initOTA();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  irrecv.enableIRIn(); // start IR receiver 
  starttime = millis();
}

void loop() 
{
  exchange_payloads();
  if (WiFi.status() == WL_CONNECTED)
  {
    ArduinoOTA.handle();
    if (!client.connected()) reconnect(); // Reconnect to MQTT broker if required
    client.loop(); // MQTT client loop
    if (irrecv.decode(&results))
    {
      // create mqtt string for IR Remote Data
      sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%s;0\" }", idxirremotedata, uint64ToString(results.value, HEX).c_str());
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);      
      irrecv.resume(); 
    }
    if (newPayloadReceived && millis() - starttime > 1000)
    {
      float roomtemp = (mosi_frame[6] - 61) / 4;
      //float tempsetpoint = (mosi_frame[5] & 0x7F) /2; // bit masked so MSB ignored as we only need mosiframe[5](6:0)
      int errorcode = mosi_frame[7];
      // create mqtt string for roomtemp
      sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;0\" }", idxroomtemp, roomtemp);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);
      // create mqtt string for tempsetpoint
      //sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;0\" }", idxtempsetpoint, tempsetpoint);
      // send data to the MQTT topic
      //client.publish(mqtt_domoticz_topic_in, mqttbuffer);
      // create mqtt string for errorcode
      sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;0\" }", idxerrorcode, errorcode);
      // send data to the MQTT topic
      client.publish(mqtt_domoticz_topic_in, mqttbuffer);


      /*if ((mosi_frame[9] & 0x80) == 0)
      {
        float outdoortemp = mosi_frame[14];
        // create mqtt string for outdoortemp
        sprintf(mqttbuffer, "{ \"idx\" : %d, \"nvalue\" : 0, \"svalue\" : \"%3.1f;0\" }", idxoutdoortemp, outdoortemp);
        // send data to the MQTT topic
        client.publish(mqtt_domoticz_topic_in, mqttbuffer); 
      }*/
               
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
  yield();
}
