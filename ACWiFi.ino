#define SCK_PIN  14
#define MOSI_PIN 13
#define MISO_PIN 12

byte variant_no = 0; // Frame variation that is currently being sent (0, 1 or 2 in frameVariant[])
byte frame_no = 1; // Counter for how many times a frame variation has been sent (max. = 48)

bool checksumError = false;
bool newPayloadReceived = false;

byte newMode     = 0;
byte newVanes    = 0;
byte newFanspeed = 0;
byte newSetpoint = 0;

  int cmdTimeStamp = millis();
 
const byte mosi_frame_sig[3] = {0x6d, 0x80, 0x04}; // SPI frame start signature: first 3 bytes in a full SPI data frame. Used to sync to MHI SPI data in SPI_sync() routine. Might be different on other unit types!

// Bitfield:             1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
byte miso_frame[20] = {0xA9, 0x00, 0x07, 0x4C, 0x00, 0x2A, 0xFF, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x0F, 0x04, 0x05, 0xF5};

byte mosi_frame[20]; // Array to collect a single frame of SPI data received from the MHI unit

byte mosi_bitfield4_10[7]; // Array containing bitfields 4-10 from rx_SPIframe, which holds current MHI mode, vanes, fans speed, ambient temperature and setpoint

//Alternating bitfield 10-18 variations, each successively send for 48 frames. Bit 3 in bitfield 18 functions as a clock and is 1 for 24 frames and 0 for the subsequent 24 frames. I have never seen bit fields 11-12 changing, so don't know what they are for.
//                         Bitfield:   10    11    12    13    14    15    16    17    18
const byte frameVariant[3][9]  {   { 0x40, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x0F, 0x04 },  //variant number 0
                                   { 0x80, 0x00, 0x00, 0x32, 0xD6, 0x01, 0x00, 0x0F, 0x04 },  //variant number 1
                                   { 0x80, 0x00, 0x00, 0xF1, 0xF7, 0xFF, 0xFF, 0x0F, 0x04 }};   //variant number 2

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
  switch (frame_no)
  {
    case 2:                                                                                                //<FRAME 2> Verify checksum on SPI frame that was just received and send to ESP8266 if correct
      if (verify_checksum()) checksumError = false;
      else checksumError = true;
      break;                                                                                               //Start from beginning of loop() and wait for next complete SPI frame

    case 24:                                                                                               //<FRAME 24> Current frame variation has been sent 24 times -> clear clock bit in bit field 18 for the next 24 frames
      bitClear(miso_frame[17], 2);                                                                        //Clear clock bit 3 in bitfield 18-> update checksum and resend for 24 cycles
      update_checksum();                                                                                   //Recalculate checksum of tx_SPIframe
      break;                                                                                               //Start from beginning of loop() and wait for next complete SPI frame

    case 47:                                                                                               //<FRAME 47> Collect the most recent bit fields 4-10 for constructing an updated tx_SPIframe after the upcoming frame (48)
      if (verify_checksum())                                                                               //Verify checksum
      {
        memcpy(&mosi_bitfield4_10, &mosi_frame[3], 7);                                                    //Get bitfields 4-10 from the last MHI SPI frame to use for the upcoming tx_SPIframe update
        checksumError = false;
      }
      else checksumError = true;
      break;

    case 48:                                                                                               //<FRAME 48> Current frame variation has been send 48 times -> construct next frame variant using most recent bit fields 4-10 collected in frame 47
      frame_no = 0;   
      if (++variant_no > 2) variant_no = 0;
      memcpy(&miso_frame[9], &frameVariant[variant_no][0], 9);
      /*Serial.printf("VarNo:%i . ", variant_no);
      Serial.print("Miso:");
      for (uint8_t i = 9; i <18; i++)
        Serial.printf("%.2X ", miso_frame[i]);
      Serial.println();
      Serial.print("  FrameVariant:");
      for (uint8_t i = 0; i <9; i++)
        Serial.printf("%.2X ", frameVariant[variant_no][i]);
      Serial.println();*/
      
      //******************* CONSTRUCTION OF UPDATED BIT FIELDS *******************
      //Set 'state change' bits and 'write' bits if MQTT update received from ESP
      //otherwise only clear 'write' bits using masks from the xxxMask[0][] arrays
      //Bitfields 4, 5, 6, 10 are based on the last received MHI values (frame 47)
      miso_frame[3]  =  mosi_bitfield4_10[0] & ~modeMask[newMode][0];                                       //Clear mode bits (bitfield 4)
      miso_frame[3] |=  modeMask[newMode][1];                                                             //Set mode bits

      miso_frame[3] &= ~vanesMask[newVanes][0];                                                           //Clear vanes bits (bitfield 4)
      miso_frame[3] |=  vanesMask[newVanes][1];                                                           //Set vanes bits

      miso_frame[4]  =  mosi_bitfield4_10[1] & ~vanesMask[newVanes][2];                                     //Clear vanes bits (bitfield 5)
      miso_frame[4] |=  vanesMask[newVanes][3];                                                           //Set vanes bits

      miso_frame[4] &= ~fanspeedMask[newFanspeed][0];                                                     //Clear fanspeed bits (bitfield 5)
      miso_frame[4] |=  fanspeedMask[newFanspeed][1];                                                     //Set fanspeed bits

      bitWrite(mosi_bitfield4_10[6], 0, bitRead(mosi_bitfield4_10[6], 6));                                     //Copy bit 7 from rx_SPIframe[9] to bit 1 as the status bits for fan speed 4 appear to be swapped (!?) between MISO and MOSI
      miso_frame[9] &=  ~0b00111111;                                                                      //Clear bits 1-6 and keep variant bits 7-8

      miso_frame[9] |=  (mosi_bitfield4_10[6] & ~fanspeedMask[newFanspeed][2]);
      miso_frame[9] |=  fanspeedMask[newFanspeed][3];                                                     //Set fanspeed bits

      //Construct setpoint bitfield (#6) from last MHI value or MQTT update
      if (newSetpoint == 0) miso_frame[5] = mosi_bitfield4_10[2] & ~0b10000000;                           //Copy last received MHI setpoint and clear the write bit
      else miso_frame[5] = (newSetpoint << 1) | 0b10000000;                                              //MQTT updated setpoint in degrees Celsius -> shift 1 bit left and set write bit (#8)

      update_checksum();                                                                                   //Recalculate checksum of tx_SPIframe

      //Reset all state changes
      newMode     = 0;
      newVanes    = 0;
      newFanspeed = 0;
      newSetpoint = 0;
      break;                                                                                               //Start from beginning of loop() and wait for next complete SPI frame.

    default:
      break;
  }
  /*for (uint8_t i = 0; i < 20; i++) {
    Serial.printf("%.2X ", miso_frame[i]);
  }
  Serial.print(frame_no);
  Serial.println();*/
  frame_no++;

  int startMillis = millis();             // start time of this loop run
  int SCKMillis = millis();               // time of last SCK low level
  while (millis() - SCKMillis < 5) { // wait for 5ms stable high signal to detect a frame start
    if (!digitalRead(SCK_PIN))
      SCKMillis = millis();
  }

  // read/write MOSI/MISO frame
  for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { // read and write a data packet of 20 bytes
    byte MOSI_byte = 0;
    byte bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) { // read and write 1 byte
      while (digitalRead(SCK_PIN)) {} // wait for falling edge
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
  }
  //if (newPayloadReceived)
  //{
    for (uint8_t i = 0; i < 20; i++) {
      Serial.printf("%.2x/%.2x ", mosi_frame[i], miso_frame[i]);
    }
    //if (verify_checksum() and !checksumError) Serial.printf(" Verfied checksum.");
    float roomtemp = (mosi_frame[6] - 61) / 4;
    Serial.printf(" Room Temp:%.2f", roomtemp);
    Serial.println();
    
    newPayloadReceived = false;
  //}
}

void setup() {
  Serial.begin(115200);
  pinMode(16, OUTPUT); // turn on level shifter
  digitalWrite(16, 1); // turn on level shifter
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);

  cmdTimeStamp = millis();             // start time of this loop run
}

void loop() {
  exchange_payloads();

  if (millis() - cmdTimeStamp > 5000)
  {
      // cmdTimeStamp = millis();
      newMode     = 0;
      newVanes    = 0;
      newFanspeed = 0;
      newSetpoint = 0;
  }
}
