#define SCK_PIN  14
#define MOSI_PIN 13
#define MISO_PIN 12

void setup() {
  Serial.begin(115200);
  pinMode(16, OUTPUT); // turn on level shifter
  digitalWrite(16, 1); // turn on level shifter
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
}

void loop() {
  int startMillis = millis();             // start time of this loop run
  static byte MOSI_frame[20];
  int SCKMillis = millis();               // time of last SCK low level
  while (millis() - SCKMillis < 5) { // wait for 5ms stable high signal to detect a frame start
    if (!digitalRead(SCK_PIN))
      SCKMillis = millis();
  }

  // read/write MOSI/MISO frame
  for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { // read and write a data packet of 20 bytes
    //Serial.printf("x%02x ", MISO_frame[byte_cnt]);
    byte MOSI_byte = 0;
    byte bit_mask = 1;
    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) { // read and write 1 byte
      while (digitalRead(SCK_PIN)) {} // wait for falling edge
      while (!digitalRead(SCK_PIN)) {} // wait for rising edge
      if (digitalRead(MOSI_PIN))
        MOSI_byte += bit_mask;
      bit_mask = bit_mask << 1;
    }
    if (MOSI_frame[byte_cnt] != MOSI_byte) {
      MOSI_frame[byte_cnt] = MOSI_byte;
    }
  }
  for (uint8_t i = 0; i < 19; i++) {
    Serial.printf("%.2X ", MOSI_frame[i]);
  }
  Serial.println();
}
