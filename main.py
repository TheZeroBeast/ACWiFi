from machine import Pin, freq
import time

freq(160000000)
lvlshiften = Pin(16, Pin.OUT)
lvlshiften.on()

mosipin = Pin(13, Pin.IN, Pin.PULL_UP)
misopin = Pin(12, Pin.OUT)
misopin.value(1)
sckpin = Pin(14, Pin.IN, Pin.PULL_UP)

# constants for the frame
SB0 = 0
SB1 = SB0 + 1
SB2 = SB0 + 2
DB0 = SB2 + 1
DB1 = SB2 + 2
DB2 = SB2 + 3
DB3 = SB2 + 4
DB4 = SB2 + 5
DB6 = SB2 + 7
DB9 = SB2 + 10
DB10 = SB2 + 11
DB11 = SB2 + 12
DB12 = SB2 + 13
DB14 = SB2 + 15
CBH = DB14 + 1
CBL = DB14 + 2

doubleframe = False
frame = 0
erropdataCnt = 0
MISO_frame = [0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f,
              0x00, 0x00,
              0x00]

def calc_checksum(frame):
    checksum = 0
    for i in range(0,CBH):
        checksum += frame[i]
        i += 1
    return checksum

def payload():
    global doubleframe, frame, erropdataCnt, MISO_frame
    payload = bytearray(20)



    # wait for 5ms high clock to detect a frame start
    sckmillis = time.ticks_ms()
    while (time.ticks_ms() - sckmillis < 5):
        if not sckpin:
            sckmillis = time.ticks_ms()

    # build the next MISO frame
    frame += 1
    if frame >= 20:
        doubleframe = not doubleframe
        MISO_frame[DB14] == doubleframe << 2
        frame = 1
        if doubleframe:
            MISO_frame[DB0] = 0x00
            MISO_frame[DB1] = 0x00
            MISO_frame[DB2]  = 0x00
            if erropdataCnt == 0:
                MISO_frame[DB6] = 0x80
                MISO_frame[DB9] = 0xff
                erropdataCnt = erropdataCnt - 1
                
        # Set Power, Mode, Testpoint, Fan, Vanes
        # TODO - Create vars for all parameters
        MISO_frame[DB0] = 0b00      # POWER
        MISO_frame[DB0] |= 0b00     # MODE
        MISO_frame[DB2] = 0b00      # Testpoint
        MISO_frame[DB1] = 0b00      # Fan1
        MISO_frame[DB6] |= 0b00     # Fan6
        MISO_frame[DB0] |= 0b00     # Vanes0
        MISO_frame[DB1] |= 0b00     # Vanes1

    checksum = calc_checksum(MISO_frame)
    MISO_frame[CBH] = checksum >> 8
    MISO_frame[CBL] = checksum & 0xFF

    for byte_cnt in range(0, 20):  # change to range(0,20) for deployment
        MOSI_byte = 0
        bit = 0
        for bit_cnt in range(0, 8):
            while sckpin.value():  # wait for clock falling edge
                pass  # insert write code here later ...
            if MISO_frame[byte_cnt] & bit > 0:
                misopin.on()
            else:
                misopin.off()
            while not sckpin.value():  # wait for clock rising edge
                pass
            if mosipin.value():
                bit = 1
            else:
                bit = 0
            MOSI_byte = (MOSI_byte << 1) | bit
        payload[byte_cnt] = MOSI_byte
    #print(payload)
    print("".join("\\x%02x" % i for i in payload))

def main():
    while 1:
        payload()

if __name__== "__main__":
    main()