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
    return checksum

def verify_checksum(frame):
    sum = 0
    for i in range(0,18):
        sum += frame[i]

    #print('LSB' + str((sum >> 8) & 0xfF) + ', ' + str(frame[18]))
    #print('HSB' + str(sum & 0xfF) + ', ' + str(frame[19]))
    return (frame[18] == ((sum >> 8) & 0xfF) and frame[19] == (sum & 0xFF))

def mosi_sig_sync():
    sig = bytes([0x6d, 0x80, 0x04])
    readbuffer = bytearray(3)

    # wait for 200us high clock to detect a frame start
    sckus = time.ticks_us()
    while True:
        if not sckpin.value():
            while not sckpin.value():  # wait for clock rising edge
                pass
            sckus = time.ticks_us()
        if time.ticks_diff(time.ticks_us(), sckus) > 200:
            break
    while True:
        MOSI_byte = 0
        bit_mask = 1
        for bit_cnt in range(0, 8):
            while sckpin.value():  # wait for clock falling edge
                pass  # insert write code here later ...
            while not sckpin.value():  # wait for clock rising edge
                pass
            # time stamp rising edge
            if mosipin.value():
                MOSI_byte += bit_mask
            bit_mask = bit_mask << 1
        readbuffer[2] = readbuffer[1]
        readbuffer[1] = readbuffer[0]
        readbuffer[0] = MOSI_byte
        #print("".join("\\x%02x" % i for i in readbuffer))
        #print("".join("\\x%02x" % i for i in sig))
        #if sig[0] == readbuffer[0] and sig[1] == readbuffer[1] and sig[2] == readbuffer[2]:
        if sig[0] == readbuffer[0]:
            #print('synced')
            return readbuffer

def payload():
    global doubleframe, frame, erropdataCnt, MISO_frame
    payload = bytearray(20)
    sig = mosi_sig_sync()
    payload[0] = sig[0]
    payload[1] = sig[1]
    payload[2] = sig[2]

    # wait for 200us high clock to detect a frame start
    sckus = time.ticks_us()
    while True:
        if not sckpin.value():
            while not sckpin.value():  # wait for clock rising edge
                pass
            sckus = time.ticks_us()
        if time.ticks_diff(time.ticks_us(), sckus) > 200:
            break

    for byte_cnt in range(3, 20):  # change to range(0,20) for deployment
        MOSI_byte = 0
        bit_mask = 1
        for bit_cnt in range(0, 8):
            while sckpin.value():  # wait for clock falling edge
                pass  # insert write code here later ...
            while not sckpin.value():  # wait for clock rising edge
                pass
            # time stamp rising edge
            if mosipin.value():
                MOSI_byte += bit_mask
            bit_mask = bit_mask << 1
        payload[byte_cnt] = MOSI_byte

    print("".join("\\x%02x" % i for i in payload))
    if verify_checksum(payload):
        print('Payload CSC Verified!')

def main():
    while 1:
        payload()

if __name__== "__main__":
    main()