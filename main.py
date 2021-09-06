from machine import Pin, freq
import time

freq(160000000)
lvlshiften = Pin(16, Pin.OUT)
lvlshiften.on()

mosipin = Pin(13, Pin.IN)
misopin = Pin(12, Pin.OUT)
misopin.value(1)
sckpin = Pin(14, Pin.IN)

variant_no = 0                                      # Frame variation that is currently being sent (0, 1 or 2 in frameVariant[])
frame_no = 1                                        # Counter for how many times a frame variation has been sent (max. = 48)
state = 0                                           # //'State machine' in loop(): [State=0] Collect a full 20-byte SPI data frame. [State=1] Pulse clock, set new/updated frames etc.. [State=2] Check for updated states from user.

checksum_error = False                              # Flag for checksum error in frame 2 or 47. If two errors occur at both these positions in a single 48-frame cycle -> SPI sync lost -> resync SPI

newMode = 0x00
newVanes = 0x00
newFanspeed = 0x00
newSetpoint = 0x00

mosi_frame_sig = [0x6c, 0x80, 0x04]                 # SPI frame start signature: first 3 bytes in a full SPI data frame. Used to sync to MHI SPI data in SPI_sync() routine. Might be different on other unit types!

# Bitfield:     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17    18    19    20
miso_frame = [0xA9, 0x00, 0x07, 0x4C, 0x00, 0x2A, 0xFF, 0x00, 0x00, 0x40, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x0F, 0x04, 0x05, 0xF5]

mosi_frame = bytearray(20)                          # Array to collect a single frame of SPI data received from the MHI unit

mosi_bitfield4_10 =bytearray(7)                     # Array containing bitfields 4-10 from rx_SPIframe, which holds current MHI mode, vanes, fans speed, ambient temperature and setpoint
# Bitfield:         10    11    12    13    14    15    16    17    18
frame_variants = [[0x40, 0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0x0F, 0x04],
                  [0x80, 0x00, 0x00, 0x32, 0xD6, 0x01, 0x00, 0x0F, 0x04],
                  [0x80, 0x00, 0x00, 0xF1, 0xF7, 0xFF, 0xFF, 0x0F, 0x04]]

# MODE bitmasks    Bitfield #4
#                CLR     |    SET
mode_mask = [[0b00100010, 0b00000000], # 0 = Unchanged(only clear 'write' bits)
             [0b00100011, 0b00000010], # 1 = OFF
             [0b00111111, 0b00110011], # 2 = HEAT
             [0b00111111, 0b00101011], # 3 = COOL
             [0b00111111, 0b00100011], # 4 = AUTO
             [0b00111111, 0b00100111], # 5 = DRY
             [0b00111111, 0b00101111], # 6 = FAN
             [0b00100011, 0b00000011]] # 7 = ON(using last mode)

# VANES bitmasks     Bitfield #4             Bitfield #5
#                 CLR     |    SET        CLR    |    SET
vanes_mask = [[0b10000000, 0b00000000, 0b10000000, 0b00000000], # 0 = Unchanged (only clear 'write' bits)
              [0b11000000, 0b10000000, 0b10110000, 0b10000000], # 1 = 1 (up)
              [0b11000000, 0b10000000, 0b10110000, 0b10010000], # 2 = 2
              [0b11000000, 0b10000000, 0b10110000, 0b10100000], # 3 = 3
              [0b11000000, 0b10000000, 0b10110000, 0b10110000], # 4 = 4 (down)
              [0b11000000, 0b11000000, 0b10000000, 0b00000000]] # 5 = swing

# FANSPEED bitmasks    Bitfield #5             Bitfield #10
#                   CLR     |    SET        CLR    |    SET
fanspeed_mask = [[0b00001000, 0b00000000, 0b11011000, 0b00000000], # 0 = Unchanged (only clear 'write' bits)
                 [0b00001111, 0b00001000, 0b11011000, 0b00000000], # 1 = Speed 1
                 [0b00001111, 0b00001001, 0b11011000, 0b00000000], # 2 = Speed 2
                 [0b00001111, 0b00001010, 0b11011000, 0b00000000], # 3 = Speed 3
                 [0b00001111, 0b00001010, 0b11011000, 0b00010001]] # 4 = Speed 4

def update_checksum():
    global miso_frame
    sum = 0
    for i in range(0, 18):
        sum += miso_frame[i]
    miso_frame[18] = (sum >> 8) & 0xff
    miso_frame[19] = sum & 0xff

def verify_checksum():
    sum = 0
    for i in range(0, 18):
        sum += mosi_frame[i]
    #print('LSB' + str((sum >> 8) & 0xfF) + ', ' + str(mosi_frame[18]))
    #print('HSB' + str(sum & 0xfF) + ', ' + str(mosi_frame[19]))
    return (mosi_frame[18] == ((sum >> 8) & 0xfF) and mosi_frame[19] == (sum & 0xFF))

def update_miso_frame_variant():
    global frame_no, miso_frame
    for i in range(10, 19):
        miso_frame[i] = frame_variants[frame_no][i-10]
    frame_no += 1
    if frame_no == 3:
        frame_no = 0

def exchange_payloads():
    global doubleframe, frame, erropdataCnt, miso_frame, mosi_frame
    global state, frame_no, checksum_error, mosi_bitfield4_10
    global newMode, newVanes, newFanspeed, newSetpoint

    # wait for 5ms high clock to detect a frame start
    sckus = time.ticks_us()
    while True:
        if not sckpin.value():
            while not sckpin.value():  # wait for clock rising edge
                pass
            sckus = time.ticks_us()
        if time.ticks_diff(time.ticks_us(), sckus) > 5000:
            break
    sckus = time.ticks_us()
    for byte_cnt in range(0, 20):  # change to range(0,20) for deployment
        MOSI_byte = 0
        bit_mask = 0b10000000
        for bit_cnt in range(0, 8):
            while sckpin.value():  # wait for clock falling edge
                pass  # insert write code here later ...
            #if miso_frame[byte_cnt] & bit_mask:
            #    misopin.on()
            #else:
            #    misopin.off()
            while not sckpin.value():  # wait for clock rising edge
                pass
            # time stamp rising edge
            if mosipin.value():
                MOSI_byte += bit_mask
            bit_mask = bit_mask >> 1
        mosi_frame[byte_cnt] = MOSI_byte

    print("".join("\\x%02x" % i for i in mosi_frame) + ' Frame time(us):' + str(time.ticks_diff(time.ticks_us(), sckus)))
    if verify_checksum():
        print('Payload CSC Verified!')
    frame_no += 1

def main():
    while 1:
        # exchange_payloads()
        while sckpin.value():  # wait for clock falling edge
            pass
        stamp = time.ticks_us()
        while not sckpin.value():  # wait for clock rising edge
            pass
        stamp2 = time.ticks_us()
        print(str(time.ticks_diff(stamp2, stamp)))

if __name__== "__main__":
    main()