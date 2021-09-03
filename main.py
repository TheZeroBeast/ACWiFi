from machine import Pin
import time

lvlshiften = Pin(16, Pin.OUT)
lvlshiften.on()

mosipin = Pin(13, Pin.IN, Pin.PULL_UP)
misopin = Pin(12, Pin.OUT)
misopin.value(1)
sckpin = Pin(14, Pin.IN, Pin.PULL_UP)

def recvpayload():
    payload = bytearray(20)
    # wait for 5ms high clock to detect a frame start
    sckmillis = time.ticks_ms()
    while (time.ticks_ms() - sckmillis < 5):
        if not sckpin:
            sckmillis = time.ticks_ms()
    for byte_cnt in range(0, 20):  # change to range(0,20) for deployment
        MOSI_byte = 0
        bit = 0
        for bit_cnt in range(0, 8):
            while sckpin.value():  # wait for clock falling edge
                pass  # insert write code here later ...
            while not sckpin.value():  # wait for clock rising edge
                pass
            if mosipin.value():
                bit = 1
            else:
                bit = 0
            MOSI_byte = (MOSI_byte << 1) | bit
        payload[byte_cnt] = MOSI_byte
    return payload

def main():
    while 1:
        print(recvpayload())

if __name__== "__main__":
    main()