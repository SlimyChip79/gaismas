import time
import board
import busio
from adafruit_pcf8575 import PCF8575

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf = PCF8575(i2c, address=0x27)

# All relays OFF initially (active-low)
pcf.value = 0xFFFF

while True:
    for i in range(8):
        mask = 1 << i

        # Turn ON
        pcf.value &= ~mask
        time.sleep(0.5)

        # Turn OFF
        pcf.value |= mask
        time.sleep(0.5)
