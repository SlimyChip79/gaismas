from adafruit_pcf8574 import PCF8574
import board
import busio
import time

i2c = busio.I2C(board.SCL, board.SDA)
pcf = PCF8574(i2c, address=0x27)

# Set all pins as output
for pin in range(8):
    pcf.pin(pin).direction = PCF8574.OUTPUT

# Turn on/off pins one by one
while True:
    for pin in range(8):
        pcf.pin(pin).value = True   # ON
        time.sleep(0.5)
        pcf.pin(pin).value = False  # OFF
