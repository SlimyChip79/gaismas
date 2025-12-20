import board
import busio
import adafruit_pcf8575
import time

# Use the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# PCF8575 address (0x27 is from your i2cdetect)
pcf = adafruit_pcf8575.PCF8575(i2c, 0x27)

# There are 16 pins: 0-15
pins = list(range(16))

# Loop to turn them on/off one by one
while True:
    for pin in pins:
        pcf.pin(pin).value = True   # Turn ON
        time.sleep(0.5)
        pcf.pin(pin).value = False  # Turn OFF
        time.sleep(0.5)
