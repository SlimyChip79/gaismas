#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pcf8575 import PCF8575

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PCF8575 at detected address
pcf = PCF8575(i2c, 0x27)  # change if your i2cdetect shows a different address

# Set all 16 pins as outputs
for pin in range(16):
    pcf.setup(pin, True)  # True = output

print("Starting relay test...")

try:
    while True:
        # Turn relays ON one by one
        for pin in range(16):
            print(f"Relay {pin} ON")
            pcf.output(pin, False)  # Active LOW = ON
            time.sleep(0.5)
            pcf.output(pin, True)   # OFF after delay

        time.sleep(1)  # pause between cycles

except KeyboardInterrupt:
    # Turn all relays OFF on exit
    for pin in range(16):
        pcf.output(pin, True)
    print("Relay test stopped, all OFF")
