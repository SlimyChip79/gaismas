#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pcf8575 import PCF8575

# Initialize I2C and PCF8575 at detected address 0x27
i2c = busio.I2C(board.SCL, board.SDA)
pcf = PCF8575(i2c, 0x27)

# Configure all 16 pins as outputs
for pin in range(16):
    pcf.pin_mode(pin, True)  # True = output

print("Starting sequence...")

while True:
    # Turn pins ON one by one
    for pin in range(16):
        pcf[pin] = True
        print(f"Pin {pin} ON")
        time.sleep(0.5)

    # Turn pins OFF one by one
    for pin in range(16):
        pcf[pin] = False
        print(f"Pin {pin} OFF")
        time.sleep(0.5)
