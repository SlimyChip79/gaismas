#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pcf8575 import PCF8575

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# PCF8575 at address 0x27
pcf = PCF8575(i2c, 0x27)

# Initialize all pins to HIGH (PCF8575 is active LOW, adjust if needed)
for pin in range(16):
    pcf[pin] = True  # Set HIGH = OFF if using active LOW relays

print("Starting sequence...")

while True:
    # Turn pins ON one by one
    for pin in range(16):
        pcf[pin] = False  # LOW = ON for active LOW
        print(f"Pin {pin} ON")
        time.sleep(0.5)

    # Turn pins OFF one by one
    for pin in range(16):
        pcf[pin] = True  # HIGH = OFF
        print(f"Pin {pin} OFF")
        time.sleep(0.5)
