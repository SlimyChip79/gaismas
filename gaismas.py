#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pcf8575 import PCF8575

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)

# PCF8575 address (replace 0x27 with your detected address)
pcf = PCF8575(i2c, 0x27)

# Number of relays
NUM_RELAYS = 16

# Initialize all pins as outputs and off
for pin in range(NUM_RELAYS):
    pcf[pin] = False

print("All relays initialized to OFF.")

# Main loop
try:
    while True:
        # Turn on each relay one by one
        for pin in range(NUM_RELAYS):
            pcf[pin] = True
            print(f"Relay {pin} ON")
            time.sleep(0.5)  # half-second delay
            pcf[pin] = False
            print(f"Relay {pin} OFF")
            time.sleep(0.2)
except KeyboardInterrupt:
    print("Exiting, turning all relays OFF.")
    for pin in range(NUM_RELAYS):
        pcf[pin] = False
