#!/usr/bin/env python3
from pcf8575 import PCF8575
import time

PCF_ADDR = 0x27  # I2C address of your relay board
pcf = PCF8575(1, PCF_ADDR)  # bus 1

print("[RELAYS] Initializing PCF8575...")

# Set all relays OFF initially
pcf.write_port(0xFFFF)  # 1 = off (PCF8575 is active-low for most relay boards)
time.sleep(0.5)

try:
    while True:
        # Turn on each relay one by one
        for i in range(16):
            # Create mask for this relay (active low)
            mask = ~(1 << i) & 0xFFFF
            pcf.write_port(mask)
            print(f"Relay {i+1} ON")
            time.sleep(0.5)

            # Turn it off
            pcf.write_port(0xFFFF)
            print(f"Relay {i+1} OFF")
            time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopping, turning all relays off...")
    pcf.write_port(0xFFFF)
