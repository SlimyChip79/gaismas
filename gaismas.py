#!/usr/bin/env python3
from pcf8575 import PCF8575
import time

# I2C address of your relay board
PCF_ADDR = 0x27
I2C_BUS = 1  # Usually 1 on Raspberry Pi

print("[RELAYS] Initializing PCF8575...")
pcf = PCF8575(I2C_BUS, PCF_ADDR)
time.sleep(0.2)  # Allow the chip to initialize

# Make sure all relays are OFF at start (PCF8575 is active low for most relay boards)
for pin in range(16):
    pcf.digital_write(pin, 1)

print("[RELAYS] Starting relay test...")

try:
    while True:
        for pin in range(16):
            # Turn ON the relay (active low)
            pcf.digital_write(pin, 0)
            print(f"Relay {pin+1} ON")
            time.sleep(0.5)  # keep it on long enough to see LED

            # Turn OFF the relay
            pcf.digital_write(pin, 1)
            print(f"Relay {pin+1} OFF")
            time.sleep(0.2)  # short delay before next relay

except KeyboardInterrupt:
    print("\n[RELAYS] Test stopped by user, turning all relays OFF...")
    for pin in range(16):
        pcf.digital_write(pin, 1)
    print("[RELAYS] All relays OFF.")
