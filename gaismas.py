#!/usr/bin/env python3
import time
from pcf8575 import PCF8575  # PCF8575 library for relay board

# ================= CONFIG =================
PCF_ADDR = 0x27          # I2C address of your PCF8575 relay board
DELAY = 1.0              # seconds each relay stays on

# ================= INIT =================
print("[RELAYS] Initializing PCF8575...")
pcf = PCF8575(PCF_ADDR)

# Set all 16 pins to HIGH (relays OFF if board is active low)
for pin in range(16):
    pcf.pin_mode(pin, pcf.OUTPUT)
    pcf.write(pin, 1)  # 1 = off for active-low relays

print("[RELAYS] All relays cleared. Starting test...")

# ================= MAIN LOOP =================
try:
    while True:
        # Loop through all 16 relays
        for pin in range(16):
            print(f"[RELAYS] Turning ON relay {pin + 1}")
            pcf.write(pin, 0)  # 0 = ON for active-low relay
            time.sleep(DELAY)
            print(f"[RELAYS] Turning OFF relay {pin + 1}")
            pcf.write(pin, 1)
            time.sleep(0.1)  # small pause between relays

except KeyboardInterrupt:
    print("\n[RELAYS] Stopping test, clearing all relays...")
    for pin in range(16):
        pcf.write(pin, 1)  # turn all off
    print("[RELAYS] Done.")
