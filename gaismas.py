#!/usr/bin/env python3
from smbus2 import SMBus
import time

# ================= CONFIG =================
I2C_BUS = 1
PCF_ADDR = 0x27  # PCF8575 relay board
DELAY = 0.5      # seconds between toggles

# ================= HELPER =================
def write_pcf(value):
    """
    Write 16-bit value to PCF8575 safely.
    Handles high and low bytes correctly.
    """
    try:
        low = value & 0xFF
        high = (value >> 8) & 0xFF
        bus.write_i2c_block_data(PCF_ADDR, low, [high])
    except Exception as e:
        print(f"Failed to write PCF8575: {e}")


# ================= START =================
bus = SMBus(I2C_BUS)
print("PCF8575 test starting...")

# Initialize all relays off (assuming active-low)
relays = 0xFFFF
write_pcf(relays)
time.sleep(1)

try:
    while True:
        # Turn ON each relay one by one
        for i in range(16):
            print(f"Turning ON relay {i+1}")
            relays &= ~bitmask(i)  # clear bit → active
            write_pcf(relays)
            time.sleep(DELAY)
            
            print(f"Turning OFF relay {i+1}")
            relays |= bitmask(i)   # set bit → inactive
            write_pcf(relays)
            time.sleep(DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays off...")
    write_pcf(0xFFFF)  # all off
    bus.close()
