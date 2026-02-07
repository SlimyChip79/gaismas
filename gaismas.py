#!/usr/bin/env python3
from smbus2 import SMBus
import time

# ================= CONFIG =================
I2C_BUS = 1
PCF_ADDR = 0x27  # PCF8575 address
DELAY = 0.5      # seconds between relay changes

# ================= START =================
print("[GAISMAS] Relay test starting")

try:
    bus = SMBus(I2C_BUS)
    print("[GAISMAS] I2C bus opened")
except Exception as e:
    print(f"[GAISMAS] I2C init failed: {e}")
    bus = None

# ================= HELPER =================
def write_pcf(relay_state):
    """
    Write 16-bit relay_state to PCF8575.
    relay_state: 16-bit int, bit0 = relay 1, bit15 = relay 16
    """
    if not bus:
        return
    low_byte  = relay_state & 0xFF
    high_byte = (relay_state >> 8) & 0xFF
    try:
        bus.write_byte_data(PCF_ADDR, 0x00, low_byte)   # pins P0-P7
        time.sleep(0.002)  # small delay
        bus.write_byte_data(PCF_ADDR, 0x01, high_byte)  # pins P8-P15
    except Exception as e:
        print(f"[GAISMAS] PCF write error: {e}")

# ================= MAIN LOOP =================
try:
    # start with all relays off (PCF8575 is high=off for many relay boards)
    relay_state = 0xFFFF
    write_pcf(relay_state)
    print("[GAISMAS] All relays cleared")
    time.sleep(1)

    while True:
        # Cycle through relays 1..16
        for i in range(16):
            # turn one relay on (active low for typical relay board)
            relay_state = 0xFFFF ^ (1 << i)
            write_pcf(relay_state)
            print(f"[GAISMAS] Relay {i+1} ON")
            time.sleep(DELAY)

            # turn it off again
            relay_state = 0xFFFF
            write_pcf(relay_state)
            print(f"[GAISMAS] Relay {i+1} OFF")
            time.sleep(DELAY)

except KeyboardInterrupt:
    print("[GAISMAS] Test stopped by user")

finally:
    # turn all relays off
    relay_state = 0xFFFF
    write_pcf(relay_state)
    print("[GAISMAS] All relays cleared")
    if bus:
        bus.close()
