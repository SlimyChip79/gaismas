#!/usr/bin/env python3
import time
from smbus2 import SMBus

# ================= CONFIG =================
I2C_BUS = 1
PCF_ADDR = 0x27  # Relay extender (PCF8575)

# ================= HELPERS =================
def log(msg):
    print(f"[GAISMAS] {time.strftime('%H:%M:%S')} | {msg}", flush=True)

def write_relays(bus, relay_bits):
    """
    Write 16 relays to PCF8575 using proven working method
    relay_bits: 16-bit int, bit0 = relay1, bit15 = relay16
    """
    # Split low/high bytes
    low_byte  = relay_bits & 0xFF       # relays 1-8
    high_byte = (relay_bits >> 8) & 0xFF  # relays 9-16

    # Invert for active-low relays
    low_byte  ^= 0xFF
    high_byte ^= 0xFF

    try:
        # Write low byte (relays 1-8)
        bus.write_byte_data(PCF_ADDR, 0x00, low_byte)
        time.sleep(0.01)
        # Write high byte (relays 9-16)
        bus.write_byte_data(PCF_ADDR, 0x01, high_byte)
        time.sleep(0.01)
    except OSError as e:
        log(f"PCF write error: {e}")

# ================= INIT =================
log("Service starting")
try:
    bus = SMBus(I2C_BUS)
    log("I2C bus opened")
except Exception as e:
    log(f"I2C init failed: {e}")
    bus = None

# Clear all relays at startup
if bus:
    write_relays(bus, 0x0000)
    log("PCF8575 relays cleared")

# ================= MAIN LOOP =================
try:
    relay_state = 0x0000
    while True:
        if not bus:
            time.sleep(1)
            continue

        # Example: turn on each relay one by one
        for i in range(16):
            relay_state = 1 << i
            write_relays(bus, relay_state)
            log(f"RELAYS : {relay_state:016b}")
            time.sleep(0.5)

        # Turn all off
        relay_state = 0x0000
        write_relays(bus, relay_state)
        log(f"RELAYS : {relay_state:016b}")
        time.sleep(1)

except KeyboardInterrupt:
    log("Service stopped by user")

finally:
    if bus:
        # Turn off all relays
        write_relays(bus, 0x0000)
        bus.close()
    log("Service exited")
