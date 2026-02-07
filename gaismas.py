#!/usr/bin/env python3
from smbus2 import SMBus
import time

I2C_BUS = 1
PCF_ADDR = 0x27  # your PCF8575 relay board address
DELAY = 1  # seconds

def write_relays(bus, relay_bits):
    """
    Writes 16-bit relay state to PCF8575.
    Corrects for active-low and physical mapping:
      - physical relays 1-8 = P8-P15 (high byte)
      - physical relays 9-16 = P0-P7 (low byte)
    """
    high_byte = relay_bits & 0xFF       # bits 0-7 = physical relays 9-16
    low_byte  = (relay_bits >> 8) & 0xFF  # bits 8-15 = physical relays 1-8
    # invert for active-low
    high_byte ^= 0xFF
    low_byte  ^= 0xFF
    try:
        bus.write_byte_data(PCF_ADDR, 0x00, low_byte)
        time.sleep(0.01)
        bus.write_byte_data(PCF_ADDR, 0x01, high_byte)
        time.sleep(0.01)
    except Exception as e:
        print(f"PCF write error: {e}")

def main():
    bus = SMBus(I2C_BUS)
    try:
        print("Clearing all relays...")
        write_relays(bus, 0x0000)
        time.sleep(1)

        # Test each relay individually
        for i in range(16):
            relay_state = 1 << i
            print(f"Relay {i+1} ON")
            write_relays(bus, relay_state)
            time.sleep(DELAY)
            print(f"Relay {i+1} OFF")
            write_relays(bus, 0x0000)
            time.sleep(DELAY)

    finally:
        print("Clearing all relays at end")
        write_relays(bus, 0x0000)
        bus.close()

if __name__ == "__main__":
    main()
