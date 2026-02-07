#!/usr/bin/env python3
import time
from smbus2 import SMBus

I2C_BUS = 1
PCF_ADDR = 0x27  # PCF8575 relay board

def write_relays(bus, value):
    """
    Write 16-bit value to PCF8575
    Each bit = one relay (0=on, 1=off for active-low board)
    """
    low_byte  = value & 0xFF
    high_byte = (value >> 8) & 0xFF

    # Active-low, invert bytes
    low_byte  ^= 0xFF
    high_byte ^= 0xFF

    bus.write_byte_data(PCF_ADDR, 0x00, low_byte)
    time.sleep(0.01)
    bus.write_byte_data(PCF_ADDR, 0x01, high_byte)
    time.sleep(0.01)

def main():
    bus = SMBus(I2C_BUS)
    print("I2C bus opened, starting relay test")

    # Turn all relays off at start
    write_relays(bus, 0x0000)

    try:
        while True:
            for i in range(16):
                state = 1 << i
                write_relays(bus, state)
                print(f"RELAYS: {state:016b}")
                time.sleep(0.5)

            # Turn all relays off
            write_relays(bus, 0x0000)
            print("RELAYS: all off")
            time.sleep(1)

    except KeyboardInterrupt:
        write_relays(bus, 0x0000)
        print("Relay test stopped, all off")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
