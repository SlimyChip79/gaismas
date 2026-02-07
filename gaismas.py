#!/usr/bin/env python3

import time
import smbus2
from datetime import datetime

I2C_BUS = 1
PCF_ADDR = 0x20

bus = smbus2.SMBus(I2C_BUS)

def log(msg):
    print(f"[GAISMAS] {datetime.now().strftime('%H:%M:%S')} | {msg}")

def read_pcf():
    # PCF8575: low byte first, then high byte
    data = bus.read_word_data(PCF_ADDR, 0)
    return data & 0xFFFF

def write_pcf(value):
    bus.write_word_data(PCF_ADDR, 0, value & 0xFFFF)

# ---- STARTUP FIX ----
# Force all relay pins OFF (active-low → write 1)
write_pcf(0xFFFF)
time.sleep(0.1)

last_inputs = 0xFFFF

while True:
    inputs = read_pcf()

    # Inputs are active-low → invert for relays
    relays = (~inputs) & 0xFFFF

    # Write relay outputs (forces pins to OUTPUT)
    write_pcf(relays)

    if inputs != last_inputs:
        log(f"INPUTS : {format(inputs, '016b')}")
        log(f"RELAYS : {format(relays, '016b')}")
        last_inputs = inputs

    time.sleep(0.05)
