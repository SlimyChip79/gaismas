#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575
from datetime import datetime

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

# ---------------- TIMING SETTINGS ----------------
LOOP_DELAY = 0.005           # 5ms main loop delay
debounce_delay = 50           # ms for button debounce
long_press_threshold = 350   # ms to detect long press

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# All relays OFF (active-low)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# ---------------- BUTTON ARRAYS ----------------
# Structure 1: simple toggle
simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 0),
    (PCA1_ADDR, 1, pcf1, 1 << 1),
]

# Structure 2: short press relay + long press relay
debounce_buttons = [
    (PCA2_ADDR, 0, pcf2, 1 << 0, 1 << 1),
    (PCA2_ADDR, 1, pcf2, 1 << 2, 1 << 3),
]

# ---------------- STATE TRACKERS ----------------
# Structure 1
last_input_simple = [1]*len(simple_buttons)

# Structure 2
last_input_debounce = [1]*len(debounce_buttons)
last_debounce_time = [0]*len(debounce_buttons)
button_press_start_time = [0]*len(debounce_buttons)
long_press_triggered = [False]*len(debounce_buttons)

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ---------------- MAIN LOOP ----------------
try:
    while True:
        current_time = int(time.time() * 1000)  # ms

        # --- Structure 1: simple toggle ---
        for idx, (addr, pin, pcf, relay_mask) in enumerate(simple_buttons):
            inputs = read_pca_inputs(addr)
            val = inputs[pin]
            if val == 0 and last_input_simple[idx] == 1:
                # Toggle relay
                if pcf_state := pcf.read_gpio() & relay_mask:
                    pcf_state &= ~relay_mask
                    print(f"[DEBUG] Simple Button idx={idx} ON")
                else:
                    pcf_state |= relay_mask
                    print(f"[DEBUG] Simple Button idx={idx} OFF")
                pcf.write_gpio(pcf_state)
            last_input_simple[idx] = val

        # --- Structure 2: short + long press ---
        for idx, (addr, pin, pcf, relay_short, relay_long) in enumerate(debounce_buttons):
            inputs = read_pca_inputs(addr)
            val = inputs[pin]

            # Debounce check
            if val != last_input_debounce[idx]:
                last_debounce_time[idx] = current_time

            if (current_time - last_debounce_time[idx]) > debounce_delay:
                # Button pressed
                if val == 1:
                    if button_press_start_time[idx] == 0:
                        button_press_start_time[idx] = current_time
                        long_press_triggered[idx] = False
                    elif not long_press_triggered[idx] and (current_time - button_press_start_time[idx] >= long_press_threshold):
                        # Long press triggered
                        relay_state = pcf.read_gpio() ^ relay_long
                        pcf.write_gpio(relay_state)
                        print(f"[DEBUG] Debounce Button idx={idx} LONG PRESS")
                        long_press_triggered[idx] = True
                else:
                    # Button released
                    if button_press_start_time[idx] != 0 and not long_press_triggered[idx]:
                        # Short press
                        relay_state = pcf.read_gpio() ^ relay_short
                        pcf.write_gpio(relay_state)
                        print(f"[DEBUG] Debounce Button idx={idx} SHORT PRESS")
                    button_press_start_time[idx] = 0
                    long_press_triggered[idx] = False

            last_input_debounce[idx] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
