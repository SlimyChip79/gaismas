#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575

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
debounce_delay = 50          # debounce time in ms
long_press_threshold = 1000  # how long button must be held (ms)

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# All relays OFF (active-low → HIGH = off)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# ---------------- BUTTON STRUCTURES ----------------
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
last_input_simple = [1] * len(simple_buttons)

last_input_debounce = [1] * len(debounce_buttons)
last_debounce_time = [0] * len(debounce_buttons)
button_press_start_time = [0] * len(debounce_buttons)
long_press_triggered = [False] * len(debounce_buttons)

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT_0)
        p1 = bus.read_byte_data(addr, REG_INPUT_1)
        return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]
    except Exception as e:
        print(f"[ERROR] PCA 0x{addr:02X} read failed: {e}")
        return [1]*16

# ---------------- MAIN LOOP ----------------
try:
    while True:
        current_time = int(time.time() * 1000)  # ms

        # ====================================================
        # STRUCTURE 1 — SIMPLE TOGGLE
        # ====================================================
        for idx, (addr, pin, pcf, relay_mask) in enumerate(simple_buttons):

            inputs = read_pca_inputs(addr)
            val = inputs[pin]  # 0 = pressed (active-low)

            # Detect press edge (released -> pressed)
            if val == 0 and last_input_simple[idx] == 1:

                if pcf is pcf1:
                    global_state = pcf1_state
                else:
                    global_state = pcf2_state

                # Toggle relay
                global_state ^= relay_mask

                if pcf is pcf1:
                    pcf1_state = global_state
                    pcf1.write_gpio(pcf1_state)
                else:
                    pcf2_state = global_state
                    pcf2.write_gpio(pcf2_state)

                print(f"[SIMPLE] Button {idx} toggled relay")

            last_input_simple[idx] = val

        # ====================================================
        # STRUCTURE 2 — SHORT + LONG PRESS
        # ====================================================
        for idx, (addr, pin, pcf, relay_short, relay_long) in enumerate(debounce_buttons):

            inputs = read_pca_inputs(addr)
            val = inputs[pin]  # 0 = pressed

            # Debounce timing
            if val != last_input_debounce[idx]:
                last_debounce_time[idx] = current_time

            if (current_time - last_debounce_time[idx]) > debounce_delay:

                # ---------------- BUTTON PRESSED ----------------
                if val == 0:

                    if button_press_start_time[idx] == 0:
                        # First moment of press
                        button_press_start_time[idx] = current_time
                        long_press_triggered[idx] = False

                    elif (
                        not long_press_triggered[idx]
                        and (current_time - button_press_start_time[idx] >= long_press_threshold)
                    ):
                        # LONG PRESS TRIGGER
                        if pcf is pcf1:
                            pcf1_state ^= relay_long
                            pcf1.write_gpio(pcf1_state)
                        else:
                            pcf2_state ^= relay_long
                            pcf2.write_gpio(pcf2_state)

                        print(f"[DEBOUNCE] Button {idx} LONG PRESS")
                        long_press_triggered[idx] = True

                # ---------------- BUTTON RELEASED ----------------
                else:
                    if button_press_start_time[idx] != 0 and not long_press_triggered[idx]:
                        # SHORT PRESS TRIGGER
                        if pcf is pcf1:
                            pcf1_state ^= relay_short
                            pcf1.write_gpio(pcf1_state)
                        else:
                            pcf2_state ^= relay_short
                            pcf2.write_gpio(pcf2_state)

                        print(f"[DEBOUNCE] Button {idx} SHORT PRESS")

                    # Reset press tracking
                    button_press_start_time[idx] = 0
                    long_press_triggered[idx] = False

            last_input_debounce[idx] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
