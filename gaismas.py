#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575

# ================= CONFIG =================
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x27
PCF2_ADDR = 0x26

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

# ---------------- TIMING ----------------
LOOP_DELAY = 0.05
debounce_delay = 100
long_press_threshold = 350

# ================= INIT =================
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# All relays OFF (active LOW)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# ================= BUTTON STRUCTURES =================

simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 4),
    (PCA1_ADDR, 8, pcf1, 1 << 0),
    (PCA1_ADDR, 3, pcf1, 1 << 6),
    (PCA1_ADDR, 11, pcf1, 1 << 7),
    (PCA1_ADDR, 2, pcf2, 1 << 7),
    (PCA1_ADDR, 5, pcf1, 1 << 2),
    (PCA1_ADDR, 14, pcf1, 1 << 9),
    (PCA1_ADDR, 6, pcf1, 1 << 3),
    (PCA2_ADDR, 7, pcf1, 1 << 8),
    (PCA1_ADDR, 7, pcf1, 1 << 10),
    (PCA1_ADDR, 1, pcf1, 1 << 11),
    (PCA1_ADDR, 12, pcf1, 1 << 12),
    (PCA1_ADDR, 9, pcf1, 1 << 13),
    (PCA2_ADDR, 14, pcf1, 1 << 14),
    (PCA2_ADDR, 0, pcf2, 1 << 8),
]

debounce_buttons = [
    (PCA2_ADDR, 6, pcf1, 1 << 8,  pcf1, 1 << 4),
    (PCA1_ADDR, 13, pcf1, 1 << 12,  pcf1, 1 << 0),
    (PCA2_ADDR, 15, pcf1, 1 << 14,  pcf1, 1 << 2),
    (PCA1_ADDR, 4, pcf1, 1 << 5,  pcf1, 1 << 7),
    (PCA2_ADDR, 2, pcf2, 1 << 10,  pcf2, 1 << 9),
    (PCA1_ADDR, 15, pcf1, 1 << 5,  pcf1, 1 << 7),
    (PCA2_ADDR, 4, pcf1, 1 << 9,  pcf1, 1 << 7),
    (PCA2_ADDR, 1, pcf1, 1 << 15,  pcf1, 1 << 1),
    (PCA2_ADDR, 5, pcf1, 1 << 15,  pcf1, 1 << 1),
]

# ================= STATE TRACKERS =================

last_simple_state = [1] * len(simple_buttons)

last_debounce_state = [1] * len(debounce_buttons)
last_debounce_time = [0] * len(debounce_buttons)
press_start_time = [0] * len(debounce_buttons)
long_press_triggered = [False] * len(debounce_buttons)

# ================= HELPERS =================

def read_pca_inputs(addr):
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT_0)
        p1 = bus.read_byte_data(addr, REG_INPUT_1)
        return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]
    except:
        return [1]*16

# ================= MAIN LOOP =================

try:
    while True:

        current_time = int(time.time() * 1000)

        pca_inputs = {
            PCA1_ADDR: read_pca_inputs(PCA1_ADDR),
            PCA2_ADDR: read_pca_inputs(PCA2_ADDR),
        }

        # =====================================================
        # STRUCTURE 1 — SIMPLE TOGGLE (ON RELEASE)
        # =====================================================
        for idx, (addr, pin, pcf, relay_mask) in enumerate(simple_buttons):

            val = pca_inputs[addr][pin]  # 0 = pressed

            # Toggle on release (pressed -> released)
            if last_simple_state[idx] == 0 and val == 1:

                if pcf is pcf1:
                    pcf1_state ^= relay_mask
                    pcf1.write_gpio(pcf1_state)
                else:
                    pcf2_state ^= relay_mask
                    pcf2.write_gpio(pcf2_state)

                print(f"[SIMPLE] Button {idx} toggled relay")

            last_simple_state[idx] = val

        # =====================================================
        # STRUCTURE 2 — SHORT + LONG PRESS
        # =====================================================
        for idx, (addr, pin, short_pcf, short_mask, long_pcf, long_mask) in enumerate(debounce_buttons):

            val = pca_inputs[addr][pin]

            if val != last_debounce_state[idx]:
                last_debounce_time[idx] = current_time

            if (current_time - last_debounce_time[idx]) > debounce_delay:

                if val == 0:

                    if press_start_time[idx] == 0:
                        press_start_time[idx] = current_time
                        long_press_triggered[idx] = False

                    elif (
                        not long_press_triggered[idx]
                        and (current_time - press_start_time[idx] >= long_press_threshold)
                    ):

                        if long_pcf is pcf1:
                            pcf1_state ^= long_mask
                            pcf1.write_gpio(pcf1_state)
                        else:
                            pcf2_state ^= long_mask
                            pcf2.write_gpio(pcf2_state)

                        print(f"[DEBOUNCE] Button {idx} LONG PRESS")
                        long_press_triggered[idx] = True

                else:
                    if press_start_time[idx] != 0 and not long_press_triggered[idx]:

                        if short_pcf is pcf1:
                            pcf1_state ^= short_mask
                            pcf1.write_gpio(pcf1_state)
                        else:
                            pcf2_state ^= short_mask
                            pcf2.write_gpio(pcf2_state)

                        print(f"[DEBOUNCE] Button {idx} SHORT PRESS")

                    press_start_time[idx] = 0
                    long_press_triggered[idx] = False

            last_debounce_state[idx] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
