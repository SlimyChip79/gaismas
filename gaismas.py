#!/usr/bin/env python3
import time
from smbus2 import SMBus
import RPi.GPIO as GPIO

# ================= CONFIG =================
I2C_BUS = 1

# PCA (buttons)
PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

# PCF (relays)
PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

# PCA registers
REG_INPUT_0 = 0x00

# Interrupt pins
INT1_PIN = 17
INT2_PIN = 27

# Timing
LOOP_DELAY = 0.001
DEBOUNCE_MS = 100
LONG_PRESS_MS = 350

# ==========================================

print("[SYSTEM] Starting...")

bus = SMBus(I2C_BUS)

# ---------------- GPIO (INT) ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- RELAY STATES --------------
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

bus.write_word_data(PCF1_ADDR, 0x00, pcf1_state)
bus.write_word_data(PCF2_ADDR, 0x00, pcf2_state)

# ================= BUTTON MAPS =================

# (structure kept same behavior as your sketch)

# --- SIMPLE TOGGLE (on release) ---
simple_buttons = [
    (PCA1_ADDR, 0,  PCF1_ADDR, 1<<4),
    (PCA1_ADDR, 8,  PCF1_ADDR, 1<<0),
    (PCA1_ADDR, 3,  PCF1_ADDR, 1<<6),
    (PCA1_ADDR, 11, PCF1_ADDR, 1<<7),
    (PCA2_ADDR, 7,  PCF1_ADDR, 1<<8),
]

# --- SHORT + LONG PRESS ---
debounce_buttons = [
    (PCA2_ADDR, 6,  PCF1_ADDR, 1<<4,  PCF1_ADDR, 1<<0),
    (PCA1_ADDR, 13, PCF1_ADDR, 1<<12, PCF1_ADDR, 1<<1),
]

# ================= STATE TRACKING =================

last_simple = [1]*len(simple_buttons)

last_state = {}
press_time = {}
long_done = {}

# ================= HELPERS =================

def read_pca(addr):
    return bus.read_word_data(addr, REG_INPUT_0)

def get_bit(value, bit):
    return (value >> bit) & 1

# ================= MAIN LOOP =================

print("[SYSTEM] Ready.")

try:
    while True:

        # --------- PCA1 ----------
        if GPIO.input(INT1_PIN) == 0:
            inputs1 = read_pca(PCA1_ADDR)

        else:
            inputs1 = None

        # --------- PCA2 ----------
        if GPIO.input(INT2_PIN) == 0:
            inputs2 = read_pca(PCA2_ADDR)
        else:
            inputs2 = None

        # =====================================================
        # STRUCTURE 1 — TOGGLE ON RELEASE
        # =====================================================
        for idx, (addr, pin, pcf_addr, mask) in enumerate(simple_buttons):

            if addr == PCA1_ADDR and inputs1 is not None:
                val = get_bit(inputs1, pin)
            elif addr == PCA2_ADDR and inputs2 is not None:
                val = get_bit(inputs2, pin)
            else:
                continue

            # release detection
            if last_simple[idx] == 0 and val == 1:

                current = pcf1_state if pcf_addr == PCF1_ADDR else pcf2_state

                current ^= mask

                if pcf_addr == PCF1_ADDR:
                    pcf1_state = current
                else:
                    pcf2_state = current

                bus.write_word_data(pcf_addr, 0x00, current)
                print(f"[SIMPLE] Toggle relay")

            last_simple[idx] = val

        # =====================================================
        # STRUCTURE 2 — SHORT + LONG PRESS
        # =====================================================
        now = int(time.time()*1000)

        for idx, (addr, pin, short_addr, short_mask, long_addr, long_mask) in enumerate(debounce_buttons):

            if addr == PCA1_ADDR and inputs1 is not None:
                val = get_bit(inputs1, pin)
            elif addr == PCA2_ADDR and inputs2 is not None:
                val = get_bit(inputs2, pin)
            else:
                continue

            key = (addr, pin)

            if key not in last_state:
                last_state[key] = 1
                press_time[key] = 0
                long_done[key] = False

            if val == 0:  # pressed

                if press_time[key] == 0:
                    press_time[key] = now
                    long_done[key] = False

                elif (not long_done[key] and
                      now - press_time[key] >= LONG_PRESS_MS):

                    current = pcf1_state if long_addr == PCF1_ADDR else pcf2_state
                    current ^= long_mask

                    if long_addr == PCF1_ADDR:
                        pcf1_state = current
                    else:
                        pcf2_state = current

                    bus.write_word_data(long_addr, 0x00, current)
                    print("[LONG PRESS]")
                    long_done[key] = True

            else:  # released

                if press_time[key] != 0 and not long_done[key]:

                    current = pcf1_state if short_addr == PCF1_ADDR else pcf2_state
                    current ^= short_mask

                    if short_addr == PCF1_ADDR:
                        pcf1_state = current
                    else:
                        pcf2_state = current

                    bus.write_word_data(short_addr, 0x00, current)
                    print("[SHORT PRESS]")

                press_time[key] = 0
                long_done[key] = False

            last_state[key] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\nShutting down...")

    bus.write_word_data(PCF1_ADDR, 0x00, 0xFFFF)
    bus.write_word_data(PCF2_ADDR, 0x00, 0xFFFF)

    GPIO.cleanup()
    bus.close()
