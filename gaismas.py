#!/usr/bin/env python3
"""
lights.py — Raspberry Pi 4 light controller
Hardware:
  - 2x PCA9555 input expanders  (0x20, 0x22) — read via smbus2
  - 2x PCF8575 relay expanders  (0x27, 0x26) — written via smbus2 (no register, raw 2-byte write)
  - INT lines: PCA1→GPIO17, PCA2→GPIO27 (active LOW)

Button structures:
  1. SIMPLE  — toggle relay on button release
  2. DEBOUNCE — short press toggles relay A, long press toggles relay B
"""

import time
import RPi.GPIO as GPIO
from smbus2 import SMBus

# ===================== CONFIG =====================

I2C_BUS  = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x27   # pcf1
PCF2_ADDR = 0x26   # pcf2

INT1_PIN  = 17     # active-LOW interrupt from PCA1
INT2_PIN  = 27     # active-LOW interrupt from PCA2

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01

# Timing (milliseconds)
DEBOUNCE_DELAY       = 100
LONG_PRESS_THRESHOLD = 350

POLL_INTERVAL = 0.001  # 1 ms

# ===================== INIT =======================

print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)
time.sleep(0.5)

# PCF8575: active-LOW relays → all OFF = all bits HIGH = 0xFFFF
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

def pcf_write(addr, state):
    """Write 16-bit state to a PCF8575 (no register — raw 2-byte write)."""
    low  = state & 0xFF
    high = (state >> 8) & 0xFF
    try:
        bus.write_i2c_block_data(addr, low, [high])
        print(f"[I2C WRITE] PCF 0x{addr:02X} → state=0x{state:04X}  "
              f"(low=0b{low:08b}  high=0b{high:08b})")
    except OSError as e:
        print(f"[I2C ERROR] PCF 0x{addr:02X} write failed: {e}")

pcf_write(PCF1_ADDR, pcf1_state)
pcf_write(PCF2_ADDR, pcf2_state)

# GPIO interrupt pins
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("[GAISMAS] Ready (interrupt-driven polling)")

# ===================== INPUT MAP ==================

# SIMPLE buttons: (pca_addr, pin, pcf_addr, relay_mask)
#   Toggle relay_mask on pcf_addr when button released.
simple_buttons = [
    (PCA1_ADDR,  0, PCF1_ADDR, 1 << 4),
    (PCA1_ADDR,  8, PCF1_ADDR, 1 << 0),
    (PCA1_ADDR,  3, PCF1_ADDR, 1 << 6),
    (PCA1_ADDR, 11, PCF1_ADDR, 1 << 7),
    (PCA1_ADDR,  2, PCF2_ADDR, 1 << 7),
    (PCA1_ADDR,  5, PCF1_ADDR, 1 << 2),
    (PCA1_ADDR, 14, PCF1_ADDR, 1 << 9),
    (PCA1_ADDR,  6, PCF1_ADDR, 1 << 3),
    (PCA2_ADDR,  7, PCF1_ADDR, 1 << 8),
    (PCA1_ADDR,  7, PCF1_ADDR, 1 << 10),
    (PCA1_ADDR,  1, PCF1_ADDR, 1 << 11),
    (PCA1_ADDR, 12, PCF1_ADDR, 1 << 12),
    (PCA1_ADDR,  9, PCF1_ADDR, 1 << 13),
    (PCA2_ADDR, 14, PCF1_ADDR, 1 << 14),
    (PCA2_ADDR,  0, PCF2_ADDR, 1 << 8),
]

# DEBOUNCE buttons: (pca_addr, pin, short_pcf, short_mask, long_pcf, long_mask)
#   Short press → toggle short relay
#   Long press  → toggle long relay
debounce_buttons = [
    (PCA2_ADDR,  6, PCF1_ADDR, 1 <<  8, PCF1_ADDR, 1 <<  4),
    (PCA1_ADDR, 13, PCF1_ADDR, 1 << 12, PCF1_ADDR, 1 <<  0),
    (PCA2_ADDR, 15, PCF1_ADDR, 1 << 14, PCF1_ADDR, 1 <<  2),
    (PCA1_ADDR,  4, PCF1_ADDR, 1 <<  5, PCF1_ADDR, 1 <<  7),
    (PCA2_ADDR,  2, PCF2_ADDR, 1 << 10, PCF2_ADDR, 1 <<  9),
    (PCA1_ADDR, 15, PCF1_ADDR, 1 <<  5, PCF1_ADDR, 1 <<  7),
    (PCA2_ADDR,  4, PCF1_ADDR, 1 <<  9, PCF1_ADDR, 1 <<  7),
    (PCA2_ADDR,  1, PCF1_ADDR, 1 << 15, PCF1_ADDR, 1 <<  1),
    (PCA2_ADDR,  5, PCF1_ADDR, 1 << 15, PCF1_ADDR, 1 <<  1),
]

# ===================== STATE ======================

# Simple button last pin state (1 = released/idle)
s_last = [1] * len(simple_buttons)

# Debounce button state tracking
d_last        = [1]     * len(debounce_buttons)  # last raw pin value
d_last_change = [0]     * len(debounce_buttons)  # time of last edge (ms)
d_press_start = [0]     * len(debounce_buttons)  # when stable press began
d_long_fired  = [False] * len(debounce_buttons)  # long press already triggered

# ===================== HELPERS ====================

def read_pca(addr):
    """Read 16 input bits from a PCA9555. Returns list of 16 ints (0/1)."""
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT_0)
        p1 = bus.read_byte_data(addr, REG_INPUT_1)
        pins = [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]
        pressed = [i for i, v in enumerate(pins) if v == 0]
        label = f"PCA 0x{addr:02X}"
        print(f"[I2C READ ] {label} → P0=0b{p0:08b} ({p0:#04x})  P1=0b{p1:08b} ({p1:#04x})"
              f"  pressed pins: {pressed if pressed else 'none'}")
        return pins
    except OSError as e:
        print(f"[I2C ERROR] PCA 0x{addr:02X} read failed: {e}")
        return [1] * 16   # fail-safe: treat all as released

def toggle_relay(pcf_addr, mask):
    """Toggle bits in mask on the given PCF8575."""
    global pcf1_state, pcf2_state
    if pcf_addr == PCF1_ADDR:
        pcf1_state ^= mask
        pcf_write(PCF1_ADDR, pcf1_state)
    else:
        pcf2_state ^= mask
        pcf_write(PCF2_ADDR, pcf2_state)

# ===================== MAIN LOOP ==================

# Track INT pin state to log only on edge (change), not every loop
_last_int1 = 1
_last_int2 = 1
_last_heartbeat = 0

try:
    while True:
        now_ms = int(time.time() * 1000)

        # --- Sample INT pins ---
        int1 = GPIO.input(INT1_PIN)
        int2 = GPIO.input(INT2_PIN)

        # Log INT edges (HIGH→LOW = interrupt fired, LOW→HIGH = cleared)
        if int1 != _last_int1:
            state_str = "FIRED (LOW)" if int1 == 0 else "cleared (HIGH)"
            print(f"[INT EDGE ] GPIO{INT1_PIN} (PCA1 0x{PCA1_ADDR:02X}) → {state_str}")
            _last_int1 = int1
        if int2 != _last_int2:
            state_str = "FIRED (LOW)" if int2 == 0 else "cleared (HIGH)"
            print(f"[INT EDGE ] GPIO{INT2_PIN} (PCA2 0x{PCA2_ADDR:02X}) → {state_str}")
            _last_int2 = int2

        # Periodic heartbeat every 10 seconds so you can see the loop is alive
        if now_ms - _last_heartbeat >= 10000:
            print(f"[HEARTBEAT] INT1(GPIO{INT1_PIN})={'LOW' if int1==0 else 'HIGH'}  "
                  f"INT2(GPIO{INT2_PIN})={'LOW' if int2==0 else 'HIGH'}  "
                  f"pcf1=0x{pcf1_state:04X}  pcf2=0x{pcf2_state:04X}")
            _last_heartbeat = now_ms

        # --- Read inputs only when the relevant INT line fires ---
        pca_data = {}
        if int1 == 0:
            print(f"[INT ACTIVE] PCA1 0x{PCA1_ADDR:02X} — reading inputs...")
            pca_data[PCA1_ADDR] = read_pca(PCA1_ADDR)
        if int2 == 0:
            print(f"[INT ACTIVE] PCA2 0x{PCA2_ADDR:02X} — reading inputs...")
            pca_data[PCA2_ADDR] = read_pca(PCA2_ADDR)

        # Skip processing if no interrupt is active
        if not pca_data:
            time.sleep(POLL_INTERVAL)
            continue

        # Fill in any missing PCA with its last-known (all-released) state
        # so that per-button logic can still run for buttons on the active PCA.
        if PCA1_ADDR not in pca_data:
            pca_data[PCA1_ADDR] = [1] * 16
        if PCA2_ADDR not in pca_data:
            pca_data[PCA2_ADDR] = [1] * 16

        # ── STRUCTURE 1: SIMPLE TOGGLE ──────────────────────────────────
        for idx, (addr, pin, pcf_addr, relay_mask) in enumerate(simple_buttons):
            val = pca_data[addr][pin]   # 0 = pressed

            if s_last[idx] == 0 and val == 1:   # released
                toggle_relay(pcf_addr, relay_mask)
                print(f"[SIMPLE #{idx}] toggled relay 0x{relay_mask:04X} on PCF 0x{pcf_addr:02X}")

            s_last[idx] = val

        # ── STRUCTURE 2: SHORT / LONG PRESS ─────────────────────────────
        for idx, (addr, pin, s_pcf, s_mask, l_pcf, l_mask) in enumerate(debounce_buttons):
            val = pca_data[addr][pin]

            # Detect edge → reset debounce timer
            if val != d_last[idx]:
                d_last_change[idx] = now_ms

            # Only act after signal has been stable for DEBOUNCE_DELAY ms
            if (now_ms - d_last_change[idx]) >= DEBOUNCE_DELAY:

                if val == 0:  # button held
                    if d_press_start[idx] == 0:
                        # First stable low: record press start
                        d_press_start[idx] = now_ms
                        d_long_fired[idx]  = False

                    elif (
                        not d_long_fired[idx]
                        and (now_ms - d_press_start[idx]) >= LONG_PRESS_THRESHOLD
                    ):
                        # Long press threshold crossed
                        toggle_relay(l_pcf, l_mask)
                        print(f"[DEBOUNCE #{idx}] LONG PRESS → relay 0x{l_mask:04X} on PCF 0x{l_pcf:02X}")
                        d_long_fired[idx] = True

                else:  # button released
                    if d_press_start[idx] != 0 and not d_long_fired[idx]:
                        # Short press: fire on release
                        toggle_relay(s_pcf, s_mask)
                        print(f"[DEBOUNCE #{idx}] SHORT PRESS → relay 0x{s_mask:04X} on PCF 0x{s_pcf:02X}")

                    # Reset press tracking
                    d_press_start[idx] = 0
                    d_long_fired[idx]  = False

            d_last[idx] = val

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("\n[GAISMAS] Stopping — all relays OFF")
    pcf_write(PCF1_ADDR, 0xFFFF)
    pcf_write(PCF2_ADDR, 0xFFFF)
    GPIO.cleanup()
    bus.close()
