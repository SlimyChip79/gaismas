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

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01

POLL_INTERVAL = 0.001  # fast polling

# ---------------- INIT ----------------
bus = SMBus(I2C_BUS)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, PCF1_ADDR)
pcf2 = PCF8575(i2c, PCF2_ADDR)

# All relays OFF (active-low)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF
pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# ---------------- BUTTON ARRAYS ----------------
# STRUCTURE 1: simple toggle
# Format: (pca_addr, pin_index, pcf_obj, relay_mask)
simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 0),
    (PCA1_ADDR, 1, pcf1, 1 << 1),
]

# STRUCTURE 2: short press relay + long press relay
# Format: (pca_addr, pin_index, pcf_obj, short_mask, long_mask)
debounce_buttons = [
    (PCA2_ADDR, 1, pcf2, 1 << 1, 1 << 2),
    (PCA2_ADDR, 2, pcf2, 1 << 3, 1 << 4),
]

# ---------------- STATE TRACKERS ----------------
last_state_simple = [1]*len(simple_buttons)
last_state_debounce = [1]*len(debounce_buttons)
press_start_time = [0]*len(debounce_buttons)
long_press_triggered = [False]*len(debounce_buttons)
debounce_time = [0]*len(debounce_buttons)

DEBOUNCE_DELAY = 0.02  # 20ms
LONG_PRESS_THRESHOLD = 1.0  # 1 sec

# ---------------- HELPERS ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def pcf_state(pcf):
    return pcf1_state if pcf is pcf1 else pcf2_state

def set_pcf_bit(pcf, mask, off=True):
    global pcf1_state, pcf2_state
    if pcf is pcf1:
        if off:
            pcf1_state |= mask
        else:
            pcf1_state &= ~mask
        pcf1.write_gpio(pcf1_state)
    else:
        if off:
            pcf2_state |= mask
        else:
            pcf2_state &= ~mask
        pcf2.write_gpio(pcf2_state)

def toggle_pcf(pcf, mask):
    if pcf_state(pcf) & mask:
        set_pcf_bit(pcf, mask, False)
    else:
        set_pcf_bit(pcf, mask, True)

# ---------------- PROCESS FUNCTIONS ----------------
def process_simple_buttons(inputs):
    for idx, (addr, pin, pcf, mask) in enumerate(simple_buttons):
        val = inputs[addr][pin]
        if val == 0 and last_state_simple[idx] == 1:
            toggle_pcf(pcf, mask)
            print(f"[SIMPLE] Relay {mask} TOGGLED")
        last_state_simple[idx] = val

def process_debounce_buttons(inputs):
    now = time.time()
    for idx, (addr, pin, pcf, short_mask, long_mask) in enumerate(debounce_buttons):
        val = inputs[addr][pin]
        # debounce
        if val != last_state_debounce[idx]:
            debounce_time[idx] = now
        if (now - debounce_time[idx]) > DEBOUNCE_DELAY:
            if val == 0:
                if press_start_time[idx] == 0:
                    press_start_time[idx] = now
                    long_press_triggered[idx] = False
                elif not long_press_triggered[idx] and (now - press_start_time[idx] >= LONG_PRESS_THRESHOLD):
                    toggle_pcf(pcf, long_mask)
                    print(f"[LONG PRESS] Relay {long_mask} TOGGLED")
                    long_press_triggered[idx] = True
            else:
                if press_start_time[idx] != 0 and not long_press_triggered[idx]:
                    toggle_pcf(pcf, short_mask)
                    print(f"[SHORT PRESS] Relay {short_mask} TOGGLED")
                press_start_time[idx] = 0
                long_press_triggered[idx] = False
        last_state_debounce[idx] = val

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # read PCA inputs
        inputs = {
            PCA1_ADDR: read_pca_inputs(PCA1_ADDR),
            PCA2_ADDR: read_pca_inputs(PCA2_ADDR)
        }

        process_simple_buttons(inputs)
        process_debounce_buttons(inputs)

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
