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

# ---------------- BUTTON STRUCTURES ----------------
# Structure 1: simple toggle
simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 0),
    (PCA1_ADDR, 1, pcf1, 1 << 1),
]

# Structure 2: short press + long press
debounce_buttons = [
    (PCA2_ADDR, 0, pcf1, 1 << 1, 1 << 2),
    (PCA2_ADDR, 1, pcf2, 1 << 3, 1 << 4),
]

# ---------------- STATE TRACKERS ----------------
last_simple_state = [1] * len(simple_buttons)
last_debounce_reading = [1] * len(debounce_buttons)
last_debounce_time = [0] * len(debounce_buttons)
button_press_start = [0] * len(debounce_buttons)
long_press_triggered = [False] * len(debounce_buttons)

debounce_delay = 50       # ms
long_press_threshold = 1000  # ms

# ---------------- HELPERS ----------------
def read_pca_inputs(addr):
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT_0)
        p1 = bus.read_byte_data(addr, REG_INPUT_1)
        return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]
    except Exception as e:
        print(f"[ERROR] PCA 0x{addr:02X} read failed: {e}")
        return [1]*16

def process_simple_buttons(inputs):
    global pcf1_state, pcf2_state
    for idx, (addr, pin, relay_board, relay_mask) in enumerate(simple_buttons):
        val = inputs[pin] if addr == PCA1_ADDR else read_pca_inputs(addr)[pin]
        # Edge detection: HIGH->LOW
        if val == 0 and last_simple_state[idx] == 1:
            if relay_board is pcf1:
                if pcf1_state & relay_mask:
                    pcf1_state &= ~relay_mask
                    print(f"[SIMPLE] Relay ON (PCF1, mask={relay_mask:04X})")
                else:
                    pcf1_state |= relay_mask
                    print(f"[SIMPLE] Relay OFF (PCF1, mask={relay_mask:04X})")
                relay_board.write_gpio(pcf1_state)
            else:
                if pcf2_state & relay_mask:
                    pcf2_state &= ~relay_mask
                    print(f"[SIMPLE] Relay ON (PCF2, mask={relay_mask:04X})")
                else:
                    pcf2_state |= relay_mask
                    print(f"[SIMPLE] Relay OFF (PCF2, mask={relay_mask:04X})")
                relay_board.write_gpio(pcf2_state)

        last_simple_state[idx] = val

def process_debounce_buttons(pca_inputs_dict):
    global pcf1_state, pcf2_state
    current_time = int(time.time() * 1000)  # ms
    for idx, (addr, pin, relay_board, short_mask, long_mask) in enumerate(debounce_buttons):
        inputs = pca_inputs_dict.get(addr, read_pca_inputs(addr))
        reading = inputs[pin]

        # Debounce
        if reading != last_debounce_reading[idx]:
            last_debounce_time[idx] = current_time

        if (current_time - last_debounce_time[idx]) > debounce_delay:
            if reading == 0:  # pressed
                if button_press_start[idx] == 0:
                    button_press_start[idx] = current_time
                    long_press_triggered[idx] = False
                elif not long_press_triggered[idx] and (current_time - button_press_start[idx] >= long_press_threshold):
                    # Long press
                    if relay_board is pcf1:
                        pcf1_state ^= long_mask
                        pcf1.write_gpio(pcf1_state)
                    else:
                        pcf2_state ^= long_mask
                        pcf2.write_gpio(pcf2_state)
                    print(f"[DEBOUNCE] Long press triggered idx={idx}")
                    long_press_triggered[idx] = True
            else:  # released
                if button_press_start[idx] != 0 and not long_press_triggered[idx]:
                    # Short press
                    if relay_board is pcf1:
                        pcf1_state ^= short_mask
                        pcf1.write_gpio(pcf1_state)
                    else:
                        pcf2_state ^= short_mask
                        pcf2.write_gpio(pcf2_state)
                    print(f"[DEBOUNCE] Short press triggered idx={idx}")
                button_press_start[idx] = 0
                long_press_triggered[idx] = False

        last_debounce_reading[idx] = reading

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # Read both PCAs once
        pca1_inputs = read_pca_inputs(PCA1_ADDR)
        pca2_inputs = read_pca_inputs(PCA2_ADDR)

        # Process structures
        process_simple_buttons(pca1_inputs)
        process_debounce_buttons({PCA2_ADDR: pca2_inputs})

        time.sleep(0.005)  # 5ms loop for fast response

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
