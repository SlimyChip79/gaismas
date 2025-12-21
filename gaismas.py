import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_pcf8575 import PCF8575
from adafruit_mcp230xx.digital_inout import DigitalInOut

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)  # small delay for bus stability

# MCP23017 boards (two chips)
mcp1 = mcp23017.MCP23017(i2c, address=0x20)
mcp2 = mcp23017.MCP23017(i2c, address=0x21)

# PCF8575 relay boards
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- CONFIG --------------------
DEBOUNCE_DELAY = 50          # ms
LONG_PRESS_THRESHOLD = 2000  # ms

# -------------------- NORMAL INPUTS (15) --------------------
# Map 0-31 IDs to MCP pins: (chip, pin)
normal_inputs_map = [
    (mcp1, 0), (mcp1, 1), (mcp1, 2), (mcp1, 3), (mcp1, 4),
    (mcp1, 5), (mcp1, 6), (mcp1, 7), (mcp1, 8), (mcp1, 9),
    (mcp1, 10), (mcp1, 11), (mcp1, 12), (mcp1, 13), (mcp1, 14)
]
normal_relays = list(range(15))  # 0–14 relays

# -------------------- SPECIAL CLICK/HOLD INPUTS (9) --------------------
special_inputs_map = [
    (mcp1, 15),  # input 15
    (mcp2, 0),   # input 16
    (mcp2, 1),   # input 17
    (mcp2, 2),   # input 18
    (mcp2, 3),   # input 19
    (mcp2, 4),   # input 20
    (mcp2, 5),   # input 21
    (mcp2, 6),   # input 22
    (mcp2, 7)    # input 23
]
special_click_relays = list(range(15, 24))  # relays for click
special_hold_relays  = list(range(24, 33))  # relays for hold

# -------------------- INIT INPUTS --------------------
normal_inputs = [DigitalInOut(chip, pin) for chip, pin in normal_inputs_map]
for p in normal_inputs:
    p.switch_to_input()

special_inputs = [DigitalInOut(chip, pin) for chip, pin in special_inputs_map]
for p in special_inputs:
    p.switch_to_input()

# Safety: all relays OFF (active-low)
pcf1.write_gpio(0xFFFF)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

# -------------------- STATE --------------------
normal_last_values = [0]*len(normal_inputs)
normal_states = [False]*len(normal_inputs)

special_last = [0]*len(special_inputs)
special_last_debounce = [0]*len(special_inputs)
special_press_start = [0]*len(special_inputs)
special_long_triggered = [False]*len(special_inputs)

print("32-input (15 normal + 9 click/hold) controller started")

# -------------------- LOOP --------------------
while True:
    current_time = int(time.time() * 1000)  # ms
    out1 = 0xFFFF
    out2 = 0xFFFF

    # ----- NORMAL 15 TOGGLE -----
    for i, pin_obj in enumerate(normal_inputs):
        val = pin_obj.value
        if val and not normal_last_values[i]:
            normal_states[i] = not normal_states[i]
        normal_last_values[i] = val

        relay_pin = normal_relays[i]
        if normal_states[i]:
            if relay_pin < 16:
                out1 &= ~(1 << relay_pin)
            else:
                out2 &= ~(1 << (relay_pin - 16))

    # ----- SPECIAL CLICK/HOLD -----
    for i, pin_obj in enumerate(special_inputs):
        reading = pin_obj.value
        if reading != special_last[i]:
            special_last_debounce[i] = current_time

        if (current_time - special_last_debounce[i]) > DEBOUNCE_DELAY:
            if reading:
                if special_press_start[i] == 0:
                    special_press_start[i] = current_time
                    special_long_triggered[i] = False
                elif not special_long_triggered[i] and (current_time - special_press_start[i] >= LONG_PRESS_THRESHOLD):
                    relay_pin = special_hold_relays[i]
                    if relay_pin < 16:
                        out1 &= ~(1 << relay_pin)
                    else:
                        out2 &= ~(1 << (relay_pin - 16))
                    special_long_triggered[i] = True
            else:
                if special_press_start[i] != 0 and not special_long_triggered[i]:
                    relay_pin = special_click_relays[i]
                    if relay_pin < 16:
                        out1 &= ~(1 << relay_pin)
                    else:
                        out2 &= ~(1 << (relay_pin - 16))
                special_press_start[i] = 0
                special_long_triggered[i] = False

        special_last[i] = reading

    # ----- WRITE RELAYS -----
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    # ----- DEBUG -----
    print("\nNormal Inputs:", [p.value for p in normal_inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))
    print("Special Inputs:", [p.value for p in special_inputs])

    time.sleep(0.05)
