import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_pcf8575 import PCF8575

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)

# MCP boards for normal toggle inputs
mcp1 = mcp23017.MCP23017(i2c, address=0x20)
mcp2 = mcp23017.MCP23017(i2c, address=0x21)

# PCF8575 relay boards
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- CONFIG: NORMAL 15 RELAYS --------------------
# Define manually the MCP pins for 15 normal inputs
normal_inputs_pins = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

# Corresponding relay pins
normal_relays_pins = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]

# Split MCP pins for boards
inputs1_pins = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]  # MCP1
inputs2_pins = []  # empty, all 15 on MCP1

# -------------------- CONFIG: SPECIAL 9 CLICK/HOLD --------------------
special_inputs_pins = [15, 16, 17, 18, 19, 20, 21, 22, 23]  # MCP pins
special_click_relays = [15, 16, 17, 18, 19, 20, 21, 22, 23]  # relays for short click
special_hold_relays  = [24, 25, 26, 27, 28, 29, 30, 31, 32]  # relays for long press

DEBOUNCE_DELAY = 50      # ms
LONG_PRESS_THRESHOLD = 2000  # ms

# -------------------- INIT INPUTS --------------------
# Normal inputs
inputs1 = [mcp1.get_pin(pin) for pin in inputs1_pins]
inputs2 = [mcp2.get_pin(pin) for pin in inputs2_pins]
for p in inputs1 + inputs2:
    p.switch_to_input()

# Special inputs (on MCP1 for now)
special_inputs = [mcp1.get_pin(pin) for pin in special_inputs_pins]
for p in special_inputs:
    p.switch_to_input()

# Safety: all relays OFF (active-low)
pcf1.write_gpio(0xFFFF)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

# -------------------- STATE --------------------
normal_last_values = [0]*len(normal_inputs_pins)
normal_states = [False]*len(normal_inputs_pins)

special_last = [0]*len(special_inputs_pins)
special_last_debounce = [0]*len(special_inputs_pins)
special_press_start = [0]*len(special_inputs_pins)
special_long_triggered = [False]*len(special_inputs_pins)

print("15-normal + 9-click/hold controller started")

# -------------------- LOOP --------------------
while True:
    current_time = int(time.time() * 1000)  # ms
    out1 = 0xFFFF
    out2 = 0xFFFF

    # ----- NORMAL 15 TOGGLE -----
    all_normal_inputs = inputs1 + inputs2
    for i, pin_obj in enumerate(all_normal_inputs):
        val = pin_obj.value
        if val and not normal_last_values[i]:
            normal_states[i] = not normal_states[i]
        normal_last_values[i] = val

        relay_pin = normal_relays_pins[i]
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
    print("\nNormal Inputs:", [p.value for p in all_normal_inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))
    print("Special Inputs:", [p.value for p in special_inputs])

    time.sleep(0.05)
