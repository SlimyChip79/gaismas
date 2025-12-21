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

# -------------------- CONFIG: NORMAL 32 RELAYS --------------------
NUM_NORMAL_INPUTS = 32
normal_inputs_pins = list(range(NUM_NORMAL_INPUTS))  # MCP pins 0..31
normal_relays_pins = list(range(NUM_NORMAL_INPUTS))  # Corresponding relay pins (0..31)

# Split pins for two MCP boards
inputs1_pins = normal_inputs_pins[:16]
inputs2_pins = normal_inputs_pins[16:]

# -------------------- CONFIG: CLICK/HOLD SPECIAL --------------------
NUM_SPECIAL_INPUTS = 9
special_inputs_pins = list(range(NUM_SPECIAL_INPUTS))  # MCP pins for special buttons
special_click_relays = list(range(NUM_SPECIAL_INPUTS))          # relay for short click
special_hold_relays = list(range(NUM_SPECIAL_INPUTS, NUM_SPECIAL_INPUTS*2))  # relay for long press

DEBOUNCE_DELAY = 50      # ms
LONG_PRESS_THRESHOLD = 2000  # ms

# -------------------- INIT --------------------
# Normal inputs
inputs1 = [mcp1.get_pin(pin) for pin in inputs1_pins]
inputs2 = [mcp2.get_pin(pin) for pin in inputs2_pins]
for p in inputs1 + inputs2:
    p.switch_to_input()

# Special inputs (use MCP1 for simplicity)
special_inputs = [mcp1.get_pin(pin) for pin in special_inputs_pins]
for p in special_inputs:
    p.switch_to_input()

# Safety: all relays OFF (active-low)
pcf1.write_gpio(0xFFFF)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

# -------------------- STATE --------------------
# Normal toggle
normal_last_values = [0]*NUM_NORMAL_INPUTS
normal_states = [False]*NUM_NORMAL_INPUTS

# Special click/hold
special_last = [0]*NUM_SPECIAL_INPUTS
special_last_debounce = [0]*NUM_SPECIAL_INPUTS
special_press_start = [0]*NUM_SPECIAL_INPUTS
special_long_triggered = [False]*NUM_SPECIAL_INPUTS

print("Unified 32-toggle + 9-click/hold controller started")

# -------------------- LOOP --------------------
while True:
    current_time = int(time.time() * 1000)  # ms
    # Normal toggle outputs (all 16-bit per board)
    out1 = 0xFFFF
    out2 = 0xFFFF

    # ----- NORMAL 32-INPUT TOGGLE -----
    all_normal_inputs = inputs1 + inputs2
    for i, pin_obj in enumerate(all_normal_inputs):
        val = pin_obj.value
        # detect rising edge (LOW→HIGH)
        if val and not normal_last_values[i]:
            normal_states[i] = not normal_states[i]  # toggle
        normal_last_values[i] = val

        # set relay active-low
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
                    # long press → relay
                    relay_pin = special_hold_relays[i]
                    if relay_pin < 16:
                        out1 &= ~(1 << relay_pin)
                    else:
                        out2 &= ~(1 << (relay_pin - 16))
                    special_long_triggered[i] = True
            else:
                if special_press_start[i] != 0 and not special_long_triggered[i]:
                    # short click → relay
                    relay_pin = special_click_relays[i]
                    if relay_pin < 16:
                        out1 &= ~(1 << relay_pin)
                    else:
                        out2 &= ~(1 << (relay_pin - 16))
                special_press_start[i] = 0
                special_long_triggered[i] = False

        special_last[i] = reading

    # ----- WRITE TO RELAYS -----
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    # ----- DEBUG PRINT -----
    print("\nNormal Inputs:", [p.value for p in all_normal_inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))
    print("Special Inputs:", [p.value for p in special_inputs])

    time.sleep(0.05)  # 20Hz
