import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_pcf8575 import PCF8575

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)

# MCP boards for inputs
mcp1 = mcp23017.MCP23017(i2c, address=0x20)
mcp2 = mcp23017.MCP23017(i2c, address=0x21)

# PCF8575 relay boards
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- CONFIG --------------------
# Virtual pins 0-31
normal_virtual_pins = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]   # 15 normal
normal_relays_pins  = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]

special_virtual_pins = [15,16,17,18,19,20,21,22,23]  # 9 special
special_click_relays = [15,16,17,18,19,20,21,22,23]  # short press
special_hold_relays  = [24,25,26,27,28,29,30,31,32]  # long press

DEBOUNCE_DELAY = 50        # ms
LONG_PRESS_THRESHOLD = 2000  # ms

# -------------------- SPLIT INPUTS MCP1/MCP2 --------------------
inputs1_pins = []
inputs2_pins = []

for vp in normal_virtual_pins + special_virtual_pins:
    if 0 <= vp <= 15:
        inputs1_pins.append(vp)
    elif 16 <= vp <= 31:
        inputs2_pins.append(vp - 16)
    else:
        raise ValueError("Virtual pin out of 0-31 range")

# -------------------- INIT INPUTS --------------------
inputs1 = [mcp1.get_pin(pin) for pin in inputs1_pins]
inputs2 = [mcp2.get_pin(pin) for pin in inputs2_pins]

for p in inputs1 + inputs2:
    p.switch_to_input()  # pulled-down

# Special inputs mapped manually to arrays (click/hold)
special_inputs = []
for vp in special_virtual_pins:
    if vp <= 15:
        special_inputs.append(mcp1.get_pin(vp))
    else:
        special_inputs.append(mcp2.get_pin(vp - 16))
    special_inputs[-1].switch_to_input()

# -------------------- RELAY SAFETY --------------------
pcf1.write_gpio(0xFFFF)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

# -------------------- STATE --------------------
normal_last_values = [0]*len(normal_virtual_pins)
normal_states = [False]*len(normal_virtual_pins)

special_last = [0]*len(special_virtual_pins)
special_last_debounce = [0]*len(special_virtual_pins)
special_press_start = [0]*len(special_virtual_pins)
special_long_triggered = [False]*len(special_virtual_pins)

print("15-normal + 9-click/hold controller started (MCP1/MCP2 split)")

# -------------------- HELPER --------------------
def set_relay_pin(out1, out2, relay_pin):
    if relay_pin < 16:
        out1 &= ~(1 << relay_pin)
    else:
        out2 &= ~(1 << (relay_pin - 16))
    return out1, out2

# -------------------- LOOP --------------------
while True:
    current_time = int(time.time() * 1000)  # ms
    out1 = 0xFFFF
    out2 = 0xFFFF

    # ----- NORMAL TOGGLE -----
    all_normal_inputs = inputs1 + inputs2
    for i, pin_obj in enumerate(all_normal_inputs):
        val = pin_obj.value
        if val and not normal_last_values[i]:
            normal_states[i] = not normal_states[i]
        normal_last_values[i] = val

        if normal_states[i]:
            out1, out2 = set_relay_pin(out1, out2, normal_relays_pins[i])

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
                    out1, out2 = set_relay_pin(out1, out2, special_hold_relays[i])
                    special_long_triggered[i] = True
            else:
                if special_press_start[i] != 0 and not special_long_triggered[i]:
                    out1, out2 = set_relay_pin(out1, out2, special_click_relays[i])
                special_press_start[i] = 0
                special_long_triggered[i] = False

        special_last[i] = reading

    # ----- WRITE RELAYS -----
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    # ----- DEBUG -----
    print("\nInputs1:", [p.value for p in inputs1])
    print("Inputs2:", [p.value for p in inputs2])
    print("Special Inputs:", [p.value for p in special_inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))

    time.sleep(0.05)
