import time
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017
from adafruit_mcp230xx.digital_inout import DigitalInOut
from adafruit_pcf8575 import PCF8575

# -------------------- I2C --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# -------------------- BOARDS --------------------
mcp_inputs = MCP23017(i2c, address=0x20)  # 16 inputs
pcf_relays = PCF8575(i2c, address=0x26)   # 16 relays

# -------------------- CONFIG --------------------
DEBOUNCE_DELAY = 50         # ms
LONG_PRESS_THRESHOLD = 2000 # ms

# Split 16 inputs: 8 simple toggle, 8 click/hold
simple_map = [(mcp_inputs, i) for i in range(8)]
complex_map = [(mcp_inputs, i) for i in range(8, 16)]

# Relays mapping: 8 for simple, 8 for complex
simple_relays = list(range(8))
complex_click_relays = list(range(8, 12))
complex_hold_relays  = list(range(12, 16))

# -------------------- INIT --------------------
simple_inputs = [DigitalInOut(chip, pin) for chip, pin in simple_map]
for p in simple_inputs: p.switch_to_input()

complex_inputs = [DigitalInOut(chip, pin) for chip, pin in complex_map]
for p in complex_inputs: p.switch_to_input()

# Safety: all relays OFF (active-low)
pcf_relays.write_gpio(0xFFFF)
time.sleep(0.2)

# States
simple_states = [False]*len(simple_inputs)
simple_last = [0]*len(simple_inputs)

complex_last = [0]*len(complex_inputs)
complex_last_debounce = [0]*len(complex_inputs)
complex_press_start = [0]*len(complex_inputs)
complex_long_triggered = [False]*len(complex_inputs)

print("Simple + complex controller started")

# -------------------- LOOP --------------------
while True:
    now = int(time.time()*1000)
    out = 0xFFFF  # all relays OFF

    # --- SIMPLE TOGGLE ---
    for i, pin_obj in enumerate(simple_inputs):
        val = pin_obj.value
        if val and not simple_last[i]:
            simple_states[i] = not simple_states[i]
        simple_last[i] = val

        if simple_states[i]:
            out &= ~(1 << simple_relays[i])

    # --- COMPLEX CLICK/HOLD ---
    for i, pin_obj in enumerate(complex_inputs):
        val = pin_obj.value
        if val != complex_last[i]:
            complex_last_debounce[i] = now

        if (now - complex_last_debounce[i]) > DEBOUNCE_DELAY:
            if val:
                if complex_press_start[i] == 0:
                    complex_press_start[i] = now
                    complex_long_triggered[i] = False
                elif not complex_long_triggered[i] and (now - complex_press_start[i]) >= LONG_PRESS_THRESHOLD:
                    out &= ~(1 << complex_hold_relays[i])
                    complex_long_triggered[i] = True
            else:
                if complex_press_start[i] != 0 and not complex_long_triggered[i]:
                    out &= ~(1 << complex_click_relays[i])
                complex_press_start[i] = 0
                complex_long_triggered[i] = False

        complex_last[i] = val

    # --- WRITE RELAYS ---
    pcf_relays.write_gpio(out)

    # --- DEBUG ---
    print("Simple:", [p.value for p in simple_inputs])
    print("Complex:", [p.value for p in complex_inputs])
    print("Relays: {:016b}".format(out))

    time.sleep(0.05)
