import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_pcf8575 import PCF8575
from adafruit_mcp230xx.digital_inout import DigitalInOut

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)  # small delay for bus stability

# MCP23017 input boards
mcp1 = mcp23017.MCP23017(i2c, address=0x20)
mcp2 = mcp23017.MCP23017(i2c, address=0x21)

# PCF8575 relay boards
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- CONFIG --------------------
DEBOUNCE_DELAY = 50  # ms

# -------------------- INPUTS --------------------
inputs1_pins = list(range(16))  # MCP1 pins 0-15
inputs2_pins = list(range(16))  # MCP2 pins 0-15

inputs1 = [mcp1.get_pin(pin) for pin in inputs1_pins]
inputs2 = [mcp2.get_pin(pin) for pin in inputs2_pins]

for p in inputs1 + inputs2:
    p.switch_to_input()

all_inputs = inputs1 + inputs2
last_values = [0]*32
states = [False]*32  # store toggle states

# -------------------- INITIAL RELAYS --------------------
pcf1.write_gpio(0xFFFF)  # all relays off (active-low)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

print("32 pushbutton controller started")

# -------------------- LOOP --------------------
while True:
    out1 = 0xFFFF  # PCF8575 board 1 outputs
    out2 = 0xFFFF  # PCF8575 board 2 outputs
    current_time = int(time.time() * 1000)  # ms

    for i, pin_obj in enumerate(all_inputs):
        val = pin_obj.value
        if val and not last_values[i]:  # rising edge
            states[i] = not states[i]   # toggle state
        last_values[i] = val

        # Assign relay output
        relay_pin = i  # 0-31
        if states[i]:
            if relay_pin < 16:
                out1 &= ~(1 << relay_pin)
            else:
                out2 &= ~(1 << (relay_pin - 16))

    # Write to relays
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    # Debug
    print("\nInputs:", [p.value for p in all_inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))

    time.sleep(0.05)
