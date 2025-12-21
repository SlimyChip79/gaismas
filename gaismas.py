import time
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017
from adafruit_mcp230xx.digital_inout import DigitalInOut
from adafruit_pcf8575 import PCF8575

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)  # allow bus to stabilize

# MCP23017 boards (2 chips)
mcp1 = MCP23017(i2c, address=0x20)
mcp2 = MCP23017(i2c, address=0x21)

# PCF8575 relay boards (2 boards)
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- INPUTS --------------------
# Map all 32 inputs to MCP pins
inputs = [DigitalInOut(mcp1, i) for i in range(16)] + [DigitalInOut(mcp2, i) for i in range(16)]
for inp in inputs:
    inp.switch_to_input()  # pulled-down by default

# -------------------- INITIAL RELAY STATE --------------------
pcf1.write_gpio(0xFFFF)
pcf2.write_gpio(0xFFFF)
time.sleep(0.2)

# -------------------- STATE --------------------
last_values = [0]*32
toggle_states = [False]*32

print("32-button simple toggle controller started")

# -------------------- LOOP --------------------
while True:
    out1 = 0xFFFF
    out2 = 0xFFFF

    for i, inp in enumerate(inputs):
        val = inp.value
        if val and not last_values[i]:
            toggle_states[i] = not toggle_states[i]
        last_values[i] = val

        if toggle_states[i]:
            if i < 16:
                out1 &= ~(1 << i)
            else:
                out2 &= ~(1 << (i - 16))

    # Write relays
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    # Debug prints
    print("Inputs:", [inp.value for inp in inputs])
    print("Relays out1: {:016b}".format(out1))
    print("Relays out2: {:016b}".format(out2))

    time.sleep(0.05)
