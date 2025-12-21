import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_pcf8575 import PCF8575
from adafruit_mcp230xx.digital_inout import DigitalInOut

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# MCP23017 input boards
mcp1 = mcp23017.MCP23017(i2c, address=0x20)
mcp2 = mcp23017.MCP23017(i2c, address=0x21)

# PCF8575 relay boards
pcf1 = PCF8575(i2c, address=0x26)
pcf2 = PCF8575(i2c, address=0x27)

# -------------------- INPUT SETUP --------------------
inputs = []
for pin in range(16):
    p = DigitalInOut(mcp1.get_pin(pin))
    p.switch_to_input()
    inputs.append(p)

for pin in range(16):
    p = DigitalInOut(mcp2.get_pin(pin))
    p.switch_to_input()
    inputs.append(p)

# -------------------- RELAY STATE --------------------
relay_states = [False]*32  # all OFF
out1 = 0xFFFF  # PCF8575 outputs start HIGH (OFF)
out2 = 0xFFFF
pcf1.write_gpio(out1)
pcf2.write_gpio(out2)

# -------------------- BUTTON STATE TRACKING --------------------
last_values = [1]*32  # initialize to HIGH (not pressed)

print("32-input pushbutton controller started")

# -------------------- MAIN LOOP --------------------
while True:
    for i, inp in enumerate(inputs):
        currentState = inp.value  # LOW=pressed, HIGH=released

        # Arduino-style edge detection: LOW->HIGH toggle
        if currentState == 0 and last_values[i] == 1:  # pressed (LOW)
            relay_states[i] = not relay_states[i]

        last_values[i] = currentState

        # update outputs
        if i < 16:
            if relay_states[i]:
                out1 &= ~(1 << i)  # LOW = ON
            else:
                out1 |= (1 << i)   # HIGH = OFF
        else:
            idx = i - 16
            if relay_states[i]:
                out2 &= ~(1 << idx)
            else:
                out2 |= (1 << idx)

    # write outputs
    pcf1.write_gpio(out1)
    pcf2.write_gpio(out2)

    time.sleep(0.05)
