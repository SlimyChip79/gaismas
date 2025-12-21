import time
import board
import busio
from adafruit_mcp230xx import mcp23017
from adafruit_mcp230xx.digital_inout import DigitalInOut

# -------------------- I2C SETUP --------------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)  # give bus time to stabilize

# -------------------- MCP23017 BOARD --------------------
try:
    mcp2 = mcp23017.MCP23017(i2c, address=0x21)
    time.sleep(0.2)
except OSError as e:
    print("Error initializing MCP23017 at 0x21:", e)
    raise

# -------------------- INPUT SETUP --------------------
inputs = []
for pin in range(16):
    p = DigitalInOut(mcp2.get_pin(pin))
    p.switch_to_input()
    inputs.append(p)

print("Reading MCP23017 at 0x21... Press Ctrl+C to stop")

# -------------------- MAIN LOOP --------------------
while True:
    try:
        values = [p.value for p in inputs]
        print(values)
    except OSError as e:
        print("I2C read error:", e)

    time.sleep(0.1)
