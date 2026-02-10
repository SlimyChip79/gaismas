import time
import board
import busio
from adafruit_pcf8574 import PCF8574

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# PCF8574 address
pcf = PCF8574(i2c, address=0x27)

# Set all pins as outputs and OFF
pins = []
for i in range(8):
    p = pcf.get_pin(i)
    p.direction = 1      # OUTPUT
    p.value = True       # OFF (active-low relay)
    pins.append(p)

print("Starting relay test")

while True:
    for i in range(8):
        print(f"Relay {i} ON")
        pins[i].value = False   # ON
        time.sleep(1)

        print(f"Relay {i} OFF")
        pins[i].value = True    # OFF
        time.sleep(0.5)
