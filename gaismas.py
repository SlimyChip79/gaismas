#!/usr/bin/env python3
import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

print("[GAISMAS] Initializing...")

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# ---------------- DEVICES ----------------
# Inputs
inputs1 = PCF8575(i2c, address=0x20)
inputs2 = PCF8575(i2c, address=0x22)

# Relays (active LOW)
relays1 = PCF8575(i2c, address=0x26)
relays2 = PCF8575(i2c, address=0x27)

# All relays OFF initially
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

# ---------------- INTERRUPT PINS ----------------
INT1_PIN = 17
INT2_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- PRIMING INPUTS FOR INTERRUPT ----------------
# Set all pins as INPUT to enable interrupt
for i in range(16):
    inputs1.direction[i] = 1
    inputs2.direction[i] = 1

# Initial read to clear interrupt flags
_ = inputs1.read()
_ = inputs2.read()

# ---------------- TRACK LAST STATE ----------------
last_input_1 = [1]*16
last_input_2 = [1]*16

print("[GAISMAS] Ready (Polling Interrupt Mode)")

# ---------------- HELPER ----------------
def handle_inputs(inputs, relays, last_inputs, name):
    # read all 16 pins
    vals = inputs.read()
    for i, val in enumerate(vals):
        # detect rising or falling edge
        if val != last_inputs[i]:
            last_inputs[i] = val
            if val == 0:  # assuming button pressed is LOW
                # toggle relay (active LOW)
                relay_mask = 1 << i
                current_state = relays.read()
                if current_state & relay_mask:
                    relays.write_gpio(current_state & ~relay_mask)  # ON
                    print(f"[{name}] Relay {i+1} ON")
                else:
                    relays.write_gpio(current_state | relay_mask)   # OFF
                    print(f"[{name}] Relay {i+1} OFF")
            else:
                print(f"[{name}] Input {i} released")

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # Check first extender
        if GPIO.input(INT1_PIN) == GPIO.HIGH:  # High = triggered
            print("[INT1] Triggered")
            handle_inputs(inputs1, relays1, last_input_1, "PCA1")
            while GPIO.input(INT1_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        # Check second extender
        if GPIO.input(INT2_PIN) == GPIO.HIGH:
            print("[INT2] Triggered")
            handle_inputs(inputs2, relays2, last_input_2, "PCA2")
            while GPIO.input(INT2_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        time.sleep(0.01)  # main loop

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    relays1.write_gpio(0xFFFF)
    relays2.write_gpio(0xFFFF)
    GPIO.cleanup()
