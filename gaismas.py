#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

INT1_PIN = 17
INT2_PIN = 27

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

# Very small delay to prevent 100% CPU lock
POLL_DELAY = 0.001  # 1ms

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bus = SMBus(I2C_BUS)

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# All relays OFF (active-low)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# Track last PCA input state
last_input_1 = [1]*16
last_input_2 = [1]*16

# Track last INT pin state
last_int1 = GPIO.input(INT1_PIN)
last_int2 = GPIO.input(INT2_PIN)

print("[GAISMAS] Ready (INT fast polling mode)")

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + \
           [(p1 >> i) & 1 for i in range(8)]

def handle_pca(addr, last_input, pcf, pcf_state, label):
    inputs = read_pca_inputs(addr)

    for i, val in enumerate(inputs):
        if val == 0 and last_input[i] == 1:
            mask = 1 << i

            if pcf_state & mask:
                pcf_state &= ~mask
                print(f"[{label}] Relay {i+1} ON")
            else:
                pcf_state |= mask
                print(f"[{label}] Relay {i+1} OFF")

            pcf.write_gpio(pcf_state)

        last_input[i] = val

    return pcf_state

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # ---- CHECK PCA1 INT ----
        current1 = GPIO.input(INT1_PIN)
        if current1 == GPIO.LOW and last_int1 == GPIO.HIGH:
            pcf1_state = handle_pca(
                PCA1_ADDR, last_input_1,
                pcf1, pcf1_state, 26
            )
        last_int1 = current1

        # ---- CHECK PCA2 INT ----
        current2 = GPIO.input(INT2_PIN)
        if current2 == GPIO.LOW and last_int2 == GPIO.HIGH:
            pcf2_state = handle_pca(
                PCA2_ADDR, last_input_2,
                pcf2, pcf2_state, 27
            )
        last_int2 = current2

        time.sleep(POLL_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()
