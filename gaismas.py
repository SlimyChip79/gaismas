#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575
from datetime import datetime

# ---------------- LOG ----------------
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[GAISMAS] {ts} | {msg}", flush=True)

# ---------------- CONFIG ----------------
I2C_BUS = 1

# PCA extenders
PCA1_ADDR = 0x20
PCA2_ADDR = 0x22
INT1_PIN = 17  # PCA1 INT pin
INT2_PIN = 27  # PCA2 INT pin

# PCF relays (unchanged)
PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.02  # 20ms

# ---------------- INIT ----------------
log("Initializing...")

# GPIO setup
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# I2C bus
bus = SMBus(I2C_BUS)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# PCF relays
pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)
pcf1_state = 0xFFFF  # all OFF
pcf2_state = 0xFFFF
pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# PCA last INT pin state (for edge detection)
last_int1_state = GPIO.input(INT1_PIN)
last_int2_state = GPIO.input(INT2_PIN)

# Track last PCA input states
last_input_1 = [1]*16
last_input_2 = [1]*16

log("Ready (PCA interrupt mode)")

# ---------------- HELPERS ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def handle_pca(addr, last_input, pcf, pcf_state, board_name):
    """Read PCA inputs, toggle relays if edge detected"""
    inputs = read_pca_inputs(addr)
    for i, val in enumerate(inputs):
        if val == 0 and last_input[i] == 1:
            mask = 1 << i
            if pcf_state & mask:
                pcf_state &= ~mask
                log(f"[{board_name}] Relay {i+1} ON")
            else:
                pcf_state |= mask
                log(f"[{board_name}] Relay {i+1} OFF")
            pcf.write_gpio(pcf_state)
        last_input[i] = val
    return pcf_state

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # -------- POLL PCA1 --------
        current_int1 = GPIO.input(INT1_PIN)
        if current_int1 != last_int1_state:
            last_int1_state = current_int1
            if current_int1 == GPIO.LOW:  # INT active
                pcf1_state = handle_pca(PCA1_ADDR, last_input_1, pcf1, pcf1_state, 26)

        # -------- POLL PCA2 --------
        current_int2 = GPIO.input(INT2_PIN)
        if current_int2 != last_int2_state:
            last_int2_state = current_int2
            if current_int2 == GPIO.LOW:  # INT active
                pcf2_state = handle_pca(PCA2_ADDR, last_input_2, pcf2, pcf2_state, 27)

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    log("Stopping (KeyboardInterrupt), turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()

except Exception as e:
    log(f"Fatal error: {e}")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()
