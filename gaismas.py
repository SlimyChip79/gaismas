#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575
import RPi.GPIO as GPIO

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07
REG_OUTPUT_1 = 0x03

INT1_PIN = 17  # PCA1 INT
INT2_PIN = 27  # PCA2 INT

POLL_INTERVAL = 0.02  # 20ms

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

# I2C for PCF8575
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# I2C for PCA9555
bus = SMBus(I2C_BUS)

# All relays OFF (active-low)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF
pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# ---------------- PCA SETUP ----------------
def setup_pca(addr):
    # P0 = inputs (interrupts)
    bus.write_byte_data(addr, REG_CONFIG_0, 0xFF)  
    # P1 = outputs
    bus.write_byte_data(addr, REG_CONFIG_1, 0x00)
    # Start outputs HIGH (LEDs OFF if active-low)
    bus.write_byte_data(addr, REG_OUTPUT_1, 0xFF)


setup_pca(PCA1_ADDR)
setup_pca(PCA2_ADDR)

# ---------------- GPIO INT SETUP ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Track last input state (edge detection)
last_input_1 = [1]*16
last_input_2 = [1]*16

print("[GAISMAS] Ready (Interrupt Mode)")

# ---------------- HELPERS ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def handle_pca(addr, pcf, pcf_state, last_input, label):
    inputs = read_pca_inputs(addr)
    for i, val in enumerate(inputs):
        if val == 0 and last_input[i] == 1:  # falling edge
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
        # PCA1 INT
        if GPIO.input(INT1_PIN) == 0:  # INT active LOW
            pcf1_state = handle_pca(PCA1_ADDR, pcf1, pcf1_state, last_input_1, 26)
            while GPIO.input(INT1_PIN) == 0:
                time.sleep(0.001)

        # PCA2 INT
        if GPIO.input(INT2_PIN) == 0:  # INT active LOW
            pcf2_state = handle_pca(PCA2_ADDR, pcf2, pcf2_state, last_input_2, 27)
            while GPIO.input(INT2_PIN) == 0:
                time.sleep(0.001)

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
