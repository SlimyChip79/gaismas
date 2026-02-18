#!/usr/bin/env python3
import time
from smbus2 import SMBus
import RPi.GPIO as GPIO
import board
import busio
from adafruit_pcf8575 import PCF8575

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22
INT1_PIN = 17  # INT for PCA1
INT2_PIN = 27  # INT for PCA2

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

# --- I2C for PCA ---
bus = SMBus(I2C_BUS)

# --- I2C for PCF ---
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

# All relays OFF (active-low)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF
pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# Track last input state
last_input_1 = [1]*16
last_input_2 = [1]*16

# --- GPIO for PCA interrupts ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- PCA SETUP ----------------
def setup_pca(addr):
    """Set P0 inputs, P1 outputs, INT active on input change"""
    try:
        # P0 inputs, P1 outputs
        bus.write_byte_data(addr, REG_CONFIG_0, 0xFF)
        bus.write_byte_data(addr, REG_CONFIG_1, 0x00)
        bus.write_byte_data(addr, REG_OUTPUT_1, 0x00)
        print(f"[GAISMAS] PCA 0x{addr:02X} configured for interrupt mode")
    except Exception as e:
        print(f"[GAISMAS] PCA 0x{addr:02X} setup error: {e}")

setup_pca(PCA1_ADDR)
setup_pca(PCA2_ADDR)

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def handle_pca(addr, last_input, pcf, pcf_state, label):
    """Read PCA input and toggle relays"""
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

# ---------------- CALLBACKS ----------------
def pca1_callback(channel):
    global pcf1_state
    pcf1_state = handle_pca(PCA1_ADDR, last_input_1, pcf1, pcf1_state, 26)

def pca2_callback(channel):
    global pcf2_state
    pcf2_state = handle_pca(PCA2_ADDR, last_input_2, pcf2, pcf2_state, 27)

# Attach interrupt callbacks (falling edge = active LOW)
GPIO.add_event_detect(INT1_PIN, GPIO.FALLING, callback=pca1_callback, bouncetime=10)
GPIO.add_event_detect(INT2_PIN, GPIO.FALLING, callback=pca2_callback, bouncetime=10)

print("[GAISMAS] Ready (PCA interrupt mode)")

# ---------------- MAIN LOOP ----------------
try:
    while True:
        time.sleep(1)  # main loop just sleeps, all handled via PCA interrupts

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()
