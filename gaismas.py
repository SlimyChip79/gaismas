#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575
import RPi.GPIO as GPIO
from datetime import datetime

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

INT1_PIN = 17
INT2_PIN = 27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

# ---------------- INIT ----------------
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[GAISMAS] {ts} | {msg}", flush=True)

print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# PCF relays (unchanged)
pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF
pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# Track last PCF input state
last_input_1 = [1]*16
last_input_2 = [1]*16

# ---------------- GPIO for PCA interrupts ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Track last INT state
last_int1 = GPIO.input(INT1_PIN)
last_int2 = GPIO.input(INT2_PIN)

log(f"Ready (PCA interrupt mode)")

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def handle_pca_interrupt(addr, int_name):
    inputs = read_pca_inputs(addr)
    log(f"{int_name} INT triggered | Inputs: {''.join(str(b) for b in inputs)}")
    return inputs

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # ---------- PCA1 ----------
        current1 = GPIO.input(INT1_PIN)
        if current1 != last_int1:
            last_int1 = current1
            if current1 == GPIO.LOW:
                inputs1 = handle_pca_interrupt(PCA1_ADDR, "PCA1")
                for i, val in enumerate(inputs1):
                    if val == 0 and last_input_1[i] == 1:
                        mask = 1 << i
                        if pcf1_state & mask:
                            pcf1_state &= ~mask
                            print(f"[26] Relay {i+1} ON")
                        else:
                            pcf1_state |= mask
                            print(f"[26] Relay {i+1} OFF")
                        pcf1.write_gpio(pcf1_state)
                    last_input_1[i] = val

        # ---------- PCA2 ----------
        current2 = GPIO.input(INT2_PIN)
        if current2 != last_int2:
            last_int2 = current2
            if current2 == GPIO.LOW:
                inputs2 = handle_pca_interrupt(PCA2_ADDR, "PCA2")
                for i, val in enumerate(inputs2):
                    if val == 0 and last_input_2[i] == 1:
                        mask = 1 << i
                        if pcf2_state & mask:
                            pcf2_state &= ~mask
                            print(f"[27] Relay {i+1} ON")
                        else:
                            pcf2_state |= mask
                            print(f"[27] Relay {i+1} OFF")
                        pcf2.write_gpio(pcf2_state)
                    last_input_2[i] = val

        time.sleep(0.01)  # small sleep for CPU

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()
