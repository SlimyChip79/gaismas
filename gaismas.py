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

INT1_PIN = 17
INT2_PIN = 27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.02

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)

# Configure PCA exactly like your working sketch
bus.write_byte_data(PCA1_ADDR, REG_CONFIG_0, 0xFF)  # P0 inputs
bus.write_byte_data(PCA1_ADDR, REG_CONFIG_1, 0xFF)  # P1 inputs

bus.write_byte_data(PCA2_ADDR, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA2_ADDR, REG_CONFIG_1, 0xFF)

# Prime interrupt (important)
bus.read_byte_data(PCA1_ADDR, REG_INPUT_0)
bus.read_byte_data(PCA1_ADDR, REG_INPUT_1)
bus.read_byte_data(PCA2_ADDR, REG_INPUT_0)
bus.read_byte_data(PCA2_ADDR, REG_INPUT_1)

# Setup GPIO interrupt pins
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

last_int1 = GPIO.input(INT1_PIN)
last_int2 = GPIO.input(INT2_PIN)

# ---------------- PCF RELAYS ----------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# Track last input state (for toggle detection)
last_input_1 = [1]*16
last_input_2 = [1]*16

print("[GAISMAS] Ready (Interrupt Mode)")

# ---------------- HELPER ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # -------- PCA1 (0x20) --------
        current1 = GPIO.input(INT1_PIN)
        if current1 != last_int1:
            last_int1 = current1

            if current1 == GPIO.LOW:  # active LOW interrupt
                inputs1 = read_pca_inputs(PCA1_ADDR)

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

        # -------- PCA2 (0x22) --------
        current2 = GPIO.input(INT2_PIN)
        if current2 != last_int2:
            last_int2 = current2

            if current2 == GPIO.LOW:
                inputs2 = read_pca_inputs(PCA2_ADDR)

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

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    bus.close()
    GPIO.cleanup()
