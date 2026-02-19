#!/usr/bin/env python3
import smbus2
import RPi.GPIO as GPIO
import time
import signal
import sys

# =============================
# CONFIG
# =============================

I2C_BUS = 1
PCA_ADDR = 0x20
INT_PIN = 17   # GPIO where PCA9555 INT is connected

bus = smbus2.SMBus(I2C_BUS)

# PCA9555 Registers
REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01
REG_OUTPUT_0 = 0x02
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# =============================
# SETUP PCA9555
# =============================

def pca_write(reg, value):
    bus.write_byte_data(PCA_ADDR, reg, value)

def pca_read(reg):
    return bus.read_byte_data(PCA_ADDR, reg)

def setup_pca():
    # All pins input (change if needed)
    pca_write(REG_CONFIG_0, 0xFF)
    pca_write(REG_CONFIG_1, 0xFF)

    # Clear interrupt by reading inputs
    pca_read(REG_INPUT_0)
    pca_read(REG_INPUT_1)

    print("PCA9555 configured (all pins input)")

# =============================
# INTERRUPT HANDLER
# =============================

last_state0 = 0
last_state1 = 0

def handle_interrupt(channel):
    global last_state0, last_state1

    # FAST read (block read = faster)
    data = bus.read_i2c_block_data(PCA_ADDR, REG_INPUT_0, 2)
    state0 = data[0]
    state1 = data[1]

    if state0 != last_state0 or state1 != last_state1:
        print("INT! Port0:", format(state0, "08b"),
              "Port1:", format(state1, "08b"))

        last_state0 = state0
        last_state1 = state1

# =============================
# GPIO SETUP
# =============================

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(
        INT_PIN,
        GPIO.FALLING,
        callback=handle_interrupt,
        bouncetime=1  # very small debounce for speed
    )

# =============================
# CLEAN EXIT
# =============================

def cleanup(sig=None, frame=None):
    GPIO.cleanup()
    bus.close()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

# =============================
# MAIN
# =============================

if __name__ == "__main__":
    setup_pca()
    setup_gpio()

    print("Interrupt system ready")

    while True:
        time.sleep(1)
