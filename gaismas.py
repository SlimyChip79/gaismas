#!/usr/bin/env python3
import time
import board
import busio
import RPi.GPIO as GPIO
from smbus2 import SMBus
from adafruit_pcf8575 import PCF8575

print("[GAISMAS] Initializing...")

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)
bus = SMBus(1)
time.sleep(1)

# ---------------- PCA9555 INPUT ADDRESSES ----------------
PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

# PCA9555 registers
REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# ---------------- PCF8575 RELAYS ----------------
relays1 = PCF8575(i2c, address=0x26)
relays2 = PCF8575(i2c, address=0x27)

# All relays OFF (active LOW)
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

# ---------------- INTERRUPT GPIO ----------------
INT1_PIN = 17   # PCA 0x20
INT2_PIN = 27   # PCA 0x22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ---------------- SET PCA TO INTERRUPT MODE ----------------
# All pins as INPUT (this enables interrupt generation)
bus.write_byte_data(PCA1_ADDR, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA1_ADDR, REG_CONFIG_1, 0xFF)

bus.write_byte_data(PCA2_ADDR, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA2_ADDR, REG_CONFIG_1, 0xFF)

# IMPORTANT: Prime interrupt by reading once
bus.read_byte_data(PCA1_ADDR, REG_INPUT_0)
bus.read_byte_data(PCA1_ADDR, REG_INPUT_1)

bus.read_byte_data(PCA2_ADDR, REG_INPUT_0)
bus.read_byte_data(PCA2_ADDR, REG_INPUT_1)

print("[GAISMAS] Ready (PCA Interrupt Mode)")

# ---------------- HELPER ----------------
def read_pca(addr):
    low = bus.read_byte_data(addr, REG_INPUT_0)
    high = bus.read_byte_data(addr, REG_INPUT_1)
    return (high << 8) | low

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # -------- PCA 0x20 --------
        if GPIO.input(INT1_PIN) == GPIO.LOW:  # interrupt active
            value = read_pca(PCA1_ADDR)
            print(f"[INT1] Triggered value={value:016b}")

            # Mirror to relay board 1 (active LOW)
            relays1.write_gpio(~value & 0xFFFF)

            # Wait until INT clears
            while GPIO.input(INT1_PIN) == GPIO.LOW:
                time.sleep(0.001)

        # -------- PCA 0x22 --------
        if GPIO.input(INT2_PIN) == GPIO.LOW:
            value = read_pca(PCA2_ADDR)
            print(f"[INT2] Triggered value={value:016b}")

            relays2.write_gpio(~value & 0xFFFF)

            while GPIO.input(INT2_PIN) == GPIO.LOW:
                time.sleep(0.001)

        time.sleep(0.002)

except KeyboardInterrupt:
    print("[GAISMAS] Stopping, turning all relays OFF")
    relays1.write_gpio(0xFFFF)
    relays2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
