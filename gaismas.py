#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575

print("[GAISMAS] Initializing...")

# ---------------- I2C ----------------
bus = SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# ---------------- PCA9555 INPUTS ----------------
PCA1 = 0x20
PCA2 = 0x22

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# Configure all pins as INPUT (enables interrupt)
bus.write_byte_data(PCA1, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA1, REG_CONFIG_1, 0xFF)

bus.write_byte_data(PCA2, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA2, REG_CONFIG_1, 0xFF)

# Prime interrupts (VERY IMPORTANT)
bus.read_byte_data(PCA1, REG_INPUT_0)
bus.read_byte_data(PCA1, REG_INPUT_1)

bus.read_byte_data(PCA2, REG_INPUT_0)
bus.read_byte_data(PCA2, REG_INPUT_1)

# ---------------- PCF8575 RELAYS ----------------
relays1 = PCF8575(i2c, address=0x26)
relays2 = PCF8575(i2c, address=0x27)

# All relays OFF (active LOW)
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

# ---------------- INTERRUPT PINS ----------------
INT1 = 17
INT2 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(INT1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("[GAISMAS] Ready (PCA Interrupt Mode)")

def read_pca(addr):
    low  = bus.read_byte_data(addr, REG_INPUT_0)
    high = bus.read_byte_data(addr, REG_INPUT_1)
    return (high << 8) | low

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # PCA 0x20
        if GPIO.input(INT1) == GPIO.LOW:
            value = read_pca(PCA1)
            print(f"[INT1] {value:016b}")
            relays1.write_gpio(~value & 0xFFFF)

            # wait for interrupt release
            while GPIO.input(INT1) == GPIO.LOW:
                time.sleep(0.001)

        # PCA 0x22
        if GPIO.input(INT2) == GPIO.LOW:
            value = read_pca(PCA2)
            print(f"[INT2] {value:016b}")
            relays2.write_gpio(~value & 0xFFFF)

            while GPIO.input(INT2) == GPIO.LOW:
                time.sleep(0.001)

        time.sleep(0.002)

except KeyboardInterrupt:
    print("Stopping...")
    relays1.write_gpio(0xFFFF)
    relays2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
