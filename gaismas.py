#!/usr/bin/env python3
import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

# ================= CONFIG =================
# I2C addresses
PCA1_ADDR = 0x20
PCA2_ADDR = 0x22
RELAY1_ADDR = 0x26
RELAY2_ADDR = 0x27

# Interrupt pins
INT1_PIN = 17
INT2_PIN = 27

POLL_INTERVAL = 0.02  # 20ms

# ================= SETUP =================
print("[GAISMAS] Initializing...")

# I2C
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# Input extenders
pca1 = PCF8575(i2c, address=PCA1_ADDR)
pca2 = PCF8575(i2c, address=PCA2_ADDR)

# Relay extenders
relay1 = PCF8575(i2c, address=RELAY1_ADDR)
relay2 = PCF8575(i2c, address=RELAY2_ADDR)

# Turn all relays OFF initially (active-low)
relay1.write_gpio(0xFFFF)
relay2.write_gpio(0xFFFF)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Track last interrupt state
last_int1 = GPIO.input(INT1_PIN)
last_int2 = GPIO.input(INT2_PIN)

print("[GAISMAS] Ready (Dual Interrupt Mode)")

# ================= MAIN LOOP =================
try:
    while True:
        # --- PCA1 ---
        int1_state = GPIO.input(INT1_PIN)
        if int1_state != last_int1:
            print(f"INT1 changed: {int1_state}")
            last_int1 = int1_state

        # Inverted logic: interrupt active HIGH
        if int1_state == 1:
            value = pca1.gpio
            # write input directly to relay1 (active-low)
            relay1.write_gpio(~value & 0xFFFF)
            print(f"PCA1 INPUT={bin(value)} RELAY1={bin(~value & 0xFFFF)}")

        # --- PCA2 ---
        int2_state = GPIO.input(INT2_PIN)
        if int2_state != last_int2:
            print(f"INT2 changed: {int2_state}")
            last_int2 = int2_state

        # Inverted logic: interrupt active HIGH
        if int2_state == 1:
            value = pca2.gpio
            # write input directly to relay2 (active-low)
            relay2.write_gpio(~value & 0xFFFF)
            print(f"PCA2 INPUT={bin(value)} RELAY2={bin(~value & 0xFFFF)}")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("[GAISMAS] Stopping (KeyboardInterrupt)")

finally:
    GPIO.cleanup()
    relay1.write_gpio(0xFFFF)
    relay2.write_gpio(0xFFFF)
    print("[GAISMAS] All relays OFF")
