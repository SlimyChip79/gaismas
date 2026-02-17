#!/usr/bin/env python3
import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

# ---------------- CONFIG ----------------
INT1_PIN = 17  # PCA1 interrupt
INT2_PIN = 27  # PCA2 interrupt

PCF_INPUT1_ADDR = 0x20
PCF_INPUT2_ADDR = 0x22

PCF_RELAY1_ADDR = 0x26
PCF_RELAY2_ADDR = 0x27

POLL_INTERVAL = 0.01  # 10 ms

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

inputs1 = PCF8575(i2c, address=PCF_INPUT1_ADDR)
inputs2 = PCF8575(i2c, address=PCF_INPUT2_ADDR)

relays1 = PCF8575(i2c, address=PCF_RELAY1_ADDR)
relays2 = PCF8575(i2c, address=PCF_RELAY2_ADDR)

# All relays OFF initially (active LOW)
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("[GAISMAS] Ready (Dual LOW-Active Interrupt Mode)")

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # ----- First extender -----
        if GPIO.input(INT1_PIN) == GPIO.LOW:  # LOW = interrupt
            value = inputs1.read()            # read clears interrupt
            relays1.write_gpio(value)         # mirror directly
            print(f"[PCA1] {bin(value)}")

        # ----- Second extender -----
        if GPIO.input(INT2_PIN) == GPIO.LOW:
            value = inputs2.read()
            relays2.write_gpio(value)
            print(f"[PCA2] {bin(value)}")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    relays1.write_gpio(0xFFFF)
    relays2.write_gpio(0xFFFF)
    GPIO.cleanup()
