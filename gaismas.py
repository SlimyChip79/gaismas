#!/usr/bin/env python3
import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

print("[GAISMAS] Initializing...")

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# ---------------- INPUT EXTENDERS ----------------
inputs1 = PCF8575(i2c, address=0x20)  # first PCA9555
inputs2 = PCF8575(i2c, address=0x22)  # second PCA9555

# ---------------- RELAY EXTENDERS ----------------
relays1 = PCF8575(i2c, address=0x26)  # first relay board
relays2 = PCF8575(i2c, address=0x27)  # second relay board

# All relays OFF initially (active LOW)
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

# ---------------- INTERRUPT PINS ----------------
INT1_PIN = 17  # input extender 1
INT2_PIN = 27  # input extender 2

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("[GAISMAS] Ready (Polling Interrupt Mode)")

# ---------------- LAST INPUT STATE ----------------
last_input_1 = [1]*16
last_input_2 = [1]*16

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # --- First extender ---
        if GPIO.input(INT1_PIN) == GPIO.HIGH:  # HIGH = interrupt active
            value = inputs1.read()  # read all 16 pins at once
            print(f"[INT1] Triggered, value={bin(value)}")

            for i in range(16):
                bit = (value >> i) & 1
                if bit != last_input_1[i]:
                    mask = 1 << i
                    if bit == 0:  # button pressed -> turn relay ON
                        relays1.write_gpio(relays1.read() & ~mask)
                        print(f"[RELAYS1] Relay {i+1} ON")
                    else:         # button released -> turn relay OFF
                        relays1.write_gpio(relays1.read() | mask)
                        print(f"[RELAYS1] Relay {i+1} OFF")

                last_input_1[i] = bit

            # Wait until interrupt clears
            while GPIO.input(INT1_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        # --- Second extender ---
        if GPIO.input(INT2_PIN) == GPIO.HIGH:  # HIGH = interrupt active
            value = inputs2.read()
            print(f"[INT2] Triggered, value={bin(value)}")

            for i in range(16):
                bit = (value >> i) & 1
                if bit != last_input_2[i]:
                    mask = 1 << i
                    if bit == 0:
                        relays2.write_gpio(relays2.read() & ~mask)
                        print(f"[RELAYS2] Relay {i+1} ON")
                    else:
                        relays2.write_gpio(relays2.read() | mask)
                        print(f"[RELAYS2] Relay {i+1} OFF")

                last_input_2[i] = bit

            while GPIO.input(INT2_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        time.sleep(0.01)  # small loop delay

except KeyboardInterrupt:
    print("[GAISMAS] Stopping, turning all relays OFF")
    relays1.write_gpio(0xFFFF)
    relays2.write_gpio(0xFFFF)
    GPIO.cleanup()
