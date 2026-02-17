import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pcf8575 import PCF8575

print("[GAISMAS] Initializing...")

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

# ---------------- DEVICES ----------------
inputs1 = PCF8575(i2c, address=0x20)
inputs2 = PCF8575(i2c, address=0x22)

relays1 = PCF8575(i2c, address=0x26)
relays2 = PCF8575(i2c, address=0x27)

# All relays OFF (active LOW)
relays1.write_gpio(0xFFFF)
relays2.write_gpio(0xFFFF)

# ---------------- INTERRUPT PINS ----------------
INT1_PIN = 17
INT2_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("[GAISMAS] Ready (Polling Interrupt Mode)")

# ---------------- MAIN LOOP ----------------
while True:

    # --- First extender ---
    if GPIO.input(INT1_PIN) == 1:   # HIGH = interrupt active
        value = 0
        for i in range(16):
            if inputs1[i]:
                value |= 1 << i
        relays1.write_gpio(~value & 0xFFFF)   # mirror directly (active LOW)
        print(f"[PCA1] {bin(value)}")

    # --- Second extender ---
    if GPIO.input(INT2_PIN) == 1:   # HIGH = interrupt active
        value = 0
        for i in range(16):
            if inputs2[i]:
                value |= 1 << i
        relays2.write_gpio(~value & 0xFFFF)
        print(f"[PCA2] {bin(value)}")

    time.sleep(0.01)
