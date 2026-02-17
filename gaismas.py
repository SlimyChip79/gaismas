#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

INT1_PIN = 17
INT2_PIN = 27

POLL_INTERVAL = 0.01  # 10ms fast loop

# ---------------- INIT ----------------
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

last_input_1 = [1]*16
last_input_2 = [1]*16

print("[GAISMAS] Ready (Stable Loop Mode)")

# ---------------- HELPERS ----------------
def read_pca(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

def handle_pca1():
    global pcf1_state
    inputs = read_pca(PCA1_ADDR)

    for i, val in enumerate(inputs):
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


def handle_pca2():
    global pcf2_state
    inputs = read_pca(PCA2_ADDR)

    for i, val in enumerate(inputs):
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


# ---------------- MAIN LOOP ----------------
try:
    while True:

        # Check PCA1 interrupt
        if GPIO.input(INT1_PIN) == GPIO.HIGH:
            handle_pca1()
            while GPIO.input(INT1_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        # Check PCA2 interrupt
        if GPIO.input(INT2_PIN) == GPIO.HIGH:
            handle_pca2()
            while GPIO.input(INT2_PIN) == GPIO.HIGH:
                time.sleep(0.001)

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
