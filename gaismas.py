#!/usr/bin/env python3
import time
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575
import RPi.GPIO as GPIO

# ================= CONFIG =================
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x27
PCF2_ADDR = 0x26

INT1_PIN = 17
INT2_PIN = 27

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01

LOOP_DELAY = 0.001
debounce_delay = 100
long_press_threshold = 350

# ================= INIT =================
print("[GAISMAS] Starting...")

bus = SMBus(I2C_BUS)

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

# Interrupt GPIO (ACTIVE LOW)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= BUTTONS =================

simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 4),
    (PCA1_ADDR, 8, pcf1, 1 << 0),
    (PCA1_ADDR, 3, pcf1, 1 << 6),
    (PCA1_ADDR, 11, pcf1, 1 << 7),
    (PCA2_ADDR, 7, pcf1, 1 << 8),
]

debounce_buttons = [
    (PCA1_ADDR, 4, pcf1, 1 << 5, pcf1, 1 << 7),
    (PCA2_ADDR, 2, pcf2, 1 << 10, pcf2, 1 << 9),
]

# ================= STATE =================
last_simple = {}
for addr, pin, _, _ in simple_buttons:
    last_simple[(addr, pin)] = 1

last_debounce = {}
press_start = {}
long_triggered = {}

for addr, pin, _, _, _, _ in debounce_buttons:
    last_debounce[(addr, pin)] = 1
    press_start[(addr, pin)] = 0
    long_triggered[(addr, pin)] = False

# ================= HELPER =================

def read_pca(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ================= MAIN LOOP =================

try:
    while True:

        current_time = int(time.time() * 1000)

        # -------- PCA1 --------
        if GPIO.input(INT1_PIN) == 0:
            inputs = read_pca(PCA1_ADDR)

            # SIMPLE
            for addr, pin, pcf, mask in simple_buttons:
                if addr != PCA1_ADDR:
                    continue

                val = inputs[pin]

                if last_simple[(addr, pin)] == 0 and val == 1:
                    pcf1_state ^= mask
                    pcf1.write_gpio(pcf1_state)

                last_simple[(addr, pin)] = val

            # DEBOUNCE
            for addr, pin, short_pcf, short_mask, long_pcf, long_mask in debounce_buttons:
                if addr != PCA1_ADDR:
                    continue

                val = inputs[pin]

                key = (addr, pin)

                if val != last_debounce[key]:
                    press_start[key] = current_time

                if val == 0:
                    if not long_triggered[key] and \
                       (current_time - press_start[key] >= long_press_threshold):

                        pcf1_state ^= long_mask
                        pcf1.write_gpio(pcf1_state)
                        long_triggered[key] = True

                else:
                    if press_start[key] != 0 and not long_triggered[key]:
                        pcf1_state ^= short_mask
                        pcf1.write_gpio(pcf1_state)

                    press_start[key] = 0
                    long_triggered[key] = False

                last_debounce[key] = val

        # -------- PCA2 --------
        if GPIO.input(INT2_PIN) == 0:
            inputs = read_pca(PCA2_ADDR)

            for addr, pin, pcf, mask in simple_buttons:
                if addr != PCA2_ADDR:
                    continue

                val = inputs[pin]

                if last_simple[(addr, pin)] == 0 and val == 1:
                    pcf2_state ^= mask
                    pcf2.write_gpio(pcf2_state)

                last_simple[(addr, pin)] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping...")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
