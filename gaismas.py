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

# GPIO interrupts (ACTIVE LOW)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= BUTTON STRUCTURES =================
simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 4),
    (PCA1_ADDR, 8, pcf1, 1 << 0),
    (PCA1_ADDR, 3, pcf1, 1 << 6),
    (PCA1_ADDR, 11, pcf1, 1 << 7),
    (PCA1_ADDR, 2, pcf2, 1 << 7),
    (PCA1_ADDR, 5, pcf1, 1 << 2),
    (PCA1_ADDR, 14, pcf1, 1 << 9),
    (PCA1_ADDR, 6, pcf1, 1 << 3),
    (PCA2_ADDR, 7, pcf1, 1 << 8),
    (PCA1_ADDR, 7, pcf1, 1 << 10),
    (PCA1_ADDR, 1, pcf1, 1 << 11),
    (PCA1_ADDR, 12, pcf1, 1 << 12),
    (PCA1_ADDR, 9, pcf1, 1 << 13),
    (PCA2_ADDR, 14, pcf1, 1 << 14),
    (PCA2_ADDR, 0, pcf2, 1 << 8),
]

debounce_buttons = [
    (PCA2_ADDR, 6, pcf1, 1 << 8,  pcf1, 1 << 4),
    (PCA1_ADDR, 13, pcf1, 1 << 12, pcf1, 1 << 0),
    (PCA2_ADDR, 15, pcf1, 1 << 14, pcf1, 1 << 2),
    (PCA1_ADDR, 4, pcf1, 1 << 5,  pcf1, 1 << 7),
    (PCA2_ADDR, 2, pcf2, 1 << 10, pcf2, 1 << 9),
    (PCA1_ADDR, 15, pcf1, 1 << 5, pcf1, 1 << 7),
    (PCA2_ADDR, 4, pcf1, 1 << 9,  pcf1, 1 << 7),
    (PCA2_ADDR, 1, pcf1, 1 << 15, pcf1, 1 << 1),
    (PCA2_ADDR, 5, pcf1, 1 << 15, pcf1, 1 << 1),
]

# ================= STATE =================
last_simple_state = [1] * len(simple_buttons)

last_debounce_state = [1] * len(debounce_buttons)
last_debounce_time = [0] * len(debounce_buttons)
press_start_time = [0] * len(debounce_buttons)
long_press_triggered = [False] * len(debounce_buttons)

# ================= HELPERS =================
def read_pca(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ================= MAIN LOOP =================
try:
    while True:

        current_time = int(time.time() * 1000)

        # ---------- PCA1 ----------
        if GPIO.input(INT1_PIN) == 0:
            inputs1 = read_pca(PCA1_ADDR)

            for idx, (addr, pin, pcf, relay_mask) in enumerate(simple_buttons):
                if addr != PCA1_ADDR:
                    continue

                val = inputs1[pin]

                if last_simple_state[idx] == 0 and val == 1:
                    pcf1_state ^= relay_mask
                    pcf1.write_gpio(pcf1_state)

                last_simple_state[idx] = val

            for idx, (addr, pin, short_pcf, short_mask, long_pcf, long_mask) in enumerate(debounce_buttons):
                if addr != PCA1_ADDR:
                    continue

                val = inputs1[pin]

                if val != last_debounce_state[idx]:
                    last_debounce_time[idx] = current_time

                if (current_time - last_debounce_time[idx]) > debounce_delay:

                    if val == 0:
                        if press_start_time[idx] == 0:
                            press_start_time[idx] = current_time
                            long_press_triggered[idx] = False

                        elif not long_press_triggered[idx] and \
                             (current_time - press_start_time[idx] >= long_press_threshold):

                            pcf1_state ^= long_mask
                            pcf1.write_gpio(pcf1_state)
                            long_press_triggered[idx] = True

                    else:
                        if press_start_time[idx] != 0 and not long_press_triggered[idx]:
                            pcf1_state ^= short_mask
                            pcf1.write_gpio(pcf1_state)

                        press_start_time[idx] = 0
                        long_press_triggered[idx] = False

                last_debounce_state[idx] = val

        # ---------- PCA2 ----------
        if GPIO.input(INT2_PIN) == 0:
            inputs2 = read_pca(PCA2_ADDR)

            for idx, (addr, pin, pcf, relay_mask) in enumerate(simple_buttons):
                if addr != PCA2_ADDR:
                    continue

                val = inputs2[pin]

                if last_simple_state[idx] == 0 and val == 1:
                    pcf2_state ^= relay_mask
                    pcf2.write_gpio(pcf2_state)

                last_simple_state[idx] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
