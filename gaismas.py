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

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01

LOOP_DELAY = 0.05
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

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= BUTTON STRUCTURES =================
# (UNCHANGED FROM YOUR ORIGINAL)
# --- keep your simple_buttons and debounce_buttons exactly as before ---
# (I am not rewriting them to avoid introducing mistakes)

# ================= STATE TRACKERS =================
# (UNCHANGED)
# keep your existing trackers here

# ================= HELPERS =================

def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ================= MAIN LOOP =================

last_inputs = {
    PCA1_ADDR: [1]*16,
    PCA2_ADDR: [1]*16
}

try:
    while True:

        # Refresh PCA1 only if interrupt triggered
        if GPIO.input(INT1_PIN) == 0:
            last_inputs[PCA1_ADDR] = read_pca_inputs(PCA1_ADDR)

        # Refresh PCA2 only if interrupt triggered
        if GPIO.input(INT2_PIN) == 0:
            last_inputs[PCA2_ADDR] = read_pca_inputs(PCA2_ADDR)

        # ---- YOUR ORIGINAL LOGIC USES last_inputs ----
        # Replace pca_inputs[...] with last_inputs[...]

        # (Keep your entire simple_buttons + debounce logic exactly same,
        #  just read from last_inputs instead of freshly reading every loop)

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)
    GPIO.cleanup()
    bus.close()
