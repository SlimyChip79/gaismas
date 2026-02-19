#!/usr/bin/env python3
import time
from smbus2 import SMBus
import RPi.GPIO as GPIO
from datetime import datetime

# ================= CONFIG =================
I2C_BUS   = 1
PCA_ADDR  = 0x20
INT_PIN   = 17

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.01  # 10ms

# ================= LOG =================
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}", flush=True)

# ================= GPIO =================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= I2C =================
bus = SMBus(I2C_BUS)

# ================= PCA SETUP =================
# P0 = inputs
# P1 = inputs (change to 0x00 if you want outputs)
bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0xFF)

log("PCA9555 configured (all pins input)")
log(f"Initial INT state: {GPIO.input(INT_PIN)}")

last_int_state = GPIO.input(INT_PIN)

# ================= MAIN LOOP =================
try:
    while True:
        current = GPIO.input(INT_PIN)

        # Detect change on INT pin
        if current != last_int_state:
            log(f"INT changed: {last_int_state} -> {current}")
            last_int_state = current

            # INT is active LOW
            if current == GPIO.LOW:
                p0 = bus.read_byte_data(PCA_ADDR, REG_INPUT_0)
                p1 = bus.read_byte_data(PCA_ADDR, REG_INPUT_1)

                log(f"INT TRIGGERED | P0={p0:08b}  P1={p1:08b}")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    log("Stopping")

finally:
    GPIO.cleanup()
    bus.close()
