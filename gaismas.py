#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
from smbus2 import SMBus
from datetime import datetime

# ================= CONFIG =================
I2C_BUS  = 1
PCA_ADDR = 0x20
INT_PIN  = 17

REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# ================= LOG =================
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}", flush=True)

# ================= GPIO SETUP =================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= I2C SETUP =================
bus = SMBus(I2C_BUS)

# Configure PCA9555 all pins as INPUTS
bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)
bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0xFF)

log("PCA9555 configured (all pins INPUT)")
log("Monitoring GPIO17 interrupt line...")

# ================= MAIN LOOP =================
try:
    while True:
        state = GPIO.input(INT_PIN)
        log(f"GPIO17 state: {state}")
        time.sleep(0.5)

except KeyboardInterrupt:
    log("Stopping")

finally:
    GPIO.cleanup()
    bus.close()
