#!/usr/bin/env python3
import time
import logging
from smbus2 import SMBus
import board
import busio
from adafruit_pcf8575 import PCF8575
import RPi.GPIO as GPIO

# ---------------- LOGGING ----------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(message)s",
    datefmt="%H:%M:%S"
)

log = logging.getLogger("GAISMAS")

# ---------------- CONFIG ----------------
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x26
PCF2_ADDR = 0x27

INT1_PIN = 17
INT2_PIN = 27

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01

POLL_INTERVAL = 0.001

# ---------------- INIT ----------------
log.info("Initializing...")

bus = SMBus(I2C_BUS)

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(1)

pcf1 = PCF8575(i2c, address=PCF1_ADDR)
pcf2 = PCF8575(i2c, address=PCF2_ADDR)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

pcf1.write_gpio(pcf1_state)
pcf2.write_gpio(pcf2_state)

last_input_1 = [1]*16
last_input_2 = [1]*16

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

log.info("Ready (interrupt polling)")

# ---------------- PCA READ ----------------
def read_pca_inputs(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT_0)
    p1 = bus.read_byte_data(addr, REG_INPUT_1)
    return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]

# ---------------- MAIN LOOP ----------------
try:
    while True:

        # -------- PCA1 INTERRUPT --------
        if GPIO.input(INT1_PIN) == 0:

            inputs1 = read_pca_inputs(PCA1_ADDR)

            for i, val in enumerate(inputs1):

                if val == 0 and last_input_1[i] == 1:

                    mask = 1 << i

                    if pcf1_state & mask:
                        pcf1_state &= ~mask
                        log.info(f"[PCF26] Relay {i+1} ON")
                    else:
                        pcf1_state |= mask
                        log.info(f"[PCF26] Relay {i+1} OFF")

                    pcf1.write_gpio(pcf1_state)

                last_input_1[i] = val


        # -------- PCA2 INTERRUPT --------
        if GPIO.input(INT2_PIN) == 0:

            inputs2 = read_pca_inputs(PCA2_ADDR)

            for i, val in enumerate(inputs2):

                if val == 0 and last_input_2[i] == 1:

                    mask = 1 << i

                    if pcf2_state & mask:
                        pcf2_state &= ~mask
                        log.info(f"[PCF27] Relay {i+1} ON")
                    else:
                        pcf2_state |= mask
                        log.info(f"[PCF27] Relay {i+1} OFF")

                    pcf2.write_gpio(pcf2_state)

                last_input_2[i] = val

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:

    log.info("Stopping, turning all relays OFF")

    pcf1.write_gpio(0xFFFF)
    pcf2.write_gpio(0xFFFF)

    GPIO.cleanup()
    bus.close()
