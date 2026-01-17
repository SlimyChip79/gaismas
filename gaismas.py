import smbus
import RPi.GPIO as GPIO
import time
import sys

def log(msg):
    print(f"[GAISMAS][PCA9555] {msg}", flush=True)

I2C_BUS = 1
PCA_ADDR = 0x20
INT_PIN = 17

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

log("Starting")

try:
    bus = smbus.SMBus(I2C_BUS)
except Exception as e:
    log(f"I2C failed: {e}")
    sys.exit(1)

GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)  # inputs
bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0x00)  # outputs
bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, 0x00)

log("Configured PCA9555")

def read_inputs():
    return (
        bus.read_byte_data(PCA_ADDR, REG_INPUT_0),
        bus.read_byte_data(PCA_ADDR, REG_INPUT_1)
    )

def int_cb(channel):
    p0, p1 = read_inputs()  # clears interrupt
    log(f"INT | P0={p0:08b} P1={p1:08b}")

    # example: mirror inputs to outputs
    bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, p0)

GPIO.add_event_detect(
    INT_PIN,
    GPIO.FALLING,
    callback=int_cb,
    bouncetime=50
)

log("Interrupt armed")

try:
    while True:
        time.sleep(5)
        log("Alive")

except Exception as e:
    log(f"Main loop error: {e}")

finally:
    GPIO.cleanup()
    log("Exit")
