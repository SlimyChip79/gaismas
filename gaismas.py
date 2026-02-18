from smbus2 import SMBus
import RPi.GPIO as GPIO
import time
import sys
from datetime import datetime

# ================= LOG =================
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[GAISMAS] {ts} | {msg}", flush=True)

# ================= CONFIG =================
I2C_BUS = 1
PCA_ADDR = 0x20
INT_PIN = 17

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.02   # 20 ms (safe + fast)

# ================= START =================
log("Service starting")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= I2C =================
bus = None
pca_ready = False

def open_i2c():
    global bus
    try:
        bus = SMBus(I2C_BUS)
        log("I2C bus opened")
    except Exception as e:
        bus = None
        log(f"I2C open failed: {e}")

def pca_setup():
    global pca_ready
    if not bus:
        return
    try:
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)  # P0 inputs
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0x00)  # P1 outputs
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, 0x00)
        pca_ready = True
        log("PCA9555 configured")
    except Exception as e:
        pca_ready = False
        log(f"PCA not ready: {e}")

# ================= PCA READ =================
def handle_pca_interrupt():
    if not pca_ready or not bus:
        return
    try:
        p0 = bus.read_byte_data(PCA_ADDR, REG_INPUT_0)
        p1 = bus.read_byte_data(PCA_ADDR, REG_INPUT_1)
        log(f"INT | P0={p0:08b} P1={p1:08b}")

        # Example action:
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, p0)

    except Exception as e:
        log(f"PCA read error: {e}")

# ================= INIT =================
open_i2c()
pca_setup()

last_int_state = GPIO.input(INT_PIN)
log(f"Initial GPIO17 = {last_int_state}")

alive_timer = time.time()

# ================= MAIN LOOP =================
try:
    while True:
        current = GPIO.input(INT_PIN)

        # Detect state change
        if current != last_int_state:
            log(f"GPIO17 changed: {last_int_state} -> {current}")
            last_int_state = current

            # INT is active LOW
            if current == GPIO.LOW:
                handle_pca_interrupt()

        # Periodic health check
        if time.time() - alive_timer >= 5:
            log("Alive")
            alive_timer = time.time()

            if not pca_ready:
                pca_setup()

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    log("Stopping (KeyboardInterrupt)")

except Exception as e:
    log(f"Fatal error: {e}")

finally:
    GPIO.cleanup()
    if bus:
        bus.close()
    log("Service stopped")
