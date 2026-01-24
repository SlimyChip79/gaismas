from smbus2 import SMBus
import RPi.GPIO as GPIO
import time
import sys

def log(msg):
    print(f"[GAISMAS] {msg}", flush=True)

# ================= CONFIG ==================
I2C_BUS = 1
PCA_ADDR = 0x20
INT_PIN = 17

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.1  # seconds for GPIO polling fallback

# ================= START =================
log("Service starting")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# I2C
try:
    bus = SMBus(I2C_BUS)
    log("I2C bus opened")
except Exception as e:
    log(f"I2C init failed: {e}")
    bus = None

pca_ready = False
int_ready = False
last_pin_state = GPIO.input(INT_PIN)

# ================= PCA SETUP =================
def pca_setup():
    global pca_ready
    if not bus:
        return
    try:
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0x00)
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, 0x00)
        pca_ready = True
        log("PCA9555 configured")
    except Exception as e:
        pca_ready = False
        log(f"PCA not ready: {e}")

# ================= GPIO INTERRUPT =================
def interrupt_cb(channel):
    if not pca_ready:
        log("INT ignored (PCA not ready)")
        return
    try:
        p0 = bus.read_byte_data(PCA_ADDR, REG_INPUT_0)
        p1 = bus.read_byte_data(PCA_ADDR, REG_INPUT_1)
        log(f"INT | P0={p0:08b} P1={p1:08b}")
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, p0)
    except Exception as e:
        log(f"INT read error: {e}")

def setup_interrupt():
    global int_ready
    try:
        GPIO.add_event_detect(
            INT_PIN,
            GPIO.FALLING,
            callback=interrupt_cb,
            bouncetime=50
        )
        int_ready = True
        log("GPIO interrupt armed")
    except RuntimeError as e:
        int_ready = False
        log(f"GPIO interrupt not available yet: {e}")

# ================= INIT =================
pca_setup()
setup_interrupt()

# ================= MAIN LOOP =================
try:
    while True:
        time.sleep(5)
        log("Alive")

        if not pca_ready:
            pca_setup()

        if not int_ready:
            setup_interrupt()

        # ==== POLLING FALLBACK ====
        current_state = GPIO.input(INT_PIN)
        if int_ready is False and current_state != last_pin_state:
            last_pin_state = current_state
            if current_state == 0:  # falling edge simulated
                interrupt_cb(INT_PIN)

except Exception as e:
    log(f"Fatal error: {e}")

finally:
    GPIO.cleanup()
    if bus:
        bus.close()
    log("Service stopped")
