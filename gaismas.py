from smbus2 import SMBus
import RPi.GPIO as GPIO
import time

def log(msg):
    print(f"[GAISMAS] {msg}", flush=True)

# ================= CONFIG =================
I2C_BUS = 1
PCA_ADDR = 0x20
INT_PIN = 17

REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_OUTPUT_1 = 0x03
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

# ================= START =================
log("Service starting")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bus = None
pca_ready = False
interrupt_armed = False

# ================= I2C =================
try:
    bus = SMBus(I2C_BUS)
    log("I2C bus opened")
except Exception as e:
    log(f"I2C init failed: {e}")

# ================= PCA SETUP =================
def pca_setup():
    global pca_ready
    if not bus:
        return False
    try:
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)  # P0 inputs
        bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0x00)  # P1 outputs
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, 0x00)
        pca_ready = True
        log("PCA9555 configured")
        return True
    except Exception as e:
        pca_ready = False
        log(f"PCA not ready: {e}")
        return False

# ================= INTERRUPT CALLBACK =================
def interrupt_cb(channel):
    try:
        p0 = bus.read_byte_data(PCA_ADDR, REG_INPUT_0)
        p1 = bus.read_byte_data(PCA_ADDR, REG_INPUT_1)
        log(f"INT | P0={p0:08b} P1={p1:08b}")
        bus.write_byte_data(PCA_ADDR, REG_OUTPUT_1, p0)
    except Exception as e:
        log(f"INT read error: {e}")

# ================= ARM INTERRUPT (ONCE) =================
def arm_interrupt():
    global interrupt_armed
    try:
        GPIO.add_event_detect(
            INT_PIN,
            GPIO.FALLING,
            callback=interrupt_cb,
            bouncetime=50
        )
        interrupt_armed = True
        log("GPIO interrupt armed")
    except RuntimeError as e:
        interrupt_armed = False
        log(f"Interrupt unavailable (continuing without INT): {e}")

# ================= INIT =================
if pca_setup():
    arm_interrupt()

# ================= MAIN LOOP =================
try:
    while True:
        time.sleep(5)
        log("Alive")

        # PCA recovery only
        if not pca_ready:
            if pca_setup():
                log("PCA recovered")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
    if bus:
        bus.close()
    log("Service stopped")
