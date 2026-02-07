from smbus2 import SMBus
import time
from datetime import datetime

# ================= CONFIG =================
I2C_BUS = 1

PCA_ADDR = 0x20   # PCA9555 (inputs)
PCF_ADDR = 0x27   # PCF8575 (relays)

# PCA9555 registers
REG_INPUT_0  = 0x00
REG_INPUT_1  = 0x01
REG_CONFIG_0 = 0x06
REG_CONFIG_1 = 0x07

POLL_INTERVAL = 0.2  # seconds

# ================= LOG =================
def log(msg):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[GAISMAS] {ts} | {msg}", flush=True)

# ================= I2C =================
bus = SMBus(I2C_BUS)
log("I2C bus opened")

# ================= PCA SETUP =================
try:
    # Port 0 = inputs (1)
    # Port 1 = inputs (1)  ← change to 0x00 if you want half outputs
    bus.write_byte_data(PCA_ADDR, REG_CONFIG_0, 0xFF)
    bus.write_byte_data(PCA_ADDR, REG_CONFIG_1, 0xFF)

    log("PCA9555 configured (16 inputs)")
except Exception as e:
    log(f"PCA9555 init FAILED: {e}")
    raise SystemExit(1)

# ================= PCF SETUP =================
try:
    # All relays OFF
    bus.write_word_data(PCF_ADDR, 0x00, 0x0000)
    log("PCF8575 relays cleared")
except Exception as e:
    log(f"PCF8575 init FAILED: {e}")
    raise SystemExit(1)

# ================= STATE =================
last_inputs = None
relay_state = 0x0000

# ================= MAIN LOOP =================
try:
    while True:
        # Read PCA inputs
        p0 = bus.read_byte_data(PCA_ADDR, REG_INPUT_0)
        p1 = bus.read_byte_data(PCA_ADDR, REG_INPUT_1)
        inputs = (p1 << 8) | p0   # 16-bit value

        if inputs != last_inputs:
            last_inputs = inputs

            # PCA inputs are active-low usually
            # pressed = 0 → relay ON
            relay_state = (~inputs) & 0xFFFF

            bus.write_word_data(PCF_ADDR, 0x00, relay_state)

            log(f"INPUTS : {inputs:016b}")
            log(f"RELAYS : {relay_state:016b}")

        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    log("Stopping (Ctrl+C)")

finally:
    bus.write_word_data(PCF_ADDR, 0x00, 0x0000)
    bus.close()
    log("All relays OFF, bus closed")
