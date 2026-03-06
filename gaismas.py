#!/usr/bin/env python3
import time
import logging
from smbus2 import SMBus

# ================= CONFIG =================

I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x27
PCF2_ADDR = 0x26

REG_INPUT0 = 0x00
REG_INPUT1 = 0x01

# timing
LOOP_DELAY = 0.01
LONG_PRESS = 0.35

# ================= LOGGING =================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s"
)

# ================= INIT =================

print("GAISMAS starting...")

bus = SMBus(I2C_BUS)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF


# ================= LOW LEVEL I2C =================

def pcf_write(addr, value):
    low = value & 0xFF
    high = (value >> 8) & 0xFF
    bus.write_i2c_block_data(addr, low, [high])


def pcf_read(addr):
    data = bus.read_i2c_block_data(addr, 0, 2)
    return data[0] | (data[1] << 8)


def pca_read(addr):
    p0 = bus.read_byte_data(addr, REG_INPUT0)
    p1 = bus.read_byte_data(addr, REG_INPUT1)

    return (p0 | (p1 << 8))


# ================= RELAY CONTROL =================

def toggle(pcf_id, mask):

    global pcf1_state, pcf2_state

    if pcf_id == 1:
        before = pcf1_state
        pcf1_state ^= mask
        after = pcf1_state

        changed = before ^ after
        if after & mask == 0:
            logging.info(f"RELAY PCF1 MASK {mask:#06x} -> ON")
        else:
            logging.info(f"RELAY PCF1 MASK {mask:#06x} -> OFF")

    else:
        before = pcf2_state
        pcf2_state ^= mask
        after = pcf2_state

        if after & mask == 0:
            logging.info(f"RELAY PCF2 MASK {mask:#06x} -> ON")
        else:
            logging.info(f"RELAY PCF2 MASK {mask:#06x} -> OFF")


def write_outputs():
    pcf_write(PCF1_ADDR, pcf1_state)
    pcf_write(PCF2_ADDR, pcf2_state)


# ================= STARTUP =================

pcf_write(PCF1_ADDR, pcf1_state)
pcf_write(PCF2_ADDR, pcf2_state)

time.sleep(0.1)


# ================= INPUT MAPPING =================

simple_buttons = [
    (PCA1_ADDR, 0, 1, 1 << 4),
    (PCA1_ADDR, 8, 1, 1 << 0),
]

# (keeping your full mapping unchanged — omitted here for space)
# paste your full lists exactly as before


# ================= STATE =================

last_inputs = {
    PCA1_ADDR: 0xFFFF,
    PCA2_ADDR: 0xFFFF
}

press_time = {}
long_done = {}

# ================= MAIN LOOP =================

try:

    while True:

        now = time.time()

        # Read both PCA boards
        pca_data = {
            PCA1_ADDR: pca_read(PCA1_ADDR),
            PCA2_ADDR: pca_read(PCA2_ADDR)
        }

        # -------- LOG INPUT PACKAGES --------
        for addr in [PCA1_ADDR, PCA2_ADDR]:
            if pca_data[addr] != last_inputs[addr]:

                logging.info(
                    f"INPUT CHANGE @ {hex(addr)} | "
                    f"PACKAGE: {pca_data[addr]:016b}"
                )

                last_inputs[addr] = pca_data[addr]

        # (Your button logic stays the same —
        #  just call toggle() where needed)

        write_outputs()

        time.sleep(LOOP_DELAY)


except KeyboardInterrupt:

    print("Stopping, turning relays OFF")

    pcf_write(PCF1_ADDR, 0xFFFF)
    pcf_write(PCF2_ADDR, 0xFFFF)

    bus.close()
