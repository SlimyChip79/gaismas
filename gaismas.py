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

LOOP_DELAY = 0.01
LONG_PRESS = 0.35

# ================= LOGGING =================

logging.basicConfig(
    filename="/home/chip/gaismas/gaismas_debug.log",
    level=logging.DEBUG,
    format="%(asctime)s %(message)s"
)

def log(msg):
    print(msg)
    logging.debug(msg)

# ================= INIT =================

log("[GAISMAS] Starting...")

bus = SMBus(I2C_BUS)

pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

# ================= LOW LEVEL I2C =================

def pcf_write(addr, value):
    low = value & 0xFF
    high = (value >> 8) & 0xFF

    log(f"[PCF WRITE] addr=0x{addr:X} value=0x{value:04X} bytes=[{low:02X},{high:02X}]")

    bus.write_i2c_block_data(addr, low, [high])

def pcf_read(addr):
    data = bus.read_i2c_block_data(addr, 0, 2)
    value = data[0] | (data[1] << 8)

    log(f"[PCF READ] addr=0x{addr:X} raw={data} value=0x{value:04X}")

    return value

def pca_read(addr):
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT0)
        p1 = bus.read_byte_data(addr, REG_INPUT1)

        log(f"[PCA READ] addr=0x{addr:X} REG0=0x{p0:02X} REG1=0x{p1:02X}")

        pins = []

        for i in range(8):
            pins.append((p0 >> i) & 1)

        for i in range(8):
            pins.append((p1 >> i) & 1)

        return pins

    except Exception as e:
        log(f"[PCA ERROR] {e}")
        return [1]*16

# ================= RELAY CONTROL =================

def toggle(pcf_id, mask):

    global pcf1_state, pcf2_state

    if pcf_id == 1:
        before = pcf1_state
        pcf1_state ^= mask
        after = pcf1_state
    else:
        before = pcf2_state
        pcf2_state ^= mask
        after = pcf2_state

    log(f"[TOGGLE] pcf={pcf_id} mask=0x{mask:04X} "
        f"before=0x{before:04X} after=0x{after:04X}")

# ================= STARTUP RELAYS OFF =================

pcf_write(PCF1_ADDR, pcf1_state)
pcf_write(PCF2_ADDR, pcf2_state)

time.sleep(0.1)

# ================= INPUT MAPPING =================

simple_buttons = [
    (PCA1_ADDR, 0, 1, 1 << 4),
    (PCA1_ADDR, 8, 1, 1 << 0),
    (PCA1_ADDR, 3, 1, 1 << 6),
]

# ================= STATE =================

last_simple = [1] * len(simple_buttons)

# ================= MAIN LOOP =================

try:
    while True:

        now = time.time()

        pca = {
            PCA1_ADDR: pca_read(PCA1_ADDR),
            PCA2_ADDR: pca_read(PCA2_ADDR)
        }

        # SIMPLE TOGGLE

        for i, (addr, pin, pcf_id, mask) in enumerate(simple_buttons):

            val = pca[addr][pin]

            log(f"[STATE] Button {i} val={val} last={last_simple[i]}")

            if last_simple[i] == 0 and val == 1:
                log(f"[EVENT] Button {i} RELEASE detected")
                toggle(pcf_id, mask)

            last_simple[i] = val

        # WRITE OUTPUTS

        pcf_write(PCF1_ADDR, pcf1_state)
        pcf_write(PCF2_ADDR, pcf2_state)

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    log("Stopping, turning relays OFF")
    pcf_write(PCF1_ADDR, 0xFFFF)
    pcf_write(PCF2_ADDR, 0xFFFF)
    bus.close()
