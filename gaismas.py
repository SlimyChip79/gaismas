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
    level=logging.INFO,
    format="%(asctime)s | %(message)s"
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

        if before & mask:
            logging.info(f"RELAY PCF1 {mask:#06x} -> ON")
        else:
            logging.info(f"RELAY PCF1 {mask:#06x} -> OFF")

    else:
        before = pcf2_state
        pcf2_state ^= mask
        after = pcf2_state

        if before & mask:
            logging.info(f"RELAY PCF2 {mask:#06x} -> ON")
        else:
            logging.info(f"RELAY PCF2 {mask:#06x} -> OFF")


def write_outputs():
    pcf_write(PCF1_ADDR, pcf1_state)
    pcf_write(PCF2_ADDR, pcf2_state)


# ================= STARTUP =================

pcf_write(PCF1_ADDR, pcf1_state)
pcf_write(PCF2_ADDR, pcf2_state)

time.sleep(0.1)


# ================= INPUT MAPPING =================
# (YOUR EXACT ORIGINAL MAPPING — NOT REMOVED)

simple_buttons = [
    (PCA1_ADDR, 0, 1, 1 << 4),
    (PCA1_ADDR, 8, 1, 1 << 0),
    (PCA1_ADDR, 3, 1, 1 << 6),
    (PCA1_ADDR, 11, 1, 1 << 7),
    (PCA1_ADDR, 2, 2, 1 << 7),
    (PCA1_ADDR, 5, 1, 1 << 2),
    (PCA1_ADDR, 14, 1, 1 << 9),
    (PCA1_ADDR, 6, 1, 1 << 3),
    (PCA2_ADDR, 7, 1, 1 << 8),
    (PCA1_ADDR, 7, 1, 1 << 10),
    (PCA1_ADDR, 1, 1, 1 << 11),
    (PCA1_ADDR, 12, 1, 1 << 12),
    (PCA1_ADDR, 9, 1, 1 << 13),
    (PCA2_ADDR, 14, 1, 1 << 14),
    (PCA2_ADDR, 0, 2, 1 << 8),
]

debounce_buttons = [
    (PCA2_ADDR, 6, 1, 1 << 8, 1, 1 << 4),
    (PCA1_ADDR, 13, 1, 1 << 12, 1, 1 << 0),
    (PCA2_ADDR, 15, 1, 1 << 14, 1, 1 << 2),
    (PCA1_ADDR, 4, 1, 1 << 5, 1, 1 << 7),
    (PCA2_ADDR, 2, 2, 1 << 10, 2, 1 << 9),
    (PCA1_ADDR, 15, 1, 1 << 5, 1, 1 << 7),
    (PCA2_ADDR, 4, 1, 1 << 9, 1, 1 << 7),
    (PCA2_ADDR, 1, 1, 1 << 15, 1, 1 << 1),
    (PCA2_ADDR, 5, 1, 1 << 15, 1, 1 << 1),
]

# ================= STATE =================

last_simple = [1] * len(simple_buttons)
press_time = [0] * len(debounce_buttons)
long_done = [False] * len(debounce_buttons)

last_inputs = {
    PCA1_ADDR: 0xFFFF,
    PCA2_ADDR: 0xFFFF
}

# ================= MAIN LOOP =================

try:
    while True:

        now = time.time()

        pca = {
            PCA1_ADDR: pca_read(PCA1_ADDR),
            PCA2_ADDR: pca_read(PCA2_ADDR)
        }

        # -------- LOG INPUT PACKAGES --------
        for addr in [PCA1_ADDR, PCA2_ADDR]:
            if pca[addr] != last_inputs[addr]:
                logging.info(
                    f"INPUT @ {hex(addr)} | PACKAGE: {pca[addr]:016b}"
                )
                last_inputs[addr] = pca[addr]

        # -------- SIMPLE TOGGLE --------
        for i, (addr, pin, pcf, mask) in enumerate(simple_buttons):

            val = (pca[addr] >> pin) & 1

            if last_simple[i] == 0 and val == 1:
                toggle(pcf, mask)

            last_simple[i] = val

        # -------- SHORT + LONG PRESS --------
        for i, (addr, pin, spcf, smask, lpcf, lmask) in enumerate(debounce_buttons):

            val = (pca[addr] >> pin) & 1

            if val == 0:

                if press_time[i] == 0:
                    press_time[i] = now
                    long_done[i] = False

                elif not long_done[i] and now - press_time[i] >= LONG_PRESS:
                    toggle(lpcf, lmask)
                    long_done[i] = True

            else:

                if press_time[i] != 0 and not long_done[i]:
                    toggle(spcf, smask)

                press_time[i] = 0
                long_done[i] = False

        write_outputs()
        time.sleep(LOOP_DELAY)


except KeyboardInterrupt:

    print("Stopping, turning relays OFF")

    pcf_write(PCF1_ADDR, 0xFFFF)
    pcf_write(PCF2_ADDR, 0xFFFF)

    bus.close()
