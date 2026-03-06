#!/usr/bin/env python3
import time
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
DEBOUNCE = 0.1
LONG_PRESS = 0.35

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
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT0)
        p1 = bus.read_byte_data(addr, REG_INPUT1)

        pins = []

        for i in range(8):
            pins.append((p0 >> i) & 1)

        for i in range(8):
            pins.append((p1 >> i) & 1)

        return pins

    except:
        return [1]*16


# ================= RELAY CONTROL =================

def toggle(pcf_id, mask):

    global pcf1_state, pcf2_state

    if pcf_id == 1:
        pcf1_state ^= mask
    else:
        pcf2_state ^= mask


def write_outputs():

    global pcf1_state, pcf2_state

    pcf_write(PCF1_ADDR, pcf1_state)
    pcf_write(PCF2_ADDR, pcf2_state)

    # verify to prevent bit corruption
    try:
        if pcf_read(PCF1_ADDR) != pcf1_state:
            pcf_write(PCF1_ADDR, pcf1_state)

        if pcf_read(PCF2_ADDR) != pcf2_state:
            pcf_write(PCF2_ADDR, pcf2_state)

    except:
        pass


# ================= STARTUP =================

pcf_write(PCF1_ADDR, pcf1_state)
pcf_write(PCF2_ADDR, pcf2_state)

time.sleep(0.1)


# -------- Structure 1: Simple Toggle --------
# Format:
# (PCA_addr, input_pin, pcf_device, output_bit_mask)
#
# PCA_addr        -> which PCA9555 input expander the button is on
# input_pin       -> pin number on that PCA (0-15)
# pcf_device      -> which PCF8575 output chip controls the relay
# output_bit_mask -> which relay bit to toggle (1 << bit)

simple_buttons = [
    (PCA1_ADDR, 0, pcf1, 1 << 4),  # janis griesti galvenais
    (PCA1_ADDR, 8, pcf1, 1 << 0),  # ieva griesti galvenais
    (PCA1_ADDR, 3, pcf1, 1 << 6),  # virtuve
    (PCA1_ADDR, 11, pcf1, 1 << 7), # koridors vidus sledzis
    (PCA1_ADDR, 2, pcf2, 1 << 7),  # tehniska
    (PCA1_ADDR, 5, pcf1, 1 << 2),  # gulamistaba galvenais
    (PCA1_ADDR, 14, pcf1, 1 << 9), # vejtveris durvis
    (PCA1_ADDR, 6, pcf1, 1 << 3),  # maza vanna
    (PCA2_ADDR, 7, pcf1, 1 << 8),  # janis bra durvis
    (PCA1_ADDR, 7, pcf1, 1 << 10), # skapis
    (PCA1_ADDR, 1, pcf1, 1 << 11), # liela vanna
    (PCA1_ADDR, 12, pcf1, 1 << 12),# ieva bra durvis
    (PCA1_ADDR, 9, pcf1, 1 << 13), # maza vanna bra
    (PCA2_ADDR, 14, pcf1, 1 << 14),# gulamistaba bra durvis
    (PCA2_ADDR, 0, pcf2, 1 << 8),  # pagrabs
]


# -------- Structure 2: Short + Long Press --------
# Format:
# (PCA_addr, pin, short_pcf, short_mask, long_pcf, long_mask)
#
# PCA_addr   -> which PCA input chip
# pin        -> input pin
# short_pcf  -> PCF device used for short press
# short_mask -> relay bit toggled on short press
# long_pcf   -> PCF device used for long press
# long_mask  -> relay bit toggled on long press

debounce_buttons = [
    (PCA2_ADDR, 6, pcf1, 1 << 8,  pcf1, 1 << 4),  # janis bra siena
    (PCA1_ADDR, 13, pcf1, 1 << 12, pcf1, 1 << 0), # ieva bra siena
    (PCA2_ADDR, 15, pcf1, 1 << 14, pcf1, 1 << 2), # gulamiistaba bra gulta
    (PCA1_ADDR, 4, pcf1, 1 << 5,  pcf1, 1 << 7),  # viesistaba ar koridoru long
    (PCA2_ADDR, 2, pcf2, 1 << 10, pcf2, 1 << 9),  # terase labais ara
    (PCA1_ADDR, 15, pcf1, 1 << 5, pcf1, 1 << 7),  # terase kreisais
    (PCA2_ADDR, 4, pcf1, 1 << 9,  pcf1, 1 << 7),  # vejveriis koridors
    (PCA2_ADDR, 1, pcf1, 1 << 15, pcf1, 1 << 1),  # darbistaba durvis
    (PCA2_ADDR, 5, pcf1, 1 << 15, pcf1, 1 << 1),  # drabistaba siena
]


# ================= STATE =================

last_simple = [1] * len(simple_buttons)

press_time = [0] * len(debounce_buttons)
long_done = [False] * len(debounce_buttons)


# ================= MAIN LOOP =================

try:

    while True:

        now = time.time()

        pca = {
            PCA1_ADDR: pca_read(PCA1_ADDR),
            PCA2_ADDR: pca_read(PCA2_ADDR)
        }

        # SIMPLE TOGGLE

        for i, (addr, pin, pcf, mask) in enumerate(simple_buttons):

            val = pca[addr][pin]

            if last_simple[i] == 0 and val == 1:
                toggle(pcf, mask)

            last_simple[i] = val


        # SHORT + LONG PRESS

        for i, (addr, pin, spcf, smask, lpcf, lmask) in enumerate(debounce_buttons):

            val = pca[addr][pin]

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
