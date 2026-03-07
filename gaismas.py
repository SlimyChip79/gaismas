#!/usr/bin/env python3
import time
from smbus2 import SMBus
import RPi.GPIO as GPIO

# ================= CONFIG =================
I2C_BUS = 1

PCA1_ADDR = 0x20
PCA2_ADDR = 0x22

PCF1_ADDR = 0x27
PCF2_ADDR = 0x26

INT1_PIN = 17
INT2_PIN = 27

REG_INPUT_0 = 0x00
REG_INPUT_1 = 0x01

LOOP_DELAY = 0.05
debounce_delay = 100
long_press_threshold = 350

# ================= INIT =================
print("[GAISMAS] Initializing...")

bus = SMBus(I2C_BUS)

# All relays OFF (active LOW)
pcf1_state = 0xFFFF
pcf2_state = 0xFFFF

# Write 16-bit value (PCF8575 needs word write)
def write_pcf(addr, value):
    bus.write_word_data(addr, 0x00, value & 0xFFFF)

write_pcf(PCF1_ADDR, pcf1_state)
write_pcf(PCF2_ADDR, pcf2_state)

# GPIO for INT
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(INT2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ================= BUTTON STRUCTURES =================
# (UNCHANGED FROM YOUR WORKING VERSION)

simple_buttons = [
    (PCA1_ADDR, 0, PCF1_ADDR, 1 << 4),
    (PCA1_ADDR, 8, PCF1_ADDR, 1 << 0),
    (PCA1_ADDR, 3, PCF1_ADDR, 1 << 6),
    (PCA1_ADDR, 11, PCF1_ADDR, 1 << 7),
    (PCA1_ADDR, 2, PCF2_ADDR, 1 << 7),
]

debounce_buttons = [
    (PCA2_ADDR, 6, PCF1_ADDR, 1 << 8,  PCF1_ADDR, 1 << 4),
    (PCA1_ADDR, 13, PCF1_ADDR, 1 << 12, PCF1_ADDR, 1 << 0),
]

# ================= STATE TRACKERS =================
last_simple_state = [1] * len(simple_buttons)

last_debounce_state = [1] * len(debounce_buttons)
last_debounce_time = [0] * len(debounce_buttons)
press_start_time = [0] * len(debounce_buttons)
long_press_triggered = [False] * len(debounce_buttons)

last_inputs = {
    PCA1_ADDR: [1]*16,
    PCA2_ADDR: [1]*16
}

# ================= HELPERS =================
def read_pca_inputs(addr):
    try:
        p0 = bus.read_byte_data(addr, REG_INPUT_0)
        p1 = bus.read_byte_data(addr, REG_INPUT_1)
        return [(p0 >> i) & 1 for i in range(8)] + [(p1 >> i) & 1 for i in range(8)]
    except:
        return [1]*16

# ================= MAIN LOOP =================
try:
    while True:
        current_time = int(time.time() * 1000)

        # INT refresh
        if GPIO.input(INT1_PIN) == 0:
            last_inputs[PCA1_ADDR] = read_pca_inputs(PCA1_ADDR)

        if GPIO.input(INT2_PIN) == 0:
            last_inputs[PCA2_ADDR] = read_pca_inputs(PCA2_ADDR)

        # ================= SIMPLE =================
        for idx, (addr, pin, pcf_addr, mask) in enumerate(simple_buttons):

            val = last_inputs[addr][pin]

            if last_simple_state[idx] == 0 and val == 1:

                if pcf_addr == PCF1_ADDR:
                    pcf1_state ^= mask
                    write_pcf(PCF1_ADDR, pcf1_state)
                else:
                    pcf2_state ^= mask
                    write_pcf(PCF2_ADDR, pcf2_state)

            last_simple_state[idx] = val

        # ================= DEBOUNCE =================
        for idx, (addr, pin, short_addr, short_mask, long_addr, long_mask) in enumerate(debounce_buttons):

            val = last_inputs[addr][pin]

            if val != last_debounce_state[idx]:
                last_debounce_time[idx] = current_time

            if (current_time - last_debounce_time[idx]) > debounce_delay:

                if val == 0:

                    if press_start_time[idx] == 0:
                        press_start_time[idx] = current_time
                        long_press_triggered[idx] = False

                    elif (not long_press_triggered[idx] and
                          current_time - press_start_time[idx] >= long_press_threshold):

                        if long_addr == PCF1_ADDR:
                            pcf1_state ^= long_mask
                            write_pcf(PCF1_ADDR, pcf1_state)
                        else:
                            pcf2_state ^= long_mask
                            write_pcf(PCF2_ADDR, pcf2_state)

                        long_press_triggered[idx] = True

                else:
                    if press_start_time[idx] != 0 and not long_press_triggered[idx]:

                        if short_addr == PCF1_ADDR:
                            pcf1_state ^= short_mask
                            write_pcf(PCF1_ADDR, pcf1_state)
                        else:
                            pcf2_state ^= short_mask
                            write_pcf(PCF2_ADDR, pcf2_state)

                    press_start_time[idx] = 0
                    long_press_triggered[idx] = False

            last_debounce_state[idx] = val

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping, turning all relays OFF")
    write_pcf(PCF1_ADDR, 0xFFFF)
    write_pcf(PCF2_ADDR, 0xFFFF)
    GPIO.cleanup()
    bus.close()
