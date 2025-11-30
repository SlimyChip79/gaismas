#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# ----- Pins -----
BUTTON_PIN = 4
LED1_PIN = 17
LED2_PIN = 27

# ----- Timing -----
debounce_delay = 0.03  # 30 ms
long_press_threshold = 0.4  # 400 ms

# ----- State -----
last_button_reading = GPIO.LOW
last_debounce_time = 0
button_press_start_time = 0
long_press_triggered = False
led1_state = GPIO.LOW
led2_state = GPIO.LOW

# ----- Setup -----
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)

# ----- Main Loop -----
try:
    while True:
        reading = GPIO.input(BUTTON_PIN)
        current_time = time.time()

        if reading != last_button_reading:
            last_debounce_time = current_time

        if (current_time - last_debounce_time) > debounce_delay:
            if reading == GPIO.HIGH:
                if button_press_start_time == 0:
                    button_press_start_time = current_time
                    long_press_triggered = False
                elif not long_press_triggered and (current_time - button_press_start_time >= long_press_threshold):
                    # Long press: toggle LED1
                    led1_state = not led1_state
                    GPIO.output(LED1_PIN, led1_state)
                    long_press_triggered = True
            else:
                if button_press_start_time != 0 and not long_press_triggered:
                    # Short press: toggle LED2
                    led2_state = not led2_state
                    GPIO.output(LED2_PIN, led2_state)
                button_press_start_time = 0
                long_press_triggered = False

        last_button_reading = reading
        time.sleep(0.01)

except KeyboardInterrupt:
    GPIO.cleanup()
