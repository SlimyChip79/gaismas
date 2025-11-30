import RPi.GPIO as GPIO
import time

# --- GPIO setup ---
BUTTON_PIN = 5      # Button GPIO
LED1_PIN = 17       # LED1 GPIO
LED2_PIN = 27       # LED2 GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(LED1_PIN, GPIO.OUT)
GPIO.setup(LED2_PIN, GPIO.OUT)

def led_control(duration):
    """Light LEDs based on button press duration"""
    if duration < 2:
        GPIO.output(LED1_PIN, GPIO.HIGH)
        GPIO.output(LED2_PIN, GPIO.LOW)
    else:
        GPIO.output(LED1_PIN, GPIO.LOW)
        GPIO.output(LED2_PIN, GPIO.HIGH)

try:
    button_pressed_time = None
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            if button_pressed_time is None:
                button_pressed_time = time.time()
        else:
            if button_pressed_time is not None:
                press_duration = time.time() - button_pressed_time
                led_control(press_duration)
                button_pressed_time = None
        time.sleep(0.05)  # small delay to debounce
except KeyboardInterrupt:
    GPIO.cleanup()
