import RPi.GPIO as GPIO
import time

# Pin Definition
# Use the GPIO BCM number for the LED
LED_PIN = 22 # Connect your LED to GPIO 22 (Physical Pin 15)

# Function to setup the GPIO pin
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)
    print(f"LED pin {LED_PIN} set up.")

# Function to turn LED on
def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("LED ON")

# Function to turn LED off
def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)
    print("LED OFF")

# Function to clean up GPIO settings
def cleanup():
    GPIO.cleanup()
    print("GPIO cleaned up.")

# Main program block
if __name__ == '__main__':
    try:
        setup()
        print("LED test started. Press Ctrl+C to exit.")

        while True:
            led_on()
            time.sleep(1) # LED on for 1 second
            led_off()
            time.sleep(1) # LED off for 1 second

    except KeyboardInterrupt:
        print("\nLED test stopped by user.")
    finally:
        cleanup()
