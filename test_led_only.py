import RPi.GPIO as GPIO
import time

# GPIO Pin Definition (BCM numbering)
LED_PIN = 22   # <--- THIS IS SET TO GPIO 22 TO MATCH YOUR PHYSICAL WIRING

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)
    print("GPIO setup complete for LED.")

def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("LED ON (physical output should be HIGH)")

def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)
    print("LED OFF (physical output should be LOW)")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleaned up.")

if __name__ == '__main__':
    try:
        setup_gpio()
        print("\n--- Starting Isolated LED ONLY Test on GPIO 22 ---")
        print("Ensure LED is connected to GPIO 22 via resistor, and NO buzzer is connected.")
        print("Press Ctrl+C to exit.")

        while True:
            print("\nAttempting to illuminate LED...")
            led_on()
            time.sleep(1) # LED on for 1 second
            led_off()
            time.sleep(1) # LED off for 1 second

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        cleanup_gpio()
