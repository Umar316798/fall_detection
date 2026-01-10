import RPi.GPIO as GPIO
import time

# Define the GPIO pin connected to the buzzer (using BCM numbering)
# Physical Pin 11 is BCM GPIO 17
BUZZER_PIN = 17

def setup():
    # Set the GPIO modes to BCM numbering
    GPIO.setmode(GPIO.BCM)
    # Set the buzzer pin as an output
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    # Ensure buzzer is off initially
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def beep(duration_ms):
    # Turn the buzzer on (HIGH for active buzzer)
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    # Convert milliseconds to seconds for time.sleep()
    time.sleep(duration_ms / 1000.0)
    # Turn the buzzer off
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def destroy():
    # Ensure buzzer is off before cleaning up
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    # Clean up all GPIO settings (important to release pins)
    GPIO.cleanup()

if __name__ == '__main__':
    print("Buzzer test started. Press Ctrl+C to exit.")
    try:
        setup()
        # Loop indefinitely, beeping every second
        while True:
            beep(200) # Beep for 200 milliseconds
            time.sleep(1) # Wait for 1 second before the next beep
    except KeyboardInterrupt:
        print("Buzzer test stopped.")
    finally:
        # This block always executes, ensuring cleanup
        destroy()
