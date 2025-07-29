import RPi.GPIO as GPIO
import time

# --- GPIO Pin Definitions (BCM numbering) ---
# BCM GPIO 4 is Physical Pin 7
BUZZER_PIN = 4
# BCM GPIO 18 is Physical Pin 12
LED_PIN = 18 # Changed from 22 to 18 as requested

def setup_gpio():
    """
    Sets up the GPIO mode to BCM and configures the buzzer and LED pins as outputs.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.setup(LED_PIN, GPIO.OUT)
    print("GPIO setup complete.")

def play_tone(frequency, duration):
    """
    Plays a tone on the passive buzzer using PWM.

    Args:
        frequency (int): The frequency of the tone in Hz.
        duration (float): The duration of the tone in seconds.
    """
    print(f"Playing tone at {frequency}Hz for {duration}s...")
    buzzer_pwm = None
    try:
        # Create a PWM object for the buzzer pin with the given frequency
        buzzer_pwm = GPIO.PWM(BUZZER_PIN, frequency)
        buzzer_pwm.start(50)  # Start PWM with 50% duty cycle
        time.sleep(duration)  # Keep the tone playing for the specified duration
    except Exception as e:
        print(f"Error in play_tone: {e}")
    finally:
        if buzzer_pwm:
            buzzer_pwm.stop()  # Stop the PWM
        print("Tone finished.")

def led_on():
    """
    Turns the LED connected to LED_PIN ON (sets the pin to HIGH).
    """
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("LED ON")

def led_off():
    """
    Turns the LED connected to LED_PIN OFF (sets the pin to LOW).
    """
    GPIO.output(LED_PIN, GPIO.LOW)
    print("LED OFF")

def cleanup_gpio():
    """
    Cleans up all GPIO configurations, releasing the pins.
    This should always be called when the script exits.
    """
    GPIO.cleanup()
    print("GPIO cleaned up.")

if __name__ == '__main__':
    try:
        setup_gpio()
        print("\nStarting Buzzer and LED Test. Press Ctrl+C to exit.")

        while True:
            # --- Buzzer sequence ---
            print("\nPlaying buzzer tone (GPIO 4)...")
            play_tone(440, 0.5) # Play A4 note for 0.5 seconds
            time.sleep(0.2)    # Short pause
            play_tone(523, 0.5) # Play C5 note for 0.5 seconds
            time.sleep(0.5)    # Pause before repeating

            # --- LED sequence ---
            print("Illuminating LED (GPIO 18)...") # Updated print statement
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
