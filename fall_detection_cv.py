import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import os
import atexit # Import atexit to ensure cleanup on exit
from picamera2 import Picamera2

# --- GPIO Pin Definitions (BCM numbering) ---
# BCM GPIO 17 is Physical Pin 11 (for Buzzer)
BUZZER_PIN = 17
# BCM GPIO 18 is Physical Pin 12 (for LED)
LED_PIN = 18

GPIO.setmode(GPIO.BCM) # Use BCM numbering mode
GPIO.setwarnings(False) # Disable warnings

# --- Fall Detection Parameters ---
# Lowered MIN_AREA to be more sensitive to smaller detections
MIN_AREA = 3000
CONSECUTIVE_FALL_FRAMES = 10
COOLDOWN_PERIOD = 30
HAAR_CASCADE_PATH = 'haarcascade_fullbody.xml'

# --- Global Variables ---
fall_frames_count = 0
is_cooldown_active = False
fall_detected_time = None
cooldown_text = ""
display_text = "SYSTEM READY"
text_color = (0, 255, 0) # Green for SYSTEM READY

# --- GPIO Setup Functions ---
def cleanup_gpio():
    """
    Cleans up all GPIO configurations, releasing the pins.
    This function is registered with atexit to ensure it always runs.
    """
    print("\nCleaning up GPIO pins...")
    GPIO.output(BUZZER_PIN, GPIO.LOW) # Ensure buzzer is off
    GPIO.output(LED_PIN, GPIO.LOW) # Ensure LED is off
    GPIO.cleanup()
    print("GPIO cleaned up.")

def setup_gpio():
    """
    Sets up the GPIO mode to BCM and configures the buzzer and LED pins as outputs.
    Registers cleanup_gpio to run on program exit.
    """
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.LOW)
    atexit.register(cleanup_gpio) # Register the cleanup function
    print("GPIO setup complete for Buzzer (GPIO {}) and LED (GPIO {})".format(BUZZER_PIN, LED_PIN))

def play_tone(frequency, duration, pwm_object):
    """
    Plays a tone on the passive buzzer using PWM.
    """
    try:
        pwm_object.ChangeFrequency(frequency)
        pwm_object.start(50)
        time.sleep(duration)
    except Exception as e:
        print(f"Error playing tone: {e}")
    finally:
        pwm_object.stop()

def buzzer_alarm(buzzer_pwm):
    """
    Plays a simple alarm sound on the passive buzzer and turns on the LED.
    """
    print("Buzzer alarm activated!")
    GPIO.output(LED_PIN, GPIO.HIGH)

    play_tone(1000, 0.2, buzzer_pwm)
    time.sleep(0.1)
    play_tone(1200, 0.2, buzzer_pwm)
    time.sleep(0.1)
    play_tone(1000, 0.2, buzzer_pwm)
    time.sleep(0.1)
    play_tone(1200, 0.2, buzzer_pwm)

def led_on():
    """
    Turns the LED connected to LED_PIN ON.
    """
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("LED ON")

def led_off():
    """
    Turns the LED connected to LED_PIN OFF.
    """
    GPIO.output(LED_PIN, GPIO.LOW)
    print("LED OFF")

# --- Main Fall Detection Logic ---
def main():
    global fall_frames_count, is_cooldown_active, fall_detected_time, cooldown_text, display_text, text_color

    picam2 = Picamera2()
    camera_config = picam2.create_video_configuration({"size": (640, 480), "format": "XRGB8888"})
    picam2.configure(camera_config)

    fullbody_cascade = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
    if fullbody_cascade.empty():
        print(f"Error: Could not load cascade classifier from {HAAR_CASCADE_PATH}.")
        return

    try:
        picam2.start()
        print("Camera started successfully.")
        time.sleep(2)

        fall_images_dir = "fall_images"
        if not os.path.exists(fall_images_dir):
            os.makedirs(fall_images_dir)
            print(f"Created directory: {fall_images_dir}")

        buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1)

        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Adjusted detectMultiScale parameters for better detection.
            # scaleFactor is lowered to 1.05 to check more image scales.
            # minNeighbors is increased to 5 to require more neighboring rectangles,
            # which can reduce false positives.
            bodies = fullbody_cascade.detectMultiScale(gray, 1.05, 5)

            person_detected = False
            for (x, y, w, h) in bodies:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
               
                # Check for a body with a valid area
                if cv2.contourArea(np.array([(x,y),(x+w,y),(x+w,y+h),(x,y+h)])) < MIN_AREA:
                    continue
               
                aspect_ratio = float(w) / h
                if aspect_ratio > 1.8:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    person_detected = True
                    print("A potential fall has been detected.")
                    break

            if not is_cooldown_active:
                if person_detected:
                    fall_frames_count += 1
                    if fall_frames_count >= CONSECUTIVE_FALL_FRAMES:
                        print("!!! FALL DETECTED !!!")
                        fall_detected_time = time.time()
                        is_cooldown_active = True
                        buzzer_alarm(buzzer_pwm)
                        fall_frames_count = 0
                       
                        timestamp = time.strftime("%Y%m%d_%H%M%S")
                        image_filename = os.path.join(fall_images_dir, f"FALL_{timestamp}.jpg")
                        cv2.imwrite(image_filename, frame)
                        print(f"Fall image saved: {image_filename}")
                else:
                    fall_frames_count = 0
                   
            if is_cooldown_active:
                elapsed_time = time.time() - fall_detected_time
                if elapsed_time < COOLDOWN_PERIOD:
                    cooldown_text = f"COOLDOWN: {int(COOLDOWN_PERIOD - elapsed_time)}s"
                    display_text = cooldown_text
                    text_color = (0, 255, 255)
                    led_on()
                else:
                    is_cooldown_active = False
                    fall_detected_time = None
                    display_text = "SYSTEM READY"
                    text_color = (0, 255, 0)
                    cooldown_text = ""
                    print("Cooldown ended, System Ready.")
                    led_off()
            else:
                display_text = "MONITORING..."
                text_color = (255, 255, 0)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2

            text_size = cv2.getTextSize(display_text, font, font_scale, font_thickness)[0]
            text_x = (frame.shape[1] - text_size[0]) // 2
            text_y = (frame.shape[0] + text_size[1]) // 2

            cv2.putText(frame, display_text, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

            cv2.imshow("Fall Detection System", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
           
            # Add a small sleep to prevent the CPU from running at 100%
            time.sleep(0.01)

    except Exception as e:
        print(f"An error occurred during video processing: {e}")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    setup_gpio()
    main()
