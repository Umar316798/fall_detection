import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from datetime import datetime
import os

# --- GPIO Pin Definitions (BCM numbering) ---
# BCM GPIO 4 is Physical Pin 7 (for Buzzer)
BUZZER_PIN = 4
# BCM GPIO 18 is Physical Pin 12 (for LED)
LED_PIN = 18

# --- Fall Detection Parameters ---
# Minimum area of the bounding box to consider it a person (adjust as needed)
MIN_AREA = 5000
# Number of consecutive frames a fall must be detected to trigger an alert
CONSECUTIVE_FALL_FRAMES = 10
# Cooldown period in seconds after a fall detection before re-arming
COOLDOWN_PERIOD = 30

# --- Global Variables ---
fall_frames_count = 0
fall_detected_time = None
is_cooldown_active = False

# --- GPIO Setup Functions ---
def setup_gpio():
    """
    Sets up the GPIO mode to BCM and configures the buzzer and LED pins as outputs.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.setup(LED_PIN, GPIO.OUT)
    print("GPIO setup complete for Buzzer and LED.")

def play_tone(frequency, duration):
    """
    Plays a tone on the passive buzzer using PWM.
    """
    buzzer_pwm = None
    try:
        buzzer_pwm = GPIO.PWM(BUZZER_PIN, frequency)
        buzzer_pwm.start(50)  # 50% duty cycle
        time.sleep(duration)
    except Exception as e:
        print(f"Error playing tone: {e}")
    finally:
        if buzzer_pwm:
            buzzer_pwm.stop()

def led_on():
    """
    Turns the LED connected to LED_PIN ON.
    """
    GPIO.output(LED_PIN, GPIO.HIGH)

def led_off():
    """
    Turns the LED connected to LED_PIN OFF.
    """
    GPIO.output(LED_PIN, GPIO.LOW)

def cleanup_gpio():
    """
    Cleans up all GPIO configurations, releasing the pins.
    """
    GPIO.cleanup()
    print("GPIO cleaned up.")

# --- Main Fall Detection Logic ---
def run_fall_detection():
    global fall_frames_count, fall_detected_time, is_cooldown_active

    # Initialize camera (0 for default webcam, or specify path for PiCam if needed)
    # For Raspberry Pi Camera, you might need to use picamera or a specific GStreamer pipeline
    # Example for PiCamera (requires picamera library and different capture method):
    # from picamera.array import PiRGBArray
    # from picamera import PiCamera
    # camera = PiCamera()
    # camera.resolution = (640, 480)
    # camera.framerate = 32
    # rawCapture = PiRGBArray(camera, size=(640, 480))
    # time.sleep(0.1) # Allow camera to warm up

    # For USB Webcam or default camera:
    cap = cv2.VideoCapture(0) # 0 for default camera

    if not cap.isOpened():
        print("Error: Could not open video stream or file.")
        cleanup_gpio()
        return

    # Background subtractor for motion detection
    fgbg = cv2.createBackgroundSubtractorMOG2()

    # Create a directory for fall images if it doesn't exist
    FALL_IMAGES_DIR = "fall_images"
    os.makedirs(FALL_IMAGES_DIR, exist_ok=True)
    print(f"Fall images will be saved in: {os.path.abspath(FALL_IMAGES_DIR)}")

    # Set up GPIO
    setup_gpio()

    try:
        # Loop to continuously read frames from the camera
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame, exiting...")
                break

            # Flip the frame horizontally for mirror effect (optional, depends on camera orientation)
            frame = cv2.flip(frame, 1)

            # Convert to grayscale for background subtraction
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fgmask = fgbg.apply(gray)

            # Apply morphological operations to clean up the mask
            fgmask = cv2.erode(fgmask, None, iterations=2)
            fgmask = cv2.dilate(fgmask, None, iterations=2)

            # Find contours in the foreground mask
            contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            person_detected = False
            for contour in contours:
                if cv2.contourArea(contour) < MIN_AREA:
                    continue

                (x, y, w, h) = cv2.boundingRect(contour)

                # Check for aspect ratio (height > width for standing person)
                # If width is significantly greater than height, it might indicate a fall
                aspect_ratio = float(w) / h
                if aspect_ratio > 1.8:  # Threshold for fall detection (adjust as needed)
                    # This is a potential fall
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) # Red bounding box
                    person_detected = True
                    break # Only need to find one fall to trigger

            # --- Fall Detection Logic with Cooldown ---
            if person_detected:
                if not is_cooldown_active:
                    fall_frames_count += 1
                    if fall_frames_count >= CONSECUTIVE_FALL_FRAMES:
                        # Fall confirmed! Trigger alarm, capture image, start cooldown
                        print("!!! FALL DETECTED !!!")
                        fall_detected_time = time.time()
                        is_cooldown_active = True
                        fall_frames_count = 0 # Reset count after detection

                        # Activate Buzzer and LED
                        play_tone(1000, 1) # High pitch for 1 second
                        led_on()
                        # LED will stay on during cooldown or until explicitly turned off

                        # Capture Image
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        image_filename = os.path.join(FALL_IMAGES_DIR, f"fall_{timestamp}.jpg")
                        cv2.imwrite(image_filename, frame)
                        print(f"Fall image saved: {image_filename}")
                else:
                    # Cooldown is active, do not increment fall_frames_count
                    pass
            else:
                # No person detected or aspect ratio not indicating fall, reset count
                fall_frames_count = 0
                if not is_cooldown_active:
                    # Turn off LED if not in cooldown and no fall is detected
                    led_off()

            # --- Cooldown Management ---
            if is_cooldown_active:
                elapsed_time = time.time() - fall_detected_time
                if elapsed_time < COOLDOWN_PERIOD:
                    # Display cooldown message
                    cooldown_text = f"COOLDOWN: {int(COOLDOWN_PERIOD - elapsed_time)}s"
                    text_color = (0, 255, 255) # Yellow
                    # Keep LED on during cooldown
                    led_on()
                else:
                    # Cooldown finished
                    is_cooldown_active = False
                    fall_detected_time = None
                    cooldown_text = "SYSTEM READY"
                    text_color = (0, 255, 0) # Green
                    led_off() # Turn off LED after cooldown
                    print("Cooldown ended. System ready.")
            else:
                cooldown_text = "Monitoring..."
                text_color = (255, 255, 0) # Cyan

            # --- Display Status and Fall Detected Message ---
            # Determine text to display
            display_text = ""
            if is_cooldown_active:
                display_text = "!!! FALL DETECTED !!!"
                text_color = (0, 0, 255) # Red for Fall Detected
            else:
                display_text = cooldown_text # Monitoring or System Ready

            # Calculate text size and position for centering
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2
            text_size = cv2.getTextSize(display_text, font, font_scale, font_thickness)[0]
            text_x = (frame.shape[1] - text_size[0]) // 2
            text_y = (frame.shape[0] + text_size[1]) // 2 # Centered vertically

            cv2.putText(frame, display_text, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

            # Display the frame
            cv2.imshow('Fall Detection System', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred during video processing: {e}")
    finally:
        # Release resources
        cap.release()
        cv2.destroyAllWindows()
        cleanup_gpio() # Ensure GPIO pins are cleaned up

if __name__ == '__main__':
    run_fall_detection()
