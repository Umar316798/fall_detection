import cv2
import time
from picamera2 import Picamera2 # Use if you have the official Pi camera module

print("Starting camera test...")

try:
    # Initialize Picamera2 for the official Raspberry Pi Camera Module (CSI camera)
    picam2 = Picamera2()

    # Explicitly configure for a standard video stream format
    # This format (BGR888) is directly compatible with OpenCV's expectations
    camera_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(camera_config)

    picam2.start() # Start Picamera2 (no show_preview here, we'll use imshow)
    print("Picamera2 started.")
    time.sleep(2) # Give camera time to warm up

    while True:
        # Capture frame from Picamera2
        frame = picam2.capture_array()

        # Save a test image to confirm capture (this will save one image per loop)
        # You can comment this line out after confirming it works, or move it outside the loop
        # to save just one image at the start if needed.
        cv2.imwrite("test_capture.jpg", frame)

        # *** DIAGNOSTIC: Print frame shape and data type ***
        print(f"Frame shape: {frame.shape}, Dtype: {frame.dtype}")

        # Display the frame in a window
        cv2.imshow("Camera Test", frame)

        # Wait for 1ms and check for 'q' key press to exit
        # This is crucial for OpenCV windows to update and remain responsive
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Cleanup resources
    print("Cleaning up camera and windows.")
    if 'picam2' in locals() and picam2.started: # Ensure picam2 was initialized and started
        picam2.stop()
    cv2.destroyAllWindows() # Close all OpenCV windows
    print("Camera test finished.")
