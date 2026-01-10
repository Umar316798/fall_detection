# Fall Detection and Alert System

This is a self-contained, low-cost fall detection system built on a Raspberry Pi. It uses computer vision to autonomously monitor for falls and triggers multi-modal alerts, providing an effective solution for at-home safety monitoring.

# 💡 Project Motivation

This project was developed to address the significant risk of falls faced by individuals with neurological disorders such as Parkinson's, Alzheimer's, and multiple sclerosis. A fall can have severe consequences for these individuals. The goal was to create a non-invasive, reliable system that can provide a timely alert to caregivers without the need for wearable devices, offering peace of mind and enhancing patient safety.

# ✨ Features

Real-Time Monitoring: Continuously monitors a live camera feed for falls.

Computer Vision-Based Detection: Utilizes a robust aspect ratio algorithm to differentiate between normal movements and a fall.

Multi-modal Alerts: Triggers a loud buzzer and a flashing LED to provide immediate auditory and visual alerts.

Automated Image Capture: Saves an image of the event to a local directory for incident logging and review.

Cooldown Mechanism: Prevents repeated false alarms by entering a cooldown period after a fall is detected.

# 🛠️ Technologies Used

##Hardware:

Raspberry Pi (any model, e.g., Pi 4 or Pi 5)

Raspberry Pi Camera Module (CSI or USB)

Passive Buzzer

LED

Breadboard and jumper wires

Software:

Python 3: The primary programming language.

OpenCV: An open-source library for computer vision tasks.

RPi.GPIO: A library for controlling the GPIO pins on the Raspberry Pi.

Picamera2: The official library for interfacing with the Raspberry Pi camera module.

# 🚀 Getting Started
To set up the system, you will need to wire the components and install the necessary software libraries.

Hardware Setup: Follow the wiring diagrams to connect the buzzer and LED to the appropriate GPIO pins on your Raspberry Pi.

Software Installation: Install the required Python libraries using pip.

pip install opencv-python RPi.GPIO picamera2

Run the Script: Navigate to the project directory and run the main Python script.

python3 fall_detection_cv.py

Note: Ensure your Raspberry Pi's camera is enabled in the configuration settings (sudo raspi-config or sudo rpi-config).
