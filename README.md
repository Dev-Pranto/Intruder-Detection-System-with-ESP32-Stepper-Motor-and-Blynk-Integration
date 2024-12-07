# Project Summary: Smart Intruder Detection and Surveillance System
This project is a smart IoT-based security system designed to detect and respond to intrusions in defined zones. It integrates motion detection, sound monitoring, and real-time notifications to enhance security. The system is built using an ESP32 microcontroller and features advanced functionalities such as stepper motor-driven camera tracking and a relay-controlled alarm mechanism.

Key Features:
Intruder Detection:

Utilizes two IR sensors to monitor motion in distinct zones (Zone 1 and Zone 2).
Triggers specific actions based on the zone of intrusion.
Sound Monitoring:

A KY-03 sound sensor detects unusual noise levels, triggering an alarm and system response.
Camera Control:

A stepper motor rotates to a fixed position based on the zone of motion, simulating a CCTV camera tracking system.
Ensures the camera points to the relevant zone, even with multiple intrusions.
Real-Time Alerts with Blynk:

Sends live updates to a mobile app, including temperature, humidity, sound levels, and zone status.
Allows remote control of the system and its components.
Integrated Relay Control:

A relay module can activate additional devices like floodlights or alarms remotely via the app.
User-Friendly Interface:

A 16x2 LCD provides live feedback on the system status, intrusions, and sensor readings.
Automation & Safety:

A buzzer alerts the user in case of intrusions or loud noise.
Operates in an "Armed" or "OFF" state controlled remotely.
Technologies Used:
Microcontroller: ESP32 Devkit V1
Sensors: IR sensors, KY-03 sound sensor, DHT11 temperature and humidity sensor
Actuators: 28BYJ-48 stepper motor, buzzer, relay module
Display: 16x2 LCD
Software: Arduino IDE, Blynk IoT platform, Wi-Fi for app connectivity
Applications:
Home and office security systems
Perimeter monitoring and surveillance
Automated zone-based camera control
IoT-enabled safety and automation solutions
