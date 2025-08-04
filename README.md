# MediBox--Embedded-System-for-Smart-Medicine-Intake-
This project is a Medibox, a smart medication storage system, made by integrating real-time environmental monitoring (light and temperature) and dynamic control of a shaded sliding window via a servo motor. Key features include:

Light Intensity Monitoring: Uses an LDR to measure ambient light, with configurable sampling/sending intervals.

Temperature Sensing: DHT11 sensor tracks storage conditions.

Dynamic Shading Control: Servo motor adjusts window angle based on light/temperature using a derived equation.

Node-RED Dashboard: Visualizes sensor data, plots historical trends, and allows parameter adjustments (sampling intervals, motor angle limits, etc.).

MQTT (Mosquitto broker) facilitates lightweight publish-subscribe messaging between the ESP32 and Node-RED, ensuring low-latency updates for sensor data and parameter adjustments
