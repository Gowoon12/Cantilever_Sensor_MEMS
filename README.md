# Cantilever Sensor Monitoring

This project is a collaborative research effort between the MeRIC-Lab and MNTL-Lab of the Department of Mechanical Engineering at Chonnam National University.

We are developing a wireless long-term monitoring system for cantilever sensors that measure contractile forces during cardiomyocyte maturation.

## Environment

You can find more information in the link below.

 + Tested on Python 3.8
   
 + Platforms: Ubuntu 20.04 LTS and Windows 10

## System Overview

The system is designed for the wireless long-term monitoring of cantilever sensors that measure the contractile forces of cardiomyocytes. Its architecture consists of three main components:
### Sensing Module
+ Amplified signals from cantilever sensors are acquired through the ADC1 pins of the ESP32 board (pins 32, 33, 34, 35, 36, 39).
+ Each ESP32 currently supports up to three sensors, with scalability to six sensors per board.
### Communication Module
+ Data transmission is carried out using UDP communication via a wifi router.
+ Each ESP32 is assigned a unique IP address and port number (e.g., 7001, 7002, 7003, …).
+ The transmitted data packets are formatted as: {time, Sensor1, Sensor2, Sensor3}

### Visualization & Monitoring Module
+ A custom GUI visualizes the sensor signals in real time, enabling long-term monitoring and analysis.

