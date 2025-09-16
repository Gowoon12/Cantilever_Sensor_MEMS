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

## Programs

  ### single_port_gw.ino 
  + This code is for connecting one cantilever sensor per ESP32 board. 
  + Each ESP32 must use a different UDP port number (e.g., 7001, 7002, 7003 …)
  + Connect the cantilever sensor output to the ESP32 ADC pins:
  ++ GPIO 33 → V+ (positive output)
  ++GPIO 35 → V- (negative output)

  ### multi_port_gw.ino
  + This code is for connecting multi(~6) cantilever sensor per ESP32 board. 
  + Each ESP32 must use a different UDP port number (e.g., 7001, 7002, 7003 …)
  + Connect the cantilever sensor output to the ESP32 ADC pins:
  + GPIO 32, 33, 34, 35, 36, 39
  + UDP Setup

    #define DEST_IP   "192.168.2.2"   // PC IPv4
    #define DEST_PORT 7006            // Use sequential ports: 7001, 7002, 7003, ...
    const char *SSID = "meric_drone";
    const char *PWD  = "***********";

  ### Multi_sensing
  + This code is for the data reciever and GUI in the PC.
  + This code is upgrade version of the [realtime-plot](https://github.com/Gowoon12/Microsystem_team4/tree/main/Dynamixel_Linux-main/realtimeplot_forGW) from Yang Yang. 
  ++ adc_plot_widget_multi.py
  ++ main_multi.py
  ++ main.ui
  ++ sensorplot.ui
  ++ sensorsetup.ui



