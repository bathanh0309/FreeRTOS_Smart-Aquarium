# FreeRTOS

## Introduction
FreeRTOS is an open source, lightweight real-time operating system designed to manage tasks on microcontrollers and embedded systems. The project takes advantage of FreeRTOS features such as multitasking, queuing, and interrupts to manage tasks such as:
- Sensor reading: The SensorTask task continuously collects water temperature and clarity data from the DS18B20 sensor and clarity sensor, sending the data via queue for processing.
- Device Control: The ControlTask ​​handles automatic or manual control logic for the filter motor, heater, and feeder motor, based on sensor data and pushbutton commands.
- Display and Monitoring: The DisplayTask updates the OLED interface with time, device status, and sensor data, while the SerialPrintTask prints information via Serial for monitoring.
## How the System Works
### System Overview:
- AUTO (automatically adjusts filtration and heating based on sensor thresholds and automatically feeds fish at set times)
- MANUAL (user controls directly via push button)
