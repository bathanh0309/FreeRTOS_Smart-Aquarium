# Introduction
The "Smart Aquarium System" is an aquarium automation solution, using FreeRTOS to coordinate tasks on the microcontroller.

The system integrates sensors and devices to monitor and control the aquarium environment, with the following main features:
## How the System Works
DS18B20 temperature sensor: Measures water temperature, controls the heating lamp according to the threshold:
- Temperature < 30°C: PWM heating lamp = 255 (strong).
- 30°C ≤ Temperature < 32°C: PWM = 100 (medium).
- Temperature ≥ 32°C: PWM = 0 (off).

Turbidity sensor: Measures water clarity, controls the filter motor:
- Turbidity > 3000: PWM = 0 (off).
- 2000 < Turbidity ≤ 3000: PWM = 100 (medium).
- Turbidity ≤ 2000: PWM = 255 (strong).

RTC DS3231: Automatically activates the feeding motor (PWM = 25) for 5 seconds at 11:00 every day, not repeating on the same day.

Operation mode:
- Automatic (AUTO): Controls filtration, heating, and feeding based on sensor data and time.
- Manual (MANUAL): User controls directly via push button, switches device status (OFF → WEAK → STRONG → OFF).
## FreeRTOS

- SensorTask: Reads sensor data every 500ms, sends via queue.
- ControlTask: Processes control logic, receives commands from push button via interrupt and queue.
- DisplayTask: Display information (time, temperature, turbidity, status) on OLED every 200ms.
- SerialPrintTask: Print information via Serial for monitoring every 1 second.

## RainMaker App

Users can switch AUTO/MANUAL mode, turn on/off the filter motor, heating lamp, or activate feeding via the RainMaker app on iOS/Android.

## Quick View of the Project

**Design Circuit Schematic on Altium software**
![image](Media/Schematic.jpg)

**Design 2D Circuit Schematic on Altium software**
![image](Media/2D.jpg)

**System Model**
![image](Media/System.jpg)


https://github.com/user-attachments/assets/afabf5fa-f88d-4a85-839b-8f9cc426c269


https://github.com/user-attachments/assets/8d7514b6-308d-45bb-afe0-62e412b6e779

**use-case-diagram**
![image](usecasediagram.jpg)





