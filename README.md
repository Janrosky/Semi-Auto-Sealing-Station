# Semi-Auto-Sealing-Station

Prototype of a semi-automatic thermal sealing and process-monitoring station built with an Arduino Mega 2560 and PlatformIO.

The project models a small manufacturing cell in which a tray is detected, retained by a stopper, positioned under a simulated sealing head, validated for process temperature and head position, held for a configurable dwell time, inspected, and released only when all process conditions are valid.

## Main functions

- Tray presence detection using an optical sensor prototype.
- Automatic stopper actuation with a servo.
- Manual cycle start from the local HMI.
- Sealing-head position validation with an HC-SR04 ultrasonic sensor.
- Continuous dwell-time validation before release.
- Temperature interlock using a DS18B20 sensor.
- Latched process faults with operator reset and controlled recovery.
- RGB status indication and critical audible alarm.
- 16x2 I2C LCD and joystick-based HMI.
- Production counters, alarms, settings and diagnostics.
- Serial telemetry for future PC/SCADA/dashboard integration.

## Hardware

- Arduino Mega 2560
- HC-SR04 ultrasonic sensor
- DS18B20 temperature sensor
- Photoresistor / light sensor module
- Servo motor
- RGB LED module
- Active buzzer
- 16x2 I2C LCD
- Joystick module
- Potentiometer

## Process sequence

```text
WAITING TRAY
    ↓
TRAY DETECTED
    ↓
STOPPER HOLD
    ↓
OPERATOR START
    ↓
POSITION HEAD
    ↓
SEALING / DWELL
    ↓
FINAL INSPECTION
    ↓
RELEASE TRAY
    ↓
CONFIRM EXIT
    ↓
CYCLE COMPLETE
```

A process fault keeps the stopper in HOLD until the operator presses RESET. The controller then opens the stopper, requests removal of the tray, confirms its exit, and returns to the waiting state.

## Development environment

The firmware is developed with PlatformIO using the Arduino framework for the ATmega2560.

```bash
pio run
pio run --target upload
pio device monitor
```

## Project status

Current version: functional embedded-system prototype. Formal architecture, I/O mapping, state-machine documentation and validation test cases will be added in the next documentation phase.
