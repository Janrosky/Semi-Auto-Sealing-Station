#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

constexpr uint8_t JOYSTICK_X_PIN = A0;
constexpr uint8_t JOYSTICK_Y_PIN = A1;
constexpr uint8_t JOYSTICK_BUTTON_PIN = 2;
constexpr uint8_t POTENTIOMETER_PIN = A2;
constexpr uint8_t PHOTO_SENSOR_PIN = A3;
constexpr uint8_t TEMPERATURE_PIN = 4;
constexpr uint8_t SERVO_PIN = 5;
constexpr uint8_t ECHO_PIN = 6;
constexpr uint8_t TRIG_PIN = 7;
constexpr uint8_t RGB_RED_PIN = 8;
constexpr uint8_t RGB_GREEN_PIN = 9;
constexpr uint8_t RGB_BLUE_PIN = 10;
constexpr uint8_t BUZZER_PIN = 11;

OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature temperatureSensor(&oneWire);
Servo stopperServo;

constexpr bool RGB_ACTIVE_LOW = true;
constexpr bool BUZZER_ACTIVE_LOW = true;
constexpr bool PHOTO_PRESENT_WHEN_HIGH = true;

int photoThreshold = 700;
constexpr int PHOTO_HYSTERESIS = 30;
constexpr float HEAD_CRITICAL_DISTANCE = 5.0f;
float headOkMin = 10.0f;
float headOkMax = 30.0f;
float sealTempMin = 20.0f;
float sealTempMax = 35.0f;
uint8_t dwellSeconds = 5;
uint8_t positioningTimeoutSeconds = 15;

constexpr unsigned long FINAL_INSPECTION_TIME = 1000;
constexpr unsigned long EXIT_CONFIRM_TIME = 400;
constexpr unsigned long PART_LOST_TIME = 400;
constexpr unsigned long COMPLETE_DISPLAY_TIME = 1500;

constexpr int SERVO_PASS_POSITION = 10;
constexpr int SERVO_HOLD_POSITION = 55;

enum class ServoState { PASS, HOLD };
ServoState servoState = ServoState::PASS;

constexpr unsigned long SENSOR_INTERVAL = 80;
constexpr unsigned long DISPLAY_INTERVAL = 200;
constexpr unsigned long SERIAL_INTERVAL = 500;
unsigned long previousSensorTime = 0;
unsigned long previousDisplayTime = 0;
unsigned long previousSerialTime = 0;

float rawDistance = -1.0f;
float headDistance = -1.0f;
constexpr float DISTANCE_FILTER_ALPHA = 0.35f;
uint8_t ultrasonicFailureCount = 0;
constexpr uint8_t MAX_ULTRASONIC_FAILURES = 3;

int photoRaw = 0;
float photoFiltered = -1.0f;
constexpr float PHOTO_FILTER_ALPHA = 0.25f;
bool trayPresent = false;

int potentiometerRaw = 0;
int lineSpeedSetpoint = 0;

float currentTemperature = 0.0f;
bool temperatureValid = false;
bool temperatureSensorFault = false;
constexpr unsigned long TEMPERATURE_INTERVAL = 1000;
constexpr unsigned long TEMPERATURE_CONVERSION_TIME = 200;
unsigned long previousTemperatureTime = 0;
unsigned long temperatureRequestTime = 0;
bool temperatureConversionPending = false;

enum class HeadState {
    SENSOR_FAULT,
    CRITICAL,
    TOO_CLOSE,
    POSITION_OK,
    TOO_FAR
};
HeadState headState = HeadState::SENSOR_FAULT;

enum class CycleState {
    WAITING_TRAY,
    TRAY_DETECTED,
    POSITIONING,
    SEALING,
    INSPECTING,
    RELEASE_WAIT_EXIT,
    COMPLETE,
    FAULT,
    FAULT_RELEASE_WAIT_EXIT
};
CycleState cycleState = CycleState::WAITING_TRAY;

enum class FaultCode : uint8_t {
    NONE = 0,
    ULTRASONIC_SENSOR = 1,
    HEAD_CRITICAL = 2,
    TEMPERATURE_SENSOR = 3,
    PART_LOST = 4,
    POSITION_TIMEOUT = 5,
    PROCESS_INVALID = 6
};
FaultCode activeFault = FaultCode::NONE;
FaultCode lastFault = FaultCode::NONE;

unsigned long cyclesStarted = 0;
unsigned long okPieces = 0;
unsigned long ngPieces = 0;
unsigned long abortedCycles = 0;
unsigned long alarmCount = 0;
unsigned long cycleStartTime = 0;
unsigned long dwellStartTime = 0;
unsigned long inspectionStartTime = 0;
unsigned long resultTime = 0;
unsigned long lastCycleTime = 0;

bool partLostTimerActive = false;
unsigned long partLostStartTime = 0;
bool exitTimerActive = false;
unsigned long exitStartTime = 0;

enum class Screen {
    PROCESS,
    HEAD,
    TEMPERATURE,
    PRODUCTION,
    SETPOINT,
    ALARMS,
    SETTINGS,
    DIAGNOSTICS
};
Screen currentScreen = Screen::PROCESS;

enum class SettingItem {
    HEAD_MIN,
    HEAD_MAX,
    PHOTO_THRESHOLD,
    TEMP_MIN,
    TEMP_MAX,
    DWELL_TIME,
    POSITION_TIMEOUT
};
SettingItem selectedSetting = SettingItem::HEAD_MIN;
bool editingSetting = false;

int joystickX = 512;
int joystickY = 512;
constexpr int JOYSTICK_LOW = 300;
constexpr int JOYSTICK_HIGH = 700;

enum class JoystickDirection { CENTER, UP, DOWN, LEFT, RIGHT };
bool joystickReady = true;
bool previousButtonPressed = false;
unsigned long previousButtonTime = 0;
constexpr unsigned long BUTTON_DEBOUNCE = 150;

void setRgb(bool red, bool green, bool blue) {
    digitalWrite(RGB_RED_PIN, red ? (RGB_ACTIVE_LOW ? LOW : HIGH) : (RGB_ACTIVE_LOW ? HIGH : LOW));
    digitalWrite(RGB_GREEN_PIN, green ? (RGB_ACTIVE_LOW ? LOW : HIGH) : (RGB_ACTIVE_LOW ? HIGH : LOW));
    digitalWrite(RGB_BLUE_PIN, blue ? (RGB_ACTIVE_LOW ? LOW : HIGH) : (RGB_ACTIVE_LOW ? HIGH : LOW));
}

void rgbOff() { setRgb(false, false, false); }
void rgbRed() { setRgb(true, false, false); }
void rgbGreen() { setRgb(false, true, false); }
void rgbYellow() { setRgb(true, true, false); }
void rgbBlue() { setRgb(false, false, true); }
void rgbCyan() { setRgb(false, true, true); }

void buzzerOn() { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? LOW : HIGH); }
void buzzerOff() { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW); }

void setServoPass() {
    if (servoState == ServoState::PASS) return;
    stopperServo.write(SERVO_PASS_POSITION);
    servoState = ServoState::PASS;
}

void setServoHold() {
    if (servoState == ServoState::HOLD) return;
    stopperServo.write(SERVO_HOLD_POSITION);
    servoState = ServoState::HOLD;
}

float readDistanceCm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
    if (duration == 0) return -1.0f;

    float distance = duration * 0.0343f / 2.0f;
    if (distance < 2.0f || distance > 400.0f) return -1.0f;
    return distance;
}

void updateDistance() {
    rawDistance = readDistanceCm();

    if (rawDistance < 0) {
        ultrasonicFailureCount++;
        if (ultrasonicFailureCount >= MAX_ULTRASONIC_FAILURES) headDistance = -1.0f;
        return;
    }

    ultrasonicFailureCount = 0;
    if (headDistance < 0) {
        headDistance = rawDistance;
        return;
    }

    headDistance = DISTANCE_FILTER_ALPHA * rawDistance +
                   (1.0f - DISTANCE_FILTER_ALPHA) * headDistance;
}

void updatePhotoSensor() {
    photoRaw = analogRead(PHOTO_SENSOR_PIN);

    if (photoFiltered < 0) photoFiltered = photoRaw;
    else photoFiltered = PHOTO_FILTER_ALPHA * photoRaw +
                         (1.0f - PHOTO_FILTER_ALPHA) * photoFiltered;

    if (PHOTO_PRESENT_WHEN_HIGH) {
        if (!trayPresent && photoFiltered > photoThreshold + PHOTO_HYSTERESIS) trayPresent = true;
        else if (trayPresent && photoFiltered < photoThreshold - PHOTO_HYSTERESIS) trayPresent = false;
    } else {
        if (!trayPresent && photoFiltered < photoThreshold - PHOTO_HYSTERESIS) trayPresent = true;
        else if (trayPresent && photoFiltered > photoThreshold + PHOTO_HYSTERESIS) trayPresent = false;
    }
}

void updatePotentiometer() {
    potentiometerRaw = analogRead(POTENTIOMETER_PIN);
    lineSpeedSetpoint = map(potentiometerRaw, 0, 1023, 0, 100);
}

void requestTemperature(unsigned long currentTime) {
    temperatureSensor.requestTemperatures();
    temperatureRequestTime = currentTime;
    temperatureConversionPending = true;
}

void updateTemperature(unsigned long currentTime) {
    if (temperatureConversionPending &&
        currentTime - temperatureRequestTime >= TEMPERATURE_CONVERSION_TIME) {

        float temperature = temperatureSensor.getTempCByIndex(0);
        temperatureConversionPending = false;
        previousTemperatureTime = currentTime;

        if (temperature == DEVICE_DISCONNECTED_C || temperature < -55.0f || temperature > 125.0f) {
            temperatureSensorFault = true;
            temperatureValid = false;
        } else {
            currentTemperature = temperature;
            temperatureSensorFault = false;
            temperatureValid = true;
        }
    }

    if (!temperatureConversionPending &&
        currentTime - previousTemperatureTime >= TEMPERATURE_INTERVAL) {
        requestTemperature(currentTime);
    }
}

bool temperatureReady() {
    return temperatureValid &&
           !temperatureSensorFault &&
           currentTemperature >= sealTempMin &&
           currentTemperature <= sealTempMax;
}

HeadState calculateHeadState() {
    if (headDistance < 0) return HeadState::SENSOR_FAULT;
    if (headDistance <= HEAD_CRITICAL_DISTANCE) return HeadState::CRITICAL;
    if (headDistance < headOkMin) return HeadState::TOO_CLOSE;
    if (headDistance <= headOkMax) return HeadState::POSITION_OK;
    return HeadState::TOO_FAR;
}

void resetPartLostTimer() {
    partLostTimerActive = false;
    partLostStartTime = 0;
}

bool trayConfirmedLost(unsigned long currentTime) {
    if (trayPresent) {
        resetPartLostTimer();
        return false;
    }

    if (!partLostTimerActive) {
        partLostTimerActive = true;
        partLostStartTime = currentTime;
        return false;
    }

    return currentTime - partLostStartTime >= PART_LOST_TIME;
}

void resetExitTimer() {
    exitTimerActive = false;
    exitStartTime = 0;
}

bool trayExitConfirmed(unsigned long currentTime) {
    if (trayPresent) {
        resetExitTimer();
        return false;
    }

    if (!exitTimerActive) {
        exitTimerActive = true;
        exitStartTime = currentTime;
        return false;
    }

    return currentTime - exitStartTime >= EXIT_CONFIRM_TIME;
}

uint8_t faultNumber(FaultCode fault) {
    return static_cast<uint8_t>(fault);
}

const char* faultText(FaultCode fault) {
    switch (fault) {
        case FaultCode::NONE: return "NONE";
        case FaultCode::ULTRASONIC_SENSOR: return "HEAD SENSOR ERR";
        case FaultCode::HEAD_CRITICAL: return "HEAD TOO CLOSE";
        case FaultCode::TEMPERATURE_SENSOR: return "TEMP SENSOR ERR";
        case FaultCode::PART_LOST: return "TRAY LOST";
        case FaultCode::POSITION_TIMEOUT: return "POS TIMEOUT";
        case FaultCode::PROCESS_INVALID: return "PROCESS INVALID";
    }
    return "UNKNOWN";
}

void enterFault(FaultCode fault, bool abortCycle, unsigned long currentTime) {
    if (cycleState == CycleState::FAULT ||
        cycleState == CycleState::FAULT_RELEASE_WAIT_EXIT) return;

    activeFault = fault;
    lastFault = fault;
    alarmCount++;

    if (abortCycle) abortedCycles++;
    else ngPieces++;

    cycleState = CycleState::FAULT;
    resultTime = currentTime;
    setServoHold();
    currentScreen = Screen::PROCESS;
}

bool startInterlocksOk() {
    if (!trayPresent) return false;
    if (headDistance < 0) return false;
    if (temperatureSensorFault) return false;
    if (!temperatureReady()) return false;
    return true;
}

void startCycle(unsigned long currentTime) {
    if (!startInterlocksOk()) return;

    cyclesStarted++;
    cycleStartTime = currentTime;
    dwellStartTime = 0;
    activeFault = FaultCode::NONE;
    resetPartLostTimer();
    setServoHold();
    cycleState = CycleState::POSITIONING;
    currentScreen = Screen::PROCESS;
}

void resetFault() {
    setServoPass();
    resetPartLostTimer();
    resetExitTimer();
    cycleState = CycleState::FAULT_RELEASE_WAIT_EXIT;
    currentScreen = Screen::PROCESS;
}

void updateCycle(unsigned long currentTime) {
    switch (cycleState) {
        case CycleState::WAITING_TRAY:
            setServoPass();
            if (trayPresent) {
                setServoHold();
                cycleState = CycleState::TRAY_DETECTED;
                currentScreen = Screen::PROCESS;
            }
            break;

        case CycleState::TRAY_DETECTED:
            setServoHold();
            if (!trayPresent) {
                setServoPass();
                cycleState = CycleState::WAITING_TRAY;
            }
            break;

        case CycleState::POSITIONING:
            setServoHold();

            if (trayConfirmedLost(currentTime)) {
                enterFault(FaultCode::PART_LOST, true, currentTime);
                break;
            }

            if (temperatureSensorFault) {
                enterFault(FaultCode::TEMPERATURE_SENSOR, false, currentTime);
                break;
            }

            if (headState == HeadState::SENSOR_FAULT) {
                enterFault(FaultCode::ULTRASONIC_SENSOR, false, currentTime);
                break;
            }

            if (headState == HeadState::CRITICAL) {
                enterFault(FaultCode::HEAD_CRITICAL, false, currentTime);
                break;
            }

            if (currentTime - cycleStartTime >=
                static_cast<unsigned long>(positioningTimeoutSeconds) * 1000UL) {
                enterFault(FaultCode::POSITION_TIMEOUT, false, currentTime);
                break;
            }

            if (headState == HeadState::POSITION_OK && temperatureReady()) {
                dwellStartTime = currentTime;
                cycleState = CycleState::SEALING;
                currentScreen = Screen::PROCESS;
            }
            break;

        case CycleState::SEALING:
            setServoHold();

            if (trayConfirmedLost(currentTime)) {
                enterFault(FaultCode::PART_LOST, true, currentTime);
                break;
            }

            if (temperatureSensorFault) {
                enterFault(FaultCode::TEMPERATURE_SENSOR, false, currentTime);
                break;
            }

            if (headState == HeadState::SENSOR_FAULT) {
                enterFault(FaultCode::ULTRASONIC_SENSOR, false, currentTime);
                break;
            }

            if (headState == HeadState::CRITICAL) {
                enterFault(FaultCode::HEAD_CRITICAL, false, currentTime);
                break;
            }

            if (headState != HeadState::POSITION_OK || !temperatureReady()) {
                dwellStartTime = 0;
                cycleState = CycleState::POSITIONING;
                break;
            }

            if (currentTime - dwellStartTime >=
                static_cast<unsigned long>(dwellSeconds) * 1000UL) {
                inspectionStartTime = currentTime;
                cycleState = CycleState::INSPECTING;
                currentScreen = Screen::PROCESS;
            }
            break;

        case CycleState::INSPECTING:
            setServoHold();

            if (trayConfirmedLost(currentTime)) {
                enterFault(FaultCode::PART_LOST, true, currentTime);
                break;
            }

            if (headState != HeadState::POSITION_OK || !temperatureReady()) {
                enterFault(FaultCode::PROCESS_INVALID, false, currentTime);
                break;
            }

            if (currentTime - inspectionStartTime >= FINAL_INSPECTION_TIME) {
                setServoPass();
                resetExitTimer();
                cycleState = CycleState::RELEASE_WAIT_EXIT;
                currentScreen = Screen::PROCESS;
            }
            break;

        case CycleState::RELEASE_WAIT_EXIT:
            setServoPass();

            if (trayExitConfirmed(currentTime)) {
                okPieces++;
                lastCycleTime = currentTime - cycleStartTime;
                resultTime = currentTime;
                cycleState = CycleState::COMPLETE;
                currentScreen = Screen::PROCESS;
            }
            break;

        case CycleState::COMPLETE:
            setServoPass();
            if (currentTime - resultTime >= COMPLETE_DISPLAY_TIME) {
                cycleState = CycleState::WAITING_TRAY;
            }
            break;

        case CycleState::FAULT:
            setServoHold();
            break;

        case CycleState::FAULT_RELEASE_WAIT_EXIT:
            setServoPass();

            if (trayExitConfirmed(currentTime)) {
                activeFault = FaultCode::NONE;
                resetPartLostTimer();
                resetExitTimer();
                cycleState = CycleState::WAITING_TRAY;
                currentScreen = Screen::PROCESS;
            }
            break;
    }
}

void updateRgb(unsigned long currentTime) {
    if (cycleState == CycleState::FAULT) {
        if ((currentTime / 300) % 2 == 0) rgbRed();
        else rgbOff();
        return;
    }

    if (cycleState == CycleState::FAULT_RELEASE_WAIT_EXIT) {
        rgbYellow();
        return;
    }

    switch (cycleState) {
        case CycleState::WAITING_TRAY: rgbOff(); break;
        case CycleState::TRAY_DETECTED: rgbYellow(); break;
        case CycleState::POSITIONING:
            if (headState == HeadState::TOO_CLOSE) rgbRed();
            else rgbYellow();
            break;
        case CycleState::SEALING: rgbBlue(); break;
        case CycleState::INSPECTING: rgbCyan(); break;
        case CycleState::RELEASE_WAIT_EXIT: rgbGreen(); break;
        case CycleState::COMPLETE: rgbGreen(); break;
        case CycleState::FAULT:
        case CycleState::FAULT_RELEASE_WAIT_EXIT:
            break;
    }
}

void updateBuzzer() {
    if (headState == HeadState::CRITICAL) buzzerOn();
    else buzzerOff();
}

void readJoystick() {
    joystickX = analogRead(JOYSTICK_X_PIN);
    joystickY = analogRead(JOYSTICK_Y_PIN);
}

JoystickDirection getJoystickDirection() {
    if (joystickY < JOYSTICK_LOW) return JoystickDirection::UP;
    if (joystickY > JOYSTICK_HIGH) return JoystickDirection::DOWN;
    if (joystickX < JOYSTICK_LOW) return JoystickDirection::LEFT;
    if (joystickX > JOYSTICK_HIGH) return JoystickDirection::RIGHT;
    return JoystickDirection::CENTER;
}

void nextScreen() {
    switch (currentScreen) {
        case Screen::PROCESS: currentScreen = Screen::HEAD; break;
        case Screen::HEAD: currentScreen = Screen::TEMPERATURE; break;
        case Screen::TEMPERATURE: currentScreen = Screen::PRODUCTION; break;
        case Screen::PRODUCTION: currentScreen = Screen::SETPOINT; break;
        case Screen::SETPOINT: currentScreen = Screen::ALARMS; break;
        case Screen::ALARMS: currentScreen = Screen::SETTINGS; break;
        case Screen::SETTINGS: currentScreen = Screen::DIAGNOSTICS; break;
        case Screen::DIAGNOSTICS: currentScreen = Screen::PROCESS; break;
    }
}

void previousScreen() {
    switch (currentScreen) {
        case Screen::PROCESS: currentScreen = Screen::DIAGNOSTICS; break;
        case Screen::HEAD: currentScreen = Screen::PROCESS; break;
        case Screen::TEMPERATURE: currentScreen = Screen::HEAD; break;
        case Screen::PRODUCTION: currentScreen = Screen::TEMPERATURE; break;
        case Screen::SETPOINT: currentScreen = Screen::PRODUCTION; break;
        case Screen::ALARMS: currentScreen = Screen::SETPOINT; break;
        case Screen::SETTINGS: currentScreen = Screen::ALARMS; break;
        case Screen::DIAGNOSTICS: currentScreen = Screen::SETTINGS; break;
    }
}

void nextSetting() {
    switch (selectedSetting) {
        case SettingItem::HEAD_MIN: selectedSetting = SettingItem::HEAD_MAX; break;
        case SettingItem::HEAD_MAX: selectedSetting = SettingItem::PHOTO_THRESHOLD; break;
        case SettingItem::PHOTO_THRESHOLD: selectedSetting = SettingItem::TEMP_MIN; break;
        case SettingItem::TEMP_MIN: selectedSetting = SettingItem::TEMP_MAX; break;
        case SettingItem::TEMP_MAX: selectedSetting = SettingItem::DWELL_TIME; break;
        case SettingItem::DWELL_TIME: selectedSetting = SettingItem::POSITION_TIMEOUT; break;
        case SettingItem::POSITION_TIMEOUT: selectedSetting = SettingItem::HEAD_MIN; break;
    }
}

void previousSetting() {
    switch (selectedSetting) {
        case SettingItem::HEAD_MIN: selectedSetting = SettingItem::POSITION_TIMEOUT; break;
        case SettingItem::HEAD_MAX: selectedSetting = SettingItem::HEAD_MIN; break;
        case SettingItem::PHOTO_THRESHOLD: selectedSetting = SettingItem::HEAD_MAX; break;
        case SettingItem::TEMP_MIN: selectedSetting = SettingItem::PHOTO_THRESHOLD; break;
        case SettingItem::TEMP_MAX: selectedSetting = SettingItem::TEMP_MIN; break;
        case SettingItem::DWELL_TIME: selectedSetting = SettingItem::TEMP_MAX; break;
        case SettingItem::POSITION_TIMEOUT: selectedSetting = SettingItem::DWELL_TIME; break;
    }
}

void adjustSetting(int direction) {
    switch (selectedSetting) {
        case SettingItem::HEAD_MIN:
            headOkMin += direction;
            if (headOkMin < HEAD_CRITICAL_DISTANCE + 1.0f) headOkMin = HEAD_CRITICAL_DISTANCE + 1.0f;
            if (headOkMin > headOkMax - 1.0f) headOkMin = headOkMax - 1.0f;
            break;

        case SettingItem::HEAD_MAX:
            headOkMax += direction;
            if (headOkMax < headOkMin + 1.0f) headOkMax = headOkMin + 1.0f;
            if (headOkMax > 200.0f) headOkMax = 200.0f;
            break;

        case SettingItem::PHOTO_THRESHOLD:
            photoThreshold += direction * 10;
            photoThreshold = constrain(photoThreshold, 50, 950);
            break;

        case SettingItem::TEMP_MIN:
            sealTempMin += direction;
            if (sealTempMin > sealTempMax - 1.0f) sealTempMin = sealTempMax - 1.0f;
            break;

        case SettingItem::TEMP_MAX:
            sealTempMax += direction;
            if (sealTempMax < sealTempMin + 1.0f) sealTempMax = sealTempMin + 1.0f;
            break;

        case SettingItem::DWELL_TIME:
            if (direction > 0 && dwellSeconds < 30) dwellSeconds++;
            if (direction < 0 && dwellSeconds > 1) dwellSeconds--;
            break;

        case SettingItem::POSITION_TIMEOUT:
            if (direction > 0 && positioningTimeoutSeconds < 60) positioningTimeoutSeconds++;
            if (direction < 0 && positioningTimeoutSeconds > 5) positioningTimeoutSeconds--;
            break;
    }
}

void processJoystick() {
    JoystickDirection direction = getJoystickDirection();

    if (direction == JoystickDirection::CENTER) {
        joystickReady = true;
        return;
    }

    if (!joystickReady) return;
    joystickReady = false;

    if (currentScreen == Screen::SETTINGS && editingSetting) {
        if (direction == JoystickDirection::LEFT) adjustSetting(-1);
        if (direction == JoystickDirection::RIGHT) adjustSetting(1);
        return;
    }

    if (currentScreen == Screen::SETTINGS) {
        if (direction == JoystickDirection::LEFT) {
            previousSetting();
            return;
        }
        if (direction == JoystickDirection::RIGHT) {
            nextSetting();
            return;
        }
    }

    if (direction == JoystickDirection::UP) previousScreen();
    if (direction == JoystickDirection::DOWN) nextScreen();
}

void processButton(unsigned long currentTime) {
    bool pressed = digitalRead(JOYSTICK_BUTTON_PIN) == LOW;
    bool newPress = pressed && !previousButtonPressed;

    if (newPress && currentTime - previousButtonTime > BUTTON_DEBOUNCE) {
        previousButtonTime = currentTime;

        if (cycleState == CycleState::TRAY_DETECTED) {
            startCycle(currentTime);
            previousButtonPressed = pressed;
            return;
        }

        if (cycleState == CycleState::FAULT) {
            resetFault();
            previousButtonPressed = pressed;
            return;
        }

        if (currentScreen == Screen::SETTINGS) editingSetting = !editingSetting;
    }

    previousButtonPressed = pressed;
}

const char* headStateText() {
    switch (headState) {
        case HeadState::SENSOR_FAULT: return "SENSOR ERR";
        case HeadState::CRITICAL: return "CRITICAL";
        case HeadState::TOO_CLOSE: return "TOO CLOSE";
        case HeadState::POSITION_OK: return "POSITION OK";
        case HeadState::TOO_FAR: return "TOO FAR";
    }
    return "?";
}

const char* cycleStateText() {
    switch (cycleState) {
        case CycleState::WAITING_TRAY: return "WAITING";
        case CycleState::TRAY_DETECTED: return "TRAY_DETECTED";
        case CycleState::POSITIONING: return "POSITIONING";
        case CycleState::SEALING: return "SEALING";
        case CycleState::INSPECTING: return "INSPECTING";
        case CycleState::RELEASE_WAIT_EXIT: return "RELEASING";
        case CycleState::COMPLETE: return "COMPLETE";
        case CycleState::FAULT: return "FAULT";
        case CycleState::FAULT_RELEASE_WAIT_EXIT: return "FAULT_RELEASE";
    }
    return "?";
}

void clearLine(uint8_t row) {
    lcd.setCursor(0, row);
    lcd.print("                ");
    lcd.setCursor(0, row);
}

void displayProcess(unsigned long currentTime) {
    clearLine(0);
    clearLine(1);

    switch (cycleState) {
        case CycleState::WAITING_TRAY:
            lcd.setCursor(0, 0); lcd.print("WAITING TRAY");
            lcd.setCursor(0, 1); lcd.print("STOPPER OPEN");
            break;

        case CycleState::TRAY_DETECTED:
            lcd.setCursor(0, 0); lcd.print("TRAY DETECTED");
            lcd.setCursor(0, 1);
            if (!temperatureValid) lcd.print("TEMP STARTING");
            else if (!temperatureReady()) lcd.print("TEMP NOT READY");
            else if (headState == HeadState::SENSOR_FAULT) lcd.print("HEAD SENSOR ERR");
            else lcd.print("PRESS BTN START");
            break;

        case CycleState::POSITIONING:
            lcd.setCursor(0, 0); lcd.print("POSITION HEAD");
            lcd.setCursor(0, 1);
            if (!temperatureReady()) {
                lcd.print("TEMP NOT READY");
                break;
            }
            if (headState == HeadState::TOO_FAR) lcd.print("MOVE DOWN ");
            else if (headState == HeadState::TOO_CLOSE) lcd.print("MOVE UP ");
            else if (headState == HeadState::POSITION_OK) lcd.print("HEAD OK ");
            else if (headState == HeadState::CRITICAL) lcd.print("CRITICAL ");
            if (headDistance >= 0) {
                lcd.print(headDistance, 1);
                lcd.print("cm");
            }
            break;

        case CycleState::SEALING: {
            float elapsedSeconds = (currentTime - dwellStartTime) / 1000.0f;
            lcd.setCursor(0, 0);
            lcd.print("SEALING ");
            lcd.print(elapsedSeconds, 1);
            lcd.print("/");
            lcd.print(dwellSeconds);
            lcd.print("s");
            lcd.setCursor(0, 1);
            lcd.print("HOLD HEAD STEADY");
            break;
        }

        case CycleState::INSPECTING:
            lcd.setCursor(0, 0); lcd.print("FINAL INSPECTION");
            lcd.setCursor(0, 1); lcd.print("TEMP + POS OK");
            break;

        case CycleState::RELEASE_WAIT_EXIT:
            lcd.setCursor(0, 0); lcd.print("SEAL COMPLETE");
            lcd.setCursor(0, 1); lcd.print("RELEASING TRAY");
            break;

        case CycleState::COMPLETE:
            lcd.setCursor(0, 0); lcd.print("CYCLE COMPLETE");
            lcd.setCursor(0, 1);
            lcd.print("OK:"); lcd.print(okPieces);
            lcd.print(" NG:"); lcd.print(ngPieces);
            break;

        case CycleState::FAULT:
            lcd.setCursor(0, 0);
            lcd.print("FAULT A00");
            lcd.print(faultNumber(activeFault));
            lcd.setCursor(0, 1);
            lcd.print("PRESS BTN RESET");
            break;

        case CycleState::FAULT_RELEASE_WAIT_EXIT:
            lcd.setCursor(0, 0); lcd.print("RESET ACCEPTED");
            lcd.setCursor(0, 1); lcd.print("REMOVE TRAY");
            break;
    }
}

void displayHead() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print("HEAD:");
    if (headDistance < 0) lcd.print("ERROR");
    else {
        lcd.print(headDistance, 1);
        lcd.print("cm");
    }
    lcd.setCursor(0, 1);
    lcd.print(headStateText());
}

void displayTemperature() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print("SEAL TEMP:");
    if (!temperatureValid || temperatureSensorFault) lcd.print("ERR");
    else {
        lcd.print(currentTemperature, 1);
        lcd.print("C");
    }
    lcd.setCursor(0, 1);
    lcd.print(temperatureReady() ? "TEMP READY" : "TEMP NOT READY");
}

void displayProduction() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print("START:"); lcd.print(cyclesStarted);
    lcd.print(" OK:"); lcd.print(okPieces);
    lcd.setCursor(0, 1);
    lcd.print("NG:"); lcd.print(ngPieces);
    lcd.print(" AB:"); lcd.print(abortedCycles);
}

void displaySetpoint() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print("LINE SPEED SET");
    lcd.setCursor(0, 1);
    lcd.print(lineSpeedSetpoint);
    lcd.print("% RAW:");
    lcd.print(potentiometerRaw);
}

void displayAlarms() {
    clearLine(0);
    clearLine(1);

    if (cycleState == CycleState::FAULT) {
        lcd.setCursor(0, 0);
        lcd.print("ACTIVE A00");
        lcd.print(faultNumber(activeFault));
        lcd.setCursor(0, 1);
        lcd.print(faultText(activeFault));
        return;
    }

    if (cycleState == CycleState::FAULT_RELEASE_WAIT_EXIT) {
        lcd.setCursor(0, 0);
        lcd.print("RECOVERY A00");
        lcd.print(faultNumber(activeFault));
        lcd.setCursor(0, 1);
        lcd.print("REMOVE TRAY");
        return;
    }

    lcd.setCursor(0, 0);
    lcd.print("NO ACTIVE ALARM");
    lcd.setCursor(0, 1);
    lcd.print("LAST:");
    if (lastFault == FaultCode::NONE) lcd.print("NONE");
    else {
        lcd.print("A00");
        lcd.print(faultNumber(lastFault));
    }
}

void displaySettings() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print(editingSetting ? "EDIT " : "SET ");

    switch (selectedSetting) {
        case SettingItem::HEAD_MIN:
            lcd.print("HEAD MIN");
            lcd.setCursor(0, 1); lcd.print(headOkMin, 1); lcd.print("cm");
            break;
        case SettingItem::HEAD_MAX:
            lcd.print("HEAD MAX");
            lcd.setCursor(0, 1); lcd.print(headOkMax, 1); lcd.print("cm");
            break;
        case SettingItem::PHOTO_THRESHOLD:
            lcd.print("PHOTO");
            lcd.setCursor(0, 1); lcd.print(photoThreshold);
            break;
        case SettingItem::TEMP_MIN:
            lcd.print("TEMP MIN");
            lcd.setCursor(0, 1); lcd.print(sealTempMin, 1); lcd.print("C");
            break;
        case SettingItem::TEMP_MAX:
            lcd.print("TEMP MAX");
            lcd.setCursor(0, 1); lcd.print(sealTempMax, 1); lcd.print("C");
            break;
        case SettingItem::DWELL_TIME:
            lcd.print("DWELL");
            lcd.setCursor(0, 1); lcd.print(dwellSeconds); lcd.print(" sec");
            break;
        case SettingItem::POSITION_TIMEOUT:
            lcd.print("POS TIMEOUT");
            lcd.setCursor(0, 1); lcd.print(positioningTimeoutSeconds); lcd.print(" sec");
            break;
    }
}

void displayDiagnostics() {
    clearLine(0);
    clearLine(1);
    lcd.setCursor(0, 0);
    lcd.print("L:"); lcd.print(photoRaw);
    lcd.print(" P:"); lcd.print(trayPresent ? 1 : 0);
    lcd.setCursor(0, 1);
    lcd.print("D:");
    if (headDistance < 0) lcd.print("ERR");
    else lcd.print(headDistance, 1);
    lcd.print(" T:");
    if (!temperatureValid) lcd.print("ERR");
    else lcd.print(currentTemperature, 1);
}

void updateDisplay(unsigned long currentTime) {
    switch (currentScreen) {
        case Screen::PROCESS: displayProcess(currentTime); break;
        case Screen::HEAD: displayHead(); break;
        case Screen::TEMPERATURE: displayTemperature(); break;
        case Screen::PRODUCTION: displayProduction(); break;
        case Screen::SETPOINT: displaySetpoint(); break;
        case Screen::ALARMS: displayAlarms(); break;
        case Screen::SETTINGS: displaySettings(); break;
        case Screen::DIAGNOSTICS: displayDiagnostics(); break;
    }
}

void sendTelemetry(unsigned long currentTime) {
    Serial.print("CYCLE="); Serial.print(cycleStateText());
    Serial.print(";TRAY="); Serial.print(trayPresent ? 1 : 0);
    Serial.print(";LIGHT="); Serial.print(photoRaw);
    Serial.print(";LIGHT_F="); Serial.print(photoFiltered, 0);
    Serial.print(";HEAD_DIST="); Serial.print(headDistance, 1);
    Serial.print(";HEAD_STATE="); Serial.print(headStateText());
    Serial.print(";TEMP=");
    if (temperatureValid) Serial.print(currentTemperature, 1);
    else Serial.print("ERR");
    Serial.print(";TEMP_READY="); Serial.print(temperatureReady() ? 1 : 0);
    Serial.print(";SPEED="); Serial.print(lineSpeedSetpoint);
    Serial.print(";SERVO="); Serial.print(servoState == ServoState::HOLD ? "HOLD" : "PASS");
    Serial.print(";DWELL_MS=");
    if (cycleState == CycleState::SEALING) Serial.print(currentTime - dwellStartTime);
    else Serial.print(0);
    Serial.print(";STARTED="); Serial.print(cyclesStarted);
    Serial.print(";OK="); Serial.print(okPieces);
    Serial.print(";NG="); Serial.print(ngPieces);
    Serial.print(";ABORT="); Serial.print(abortedCycles);
    Serial.print(";FAULT="); Serial.print(faultNumber(activeFault));
    Serial.print(";JOY_X="); Serial.print(joystickX);
    Serial.print(";JOY_Y="); Serial.println(joystickY);
}

void setup() {
    Serial.begin(9600);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    rgbOff();

    digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? HIGH : LOW);
    pinMode(BUZZER_PIN, OUTPUT);
    buzzerOff();

    stopperServo.attach(SERVO_PIN);
    stopperServo.write(SERVO_PASS_POSITION);
    servoState = ServoState::PASS;

    temperatureSensor.begin();
    temperatureSensor.setResolution(10);
    temperatureSensor.setWaitForConversion(false);
    requestTemperature(millis());

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SEALING STATION");
    lcd.setCursor(0, 1);
    lcd.print("INITIALIZING");
    delay(1500);
    lcd.clear();

    Serial.println();
    Serial.println("========================================");
    Serial.println("SEMI-AUTOMATIC THERMAL SEALING STATION");
    Serial.println("SYSTEM READY");
    Serial.println("========================================");
}

void loop() {
    unsigned long currentTime = millis();

    readJoystick();
    processJoystick();
    processButton(currentTime);

    updateTemperature(currentTime);

    if (currentTime - previousSensorTime >= SENSOR_INTERVAL) {
        previousSensorTime = currentTime;

        updateDistance();
        updatePhotoSensor();
        updatePotentiometer();

        headState = calculateHeadState();
        updateCycle(currentTime);
    }

    updateRgb(currentTime);
    updateBuzzer();

    if (currentTime - previousDisplayTime >= DISPLAY_INTERVAL) {
        previousDisplayTime = currentTime;
        updateDisplay(currentTime);
    }

    if (currentTime - previousSerialTime >= SERIAL_INTERVAL) {
        previousSerialTime = currentTime;
        sendTelemetry(currentTime);
    }
}
