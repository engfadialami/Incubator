#include <DHT.h>
#include <Servo.h>
#include <EEPROM.h>

// =========================
// Pin assignment
// =========================
#define DHTPIN1 5
#define DHTPIN2 7
#define DHTTYPE DHT11

#define RELAY_HEATER       2
#define RELAY_HUMIDITY_FAN 3

#define SERVO_LEFT_PIN   9
#define SERVO_RIGHT_PIN  11

#define STATUS_LED_PIN 13

// =========================
// Relay logic
// =========================
const bool RELAY_ACTIVE_LOW = true;

// =========================
// Temperature pulse control
// =========================
const float SETPOINT_C = 37.5;
const float EXACT_BAND = 0.0;

const float ALWAYS_ON_BELOW_C  = 37.2;
const float ALWAYS_OFF_ABOVE_C = 37.8;

const unsigned long HEATER_WINDOW_MS = 100000;

// Base duties
const int DUTY_UNDER_SETPOINT = 35;
const int DUTY_AT_SETPOINT    = 30;
const int DUTY_OVER_SETPOINT  = 25;

// =========================
// Adaptive cumulative correction
// =========================
const float CORRECTION_HIGH_TEMP_C = 37.7;
const float CORRECTION_LOW_TEMP_C  = 37.3;

const int CORRECTION_STEP_DUTY = 5;
const int MAX_CUMULATIVE_CORRECTION = 25;

const unsigned long CORRECTION_INTERVAL_MS = 15UL * 60UL * 1000UL;

int dutyCorrection = 0;
unsigned long lastCorrectionTime = 0;

// =========================
// Humidity control
// =========================
const float HUM_LOW_SETPOINT  = 50.0;
const float HUM_HIGH_SETPOINT =55.0;

// =========================
// Sensor validity limits
// =========================
const float SENSOR_TEMP_MIN_C = 10.0;
const float SENSOR_TEMP_MAX_C = 60.0;

// =========================
// Sensor offsets
// =========================
const float TEMP_OFFSET_1 = 0.0;
const float HUM_OFFSET_1  = 14.0;
const float TEMP_OFFSET_2 = 0.0;
const float HUM_OFFSET_2  = 0.0;

// =========================
// Servo settings
// =========================
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_STEP_DELAY_MS = 15;

const unsigned long SERVO_TURN_INTERVAL_MS = 4UL * 60UL * 60UL * 1000UL;

// EEPROM addresses
const int EEPROM_SERVO_ANGLE_ADDR = 0;
const int EEPROM_CORRECTION_ADDR  = 1;

// =========================
// Objects
// =========================
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

Servo servoLeft;
Servo servoRight;

// =========================
// State
// =========================
bool heaterOn = false;
bool humidityFanOn = false;
int heaterDuty = 0;

bool servoAtMax = true;
int currentServoAngle = SERVO_MAX_ANGLE;
unsigned long lastServoTurnTime = 0;

// =========================
// Helper functions
// =========================
void setRelay(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

bool isSensorValid(float t, float h) {
  if (isnan(t) || isnan(h)) return false;
  if (t < SENSOR_TEMP_MIN_C) return false;
  if (t > SENSOR_TEMP_MAX_C) return false;
  return true;
}

float round1(float x) {
  return round(x * 10.0) / 10.0;
}

int clampDuty(int value) {
  if (value < 0) return 0;
  if (value > 100) return 100;
  return value;
}

int clampCorrection(int value) {
  if (value > MAX_CUMULATIVE_CORRECTION) return MAX_CUMULATIVE_CORRECTION;
  if (value < -MAX_CUMULATIVE_CORRECTION) return -MAX_CUMULATIVE_CORRECTION;
  return value;
}

// =========================
// EEPROM correction functions
// Stored as correction + 100
// =========================
void saveCorrectionToEEPROM(int correction) {
  int storedValue = correction + 100;
  EEPROM.update(EEPROM_CORRECTION_ADDR, storedValue);
}

int readCorrectionFromEEPROM() {
  int storedValue = EEPROM.read(EEPROM_CORRECTION_ADDR);

  int correction = storedValue - 100;

  if (correction < -MAX_CUMULATIVE_CORRECTION ||
      correction > MAX_CUMULATIVE_CORRECTION) {
    correction = 0;
  }

  return correction;
}

void updateDutyCorrection(float temp) {
  unsigned long now = millis();

  if (now - lastCorrectionTime < CORRECTION_INTERVAL_MS) {
    return;
  }

  int oldCorrection = dutyCorrection;

  if (temp >= CORRECTION_HIGH_TEMP_C) {
    dutyCorrection -= CORRECTION_STEP_DUTY;
    dutyCorrection = clampCorrection(dutyCorrection);
    lastCorrectionTime = now;
  }
  else if (temp <= CORRECTION_LOW_TEMP_C) {
    dutyCorrection += CORRECTION_STEP_DUTY;
    dutyCorrection = clampCorrection(dutyCorrection);
    lastCorrectionTime = now;
  }

  if (dutyCorrection != oldCorrection) {
    saveCorrectionToEEPROM(dutyCorrection);
  }
}

int calculateHeaterDuty(float temp) {
  updateDutyCorrection(temp);

  if (temp < ALWAYS_ON_BELOW_C) return 100;
  if (temp > ALWAYS_OFF_ABOVE_C) return 0;

  int baseDuty;

  if (temp < SETPOINT_C - EXACT_BAND) {
    baseDuty = DUTY_UNDER_SETPOINT;
  }
  else if (temp > SETPOINT_C + EXACT_BAND) {
    baseDuty = DUTY_OVER_SETPOINT;
  }
  else {
    baseDuty = DUTY_AT_SETPOINT;
  }

  return clampDuty(baseDuty + dutyCorrection);
}

void applyHeaterPulseControl(int dutyPercent) {
  unsigned long windowPosition = millis() % HEATER_WINDOW_MS;
  unsigned long onTime = (HEATER_WINDOW_MS * dutyPercent) / 100;

  heaterOn = (windowPosition < onTime);
  setRelay(RELAY_HEATER, heaterOn);
}

void saveServoAngleToEEPROM(int angle) {
  EEPROM.update(EEPROM_SERVO_ANGLE_ADDR, angle);
}

int readServoAngleFromEEPROM() {
  int angle = EEPROM.read(EEPROM_SERVO_ANGLE_ADDR);

  if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE) {
    angle = SERVO_MAX_ANGLE;
  }

  return angle;
}

void moveServosSmooth(int targetAngle) {
  if (currentServoAngle < targetAngle) {
    for (int angle = currentServoAngle; angle <= targetAngle; angle++) {
      servoLeft.write(angle);
      servoRight.write(angle);
      delay(SERVO_STEP_DELAY_MS);
    }
  } else {
    for (int angle = currentServoAngle; angle >= targetAngle; angle--) {
      servoLeft.write(angle);
      servoRight.write(angle);
      delay(SERVO_STEP_DELAY_MS);
    }
  }

  currentServoAngle = targetAngle;
  saveServoAngleToEEPROM(currentServoAngle);
}

void rotateEggs() {
  if (servoAtMax) {
    moveServosSmooth(SERVO_MIN_ANGLE);
    servoAtMax = false;
    Serial.println("Servo: MIN");
  } else {
    moveServosSmooth(SERVO_MAX_ANGLE);
    servoAtMax = true;
    Serial.println("Servo: MAX");
  }

  lastServoTurnTime = millis();
}

void checkServo() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'r' || c == 'R') {
      rotateEggs();
    }
  }

  if (millis() - lastServoTurnTime >= SERVO_TURN_INTERVAL_MS) {
    rotateEggs();
  }
}

void setup() {
  Serial.begin(9600);

  dht1.begin();
  dht2.begin();

  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(RELAY_HUMIDITY_FAN, OUTPUT);

  setRelay(RELAY_HEATER, false);
  setRelay(RELAY_HUMIDITY_FAN, false);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Read saved correction
  dutyCorrection = readCorrectionFromEEPROM();

  int savedServoAngle = readServoAngleFromEEPROM();

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  servoLeft.write(savedServoAngle);
  servoRight.write(savedServoAngle);

  currentServoAngle = savedServoAngle;
  servoAtMax = (savedServoAngle == SERVO_MAX_ANGLE);

  lastServoTurnTime = millis();
  lastCorrectionTime = millis();

  Serial.print("System started | Saved Corr=");
  Serial.println(dutyCorrection);
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  checkServo();

  float h1 = dht1.readHumidity();
  float t1 = dht1.readTemperature();

  float h2 = dht2.readHumidity();
  float t2 = dht2.readTemperature();

  h1 += HUM_OFFSET_1;
  t1 += TEMP_OFFSET_1;

  h2 += HUM_OFFSET_2;
  t2 += TEMP_OFFSET_2;

  bool ok1 = isSensorValid(t1, h1);
  bool ok2 = isSensorValid(t2, h2);

  float usedTemp = 0;
  float usedHum  = 0;

  bool anyValid = false;

  if (ok1 && ok2) {
    usedTemp = (t1 + t2) / 2.0;
    usedHum  = (h1 + h2) / 2.0;
    anyValid = true;
  }
  else if (ok1) {
    usedTemp = t1;
    usedHum  = h1;
    anyValid = true;
  }
  else if (ok2) {
    usedTemp = t2;
    usedHum  = h2;
    anyValid = true;
  }

  if (anyValid) {
    float controlTemp = round1(usedTemp);

    heaterDuty = calculateHeaterDuty(controlTemp);

    applyHeaterPulseControl(heaterDuty);

    if (!humidityFanOn && usedHum < HUM_LOW_SETPOINT) {
      humidityFanOn = true;
      setRelay(RELAY_HUMIDITY_FAN, true);
    }
    else if (humidityFanOn && usedHum > HUM_HIGH_SETPOINT) {
      humidityFanOn = false;
      setRelay(RELAY_HUMIDITY_FAN, false);
    }

    Serial.print("S1: ");

    if (ok1) {
      Serial.print(t1, 1);
      Serial.print("C ");
      Serial.print(h1, 1);
      Serial.print("%");
    } else {
      Serial.print("ERR");
    }

    Serial.print(" | S2: ");

    if (ok2) {
      Serial.print(t2, 1);
      Serial.print("C ");
      Serial.print(h2, 1);
      Serial.print("%");
    } else {
      Serial.print("ERR");
    }

    Serial.print(" | T=");
    Serial.print(controlTemp, 1);

    Serial.print(" | H=");
    Serial.print(usedHum, 1);

    Serial.print(" | Duty=");
    Serial.print(heaterDuty);

    Serial.print(" | Corr=");
    Serial.print(dutyCorrection);

    Serial.print(" | Heater=");
    Serial.print(heaterOn);

    Serial.print(" | HumFan=");
    Serial.print(humidityFanOn);

    Serial.print(" | Servo=");
    Serial.println(servoAtMax ? "MAX" : "MIN");

    bool systemOK =
    (controlTemp >= 37.3 && controlTemp <= 37.7) &&
    (usedHum >= 45.0 && usedHum <= 65.0);

    digitalWrite(STATUS_LED_PIN, systemOK ? LOW : HIGH);
  }
  else {
    heaterOn = false;
    humidityFanOn = false;
    heaterDuty = 0;

    setRelay(RELAY_HEATER, false);
    setRelay(RELAY_HUMIDITY_FAN, false);

    Serial.println("Heater: OFF | HumFan: OFF (no valid sensors)");
  }

  delay(2000);
}