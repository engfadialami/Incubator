#include <DHT.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// =========================
// Mode
// =========================
bool hatchingMode = true;

// =========================
// Pin assignment
// =========================
#define DHTPIN1 4
#define DHTPIN2 5
#define DHTPIN3 6
#define DHTPIN4 7
#define DHTTYPE DHT11

#define RELAY_HEATER       2
#define RELAY_HUMIDITY_FAN 3

#define SERVO_LEFT_PIN   9
#define SERVO_RIGHT_PIN  11

#define SERVO_POWER_PIN 8

#define STATUS_LED_PIN 13

// =========================
// Bluetooth HC-05 serial
// =========================
#define BT_RX_PIN 12   // Arduino RX  <- HC-05 TX  (green wire)
#define BT_TX_PIN 10   // Arduino TX  -> HC-05 RX  (yellow wire, through voltage divider)

// =========================
// Servo power control
// =========================
const bool SERVO_POWER_ACTIVE_LOW = true;   // true for most relay modules
const unsigned long SERVO_POWER_BEFORE_MS = 5000;
const unsigned long SERVO_POWER_AFTER_MS  = 5000;

// =========================
// Edge correction
// =========================
unsigned long lowEdgeCounter = 0;
unsigned long highEdgeCounter = 0;
const unsigned long EDGE_TIME_LIMIT = 900;

// =========================
// Relay logic
// =========================
const bool RELAY_ACTIVE_LOW = true;

// =========================
// Temperature control - normal mode
// =========================
const float NORMAL_SETPOINT_C = 37.5;
const float NORMAL_ALWAYS_ON_BELOW_C  = 37.3;
const float NORMAL_ALWAYS_OFF_ABOVE_C = 37.7;

const float NORMAL_CORRECTION_HIGH_TEMP_C = 37.7;
const float NORMAL_CORRECTION_LOW_TEMP_C  = 37.3;

// =========================
// Temperature control - hatching mode
// =========================
const float HATCH_SETPOINT_C = 37.3;
const float HATCH_ALWAYS_ON_BELOW_C  = 36.9;
const float HATCH_ALWAYS_OFF_ABOVE_C = 37.5;

const float HATCH_CORRECTION_HIGH_TEMP_C = 37.4;
const float HATCH_CORRECTION_LOW_TEMP_C  = 37.0;

const float EXACT_BAND = 0.0;

const unsigned long HEATER_WINDOW_MS = 100000;

const int DUTY_UNDER_SETPOINT = 35;
const int DUTY_AT_SETPOINT    = 30;
const int DUTY_OVER_SETPOINT  = 25;

// =========================
// Adaptive cumulative correction
// =========================
const int CORRECTION_STEP_DUTY = 5;
const int MAX_CUMULATIVE_CORRECTION = 25;

const unsigned long CORRECTION_INTERVAL_MS = 15UL * 60UL * 1000UL;

int dutyCorrection = 0;
unsigned long lastCorrectionTime = 0;

// =========================
// Humidity control
// =========================
const float NORMAL_HUM_LOW_SETPOINT  = 50.0;
const float NORMAL_HUM_HIGH_SETPOINT = 55.0;

const float HATCH_HUM_LOW_SETPOINT  = 65.0;
const float HATCH_HUM_HIGH_SETPOINT = 70.0;

// =========================
// LED OK limits
// =========================
const float NORMAL_LED_TEMP_LOW  = 37.3;
const float NORMAL_LED_TEMP_HIGH = 37.7;
const float NORMAL_LED_HUM_LOW   = 45.0;
const float NORMAL_LED_HUM_HIGH  = 60.0;

const float HATCH_LED_TEMP_LOW  = 37.0;
const float HATCH_LED_TEMP_HIGH = 37.4;
const float HATCH_LED_HUM_LOW   = 55.0;
const float HATCH_LED_HUM_HIGH  = 75.0;

// =========================
// Sensor validity limits
// =========================
const float SENSOR_TEMP_MIN_C = 10.0;
const float SENSOR_TEMP_MAX_C = 60.0;

// =========================
// Sensor offsets and average selection
// =========================
const float TEMP_OFFSET_1 = 1.3;
const float HUM_OFFSET_1  = 9.0;
const bool USE_SENSOR_1_IN_AVERAGE = false;

const float TEMP_OFFSET_2 = 0.8;
const float HUM_OFFSET_2  = 12.0;
const bool USE_SENSOR_2_IN_AVERAGE = true;

const float TEMP_OFFSET_3 = 0.4;
const float HUM_OFFSET_3  = 10.0;
const bool USE_SENSOR_3_IN_AVERAGE = true;

const float TEMP_OFFSET_4 = -0.4;
const float HUM_OFFSET_4  = -6.0;
const bool USE_SENSOR_4_IN_AVERAGE = false;

// =========================
// Servo settings
// =========================
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_STEP_DELAY_MS = 15;

const unsigned long SERVO_TURN_INTERVAL_MS = 5UL * 60UL * 60UL * 1000UL;

// EEPROM addresses
const int EEPROM_SERVO_ANGLE_ADDR = 0;
const int EEPROM_NORMAL_CORRECTION_ADDR = 1;
const int EEPROM_HATCH_CORRECTION_ADDR  = 2;

// =========================
// Objects
// =========================
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);
DHT dht4(DHTPIN4, DHTTYPE);

Servo servoLeft;
Servo servoRight;

SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

String serialCommandBuffer = "";
String btCommandBuffer = "";

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
// Active mode helper functions
// =========================
float activeSetpoint() {
  return hatchingMode ? HATCH_SETPOINT_C : NORMAL_SETPOINT_C;
}

float activeAlwaysOnBelow() {
  return hatchingMode ? HATCH_ALWAYS_ON_BELOW_C : NORMAL_ALWAYS_ON_BELOW_C;
}

float activeAlwaysOffAbove() {
  return hatchingMode ? HATCH_ALWAYS_OFF_ABOVE_C : NORMAL_ALWAYS_OFF_ABOVE_C;
}

float activeCorrectionHigh() {
  return hatchingMode ? HATCH_CORRECTION_HIGH_TEMP_C : NORMAL_CORRECTION_HIGH_TEMP_C;
}

float activeCorrectionLow() {
  return hatchingMode ? HATCH_CORRECTION_LOW_TEMP_C : NORMAL_CORRECTION_LOW_TEMP_C;
}

float activeHumLow() {
  return hatchingMode ? HATCH_HUM_LOW_SETPOINT : NORMAL_HUM_LOW_SETPOINT;
}

float activeHumHigh() {
  return hatchingMode ? HATCH_HUM_HIGH_SETPOINT : NORMAL_HUM_HIGH_SETPOINT;
}

int activeCorrectionEEPROMAddress() {
  return hatchingMode ? EEPROM_HATCH_CORRECTION_ADDR : EEPROM_NORMAL_CORRECTION_ADDR;
}

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

void setServoPower(bool on) {
  if (SERVO_POWER_ACTIVE_LOW) {
    digitalWrite(SERVO_POWER_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(SERVO_POWER_PIN, on ? HIGH : LOW);
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
// Serial + Bluetooth printing helpers
// =========================
void printBoth(const char *value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(char value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(bool value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(int value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(unsigned long value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(float value) {
  Serial.print(value);
  BT.print(value);
}

void printBoth(float value, int digits) {
  Serial.print(value, digits);
  BT.print(value, digits);
}

void printlnBoth(const char *value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(int value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(char value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(bool value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(unsigned long value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(float value) {
  Serial.println(value);
  BT.println(value);
}

void printlnBoth(float value, int digits) {
  Serial.println(value, digits);
  BT.println(value, digits);
}

// =========================
// EEPROM correction functions
// =========================
void saveCorrectionToEEPROM(int correction) {
  int storedValue = correction + 100;
  EEPROM.update(activeCorrectionEEPROMAddress(), storedValue);
}

int readCorrectionFromEEPROM() {
  int storedValue = EEPROM.read(activeCorrectionEEPROMAddress());
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

  if (temp >= activeCorrectionHigh()) {
    dutyCorrection -= CORRECTION_STEP_DUTY;
    dutyCorrection = clampCorrection(dutyCorrection);
    lastCorrectionTime = now;
  }
  else if (temp <= activeCorrectionLow()) {
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

  if (temp < activeAlwaysOnBelow()) return 100;
  if (temp > activeAlwaysOffAbove()) return 0;

  int baseDuty;

  if (temp < activeSetpoint() - EXACT_BAND) {
    baseDuty = DUTY_UNDER_SETPOINT;
  }
  else if (temp > activeSetpoint() + EXACT_BAND) {
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
  setServoPower(true);
  delay(SERVO_POWER_BEFORE_MS);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  servoLeft.write(currentServoAngle);
  servoRight.write(currentServoAngle);
  delay(500);

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

  delay(SERVO_POWER_AFTER_MS);

  servoLeft.detach();
  servoRight.detach();

  setServoPower(false);
}

void rotateEggs() {
  if (hatchingMode) {
    printlnBoth("Servo: STOPPED (hatching mode)");
    return;
  }

  if (servoAtMax) {
    moveServosSmooth(SERVO_MIN_ANGLE);
    servoAtMax = false;
    printlnBoth("Servo: MIN");
  } else {
    moveServosSmooth(SERVO_MAX_ANGLE);
    servoAtMax = true;
    printlnBoth("Servo: MAX");
  }

  lastServoTurnTime = millis();
}

void switchMode(bool hatchMode) {
  if (hatchingMode == hatchMode) {
    printBoth("Mode already ");
    printlnBoth(hatchingMode ? "HATCHING" : "NORMAL");
    return;
  }

  saveCorrectionToEEPROM(dutyCorrection);

  hatchingMode = hatchMode;
  dutyCorrection = readCorrectionFromEEPROM();

  lowEdgeCounter = 0;
  highEdgeCounter = 0;
  lastCorrectionTime = millis();
  lastServoTurnTime = millis();

  printBoth("Mode changed to ");
  printBoth(hatchingMode ? "HATCHING" : "NORMAL");
  printBoth(" | Corr=");
  printlnBoth(dutyCorrection);
}

bool handleModeCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return false;
  }

  if (command.equalsIgnoreCase("Hatch")) {
    switchMode(true);
    return true;
  }

  if (command.equalsIgnoreCase("Normal")) {
    switchMode(false);
    return true;
  }

  return false;
}

void handleServoCommand(char c, bool &correctionChanged) {
  if (c == 'r' || c == 'R') {
    rotateEggs();
  }
  else if (c == '+') {
    dutyCorrection += CORRECTION_STEP_DUTY;
    dutyCorrection = clampCorrection(dutyCorrection);
    correctionChanged = true;
  }
  else if (c == '-') {
    dutyCorrection -= CORRECTION_STEP_DUTY;
    dutyCorrection = clampCorrection(dutyCorrection);
    correctionChanged = true;
  }
}

void handleIncomingChar(char c, String &commandBuffer, bool &correctionChanged) {
  if (isAlpha(c)) {
    commandBuffer += c;

    if (handleModeCommand(commandBuffer)) {
      commandBuffer = "";
    }

    return;
  }

  if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
    handleModeCommand(commandBuffer);
    commandBuffer = "";
    return;
  }

  commandBuffer = "";
  handleServoCommand(c, correctionChanged);
}

void checkServo() {
  bool correctionChanged = false;

  while (Serial.available()) {
    char c = Serial.read();
    handleIncomingChar(c, serialCommandBuffer, correctionChanged);
  }

  while (BT.available()) {
    char c = BT.read();
    handleIncomingChar(c, btCommandBuffer, correctionChanged);
  }

  if (correctionChanged) {
    saveCorrectionToEEPROM(dutyCorrection);

    printBoth("Manual Corr=");
    printlnBoth(dutyCorrection);
  }

  if (!hatchingMode &&
      millis() - lastServoTurnTime >= SERVO_TURN_INTERVAL_MS) {
    rotateEggs();
  }
}

void updateStatusLED(float controlTemp, float usedHum) {
  bool systemOK;

  if (hatchingMode) {
    systemOK =
      (controlTemp >= HATCH_LED_TEMP_LOW && controlTemp <= HATCH_LED_TEMP_HIGH) &&
      (usedHum >= HATCH_LED_HUM_LOW && usedHum <= HATCH_LED_HUM_HIGH);
  } else {
    systemOK =
      (controlTemp >= NORMAL_LED_TEMP_LOW && controlTemp <= NORMAL_LED_TEMP_HIGH) &&
      (usedHum >= NORMAL_LED_HUM_LOW && usedHum <= NORMAL_LED_HUM_HIGH);
  }

  digitalWrite(STATUS_LED_PIN, systemOK ? LOW : HIGH);
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  dht1.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();

  pinMode(RELAY_HEATER, OUTPUT);
  pinMode(RELAY_HUMIDITY_FAN, OUTPUT);

  setRelay(RELAY_HEATER, false);
  setRelay(RELAY_HUMIDITY_FAN, false);

  pinMode(SERVO_POWER_PIN, OUTPUT);
  setServoPower(false);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  dutyCorrection = readCorrectionFromEEPROM();

  int savedServoAngle = readServoAngleFromEEPROM();

  currentServoAngle = savedServoAngle;
  servoAtMax = (savedServoAngle == SERVO_MAX_ANGLE);

  lastServoTurnTime = millis();
  lastCorrectionTime = millis();

  printBoth("System started | Mode=");
  printBoth(hatchingMode ? "HATCHING" : "NORMAL");
  printBoth(" | Saved Corr=");
  printBoth(dutyCorrection);
  printBoth(" | Saved Servo=");
  printlnBoth(currentServoAngle);
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

  float h3 = dht3.readHumidity();
  float t3 = dht3.readTemperature();

  float h4 = dht4.readHumidity();
  float t4 = dht4.readTemperature();

  h1 += HUM_OFFSET_1;
  t1 += TEMP_OFFSET_1;

  h2 += HUM_OFFSET_2;
  t2 += TEMP_OFFSET_2;

  h3 += HUM_OFFSET_3;
  t3 += TEMP_OFFSET_3;

  h4 += HUM_OFFSET_4;
  t4 += TEMP_OFFSET_4;

  bool ok1 = isSensorValid(t1, h1);
  bool ok2 = isSensorValid(t2, h2);
  bool ok3 = isSensorValid(t3, h3);
  bool ok4 = isSensorValid(t4, h4);

  float usedTemp = 0;
  float usedHum  = 0;
  int usedCount = 0;

  bool use1 = ok1 && USE_SENSOR_1_IN_AVERAGE;
  bool use2 = ok2 && USE_SENSOR_2_IN_AVERAGE;
  bool use3 = ok3 && USE_SENSOR_3_IN_AVERAGE;
  bool use4 = ok4 && USE_SENSOR_4_IN_AVERAGE;

  if (use1) {
    usedTemp += t1;
    usedHum  += h1;
    usedCount++;
  }

  if (use2) {
    usedTemp += t2;
    usedHum  += h2;
    usedCount++;
  }

  if (use3) {
    usedTemp += t3;
    usedHum  += h3;
    usedCount++;
  }

  if (use4) {
    usedTemp += t4;
    usedHum  += h4;
    usedCount++;
  }

  bool anyUsedSensor = (usedCount > 0);

  if (anyUsedSensor) {
    usedTemp /= usedCount;
    usedHum  /= usedCount;

    float controlTemp = round1(usedTemp);

    if (controlTemp == 37.4) {
      lowEdgeCounter++;
    } else {
      lowEdgeCounter = 0;
    }

    if (controlTemp == 37.6) {
      highEdgeCounter++;
    } else {
      highEdgeCounter = 0;
    }

    if (lowEdgeCounter >= EDGE_TIME_LIMIT) {
      dutyCorrection += CORRECTION_STEP_DUTY;
      dutyCorrection = clampCorrection(dutyCorrection);
      saveCorrectionToEEPROM(dutyCorrection);
      lowEdgeCounter = 0;
      printlnBoth("Low edge correction applied");
    }

    if (highEdgeCounter >= EDGE_TIME_LIMIT) {
      dutyCorrection -= CORRECTION_STEP_DUTY;
      dutyCorrection = clampCorrection(dutyCorrection);
      saveCorrectionToEEPROM(dutyCorrection);
      highEdgeCounter = 0;
      printlnBoth("High edge correction applied");
    }

    heaterDuty = calculateHeaterDuty(controlTemp);

    applyHeaterPulseControl(heaterDuty);

    if (!humidityFanOn && usedHum < activeHumLow()) {
      humidityFanOn = true;
      setRelay(RELAY_HUMIDITY_FAN, true);
    }
    else if (humidityFanOn && usedHum > activeHumHigh()) {
      humidityFanOn = false;
      setRelay(RELAY_HUMIDITY_FAN, false);
    }

    updateStatusLED(controlTemp, usedHum);

    float maxTemp = -100.0;
    float minTemp = 100.0;

    if (use1) {
      maxTemp = max(maxTemp, t1);
      minTemp = min(minTemp, t1);
    }

    if (use2) {
      maxTemp = max(maxTemp, t2);
      minTemp = min(minTemp, t2);
    }

    if (use3) {
      maxTemp = max(maxTemp, t3);
      minTemp = min(minTemp, t3);
    }

    if (use4) {
      maxTemp = max(maxTemp, t4);
      minTemp = min(minTemp, t4);
    }

    float tempSpread = maxTemp - minTemp;

    int rotationRemainingMin = 0;

    if (!hatchingMode) {
      unsigned long elapsedRotationMs = millis() - lastServoTurnTime;

      if (elapsedRotationMs < SERVO_TURN_INTERVAL_MS) {
        rotationRemainingMin = (int)((SERVO_TURN_INTERVAL_MS - elapsedRotationMs) / 60000UL);
      }
    }

    printBoth("S1: ");
    if (ok1) {
      printBoth(t1, 1);
      printBoth("C ");
      printBoth(h1, 1);
      printBoth("% ");
      printBoth(USE_SENSOR_1_IN_AVERAGE ? "A" : "P");
    } else {
      printBoth("ERR");
    }

    printBoth(" | S2: ");
    if (ok2) {
      printBoth(t2, 1);
      printBoth("C ");
      printBoth(h2, 1);
      printBoth("% ");
      printBoth(USE_SENSOR_2_IN_AVERAGE ? "A" : "P");
    } else {
      printBoth("ERR");
    }

    printBoth(" | S3: ");
    if (ok3) {
      printBoth(t3, 1);
      printBoth("C ");
      printBoth(h3, 1);
      printBoth("% ");
      printBoth(USE_SENSOR_3_IN_AVERAGE ? "A" : "P");
    } else {
      printBoth("ERR");
    }

    printBoth(" | S4: ");
    if (ok4) {
      printBoth(t4, 1);
      printBoth("C ");
      printBoth(h4, 1);
      printBoth("% ");
      printBoth(USE_SENSOR_4_IN_AVERAGE ? "A" : "P");
    } else {
      printBoth("ERR");
    }

    printBoth(" | T=");
    printBoth(controlTemp, 1);

    printBoth(" | H=");
    printBoth(usedHum, 1);

    printBoth(" | Duty=");
    printBoth(heaterDuty);

    printBoth(" | Corr=");
    printBoth(dutyCorrection);

    printBoth(" | Htr=");
    printBoth(heaterOn);

    printBoth(" | HFan=");
    printBoth(humidityFanOn);

    printBoth(" | Servo=");
    if (hatchingMode) {
      printBoth("STOPPED");
    } else {
      printBoth(servoAtMax ? "MAX" : "MIN");
    }

    printBoth(" | Mod=");
    printBoth(hatchingMode ? "HTCH" : "NRML");

    printBoth(" | Sprd=");
    printBoth(tempSpread, 1);

    printBoth(" | AvgSens=");
    printBoth(usedCount);

    printBoth(" | RotMin=");
    if (hatchingMode) {
      printBoth("STOP");
    } else {
      printBoth(rotationRemainingMin);
    }

    printBoth(" | LEC=");
    printBoth(lowEdgeCounter);

    printBoth(" | HEC=");
    printlnBoth(highEdgeCounter);
  }
  else {
    heaterOn = false;
    humidityFanOn = false;
    heaterDuty = 0;

    setRelay(RELAY_HEATER, false);
    setRelay(RELAY_HUMIDITY_FAN, false);

    digitalWrite(STATUS_LED_PIN, HIGH);

    printlnBoth("Heater: OFF | HumFan: OFF | LED: ON (no sensors selected for average/control)");
  }

  delay(2000);
}