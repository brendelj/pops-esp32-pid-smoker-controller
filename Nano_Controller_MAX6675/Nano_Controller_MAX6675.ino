/*
  Pop's PID Smoker Controller - Nano Controller

  Split-project architecture:
    - Arduino Nano owns real-time smoker control and safety.
    - ESP32 companion owns Wi-Fi, web UI, SD logging, and remote commands.

  Hardware target:
    - Arduino Nano / ATmega328P
    - MAX6675 thermocouple module for pit/chamber temperature
    - Optional DS18B20 meat probes on one OneWire bus
    - 20x4 I2C LCD
    - Rotary encoder + push button
    - Heat relay + heat PWM/SSR output
    - Cooling servo/damper output
    - UART serial protocol to ESP32 companion bridge

  Serial protocol to ESP32, newline terminated:
    Nano -> ESP32 telemetry:
      {"tmpC":107.2,"tgtC":107.2,"p1C":66.1,"p2C":0,"heatPct":45,"coolPct":0,"servoDeg":0,"state":"RUN","alarm":"OK"}

    ESP32 -> Nano commands:
      {"cmd":"start"}
      {"cmd":"stop"}
      {"cmd":"set","tgtC":107.2}
      {"cmd":"set","tgtF":225}
      {"cmd":"pid","kp":6.0,"ki":0.08,"kd":15.0}
      {"cmd":"save"}
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <max6675.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------------- Pinout ----------------
const uint8_t PIN_ONEWIRE      = 2;
const uint8_t PIN_HEAT_RELAY   = 4;
const uint8_t PIN_HEAT_PWM     = 5;
const uint8_t PIN_MAX6675_CS   = 6;
const uint8_t PIN_COOL_SERVO   = 9;
const uint8_t PIN_MAX6675_SO   = 12;
const uint8_t PIN_MAX6675_SCK  = 13;
const uint8_t PIN_ENC_CLK      = A0;
const uint8_t PIN_ENC_DT       = A1;
const uint8_t PIN_ENC_SW       = A2;

// ---------------- Hardware ----------------
LiquidCrystal_I2C lcd(0x27, 20, 4);
MAX6675 pitThermo(PIN_MAX6675_SCK, PIN_MAX6675_CS, PIN_MAX6675_SO);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature dallas(&oneWire);
Servo coolServo;

// ---------------- Config ----------------
const uint32_t CFG_MAGIC = 0x504F5053UL; // POPS
const uint16_t CFG_VER   = 1;

struct Config {
  uint32_t magic;
  uint16_t version;
  float targetC;
  float alarmHighC;
  float coolStartC;
  float kp;
  float ki;
  float kd;
  uint8_t minHeatPct;
  uint8_t maxHeatPct;
  uint8_t servoClosedDeg;
  uint8_t servoOpenDeg;
  uint16_t crc;
};

Config cfg;

// ---------------- State ----------------
bool running = false;
bool sensorOk = false;
char alarmText[12] = "OK";
float pitC = NAN;
float pitFilteredC = NAN;
float probe1C = NAN;
float probe2C = NAN;
float pidI = 0;
float lastErr = 0;
uint8_t heatPct = 0;
uint8_t coolPct = 0;
uint8_t servoDeg = 0;

unsigned long lastSensorMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastLcdMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastButtonMs = 0;
long lastEncState = 0;
int menuIndex = 0;

const unsigned long SENSOR_PERIOD_MS    = 1000;
const unsigned long CONTROL_PERIOD_MS   = 1000;
const unsigned long LCD_PERIOD_MS       = 500;
const unsigned long TELEMETRY_PERIOD_MS = 1000;

// ---------------- Utilities ----------------
float cToF(float c) { return c * 9.0f / 5.0f + 32.0f; }
float fToC(float f) { return (f - 32.0f) * 5.0f / 9.0f; }

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

uint16_t configCrc(const Config& c) {
  Config tmp = c;
  tmp.crc = 0;
  return crc16((const uint8_t*)&tmp, sizeof(Config));
}

void defaultConfig() {
  cfg.magic = CFG_MAGIC;
  cfg.version = CFG_VER;
  cfg.targetC = fToC(225);
  cfg.alarmHighC = fToC(325);
  cfg.coolStartC = fToC(240);
  cfg.kp = 6.0;
  cfg.ki = 0.08;
  cfg.kd = 15.0;
  cfg.minHeatPct = 0;
  cfg.maxHeatPct = 100;
  cfg.servoClosedDeg = 0;
  cfg.servoOpenDeg = 90;
  cfg.crc = configCrc(cfg);
}

void saveConfig() {
  cfg.crc = configCrc(cfg);
  EEPROM.put(0, cfg);
}

void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC || cfg.version != CFG_VER || cfg.crc != configCrc(cfg)) {
    defaultConfig();
    saveConfig();
  }
}

void allOutputsOff() {
  heatPct = 0;
  coolPct = 0;
  digitalWrite(PIN_HEAT_RELAY, LOW);
  analogWrite(PIN_HEAT_PWM, 0);
  servoDeg = cfg.servoClosedDeg;
  coolServo.write(servoDeg);
}

void setAlarm(const char* txt) {
  strncpy(alarmText, txt, sizeof(alarmText) - 1);
  alarmText[sizeof(alarmText) - 1] = '\0';
}

// ---------------- Sensors ----------------
void readSensors() {
  float c = pitThermo.readCelsius();
  sensorOk = !isnan(c) && c > -20 && c < 500;

  if (sensorOk) {
    pitC = c;
    if (isnan(pitFilteredC)) pitFilteredC = pitC;
    pitFilteredC = pitFilteredC * 0.75f + pitC * 0.25f;
  }

  dallas.requestTemperatures();
  probe1C = dallas.getTempCByIndex(0);
  probe2C = dallas.getTempCByIndex(1);
  if (probe1C <= -100) probe1C = NAN;
  if (probe2C <= -100) probe2C = NAN;
}

// ---------------- Control ----------------
void controlLoop() {
  if (!running) {
    setAlarm("STOP");
    allOutputsOff();
    return;
  }

  if (!sensorOk || isnan(pitFilteredC)) {
    setAlarm("SENSOR");
    running = false;
    allOutputsOff();
    return;
  }

  if (pitFilteredC >= cfg.alarmHighC) {
    setAlarm("HIGH");
    running = false;
    allOutputsOff();
    return;
  }

  setAlarm("OK");

  float err = cfg.targetC - pitFilteredC;
  float dt = CONTROL_PERIOD_MS / 1000.0f;
  pidI += err * dt;
  pidI = constrain(pidI, -200.0f, 200.0f);
  float d = (err - lastErr) / dt;
  lastErr = err;

  float out = cfg.kp * err + cfg.ki * pidI + cfg.kd * d;
  out = constrain(out, 0.0f, 100.0f);
  heatPct = constrain((uint8_t)out, cfg.minHeatPct, cfg.maxHeatPct);

  if (pitFilteredC > cfg.coolStartC) {
    float coolSpan = max(1.0f, cfg.coolStartC - cfg.targetC + 10.0f);
    coolPct = constrain((uint8_t)(((pitFilteredC - cfg.coolStartC) / coolSpan) * 100.0f), 0, 100);
  } else {
    coolPct = 0;
  }

  digitalWrite(PIN_HEAT_RELAY, heatPct > 0 ? HIGH : LOW);
  analogWrite(PIN_HEAT_PWM, map(heatPct, 0, 100, 0, 255));

  servoDeg = map(coolPct, 0, 100, cfg.servoClosedDeg, cfg.servoOpenDeg);
  coolServo.write(servoDeg);
}

// ---------------- Menu ----------------
void handleEncoder() {
  int clk = digitalRead(PIN_ENC_CLK);
  int dt  = digitalRead(PIN_ENC_DT);
  long state = (clk << 1) | dt;
  if (state != lastEncState) {
    if (clk == LOW && dt == HIGH) menuIndex++;
    if (clk == HIGH && dt == LOW) menuIndex--;
    menuIndex = constrain(menuIndex, 0, 4);
    lastEncState = state;
  }

  if (digitalRead(PIN_ENC_SW) == LOW && millis() - lastButtonMs > 350) {
    lastButtonMs = millis();
    switch (menuIndex) {
      case 0: running = !running; break;
      case 1: cfg.targetC += fToC(5) - fToC(0); saveConfig(); break;
      case 2: cfg.targetC -= fToC(5) - fToC(0); saveConfig(); break;
      case 3: cfg.kp += 0.5f; saveConfig(); break;
      case 4: defaultConfig(); saveConfig(); break;
    }
  }
}

// ---------------- Serial Protocol ----------------
String serialIn;

bool findNumber(const String& s, const char* key, float& out) {
  String k = String("\"") + key + "\":";
  int p = s.indexOf(k);
  if (p < 0) return false;
  p += k.length();
  out = s.substring(p).toFloat();
  return true;
}

void processCommand(const String& s) {
  if (s.indexOf("\"cmd\":\"start\"") >= 0) {
    running = true;
    pidI = 0;
    lastErr = 0;
  } else if (s.indexOf("\"cmd\":\"stop\"") >= 0) {
    running = false;
  } else if (s.indexOf("\"cmd\":\"save\"") >= 0) {
    saveConfig();
  } else if (s.indexOf("\"cmd\":\"set\"") >= 0) {
    float v;
    if (findNumber(s, "tgtC", v)) cfg.targetC = v;
    if (findNumber(s, "tgtF", v)) cfg.targetC = fToC(v);
    saveConfig();
  } else if (s.indexOf("\"cmd\":\"pid\"") >= 0) {
    float v;
    if (findNumber(s, "kp", v)) cfg.kp = v;
    if (findNumber(s, "ki", v)) cfg.ki = v;
    if (findNumber(s, "kd", v)) cfg.kd = v;
    saveConfig();
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialIn.trim();
      if (serialIn.length()) processCommand(serialIn);
      serialIn = "";
    } else if (serialIn.length() < 160) {
      serialIn += c;
    }
  }
}

void sendTelemetry() {
  Serial.print(F("{\"tmpC\":")); Serial.print(isnan(pitFilteredC) ? 0 : pitFilteredC, 1);
  Serial.print(F(",\"tgtC\":")); Serial.print(cfg.targetC, 1);
  Serial.print(F(",\"p1C\":")); Serial.print(isnan(probe1C) ? 0 : probe1C, 1);
  Serial.print(F(",\"p2C\":")); Serial.print(isnan(probe2C) ? 0 : probe2C, 1);
  Serial.print(F(",\"heatPct\":")); Serial.print(heatPct);
  Serial.print(F(",\"coolPct\":")); Serial.print(coolPct);
  Serial.print(F(",\"servoDeg\":")); Serial.print(servoDeg);
  Serial.print(F(",\"state\":\"")); Serial.print(running ? F("RUN") : F("STOP"));
  Serial.print(F("\",\"alarm\":\"")); Serial.print(alarmText);
  Serial.println(F("\"}"));
}

// ---------------- LCD ----------------
void drawLcd() {
  lcd.setCursor(0, 0);
  lcd.print(F("Pop PID "));
  lcd.print(running ? F("RUN ") : F("STOP"));
  lcd.print(F(" "));
  lcd.print(alarmText);
  lcd.print(F("      "));

  lcd.setCursor(0, 1);
  lcd.print(F("Pit:")); lcd.print(isnan(pitFilteredC) ? 0 : cToF(pitFilteredC), 0);
  lcd.print(F("F Tgt:")); lcd.print(cToF(cfg.targetC), 0); lcd.print(F("F   "));

  lcd.setCursor(0, 2);
  lcd.print(F("Heat:")); lcd.print(heatPct); lcd.print(F("% Cool:")); lcd.print(coolPct); lcd.print(F("%   "));

  lcd.setCursor(0, 3);
  switch (menuIndex) {
    case 0: lcd.print(F(">Start/Stop        ")); break;
    case 1: lcd.print(F(">Target +5F        ")); break;
    case 2: lcd.print(F(">Target -5F        ")); break;
    case 3: lcd.print(F(">KP +0.5           ")); break;
    case 4: lcd.print(F(">Reset Config      ")); break;
  }
}

void setup() {
  pinMode(PIN_HEAT_RELAY, OUTPUT);
  pinMode(PIN_HEAT_PWM, OUTPUT);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Pop's PID Nano"));

  loadConfig();
  dallas.begin();
  coolServo.attach(PIN_COOL_SERVO);
  allOutputsOff();
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  readSerialCommands();
  handleEncoder();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    readSensors();
  }
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    controlLoop();
  }
  if (now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    drawLcd();
  }
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
