#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <math.h>

// ============================================================
// WIFI & SUPABASE CREDENTIALS
// ============================================================
const char* WIFI_SSID = "Akhi";
const char* WIFI_PASS = "12345678";

const char* SUPABASE_BASE = "https://ezkwsajhchuwpgumycab.supabase.co/rest/v1";
const char* SUPABASE_KEY  = "sb_publishable_y3fPGyRJwKoSjabqLmj2VA_OjTSWli_";
const char* DEVICE_ID     = "LUBRITORQUE_01";

// ============================================================
// HARDWARE PINS & PRE-SET 360° ENCODER CALIBRATION
// ============================================================
#define RPWM 25
#define LPWM 26
#define R_EN 27
#define L_EN 14
#define MOTOR_PWM 38

#define ENCODER_A 18
#define ENCODER_B 19
#define RPM_SEGMENTS 36
#define MAX_RUN_TIMEOUT_MS 10000UL

#define DEFAULT_QUAD_CPR      3125.0f
#define DEFAULT_SINGLE_CH_CPR 1893.0f

#define INA226_I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(INA226_I2C_ADDRESS);
bool ina226Available = false;

Preferences preferences;
float calibratedQuadCPR     = DEFAULT_QUAD_CPR;
float calibratedSingleChCPR = DEFAULT_SINGLE_CH_CPR;

SemaphoreHandle_t httpMutex = NULL;

// ============================================================
// ENCODER INTERRUPT
// ============================================================
volatile int32_t encoderCount = 0;
volatile uint32_t edgesA = 0;
volatile uint32_t edgesB = 0;
volatile uint8_t encoderLastState = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

const int8_t encoderTable[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(ENCODER_A);
  uint8_t b = digitalRead(ENCODER_B);
  uint8_t prevA = (encoderLastState >> 1) & 0x01;
  uint8_t prevB = encoderLastState & 0x01;

  uint8_t currentState = (a << 1) | b;
  uint8_t transition = (encoderLastState << 2) | currentState;
  int8_t movement = encoderTable[transition];

  portENTER_CRITICAL_ISR(&encoderMux);
  if (a != prevA) edgesA++;
  if (b != prevB) edgesB++;
  if (movement != 0) encoderCount += movement;
  portEXIT_CRITICAL_ISR(&encoderMux);

  encoderLastState = currentState;
}

void getEncoderStats(int32_t &quadCount, uint32_t &cntA, uint32_t &cntB) {
  portENTER_CRITICAL(&encoderMux);
  quadCount = encoderCount;
  cntA = edgesA;
  cntB = edgesB;
  portEXIT_CRITICAL(&encoderMux);
}

void resetEncoder() {
  portENTER_CRITICAL(&encoderMux);
  encoderCount = 0;
  edgesA = 0;
  edgesB = 0;
  encoderLastState = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  portEXIT_CRITICAL(&encoderMux);
}

void motorStart() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(LPWM, 0);
  analogWrite(RPWM, MOTOR_PWM);
}

void motorBrakeStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  delay(40);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
}

// ============================================================
// MEASUREMENT & LIVE STREAMING STATE
// ============================================================
volatile bool measurementRunning = false;
bool manualCalRunning = false;
bool offlineLocalTest = false;
unsigned long commandStartMillis = 0;
unsigned long motorStartMicros = 0;
unsigned long motorStopMicros = 0;

float instTimeSec[RPM_SEGMENTS];
float instRpmProfile[RPM_SEGMENTS];
uint8_t nextSegmentIdx = 0;
unsigned long lastSegmentMicros = 0;
uint32_t lastSegmentTicks = 0;

volatile float liveCurrent = 0.0f;
volatile float liveVoltage = 0.0f;
volatile float livePower   = 0.0f;
volatile float liveRpm     = 0.0f;

double currentSum = 0.0;
double voltageSum = 0.0;
double powerSum = 0.0;
uint32_t sampleCount = 0;

String activeMode = "TEST_UNKNOWN";
String activeSampleName = "Unknown_Sample";

// ============================================================
// THREAD-SAFE SUPABASE HTTP HELPERS
// ============================================================
int sendSupabaseRequest(String endpoint, String method, String jsonPayload, bool isBackgroundStream = false) {
  if (WiFi.status() != WL_CONNECTED) return -1;

  TickType_t waitTicks = isBackgroundStream ? pdMS_TO_TICKS(10) : pdMS_TO_TICKS(4000);
  if (httpMutex != NULL && xSemaphoreTake(httpMutex, waitTicks) != pdTRUE) {
    return -2;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(5);

  HTTPClient http;
  http.setReuse(false);
  http.setTimeout(5000);

  String url = String(SUPABASE_BASE) + endpoint;
  int code = -1;
  if (http.begin(client, url)) {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
    http.addHeader("Prefer", "return=minimal");

    code = (method == "POST") ? http.POST(jsonPayload) : http.PATCH(jsonPayload);
    if (!isBackgroundStream && (code < 200 || code >= 300)) {
      String errBody = http.getString();
      Serial.printf("Supabase %s %s failed (%d): %s\n",
                    method.c_str(), endpoint.c_str(), code, errBody.c_str());
    }
    http.end();
  }

  if (httpMutex != NULL) xSemaphoreGive(httpMutex);
  return code;
}

int updateDeviceTelemetry(String cmdOverride = "", bool isBackgroundStream = false) {
  JsonDocument doc;
  if (cmdOverride != "") doc["command"] = cmdOverride;
  doc["motor_running"] = (bool)measurementRunning;
  doc["phase"] = measurementRunning ? "1-REV SCAN" : "READY";
  doc["live_voltage"] = (float)liveVoltage;
  doc["live_current"] = (float)liveCurrent;
  doc["live_power"]   = (float)livePower;

  String payload;
  serializeJson(doc, payload);
  return sendSupabaseRequest("/device_control?device_id=eq." + String(DEVICE_ID), "PATCH", payload, isBackgroundStream);
}

void liveTelemetryTask(void *pvParameters) {
  for (;;) {
    if (measurementRunning && !offlineLocalTest && WiFi.status() == WL_CONNECTED) {
      updateDeviceTelemetry("", true);
      vTaskDelay(pdMS_TO_TICKS(350));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void logRotationRun(String name, float revTime, float rpm, float curr, float volt, float pwr) {
  JsonDocument doc;
  doc["device_id"] = DEVICE_ID;
  doc["sample_name"] = name;
  doc["mode_type"] = "1-Rev Scan";
  doc["rev_time"] = revTime;
  doc["rpm"] = rpm;
  doc["current_avg"] = curr;
  doc["voltage"] = volt;
  doc["power"] = pwr;
  doc["viscosity"] = 0.0f;
  doc["degradation_score"] = 0.0f;
  doc["status"] = "PENDING";

  JsonArray curve = doc["rpm_curve"].to<JsonArray>();
  for (uint8_t i = 0; i < RPM_SEGMENTS; i++) {
    JsonObject pt = curve.add<JsonObject>();
    pt["t"] = roundf(instTimeSec[i] * 1000.0f) / 1000.0f;
    pt["rpm"] = roundf(instRpmProfile[i] * 100.0f) / 100.0f;
  }

  String payload;
  serializeJson(doc, payload);
  int status = sendSupabaseRequest("/lubritorque_logs", "POST", payload, false);

  if (status == 400) {
    JsonDocument fallbackDoc;
    fallbackDoc["device_id"] = DEVICE_ID;
    fallbackDoc["sample_name"] = name;
    fallbackDoc["mode_type"] = "1-Rev Scan";
    fallbackDoc["rev_time"] = revTime;
    fallbackDoc["rpm"] = rpm;
    fallbackDoc["current_avg"] = curr;
    fallbackDoc["voltage"] = volt;
    fallbackDoc["power"] = pwr;
    fallbackDoc["viscosity"] = 0.0f;
    fallbackDoc["degradation_score"] = 0.0f;
    fallbackDoc["status"] = "PENDING";

    String fallbackPayload;
    serializeJson(fallbackDoc, fallbackPayload);
    sendSupabaseRequest("/lubritorque_logs", "POST", fallbackPayload, false);
  }
}

// ============================================================
// INA226 AUTO-SCANNER & HIGH-SPEED SAMPLING (CORE 1)
// ============================================================
bool initINA226Sensor() {
  for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      ina226 = INA226_WE(addr);
      if (ina226.init()) {
        ina226.setAverage(INA226_AVERAGE_4);
        ina226.setConversionTime(INA226_CONV_TIME_588);
        ina226.setResistorRange(0.1, 1.3);
        ina226Available = true;
        Serial.printf("[INA226] Connected at I2C 0x%02X\n", addr);
        return true;
      }
    }
  }
  ina226Available = false;
  return false;
}

void updateINA226() {
  static unsigned long lastSampleTime = 0;
  static unsigned long lastRetryTime = 0;

  if (!ina226Available) {
    if (!measurementRunning && (millis() - lastRetryTime >= 2000)) {
      lastRetryTime = millis();
      initINA226Sensor();
    }
    return;
  }

  if (millis() - lastSampleTime < 5) return;
  lastSampleTime = millis();

  float v = ina226.getBusVoltage_V();
  float c = fabsf(ina226.getCurrent_mA() / 1000.0f);

  if (c <= 0.0001f) {
    float shunt_mV = fabsf(ina226.getShuntVoltage_mV());
    if (shunt_mV > 0.05f) {
      c = shunt_mV / 100.0f;
    }
  }

  float p = ina226.getBusPower() / 1000.0f;
  if (p <= 0.0001f && c > 0.0001f) p = v * c;

  liveVoltage = v;
  liveCurrent = c;
  livePower   = p;

  if (measurementRunning) {
    currentSum += (double)c;
    voltageSum += (double)v;
    powerSum += (double)p;
    sampleCount++;
  }
}

// ============================================================
// 1-ROTATION STATE MACHINE (MOTOR START TO MOTOR STOP)
// ============================================================
void startMeasurement(String mode, String sampleName, bool isLocalSerialTest = false) {
  if (measurementRunning || manualCalRunning) return;

  activeMode = mode;
  activeSampleName = sampleName;
  offlineLocalTest = isLocalSerialTest;

  resetEncoder();
  currentSum = 0.0;
  voltageSum = 0.0;
  powerSum = 0.0;
  sampleCount = 0;
  nextSegmentIdx = 0;
  liveRpm = 0.0f;
  for (uint8_t i = 0; i < RPM_SEGMENTS; i++) {
    instTimeSec[i] = 0.0f;
    instRpmProfile[i] = 0.0f;
  }

  commandStartMillis = millis();
  measurementRunning = true;

  Serial.printf("\n[START] Mode=%s | Sample=%s | BusV=%.2fV | PWM=%d\n",
                mode.c_str(), sampleName.c_str(), (float)liveVoltage, MOTOR_PWM);

  if (!offlineLocalTest) {
    updateDeviceTelemetry("RUNNING", false);
  }

  motorStartMicros = micros();
  lastSegmentMicros = motorStartMicros;
  lastSegmentTicks = 0;
  motorStart();
}

void finishMeasurement(bool usedSingleChannel, uint32_t finalTicks, float targetCPR) {
  motorBrakeStop();
  motorStopMicros = micros();
  measurementRunning = false;

  unsigned long elapsedMicros = motorStopMicros - motorStartMicros;
  float revTime = (float)elapsedMicros / 1000000.0f;
  if (revTime <= 0.0001f) revTime = 0.0001f;

  float rpm = 60.0f / revTime;
  float avgCurrent = (sampleCount > 0) ? (float)(currentSum / (double)sampleCount) : (float)liveCurrent;
  float avgVoltage = (sampleCount > 0) ? (float)(voltageSum / (double)sampleCount) : (float)liveVoltage;
  float avgPower   = (sampleCount > 0) ? (float)(powerSum / (double)sampleCount)   : (avgCurrent * avgVoltage);

  liveCurrent = avgCurrent;
  liveVoltage = avgVoltage;
  livePower   = avgPower;
  liveRpm     = rpm;

  Serial.printf("[1-Rev Done] %s | Ticks:%lu/%.0f | MotorTime: %.4f s | Avg RPM: %.2f | V:%.2fV I:%.4fA P:%.2fW\n",
                usedSingleChannel ? "Single-Ch" : "Quad A+B",
                (unsigned long)finalTicks, targetCPR, revTime, rpm,
                avgVoltage, avgCurrent, avgPower);

  if (!offlineLocalTest) {
    logRotationRun(activeSampleName, revTime, rpm, avgCurrent, avgVoltage, avgPower);
    updateDeviceTelemetry("IDLE", false);
  }
  offlineLocalTest = false;
}

void updateMeasurement() {
  if (!measurementRunning) return;

  int32_t quadCount = 0;
  uint32_t cntA = 0, cntB = 0;
  getEncoderStats(quadCount, cntA, cntB);

  bool singleChannelMode = (cntA == 0 && cntB > 5) || (cntB == 0 && cntA > 5);
  uint32_t activeTicks = singleChannelMode ? (cntA + cntB) : (uint32_t)abs(quadCount);
  float targetCPR = singleChannelMode ? calibratedSingleChCPR : calibratedQuadCPR;

  unsigned long nowUs = micros();

  if (millis() - commandStartMillis > MAX_RUN_TIMEOUT_MS) {
    motorBrakeStop();
    measurementRunning = false;
    Serial.printf("[TIMEOUT STOP] Quad:%ld | EdgesA:%lu | EdgesB:%lu | BusV:%.2fV\n",
                  (long)quadCount, (unsigned long)cntA, (unsigned long)cntB, (float)liveVoltage);
    if (cntA == 0 && cntB == 0) {
      Serial.println("  -> HINT: 0 encoder edges detected. Check 12V motor power switch & BTS7960/encoder cables!");
    }
    if (!offlineLocalTest) {
      updateDeviceTelemetry("IDLE", false);
    }
    offlineLocalTest = false;
    return;
  }

  while (nextSegmentIdx < RPM_SEGMENTS) {
    uint32_t targetTicks = (uint32_t)roundf(((float)(nextSegmentIdx + 1) / (float)RPM_SEGMENTS) * targetCPR);
    if (targetTicks < 1) targetTicks = 1;
    if (activeTicks >= targetTicks) {
      unsigned long dtUs = nowUs - lastSegmentMicros;
      uint32_t dTicks = activeTicks - lastSegmentTicks;
      float instRpm = (dtUs > 0) ? (((float)dTicks / targetCPR) * (60000000.0f / (float)dtUs)) : 0.0f;
      instTimeSec[nextSegmentIdx] = (float)(nowUs - motorStartMicros) / 1000000.0f;
      instRpmProfile[nextSegmentIdx] = instRpm;
      liveRpm = instRpm;
      lastSegmentMicros = nowUs;
      lastSegmentTicks = activeTicks;
      nextSegmentIdx++;
    } else {
      break;
    }
  }

  if (activeTicks >= (uint32_t)roundf(targetCPR)) {
    finishMeasurement(singleChannelMode, activeTicks, targetCPR);
  }
}

// ============================================================
// SERIAL COMMANDS ("TEST", "SET 3125", "CAL")
// ============================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (!manualCalRunning && !measurementRunning && cmd.equalsIgnoreCase("TEST")) {
    startMeasurement("LOCAL_TEST", "Serial_Test", true);
    return;
  }

  if (!manualCalRunning && !measurementRunning && cmd.startsWith("SET ")) {
    float newQuad = cmd.substring(4).toFloat();
    if (newQuad >= 200.0f && newQuad <= 20000.0f) {
      calibratedQuadCPR = newQuad;
      calibratedSingleChCPR = roundf(newQuad * 0.605f);

      preferences.begin("lubritorque", false);
      preferences.putFloat("quad_cpr", calibratedQuadCPR);
      preferences.putFloat("single_cpr", calibratedSingleChCPR);
      preferences.end();

      Serial.printf("[CPR UPDATED!] Quad CPR = %.0f | Single-Ch CPR = %.0f\n",
                    calibratedQuadCPR, calibratedSingleChCPR);
    }
    return;
  }

  if (!manualCalRunning && cmd.equalsIgnoreCase("CAL")) {
    if (measurementRunning) return;
    resetEncoder();
    manualCalRunning = true;
    Serial.println("\n[CAL STARTED] Motor running... Press ENTER in Serial Monitor at 360°!");
    motorStartMicros = micros();
    motorStart();
  } else if (manualCalRunning) {
    motorBrakeStop();
    motorStopMicros = micros();
    manualCalRunning = false;

    int32_t qCnt = 0;
    uint32_t eA = 0, eB = 0;
    getEncoderStats(qCnt, eA, eB);

    uint32_t singleTicks = (eB >= eA) ? eB : eA;
    uint32_t quadTicks = (uint32_t)abs(qCnt);

    if (singleTicks > 100) {
      calibratedSingleChCPR = (float)singleTicks;
      calibratedQuadCPR = (quadTicks > 100) ? (float)quadTicks : (calibratedSingleChCPR * 2.0f);

      preferences.begin("lubritorque", false);
      preferences.putFloat("single_cpr", calibratedSingleChCPR);
      preferences.putFloat("quad_cpr", calibratedQuadCPR);
      preferences.end();

      float calTime = (float)(motorStopMicros - motorStartMicros) / 1000000.0f;
      Serial.printf("[CAL SAVED!] 1-Rev Time: %.3f s | Single-Ch CPR = %.0f | Quad CPR = %.0f\n",
                    calTime, calibratedSingleChCPR, calibratedQuadCPR);
    } else {
      Serial.println("[CAL ABORTED] Too few encoder pulses detected.");
    }
  }
}

// ============================================================
// CLOUD COMMAND POLLING & SETUP
// ============================================================
void checkCloudCommands() {
  if (measurementRunning || manualCalRunning) return;

  static unsigned long lastPoll = 0;
  if (millis() - lastPoll < 600) return;
  lastPoll = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  if (httpMutex != NULL && xSemaphoreTake(httpMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(5);
  HTTPClient http;
  http.setReuse(false);
  http.setTimeout(5000);

  String cmd = "IDLE", mode = "TEST_UNKNOWN", sName = "Unknown_Sample";
  bool gotResponse = false;

  String url = String(SUPABASE_BASE) + "/device_control?device_id=eq." + String(DEVICE_ID) + "&select=command,mode,sample_name";
  if (http.begin(client, url)) {
    http.addHeader("apikey", SUPABASE_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);

    int code = http.GET();
    if (code == 200) {
      String body = http.getString();
      JsonDocument doc;
      if (!deserializeJson(doc, body) && doc.size() > 0) {
        cmd = doc[0]["command"] | "IDLE";
        mode = doc[0]["mode"] | "TEST_UNKNOWN";
        sName = doc[0]["sample_name"] | "Unknown_Sample";
        gotResponse = true;
      }
    } else {
      Serial.printf("[Cloud Poll HTTP %d]\n", code);
    }
    http.end();
  }

  if (httpMutex != NULL) xSemaphoreGive(httpMutex);
  if (!gotResponse) return;

  if (cmd == "START") {
    startMeasurement(mode, sName, false);
  } else if (cmd == "STOP") {
    motorBrakeStop();
    updateDeviceTelemetry("IDLE", false);
  } else {
    updateDeviceTelemetry("", false);
  }
}

void setup() {
  Serial.begin(115200);
  httpMutex = xSemaphoreCreateMutex();

  calibratedQuadCPR     = DEFAULT_QUAD_CPR;
  calibratedSingleChCPR = DEFAULT_SINGLE_CH_CPR;
  preferences.begin("lubritorque", false);
  preferences.putFloat("quad_cpr", calibratedQuadCPR);
  preferences.putFloat("single_cpr", calibratedSingleChCPR);
  preferences.end();

  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  motorBrakeStop();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  encoderLastState = (digitalRead(ENCODER_A) << 1) | digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!initINA226Sensor()) {
    Serial.println("[INA226] Warning: Not detected at boot (will auto-retry every 2s)");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
  Serial.printf("Pre-set CPR Ready -> Quad: %.0f | Single-Ch: %.0f (Send 'TEST' in Serial Monitor for offline 1-rev check)\n",
                calibratedQuadCPR, calibratedSingleChCPR);

  // Clear any leftover START command in cloud on boot
  updateDeviceTelemetry("IDLE", false);

  xTaskCreatePinnedToCore(
    liveTelemetryTask,
    "LiveTelemetryTask",
    8192,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
  handleSerialCommands();
  updateINA226();
  updateMeasurement();
  checkCloudCommands();
}