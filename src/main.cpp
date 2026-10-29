// ESP32 load-cell node (A or B) with a local SSD1306 OLED and an ESP-NOW link
// to the handheld display. Reads a NAU7802 load-cell amplifier, auto-locks a
// stable reading above a threshold, and obeys tare/calibrate/settings commands
// sent by the display.
//
// The same source builds both nodes; the node identity comes from the
// LOAD_CELL_UNIT build flag (1 = A, 2 = B) set per environment in
// platformio.ini.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

#include <EspNowProtocol.h>

#ifndef LOAD_CELL_UNIT
#define LOAD_CELL_UNIT 1
#endif

constexpr uint8_t NODE_ID = LOAD_CELL_UNIT; // 1 = A, 2 = B

constexpr int I2C_SDA_PIN = 23;
constexpr int I2C_SCL_PIN = 22;
constexpr int LED1_PIN = 4;
constexpr int LED2_PIN = 16;
constexpr int RESET_BUTTON_PIN = 33;

constexpr int WEIGHT_SAMPLES = 4;
constexpr int SAMPLE_DELAY_MS = 10;
constexpr unsigned long SEND_INTERVAL_MS = 200;
constexpr float STABILITY_WINDOW_KG = 1.0f;
constexpr unsigned long FIXED_STABILITY_TIME_MS = 4000;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 40;

constexpr float DEFAULT_MIN_LOCK_WEIGHT_KG = 5.0f;
constexpr bool DEFAULT_AUTO_LOCK_ENABLED = true;

// MAC address of the handheld display this node reports to.
uint8_t displayMacAddress[6] = {
  0x70, 0x4B, 0xCA, 0x76, 0xC4, 0x5C
};

NAU7802 scale;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
Preferences preferences;

float calibrationFactor = 1.0f;
float minLockWeightKg = DEFAULT_MIN_LOCK_WEIGHT_KG;
bool autoLockEnabled = DEFAULT_AUTO_LOCK_ENABLED;

bool readingLocked = false;
float lockedWeightKg = 0.0f;
float stabilityMinWeightKg = 0.0f;
float stabilityMaxWeightKg = 0.0f;
unsigned long stabilityStartMs = 0;
unsigned long lastSendMs = 0;

uint8_t currentStatus = STATUS_MEASURING;
uint8_t lastCompletedCommand = COMMAND_NONE;
uint8_t lastCommandResult = COMMAND_RESULT_NONE;

volatile uint8_t pendingCommand = COMMAND_NONE;
volatile float pendingValue = 0.0f;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long buttonChangedMs = 0;


float lastOledWeight = NAN;
uint8_t lastOledStatus = 255;
bool lastOledLocked = false;

void showMessage(const char* line1, const char* line2) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB10_tr);
  oled.drawStr(0, 18, line1);
  oled.drawStr(0, 42, line2);
  oled.sendBuffer();
}

const char* statusText() {
  if (readingLocked) return "LOCKED";
  switch (currentStatus) {
    case STATUS_STABLE: return "STABLE";
    case STATUS_TARING: return "TARING";
    case STATUS_CALIBRATING: return "CALIBRATING";
    case STATUS_CALIBRATION_COMPLETE: return "CAL COMPLETE";
    case STATUS_ERROR: return "ERROR";
    default: return "MEASURING";
  }
}

void showWeight(float weightKg, bool force = false) {
  if (!force &&
      !isnan(lastOledWeight) &&
      fabsf(weightKg - lastOledWeight) < 0.001f &&
      lastOledStatus == currentStatus &&
      lastOledLocked == readingLocked) {
    return;
  }

  char weightText[20];
  snprintf(weightText, sizeof(weightText), "%.3f kg", weightKg);

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x12_tr);
  oled.drawStr(0, 11, NODE_ID == 1 ? "LOAD CELL A" : "LOAD CELL B");
  oled.drawStr(0, 24, statusText());
  oled.setFont(u8g2_font_ncenB18_tr);
  oled.drawStr(0, 51, weightText);

  char threshold[22];
  snprintf(threshold, sizeof(threshold), "Lock > %.1f kg", minLockWeightKg);
  oled.setFont(u8g2_font_5x8_tr);
  oled.drawStr(0, 63, threshold);
  oled.sendBuffer();

  lastOledWeight = weightKg;
  lastOledStatus = currentStatus;
  lastOledLocked = readingLocked;
}

void saveSettings() {
  preferences.putFloat("minLockKg", minLockWeightKg);
  preferences.putBool("autoLock", autoLockEnabled);
}

void clearLock() {
  readingLocked = false;
  lockedWeightKg = 0.0f;
  stabilityMinWeightKg = 0.0f;
  stabilityMaxWeightKg = 0.0f;
  stabilityStartMs = 0;
  currentStatus = STATUS_MEASURING;
}

float readAverageWeightKg() {
  float sum = 0.0f;
  for (int i = 0; i < WEIGHT_SAMPLES; i++) {
    sum += scale.getWeight(true, 8, 1000);
    delay(SAMPLE_DELAY_MS);
  }
  return sum / WEIGHT_SAMPLES;
}

void sendPacket(float displayedWeightKg) {
  WeightPacket packet{};
  packet.nodeId = NODE_ID;
  packet.weightKg = displayedWeightKg;
  packet.locked = readingLocked ? 1 : 0;
  packet.status = currentStatus;
  packet.lastCommand = lastCompletedCommand;
  packet.commandResult = lastCommandResult;
  packet.minLockWeightKg = minLockWeightKg;
  packet.autoLockEnabled = autoLockEnabled ? 1 : 0;
  packet.stabilityTimeMs = (uint16_t)FIXED_STABILITY_TIME_MS;

  esp_now_send(
    displayMacAddress,
    reinterpret_cast<const uint8_t*>(&packet),
    sizeof(packet)
  );
}

void finishCommand(uint8_t command, uint8_t result) {
  lastCompletedCommand = command;
  lastCommandResult = result;
}

void tareScale() {
  currentStatus = STATUS_TARING;
  showMessage("Taring", "Remove weight");
  sendPacket(0.0f);

  scale.calculateZeroOffset();
  scale.setCalibrationFactor(calibrationFactor);

  clearLock();
  finishCommand(COMMAND_TARE, COMMAND_RESULT_OK);

  showMessage("Tare", "Complete");
  sendPacket(0.0f);
  delay(700);
}

void calibrateScale(float knownWeightKg) {
  if (knownWeightKg <= 0.0f) {
    currentStatus = STATUS_ERROR;
    finishCommand(COMMAND_CALIBRATE, COMMAND_RESULT_INVALID);

    showMessage("Calibration", "Invalid weight");
    sendPacket(0.0f);

    delay(1000);
    currentStatus = STATUS_MEASURING;
    return;
  }

  currentStatus = STATUS_CALIBRATING;
  showMessage("Calibrating", "Keep still");
  sendPacket(knownWeightKg);

  scale.calculateCalibrationFactor(
    knownWeightKg,
    8,
    5000
  );

  float newCalibrationFactor = scale.getCalibrationFactor();

  bool calibrationValid =
    isfinite(newCalibrationFactor) &&
    fabsf(newCalibrationFactor) > 0.000001f;

  if (calibrationValid) {
    calibrationFactor = newCalibrationFactor;
    scale.setCalibrationFactor(calibrationFactor);

    preferences.putFloat(
      "calFactor",
      calibrationFactor
    );

    clearLock();
    currentStatus = STATUS_CALIBRATION_COMPLETE;

    finishCommand(
      COMMAND_CALIBRATE,
      COMMAND_RESULT_OK
    );

    showMessage("Calibration", "Complete");
  } else {
    currentStatus = STATUS_ERROR;

    finishCommand(
      COMMAND_CALIBRATE,
      COMMAND_RESULT_FAILED
    );

    showMessage("Calibration", "Failed");
  }

  sendPacket(knownWeightKg);
  delay(1200);

  currentStatus = STATUS_MEASURING;
}

void applySettingCommand(uint8_t command, float value) {
  uint8_t result = COMMAND_RESULT_OK;

  if (command == COMMAND_SET_LOCK_THRESHOLD) {
    if (value < 0.0f || value > 1000.0f) {
      result = COMMAND_RESULT_INVALID;
    } else {
      minLockWeightKg = value;
      clearLock();
      saveSettings();
    }
  } else if (command == COMMAND_SET_AUTO_LOCK) {
    autoLockEnabled = value >= 0.5f;
    clearLock();
    saveSettings();
  } else if (command == COMMAND_SET_STABILITY_SECONDS) {
    result = COMMAND_RESULT_INVALID;
  }

  finishCommand(command, result);
  sendPacket(readingLocked ? lockedWeightKg : 0.0f);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(
  const esp_now_recv_info_t*,
  const uint8_t* data,
  int len
) {
#else
void onDataReceived(
  const uint8_t*,
  const uint8_t* data,
  int len
) {
#endif
  if (len != sizeof(CommandPacket)) return;

  CommandPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  pendingValue = packet.value;
  pendingCommand = packet.command;
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print(NODE_ID == 1 ? "Load Cell A MAC: " : "Load Cell B MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    showMessage("ESP-NOW error", "Restart device");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataReceived);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, displayMacAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  if (!esp_now_is_peer_exist(displayMacAddress) &&
      esp_now_add_peer(&peer) != ESP_OK) {
    showMessage("Peer error", "Check MAC");
    while (true) delay(1000);
  }
}

void processCommand() {
  uint8_t command = pendingCommand;
  if (command == COMMAND_NONE) return;

  float value = pendingValue;
  pendingCommand = COMMAND_NONE;

  switch (command) {
    case COMMAND_TARE:
      tareScale();
      break;
    case COMMAND_CALIBRATE:
      calibrateScale(value);
      break;
    case COMMAND_CLEAR_LOCK:
      clearLock();
      finishCommand(COMMAND_CLEAR_LOCK, COMMAND_RESULT_OK);
      showMessage("Continue", "Measuring");
      sendPacket(0.0f);
      delay(350);
      break;
    case COMMAND_SET_LOCK_THRESHOLD:
    case COMMAND_SET_AUTO_LOCK:
    case COMMAND_SET_STABILITY_SECONDS:
      applySettingCommand(command, value);
      break;
    default:
      finishCommand(command, COMMAND_RESULT_INVALID);
      break;
  }
}

void updateStabilityAndLock(float liveWeightKg, unsigned long now) {
  if (readingLocked) return;

  if (fabsf(liveWeightKg) < 0.005f || liveWeightKg <= minLockWeightKg) {
    stabilityStartMs = 0;
    stabilityMinWeightKg = 0.0f;
    stabilityMaxWeightKg = 0.0f;
    currentStatus = STATUS_MEASURING;
    return;
  }

  if (stabilityStartMs == 0) {
    stabilityStartMs = now;
    stabilityMinWeightKg = liveWeightKg;
    stabilityMaxWeightKg = liveWeightKg;
    currentStatus = STATUS_MEASURING;
    return;
  }

  stabilityMinWeightKg = min(stabilityMinWeightKg, liveWeightKg);
  stabilityMaxWeightKg = max(stabilityMaxWeightKg, liveWeightKg);

  if ((stabilityMaxWeightKg - stabilityMinWeightKg) > STABILITY_WINDOW_KG) {
    stabilityStartMs = now;
    stabilityMinWeightKg = liveWeightKg;
    stabilityMaxWeightKg = liveWeightKg;
    currentStatus = STATUS_MEASURING;
    return;
  }

  if (now - stabilityStartMs >= FIXED_STABILITY_TIME_MS) {
    currentStatus = STATUS_STABLE;

    if (autoLockEnabled && liveWeightKg > minLockWeightKg) {
      readingLocked = true;
      lockedWeightKg = liveWeightKg;
      digitalWrite(LED1_PIN, HIGH);
    }
  }
}

void handlePhysicalButton() {
  bool reading = digitalRead(RESET_BUTTON_PIN);
  unsigned long now = millis();
  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangedMs = now;
  }
  if (now - buttonChangedMs < BUTTON_DEBOUNCE_MS || reading == stableButtonState) return;
  stableButtonState = reading;
  // Act on release only: a press clears any held lock and resumes measuring.
  // (There is no long-press action, so the press edge needs no handling.)
  if (stableButtonState == HIGH) {
    clearLock();
    finishCommand(COMMAND_CLEAR_LOCK, COMMAND_RESULT_OK);
    showMessage("Continue", "Measuring");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, HIGH);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  oled.begin();
  showMessage("Loading scale", "Please wait");

  if (!scale.begin(Wire, true)) {
    showMessage("Scale not found", "Check wiring");
    while (true) delay(1000);
  }

  scale.setGain(NAU7802_GAIN_128);
  scale.setSampleRate(NAU7802_SPS_80);
  scale.setLDO(NAU7802_LDO_3V3);

  preferences.begin(NODE_ID == 1 ? "scaleA" : "scaleB", false);
  calibrationFactor = preferences.getFloat("calFactor", 1.0f);
  minLockWeightKg = preferences.getFloat(
    "minLockKg", DEFAULT_MIN_LOCK_WEIGHT_KG
  );
  autoLockEnabled = preferences.getBool(
    "autoLock", DEFAULT_AUTO_LOCK_ENABLED
  );

  scale.setCalibrationFactor(calibrationFactor);
  setupEspNow();
  tareScale();

  showMessage(
    NODE_ID == 1 ? "Load Cell A" : "Load Cell B",
    "Wireless ready"
  );
  delay(600);
}

void loop() {
  processCommand();
  handlePhysicalButton();

  unsigned long now = millis();

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    float liveWeightKg = readAverageWeightKg();
    updateStabilityAndLock(liveWeightKg, now);

    if (!readingLocked &&
        currentStatus != STATUS_STABLE &&
        currentStatus != STATUS_TARING &&
        currentStatus != STATUS_CALIBRATING &&
        currentStatus != STATUS_CALIBRATION_COMPLETE &&
        currentStatus != STATUS_ERROR) {
      currentStatus = STATUS_MEASURING;
    }

    float displayedWeightKg =
      readingLocked ? lockedWeightKg : liveWeightKg;

    digitalWrite(LED1_PIN, readingLocked ? HIGH : LOW);
    showWeight(displayedWeightKg);
    sendPacket(displayedWeightKg);

    Serial.printf(
      "%c Weight: %.3f kg | %s | Lock > %.1f kg | Window 1.0 kg / 4 s\n",
      NODE_ID == 1 ? 'A' : 'B',
      displayedWeightKg,
      statusText(),
      minLockWeightKg
    );

    lastSendMs = now;
  }

  delay(10);
}
