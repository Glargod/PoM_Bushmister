#include "DHT.h"
#include "esp_sleep.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================== PIN DEFINES ==================
// GPIO5 is an ESP32 strapping pin — it can glitch LOW at reset and
// toggle the mister. Move the wire to GPIO18 if you still see a
// boot-on. STATUS stays on 4.
#define MISTER_PIN   5      // pulse LOW = button press
#define STATUS_PIN   4      // HIGH (~2.7V) = mister ON
#define DHT_PIN     15
#define LDR_PIN     32
#define DHTTYPE     DHT11

// Nordic UART Service — works with Web Bluetooth + nRF Connect
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ================== TUNABLE CONSTANTS ==================
const int   SHORT_PRESS_MS     = 100;
const float TEMP_OFFSET        = -0.9;

const float TEMP_MIN_C         = -5.0;
const float TEMP_MAX_C         = 25.0;
const float RH_MAX_PERCENT     = 90.0;

const int   HISTORY_SIZE       = 4;
const int   CHANGE_THRESHOLD   = 180;
const int   PRE_DUSK_OFFSET    = 400;
const int   POST_DAWN_OFFSET   = 400;
const int   MIN_RANGE_FOR_DYNAMIC = 800;

const int   BURSTS_PER_SEQUENCE = 3;
const int   FART_CHANCE_PERCENT = 20;

const uint64_t DEFAULT_SLEEP_MINUTES = 8;
const uint32_t BLE_ADVERTISE_MS      = 25000;   // window after each wake
const uint32_t BLE_CONNECTED_MAX_MS  = 180000;  // stay awake while phone is on it

const int   LOG_SIZE = 48;   // ~6.4 h at 8 min

// ================== PERSISTENT (survives deep sleep) ==================
RTC_DATA_ATTR int ldrHistory[HISTORY_SIZE] = {0};
RTC_DATA_ATTR int historyIndex = 0;
RTC_DATA_ATTR int ldrMin = 4095;
RTC_DATA_ATTR int ldrMax = 0;
RTC_DATA_ATTR uint32_t sampleCount = 0;
RTC_DATA_ATTR uint64_t sleepMinutes = DEFAULT_SLEEP_MINUTES;
RTC_DATA_ATTR bool autoMistEnabled = true;

struct Sample {
  int16_t t_x10;   // temp * 10
  uint8_t rh;
  uint16_t ldr;
  uint8_t flags;   // bit0 misted, bit1 fart, bit2 skipped, bit3 dht_fail
};

RTC_DATA_ATTR Sample logBuf[LOG_SIZE];
RTC_DATA_ATTR uint8_t logHead = 0;
RTC_DATA_ATTR uint8_t logCount = 0;

DHT dht(DHT_PIN, DHTTYPE);

float lastT = NAN;
float lastH = NAN;
int   lastLdr = 0;
int   lastAvgChange = 0;
int   lastBrightTh = 2200;
int   lastDarkTh = 1800;
bool  lastMisted = false;
bool  lastFart = false;
bool  lastSkipped = false;
bool  lastDhtFail = false;

BLEServer*         bleServer = nullptr;
BLECharacteristic* txChar    = nullptr;
bool deviceConnected    = false;
bool oldDeviceConnected = false;
volatile bool pendingOff   = false;
volatile bool pendingOn    = false;
volatile bool pendingMist  = false;
volatile bool pendingFart  = false;
volatile bool pendingReset = false;
volatile bool pendingSleep = false;
volatile bool pendingDump  = false;
String pendingSetSleep = "";

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { deviceConnected = true; }
  void onDisconnect(BLEServer* s) override { deviceConnected = false; }
};

void bleSend(const String& s);

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue().c_str();
    v.trim();
    v.toUpperCase();
    if (v.length() == 0) return;

    if (v == "STATUS") {
      // handled in loop so sensors are current
      pendingDump = false;
      // flag a status push
      pendingOff = pendingOff; // no-op keep compiler happy
    } else if (v == "OFF") {
      pendingOff = true;
    } else if (v == "ON") {
      pendingOn = true;
    } else if (v == "MIST" || v == "SPRAY") {
      pendingMist = true;
    } else if (v == "FART") {
      pendingFart = true;
    } else if (v == "RESET" || v == "RESET_CAL") {
      pendingReset = true;
    } else if (v == "SLEEP") {
      pendingSleep = true;
    } else if (v == "AUTO ON") {
      autoMistEnabled = true;
    } else if (v == "AUTO OFF") {
      autoMistEnabled = false;
    } else if (v == "LOG") {
      pendingDump = true;
    } else if (v.startsWith("SETSLEEP")) {
      pendingSetSleep = v;
    }
    // STATUS always answered from loop after command processing
    if (v == "STATUS" || v == "AUTO ON" || v == "AUTO OFF") {
      // request immediate status
      pendingDump = pendingDump;
    }
  }
};

bool misterIsOn() {
  return digitalRead(STATUS_PIN) == HIGH;
}

void toggleOnce() {
  digitalWrite(MISTER_PIN, LOW);
  delay(SHORT_PRESS_MS);
  digitalWrite(MISTER_PIN, HIGH);
  delay(200);
}

// Call this the instant pins exist. The hardware toggle can power up ON
// and GPIO5 can glitch a press at reset. Keep hammering until STATUS is LOW.
void ensureMisterOff() {
  digitalWrite(MISTER_PIN, HIGH);
  delay(80);
  int tries = 0;
  while (misterIsOn() && tries < 6) {
    Serial.println("BOOT: mister ON → forcing OFF");
    toggleOnce();
    delay(350);
    tries++;
  }
  if (misterIsOn()) {
    Serial.println("BOOT: WARNING still ON after 6 toggles");
  } else {
    Serial.println("BOOT: mister confirmed OFF");
  }
}

void ensureMisterOn() {
  int tries = 0;
  while (!misterIsOn() && tries < 6) {
    toggleOnce();
    delay(350);
    tries++;
  }
}

void performMistSequence() {
  Serial.println("Starting 3-burst sequence...");
  for (int i = 0; i < BURSTS_PER_SEQUENCE; i++) {
    Serial.print("Burst "); Serial.print(i + 1); Serial.println(" ON");
    toggleOnce();
    delay(500);
    Serial.println("Burst OFF");
    toggleOnce();
    if (i < BURSTS_PER_SEQUENCE - 1) delay(500);
  }
  delay(300);
  if (misterIsOn()) {
    Serial.println("SANITY: Still ON → forcing OFF");
    toggleOnce();
  } else {
    Serial.println("Sanity OK – mister OFF");
  }
  lastMisted = true;
}

void logSample() {
  Sample s;
  s.t_x10 = isnan(lastT) ? -999 : (int16_t)(lastT * 10);
  s.rh    = isnan(lastH) ? 255 : (uint8_t)constrain((int)lastH, 0, 100);
  s.ldr   = (uint16_t)lastLdr;
  s.flags = 0;
  if (lastMisted)  s.flags |= 0x01;
  if (lastFart)    s.flags |= 0x02;
  if (lastSkipped) s.flags |= 0x04;
  if (lastDhtFail) s.flags |= 0x08;
  logBuf[logHead] = s;
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

String statusJson() {
  String j = "{";
  j += "\"name\":\"PoM\",";
  j += "\"on\":" + String(misterIsOn() ? "true" : "false") + ",";
  j += "\"auto\":" + String(autoMistEnabled ? "true" : "false") + ",";
  j += "\"t\":";
  j += isnan(lastT) ? "null" : String(lastT, 1);
  j += ",\"rh\":";
  j += isnan(lastH) ? "null" : String(lastH, 1);
  j += ",\"ldr\":" + String(lastLdr) + ",";
  j += "\"ldrMin\":" + String(ldrMin) + ",";
  j += "\"ldrMax\":" + String(ldrMax) + ",";
  j += "\"avgChange\":" + String(lastAvgChange) + ",";
  j += "\"brightTh\":" + String(lastBrightTh) + ",";
  j += "\"darkTh\":" + String(lastDarkTh) + ",";
  j += "\"sleepMin\":" + String((uint32_t)sleepMinutes) + ",";
  j += "\"samples\":" + String(sampleCount) + ",";
  j += "\"logCount\":" + String(logCount) + ",";
  j += "\"misted\":" + String(lastMisted ? "true" : "false") + ",";
  j += "\"skipped\":" + String(lastSkipped ? "true" : "false");
  j += "}";
  return j;
}

String logJson() {
  String j = "{\"log\":[";
  for (int n = 0; n < logCount; n++) {
    int i = (logHead - logCount + n + LOG_SIZE) % LOG_SIZE;
    Sample s = logBuf[i];
    if (n) j += ",";
    j += "{\"t\":";
    j += (s.t_x10 == -999) ? "null" : String(s.t_x10 / 10.0, 1);
    j += ",\"rh\":";
    j += (s.rh == 255) ? "null" : String(s.rh);
    j += ",\"ldr\":" + String(s.ldr);
    j += ",\"f\":" + String(s.flags) + "}";
  }
  j += "]}";
  return j;
}

void bleSend(const String& s) {
  if (!deviceConnected || !txChar) return;
  // BLE notify is ~20 bytes default; chunk it
  const int CHUNK = 20;
  int len = s.length();
  for (int i = 0; i < len; i += CHUNK) {
    String part = s.substring(i, min(i + CHUNK, len));
    txChar->setValue((uint8_t*)part.c_str(), part.length());
    txChar->notify();
    delay(8);
  }
  // terminator so the webapp can reassemble
  txChar->setValue((uint8_t*)"\n", 1);
  txChar->notify();
}

void startBLE() {
  BLEDevice::init("PoM-Bushmister");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* svc = bleServer->createService(SERVICE_UUID);
  txChar = svc->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic* rx = svc->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new RxCallbacks());

  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising as PoM-Bushmister");
}

void stopBLE() {
  BLEDevice::deinit(true);
}

void readSensors() {
  lastH = dht.readHumidity();
  float t_raw = dht.readTemperature();
  lastDhtFail = isnan(lastH) || isnan(t_raw);
  if (lastDhtFail) {
    delay(2000);
    lastH = dht.readHumidity();
    t_raw = dht.readTemperature();
    lastDhtFail = isnan(lastH) || isnan(t_raw);
  }
  lastT = lastDhtFail ? NAN : (t_raw + TEMP_OFFSET);
  lastLdr = analogRead(LDR_PIN);

  ldrHistory[historyIndex] = lastLdr;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  sampleCount++;
  ldrMin = min(ldrMin, lastLdr);
  ldrMax = max(ldrMax, lastLdr);

  int range = ldrMax - ldrMin;
  lastBrightTh = 2200;
  lastDarkTh  = 1800;
  if (range >= MIN_RANGE_FOR_DYNAMIC) {
    lastBrightTh = ldrMin + (range * 70 / 100);
    lastDarkTh   = ldrMin + (range * 30 / 100);
  }

  int sumChange = 0;
  for (int i = 1; i < HISTORY_SIZE; i++) {
    sumChange += ldrHistory[i] - ldrHistory[(i - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  }
  lastAvgChange = sumChange / (HISTORY_SIZE - 1);

  Serial.print("Temp: ");
  if (isnan(lastT)) Serial.print("fail");
  else Serial.print(lastT, 1);
  Serial.print(" °C   RH: ");
  if (isnan(lastH)) Serial.print("fail");
  else Serial.print(lastH, 1);
  Serial.print(" %   LDR: ");
  Serial.println(lastLdr);
}

void runAutoLogic() {
  lastMisted = false;
  lastFart = false;
  lastSkipped = false;

  bool goodTemp = !isnan(lastT) && (lastT >= TEMP_MIN_C && lastT <= TEMP_MAX_C);
  bool goodRH   = !isnan(lastH) && (lastH <= RH_MAX_PERCENT);

  if (!autoMistEnabled) {
    Serial.println("Auto mist disabled");
    lastSkipped = true;
  } else if (goodTemp && goodRH) {
    if (lastAvgChange < -CHANGE_THRESHOLD && lastLdr > lastBrightTh - PRE_DUSK_OFFSET) {
      Serial.println("PRE-DUSK CREEP → misting");
      performMistSequence();
    } else if (lastAvgChange > CHANGE_THRESHOLD && lastLdr < lastDarkTh + POST_DAWN_OFFSET) {
      Serial.println("POST-DAWN CREEP → misting");
      performMistSequence();
    }
  } else {
    Serial.println("Skipped – bad temp or RH");
    lastSkipped = true;
  }

  if (random(100) < FART_CHANCE_PERCENT) {
    Serial.println("RANDOM FART");
    toggleOnce(); delay(200); toggleOnce();
    lastFart = true;
    delay(300);
    if (misterIsOn()) toggleOnce();
  }

  if (sampleCount > 5400) {
    ldrMin = lastLdr;
    ldrMax = lastLdr;
    sampleCount = 1;
    Serial.println("Min/Max reset for seasonal adaptation");
  }
}

void handleBleCommands() {
  if (pendingOff) {
    pendingOff = false;
    ensureMisterOff();
    bleSend(statusJson());
  }
  if (pendingOn) {
    pendingOn = false;
    ensureMisterOn();
    bleSend(statusJson());
  }
  if (pendingMist) {
    pendingMist = false;
    performMistSequence();
    bleSend(statusJson());
  }
  if (pendingFart) {
    pendingFart = false;
    toggleOnce(); delay(200); toggleOnce();
    delay(300);
    if (misterIsOn()) toggleOnce();
    lastFart = true;
    bleSend(statusJson());
  }
  if (pendingReset) {
    pendingReset = false;
    ldrMin = lastLdr;
    ldrMax = lastLdr;
    sampleCount = 1;
    historyIndex = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) ldrHistory[i] = lastLdr;
    logHead = 0;
    logCount = 0;
    bleSend("{\"ok\":\"cal reset\"}");
    bleSend(statusJson());
  }
  if (pendingSetSleep.length()) {
    int v = pendingSetSleep.substring(8).toInt();
    pendingSetSleep = "";
    if (v >= 1 && v <= 60) {
      sleepMinutes = v;
      bleSend("{\"ok\":\"sleep set\",\"sleepMin\":" + String(v) + "}");
    } else {
      bleSend("{\"err\":\"sleep 1-60 min\"}");
    }
  }
  if (pendingDump) {
    pendingDump = false;
    bleSend(logJson());
  }
}

void goToSleep() {
  ensureMisterOff();
  Serial.print("Hibernating ");
  Serial.print((uint32_t)sleepMinutes);
  Serial.println(" minutes...");
  Serial.flush();
  stopBLE();
  esp_sleep_enable_timer_wakeup(sleepMinutes * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  // Pins FIRST — before Serial delay — so we kill a boot-glitch ON ASAP
  pinMode(MISTER_PIN, OUTPUT);
  digitalWrite(MISTER_PIN, HIGH);
  pinMode(STATUS_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  Serial.begin(115200);
  delay(200);

  Serial.println("=== PoM (Piss-o-Matic) – Maritimes Edition ===");
  ensureMisterOff();

  dht.begin();
  randomSeed(esp_random());

  readSensors();
  runAutoLogic();
  logSample();
  ensureMisterOff();   // belt and suspenders after any sequence

  startBLE();
}

void loop() {
  handleBleCommands();

  static uint32_t lastStatus = 0;
  if (deviceConnected && millis() - lastStatus > 2000) {
    lastStatus = millis();
    bleSend(statusJson());
  }

  // Answer a lone STATUS write: Rx sets nothing, so push on any connection traffic
  // (status already streams every 2s while connected)

  if (!deviceConnected && oldDeviceConnected) {
    delay(80);
    bleServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    bleSend(statusJson());
  }

  if (pendingSleep) {
    pendingSleep = false;
    goToSleep();
  }

  uint32_t awakeLimit = deviceConnected ? BLE_CONNECTED_MAX_MS : BLE_ADVERTISE_MS;
  if (millis() > awakeLimit) {
    goToSleep();
  }

  delay(40);
}
