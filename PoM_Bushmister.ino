#include "DHT.h"
#include "esp_sleep.h"

// ================== PIN DEFINES ==================
// GPIO5 is a strapping pin and can glitch LOW at reset (toggles the mister).
// If boot-ON persists after this firmware, move the wire to GPIO18.
#define MISTER_PIN   5      // pulse LOW = button press
#define STATUS_PIN   4      // HIGH (~2.7V) = mister ON
#define DHT_PIN     15
#define LDR_PIN     32
#define DHTTYPE     DHT11

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
const int   FART_CHANCE_PERCENT = 7;

const uint64_t SLEEP_MINUTES   = 8;
const uint32_t WAKES_PER_DAY   = 180;

RTC_DATA_ATTR int ldrHistory[HISTORY_SIZE] = {0};
RTC_DATA_ATTR int historyIndex = 0;
RTC_DATA_ATTR int ldrMin = 4095;
RTC_DATA_ATTR int ldrMax = 0;
RTC_DATA_ATTR uint32_t sampleCount = 0;
RTC_DATA_ATTR float ldrMean = 0;
RTC_DATA_ATTR bool pendingLow = false;
RTC_DATA_ATTR bool pendingHigh = false;
RTC_DATA_ATTR int  pendingLowVal = 0;
RTC_DATA_ATTR int  pendingHighVal = 0;
RTC_DATA_ATTR bool calTrusted = false;

DHT dht(DHT_PIN, DHTTYPE);

bool misterIsOn() {
  return digitalRead(STATUS_PIN) == HIGH;
}

void toggleOnce() {
  digitalWrite(MISTER_PIN, LOW);
  delay(SHORT_PRESS_MS);
  digitalWrite(MISTER_PIN, HIGH);
  delay(200);
}

void ensureMisterOff() {
  digitalWrite(MISTER_PIN, HIGH);
  delay(80);
  int tries = 0;
  while (misterIsOn() && tries < 6) {
    Serial.println("Mister ON -> forcing OFF");
    toggleOnce();
    delay(350);
    tries++;
  }
  if (misterIsOn()) Serial.println("WARNING: still ON after 6 toggles");
  else Serial.println("Mister confirmed OFF");
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
    Serial.println("SANITY: still ON -> forcing OFF");
    toggleOnce();
  }
}

bool validLdr(int v) {
  return v > 0 && v < 4095;
}

void updateCalibration(int ldr) {
  if (!validLdr(ldr)) {
    Serial.println("LDR rail reading ignored");
    pendingLow = pendingHigh = false;
    return;
  }

  if (ldr < ldrMin) {
    if (pendingLow && pendingLowVal <= ldr + 40) {
      ldrMin = min(pendingLowVal, ldr);
      pendingLow = false;
      Serial.print("ldrMin committed: "); Serial.println(ldrMin);
    } else {
      pendingLow = true;
      pendingLowVal = ldr;
    }
  } else {
    pendingLow = false;
  }

  if (ldr > ldrMax) {
    if (pendingHigh && pendingHighVal >= ldr - 40) {
      ldrMax = max(pendingHighVal, ldr);
      pendingHigh = false;
      Serial.print("ldrMax committed: "); Serial.println(ldrMax);
    } else {
      pendingHigh = true;
      pendingHighVal = ldr;
    }
  } else {
    pendingHigh = false;
  }

  if (sampleCount == 1) ldrMean = ldr;
  else {
    float n = (sampleCount < WAKES_PER_DAY) ? (float)sampleCount : (float)WAKES_PER_DAY;
    ldrMean += (ldr - ldrMean) / n;
  }

  int range = ldrMax - ldrMin;
  calTrusted = (range >= MIN_RANGE_FOR_DYNAMIC);

  if (sampleCount > 0 && (sampleCount % WAKES_PER_DAY) == 0) {
    if (!calTrusted) {
      Serial.println("24h range too small -> reset min/max (covered LDR?)");
      ldrMin = ldr;
      ldrMax = ldr;
      pendingLow = pendingHigh = false;
      calTrusted = false;
    } else {
      ldrMin = (int)(ldrMin + (ldrMean - ldrMin) / 8.0);
      ldrMax = (int)(ldrMax + (ldrMean - ldrMax) / 8.0);
      if (ldrMin < 0) ldrMin = 0;
      if (ldrMax > 4095) ldrMax = 4095;
      Serial.print("Daily decay  min="); Serial.print(ldrMin);
      Serial.print("  max="); Serial.println(ldrMax);
    }
  }
}

void setup() {
  pinMode(MISTER_PIN, OUTPUT);
  digitalWrite(MISTER_PIN, HIGH);
  pinMode(STATUS_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  Serial.begin(115200);
  delay(200);

  Serial.println("=== PoM FIELD  set-and-forget  ===");
  ensureMisterOff();

  dht.begin();
  randomSeed(esp_random());

  float h = dht.readHumidity();
  float t_raw = dht.readTemperature();
  if (isnan(h) || isnan(t_raw)) {
    delay(2000);
    h = dht.readHumidity();
    t_raw = dht.readTemperature();
  }
  float t = t_raw + TEMP_OFFSET;
  int ldr = analogRead(LDR_PIN);

  sampleCount++;
  ldrHistory[historyIndex] = ldr;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  updateCalibration(ldr);

  if (!isnan(h) && !isnan(t_raw)) {
    Serial.print("Temp: "); Serial.print(t, 1);
    Serial.print(" C   RH: "); Serial.print(h, 1);
    Serial.print(" %   LDR: "); Serial.println(ldr);
  } else {
    Serial.print("DHT fail   LDR: "); Serial.println(ldr);
  }

  int range = ldrMax - ldrMin;
  int brightThreshold = 2200;
  int darkThreshold  = 1800;
  if (calTrusted) {
    brightThreshold = ldrMin + (range * 70 / 100);
    darkThreshold   = ldrMin + (range * 30 / 100);
    Serial.print("Dynamic bright="); Serial.print(brightThreshold);
    Serial.print("  dark="); Serial.println(darkThreshold);
  } else {
    Serial.println("Range small -> fallback thresholds");
  }

  int sumChange = 0;
  for (int i = 1; i < HISTORY_SIZE; i++) {
    sumChange += ldrHistory[i] - ldrHistory[(i - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  }
  int avgChange = sumChange / (HISTORY_SIZE - 1);

  bool goodTemp = !isnan(t_raw) && (t >= TEMP_MIN_C && t <= TEMP_MAX_C);
  bool goodRH   = !isnan(h) && (h <= RH_MAX_PERCENT);

  if (goodTemp && goodRH) {
    if (avgChange < -CHANGE_THRESHOLD && ldr > brightThreshold - PRE_DUSK_OFFSET) {
      Serial.println("PRE-DUSK CREEP -> misting");
      performMistSequence();
    } else if (avgChange > CHANGE_THRESHOLD && ldr < darkThreshold + POST_DAWN_OFFSET) {
      Serial.println("POST-DAWN CREEP -> misting");
      performMistSequence();
    }
  } else {
    Serial.println("Skipped - bad temp or RH");
  }

  if (random(100) < FART_CHANCE_PERCENT) {
    Serial.println("RANDOM FART");
    toggleOnce(); delay(200); toggleOnce();
    delay(300);
  }

  ensureMisterOff();

  Serial.print("Hibernating "); Serial.print((uint32_t)SLEEP_MINUTES); Serial.println(" min");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {}
