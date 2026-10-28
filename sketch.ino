/*
  Smart Parking Assistant with ML (Arduino Uno)
  ---------------------------------------------
  Outputs:
    - 3 Green LEDs (Slots 1, 2, 3) : Pins D8, D9, D10
    - 3 Red LEDs (Slots 1, 2, 3)   : Pins D11, D12, D13
    - Dim Light Indicator LED      : Pin D4
    - Buzzer                       : Pin A0
    - 16x2 I2C LCD                 : Pins A4 (SDA), A5 (SCL)
  Inputs:
    - Shared Ultrasonic TRIG       : Pin D2
    - Ultrasonic ECHO (1, 2, 3)    : Pins D3, D5, D7
    - Entry & Exit Gate Sensors    : Pins A1, A2 (INPUT_PULLUP)
    - LDR Sensor (AO)              : Pin A3
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define USE_RTC 0
#if USE_RTC
  #include <RTClib.h>
  RTC_DS3231 rtc;
#else
  const uint8_t START_HOUR = 8;
  const uint8_t START_MIN  = 0;
  const uint8_t START_DOW  = 1;
#endif

// Pin Allocations
const uint8_t TRIG_PIN  = 2;          // Shared trigger for all 3 ultrasonic sensors
const uint8_t ECHO[3]   = {3, 5, 7};  // Dedicated echo pins
const uint8_t DIM_LED   = 4;          // Yellow indicator LED for Dim light
const uint8_t GREEN[3]  = {8, 9, 10};
const uint8_t RED[3]    = {11, 12, 13};
const uint8_t BUZZER    = A0;
const uint8_t ENTRY_IR  = A1;
const uint8_t EXIT_IR   = A2;
const uint8_t LDR_PIN   = A3;

// Thresholds & System Settings
const uint16_t OCC_THRESHOLD_CM = 15;
const uint8_t  DEBOUNCE         = 4;
const uint16_t LOOP_MS          = 300;
const uint16_t PAGE_MS          = 3000;
const uint16_t LOG_MS           = 5000;
const uint16_t GATE_LOCKOUT_MS  = 1000;

const unsigned long BUCKET_MS    = 60000UL;
const unsigned long PARK_UNIT_MS = 300000UL;

LiquidCrystal_I2C lcd(0x27, 16, 2);

bool occupied[3]         = {false, false, false};
uint8_t diffCount[3]     = {0, 0, 0};
unsigned long startMs[3] = {0, 0, 0};
bool wasFull             = false;
uint8_t page             = 0;
unsigned long lastPage   = 0;
unsigned long lastLog    = 0;

uint8_t entryB[15] = {0}, exitB[15] = {0};
uint8_t bIdx = 0;
unsigned long lastBucket = 0;
bool entryPrev = false, exitPrev = false;
unsigned long entryLock = 0, exitLock = 0;
int ldrAvg = 0;
const char *LIGHT_TXT[3] = {"Dark", "Dim", "Bright"};

uint8_t predictFreeProb(uint8_t hour, uint8_t dow, uint8_t slot, uint8_t state,
                        uint8_t parked, uint8_t occ, uint8_t arr15, uint8_t out15,
                        uint8_t light) {
  if (state <= 0) {
    if (parked <= 0) {
      if (light <= 0) {
        if (occ <= 1) {
          if (hour <= 12) {
            return (hour <= 0) ? 98 : 99;
          } else {
            return (hour <= 20) ? 95 : 98;
          }
        } else {
          if (hour <= 9) {
            return (hour <= 3) ? 98 : 95;
          } else {
            return (hour <= 21) ? 88 : 97;
          }
        }
      } else {
        if (occ <= 0) {
          if (hour <= 7) {
            return (hour <= 6) ? 99 : 96;
          } else {
            return (slot <= 1) ? 91 : 95;
          }
        } else {
          if (occ <= 1) {
            return (hour <= 6) ? 97 : 88;
          } else {
            return (hour <= 5) ? 99 : 81;
          }
        }
      }
    } else {
      if (parked <= 25) {
        return 21;
      } else {
        return (parked <= 66) ? 0 : 16;
      }
    }
  } else {
    if (parked <= 4) {
      if (arr15 <= 0) {
        if (parked <= 0) {
          if (light <= 1) {
            return (occ <= 1) ? 98 : 87;
          } else {
            return 74;
          }
        } else {
          if (hour <= 10) {
            return (hour <= 8) ? 8 : 16;
          } else {
            return (light <= 1) ? 25 : 35;
          }
        }
      } else {
        if (hour <= 9) {
          if (dow <= 0) {
            return 23;
          } else {
            return (hour <= 5) ? 17 : 10;
          }
        } else {
          if (light <= 1) {
            return (hour <= 17) ? 13 : 23;
          } else {
            return (parked <= 0) ? 20 : 29;
          }
        }
      }
    } else {
      if (parked <= 93) {
        if (parked <= 28) {
          if (hour <= 11) {
            return (parked <= 21) ? 4 : 6;
          } else {
            return (parked <= 18) ? 11 : 21;
          }
        } else {
          if (parked <= 70) {
            return (parked <= 37) ? 4 : 0;
          } else {
            return (light <= 1) ? 8 : 11;
          }
        }
      } else {
        if (parked <= 103) {
          if (out15 <= 0) {
            return (parked <= 102) ? 14 : 23;
          } else {
            return (dow <= 3) ? 28 : 14;
          }
        } else {
          if (slot <= 2) {
            return (dow <= 4) ? 20 : 36;
          } else {
            return (light <= 1) ? 37 : 61;
          }
        }
      }
    }
  }
}

void getTime(uint8_t &hour, uint8_t &dow) {
#if USE_RTC
  DateTime now = rtc.now();
  hour = now.hour();
  dow  = now.dayOfTheWeek();
#else
  unsigned long totalMin = (unsigned long)START_HOUR * 60UL + START_MIN + millis() / 60000UL;
  hour = (uint8_t)((totalMin / 60UL) % 24UL);
  dow  = (uint8_t)((START_DOW + totalMin / 1440UL) % 7UL);
#endif
}

void pollGates() {
  unsigned long now = millis();
  if (now - lastBucket >= BUCKET_MS) {
    lastBucket += BUCKET_MS;
    bIdx = (bIdx + 1) % 15;
    entryB[bIdx] = 0;
    exitB[bIdx]  = 0;
  }
  bool e = (digitalRead(ENTRY_IR) == LOW);
  if (e && !entryPrev && (now - entryLock > GATE_LOCKOUT_MS)) {
    if (entryB[bIdx] < 255) entryB[bIdx]++;
    entryLock = now;
  }
  entryPrev = e;

  bool x = (digitalRead(EXIT_IR) == LOW);
  if (x && !exitPrev && (now - exitLock > GATE_LOCKOUT_MS)) {
    if (exitB[bIdx] < 255) exitB[bIdx]++;
    exitLock = now;
  }
  exitPrev = x;
}

void waitAndPoll(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    pollGates();
    delay(2);
  }
}

uint8_t sum15(const uint8_t *b) {
  uint16_t s = 0;
  for (uint8_t i = 0; i < 15; i++) s += b[i];
  return (s > 15) ? 15 : (uint8_t)s;
}

uint8_t lightLevel() {
  int raw = analogRead(LDR_PIN);
  ldrAvg = (ldrAvg * 7 + raw) / 8;

  // Wokwi LDR characteristics:
  // Low ADC (< 250)      = Bright
  // Mid ADC (250 - 750)  = Dim
  // High ADC (> 750)     = Dark
  if (ldrAvg >= 250 && ldrAvg <= 750) {
    return 1; // Dim
  } else if (ldrAvg > 750) {
    return 1; // Dark
  } else {
    return 2; // Bright
  }
}

uint16_t readCm(uint8_t i) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long d = pulseIn(ECHO[i], HIGH, 25000UL);
  if (d == 0) return 400;
  return (uint16_t)(d / 58UL);
}

void printRow(uint8_t row, const char *s) {
  lcd.setCursor(0, row);
  uint8_t n = 0;
  while (s[n] && n < 16) { lcd.print(s[n]); n++; }
  while (n < 16) { lcd.print(' '); n++; }
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(ECHO[i], INPUT);
    pinMode(GREEN[i], OUTPUT);
    pinMode(RED[i], OUTPUT);
  }
  pinMode(DIM_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENTRY_IR, INPUT_PULLUP);
  pinMode(EXIT_IR, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  printRow(0, "Smart Parking AI");
  printRow(1, "Starting...");

#if USE_RTC
  if (!rtc.begin()) {
    printRow(1, "RTC not found!");
    while (true) { delay(1000); }
  }
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif

  delay(1500);
  ldrAvg = analogRead(LDR_PIN);
  for (uint8_t i = 0; i < 3; i++) {
    occupied[i] = readCm(i) < OCC_THRESHOLD_CM;
    startMs[i]  = millis();
    delay(30);
  }
  entryPrev = (digitalRead(ENTRY_IR) == LOW);
  exitPrev  = (digitalRead(EXIT_IR) == LOW);
  Serial.println(F("hour,dow,s1,s2,s3,arr15,out15,light,p1,p2,p3"));
}

void loop() {
  // 1) Ultrasonic sensor sampling with debounce
  for (uint8_t i = 0; i < 3; i++) {
    bool seen = readCm(i) < OCC_THRESHOLD_CM;
    if (seen != occupied[i]) {
      if (++diffCount[i] >= DEBOUNCE) {
        occupied[i]  = seen;
        diffCount[i] = 0;
        startMs[i]   = millis();
      }
    } else {
      diffCount[i] = 0;
    }
    waitAndPoll(20);
  }

  // 2) ML Feature aggregation
  uint8_t hour, dow;
  getTime(hour, dow);
  uint8_t occCount = 0;
  for (uint8_t i = 0; i < 3; i++) if (occupied[i]) occCount++;
  uint8_t freeCount = 3 - occCount;
  uint8_t arr15  = sum15(entryB);
  uint8_t out15  = sum15(exitB);
  uint8_t light  = lightLevel();

  // Dim LED drive logic (Pin D4)
  digitalWrite(DIM_LED, (light == 1) ? HIGH : LOW);

  // 3) Model predictions
  uint8_t prob[3];
  for (uint8_t i = 0; i < 3; i++) {
    unsigned long units = occupied[i] ? (millis() - startMs[i]) / PARK_UNIT_MS : 0;
    if (units > 120) units = 120;
    prob[i] = predictFreeProb(hour, dow, i + 1, occupied[i] ? 1 : 0, (uint8_t)units,
                              occCount, arr15, out15, light);
  }

  // 4) Slot status LEDs and Full lot buzzer alert
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(GREEN[i], occupied[i] ? LOW : HIGH);
    digitalWrite(RED[i],   occupied[i] ? HIGH : LOW);
  }
  bool isFull = (freeCount == 0);
  if (isFull && !wasFull) {
    for (uint8_t k = 0; k < 2; k++) {
      digitalWrite(BUZZER, HIGH); waitAndPoll(120);
      digitalWrite(BUZZER, LOW);  waitAndPoll(120);
    }
  }
  wasFull = isFull;

  // 5) Slot recommendation logic
  uint8_t best = 0;
  if (!isFull) {
    for (uint8_t i = 0; i < 3; i++) if (!occupied[i]) { best = i; break; }
  } else {
    for (uint8_t i = 1; i < 3; i++) if (prob[i] > prob[best]) best = i;
  }

  // 6) LCD screen rotation
  if (millis() - lastPage >= PAGE_MS) { page = (page + 1) % 3; lastPage = millis(); }
  char buf[20];
  if (page == 0) {
    snprintf(buf, sizeof(buf), "S1:%c S2:%c S3:%c",
             occupied[0] ? 'X' : 'F', occupied[1] ? 'X' : 'F', occupied[2] ? 'X' : 'F');
    printRow(0, buf);
    if (!isFull) snprintf(buf, sizeof(buf), "Go to slot %d", best + 1);
    else         snprintf(buf, sizeof(buf), "FULL S%d next:%2d%%", best + 1, prob[best] > 99 ? 99 : prob[best]);
    printRow(1, buf);
  } else if (page == 1) {
    printRow(0, "Free in 15 min:");
    snprintf(buf, sizeof(buf), "%d%% %d%% %d%%",
             prob[0] > 99 ? 99 : prob[0], prob[1] > 99 ? 99 : prob[1], prob[2] > 99 ? 99 : prob[2]);
    printRow(1, buf);
  } else {
    snprintf(buf, sizeof(buf), "In:%2d Out:%2d 15m", arr15, out15);
    printRow(0, buf);
    snprintf(buf, sizeof(buf), "Light: %s", LIGHT_TXT[light]);
    printRow(1, buf);
  }

  // 7) Serial logging
  if (millis() - lastLog >= LOG_MS) {
    lastLog = millis();
    Serial.print(hour);   Serial.print(',');
    Serial.print(dow);    Serial.print(',');
    for (uint8_t i = 0; i < 3; i++) { Serial.print(occupied[i] ? 1 : 0); Serial.print(','); }
    Serial.print(arr15);  Serial.print(',');
    Serial.print(out15);  Serial.print(',');
    Serial.print(light);  Serial.print(',');
    Serial.print(prob[0]); Serial.print(',');
    Serial.print(prob[1]); Serial.print(',');
    Serial.println(prob[2]);
  }

  waitAndPoll(LOOP_MS);
}