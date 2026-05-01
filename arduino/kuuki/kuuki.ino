/*
  Air Quality Logger - Production v2 (refactored)
  Adafruit Feather 32u4 + FONA 808 + BME280 + PMS7003
  
  Features:
  - Watchdog timer (8s) prevents hangs in the field
  - Soft reset instead of infinite loop on FONA failure
  - 5x30s retry window for FONA in setup
  - PMS7003 sleep mode (sensor wakes only for measurement)
  - BME280 sanity check + auto-reinit on stuck I2C
  - Conditional debug (works without USB, prints with Serial Monitor)
  - Long delays split into chunks to feed watchdog
  
  Wiring (PMS via ST1167 level shifter):
    purple (VCC)     -> Feather USB pin (5V)
    orange (GND)     -> common GND
    blue (RX)        -> conv. LV1 RXO <- HV1 RXI <- Feather D10
    green (TX)       -> conv. LV1 TXI -> HV1 TXO -> Feather D11
    white (SET)      -> conv. LV2 TXI <- HV2 TXO <- Feather D12
                        (MOSFET channel, NOT RXI/RXO - the voltage divider
                         gives intermediate states 1.99V/2.65V instead of
                         clean 0V/3.3V)
    yellow (RESET)   -> not connected (PMS internal pull-up)
    red, black       -> not connected (NP)
  
  Level shifter:
    HV -> Feather USB pin (5V)
    LV -> Feather 3V pin (3.3V)
    GND x2 -> common GND
  
  10k pull-up between white SET and 3.3V (ensures HIGH at startup)
  100uF/10V capacitor between purple and orange close to adapter
  
  ThingSpeak fields:
    field1 = PM1.0  (ug/m3, atmospheric)
    field2 = PM2.5  (ug/m3, atmospheric)
    field3 = PM10   (ug/m3, atmospheric)
    field4 = Temperature (deg C)
    field5 = Pressure (Pa)
    field6 = Humidity (%)
    field7 = Altitude (m, computed from pressure at SEA_LEVEL_HPA)
  
  LED patterns:
    Solid ON           - cycle active (collecting/uploading)
    Short double pulse - cycle success
    Fast blink (100ms) - BME280 error (resetting in 30s)
    Med blink (200ms)  - FONA error (resetting in 60s)
    OFF                - sleeping between cycles
*/

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include <avr/wdt.h>
#include "secrets.h"   // THINGSPEAK_KEY and GPRS_APN

// ===== DEBUG =====
#define DBG_PRINT(x)    if (Serial) Serial.print(x)
#define DBG_PRINTLN(x)  if (Serial) Serial.println(x)

// ===== CONFIG =====
#define THINGSPEAK_HOST "api.thingspeak.com"
#define UPLOAD_INTERVAL 60000UL    // 1 min sleep between cycles
#define BME_ADDR        0x76
#define SEA_LEVEL_HPA   1013.25
#define NUM_SAMPLES     5
#define PMS_TIMEOUT     2000
#define MIN_SAMPLES     3
#define PMS_WARMUP      30000      // Plantower datasheet: 30s warmup after wake

// ===== PMS SLEEP POLARITY =====
#define PMS_ACTIVE  HIGH
#define PMS_SLEEP   LOW

// ===== FONA RETRY =====
#define FONA_RETRY_COUNT    5
#define FONA_RETRY_DELAY    30000UL    // 30s between retries

// ===== PINS =====
#define FONA_RX   9
#define FONA_TX   8
#define FONA_RST  4
#define PMS_RX    10
#define PMS_TX    11
#define PMS_SET   12       // GPIO controlling PMS7003 sleep mode

// ===== HARDWARE =====
Adafruit_BME280 bme;
SoftwareSerial fonaSS(FONA_TX, FONA_RX);
SoftwareSerial pmsSerial(PMS_TX, PMS_RX);
Adafruit_FONA fona(FONA_RST);

// ===== DATA =====
struct {
  uint16_t pm1_0, pm2_5, pm10_0;
} pmsData;

struct {
  uint16_t pm1_0, pm2_5, pm10_0;     // uint16_t: PMS spec max 65535 ug/m3
  float temp, pressure, humidity, altitude;
} data;

// ===== LED PATTERN ENUM (before any function using it) =====
enum LedPattern { LED_OFF, LED_ON, LED_BLINK_FAST, LED_BLINK_MEDIUM, LED_PULSE };

// ===== UTILITIES =====

// Safe delay that feeds watchdog. Use instead of delay() for >4s waits.
void wdtDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    wdt_reset();
    delay(min(ms - (millis() - start), 1000UL));
  }
}

// Soft reset MCU - jumps to bootloader address
void softReset() {
  DBG_PRINTLN(F("SOFT RESET"));
  if (Serial) Serial.flush();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void setLed(LedPattern pattern) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  switch (pattern) {
    case LED_OFF: digitalWrite(LED_BUILTIN, LOW); break;
    case LED_ON:  digitalWrite(LED_BUILTIN, HIGH); break;
    case LED_BLINK_FAST:
    case LED_BLINK_MEDIUM: {
      unsigned long interval = (pattern == LED_BLINK_FAST) ? 100 : 200;
      if (millis() - lastBlink > interval) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = millis();
      }
      break;
    }
    case LED_PULSE:
      for (uint8_t i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW);  delay(100);
        wdt_reset();
      }
      break;
  }
}

// ===== PMS7003 =====

bool readPMS() {
  static uint8_t buffer[32];
  unsigned long start = millis();
  while (millis() - start < PMS_TIMEOUT) {
    wdt_reset();
    if (pmsSerial.available() && pmsSerial.read() == 0x42) {
      if (pmsSerial.available() && pmsSerial.read() == 0x4D) {
        if (pmsSerial.readBytes(buffer, 30) != 30) return false;
        uint16_t sum = 0x42 + 0x4D;
        for (uint8_t i = 0; i < 28; i++) sum += buffer[i];
        if (sum != ((buffer[28] << 8) | buffer[29])) return false;
        pmsData.pm1_0  = (buffer[8]  << 8) | buffer[9];   // atmospheric PM1.0
        pmsData.pm2_5  = (buffer[10] << 8) | buffer[11];  // atmospheric PM2.5
        pmsData.pm10_0 = (buffer[12] << 8) | buffer[13];  // atmospheric PM10
        return true;
      }
    }
  }
  return false;
}

bool collectData() {
  DBG_PRINTLN(F("\n=== Collecting data ==="));
  
  uint32_t sum_pm1 = 0, sum_pm25 = 0, sum_pm10 = 0;
  uint8_t valid = 0;
  
  // 1. WAKE UP PMS
  DBG_PRINTLN(F("PMS WAKE"));
  digitalWrite(PMS_SET, PMS_ACTIVE);
  
  pmsSerial.begin(9600);
  pmsSerial.listen();
  
  // 2. WARMUP - 30s, with watchdog feeding
  DBG_PRINT(F("Warmup "));
  DBG_PRINT(PMS_WARMUP / 1000);
  DBG_PRINTLN(F("s..."));
  
  unsigned long warmupStart = millis();
  uint16_t totalDrained = 0;
  while (millis() - warmupStart < PMS_WARMUP) {
    wdt_reset();
    while (pmsSerial.available()) {
      pmsSerial.read();
      totalDrained++;
    }
    delay(500);
  }
  DBG_PRINT(F("Drained: ")); DBG_PRINTLN(totalDrained);
  
  // 3. SAMPLING
  for (uint8_t i = 0; i < NUM_SAMPLES * 3 && valid < NUM_SAMPLES; i++) {
    wdt_reset();
    if (readPMS()) {
      bool allZero = (pmsData.pm1_0 == 0 && pmsData.pm2_5 == 0 && pmsData.pm10_0 == 0);
      
      DBG_PRINT(F("  s")); DBG_PRINT(valid + 1);
      DBG_PRINT(F(": ")); DBG_PRINT(pmsData.pm1_0);
      DBG_PRINT(F(",")); DBG_PRINT(pmsData.pm2_5);
      DBG_PRINT(F(",")); DBG_PRINT(pmsData.pm10_0);
      
      if (allZero) {
        DBG_PRINTLN(F(" REJECTED"));
      } else {
        DBG_PRINTLN(F(""));
        sum_pm1  += pmsData.pm1_0;
        sum_pm25 += pmsData.pm2_5;
        sum_pm10 += pmsData.pm10_0;
        valid++;
      }
    } else {
      DBG_PRINTLN(F("  s FAIL"));
    }
    delay(1000);
  }
  
  // 4. SLEEP PMS - SET first (sensor stops fan), then close UART
  digitalWrite(PMS_SET, PMS_SLEEP);
  delay(50);  // let sensor settle
  pmsSerial.end();
  DBG_PRINTLN(F("PMS SLEEP"));
  
  if (valid < MIN_SAMPLES) {
    DBG_PRINT(F("Insufficient: ")); DBG_PRINTLN(valid);
    return false;
  }
  
  data.pm1_0  = sum_pm1  / valid;
  data.pm2_5  = sum_pm25 / valid;
  data.pm10_0 = sum_pm10 / valid;
  
  // 5. BME280 with full sanity check + reinit
  data.temp     = bme.readTemperature();
  data.pressure = bme.readPressure();
  data.humidity = bme.readHumidity();
  data.altitude = bme.readAltitude(SEA_LEVEL_HPA);
  
  bool bmeBad = (data.pressure < 87000 || data.pressure > 108600 ||
                 data.temp < -40 || data.temp > 80 ||
                 data.humidity < 0 || data.humidity > 100 ||
                 data.altitude < -500 || data.altitude > 8000);
  
  if (bmeBad) {
    DBG_PRINTLN(F("BME280 BAD - reinit"));
    Wire.end();
    delay(100);
    Wire.begin();
    delay(100);
    if (bme.begin(BME_ADDR) || bme.begin(0x77)) {
      DBG_PRINTLN(F("BME280 reinit OK"));
      data.temp     = bme.readTemperature();
      data.pressure = bme.readPressure();
      data.humidity = bme.readHumidity();
      data.altitude = bme.readAltitude(SEA_LEVEL_HPA);
    } else {
      DBG_PRINTLN(F("BME280 reinit FAIL"));
      return false;
    }
  }
  
  DBG_PRINT(F("AVG ")); DBG_PRINT(data.pm1_0);
  DBG_PRINT(F(",")); DBG_PRINT(data.pm2_5);
  DBG_PRINT(F(",")); DBG_PRINTLN(data.pm10_0);
  DBG_PRINT(F("T=")); DBG_PRINT(data.temp);
  DBG_PRINT(F(" RH=")); DBG_PRINT(data.humidity);
  DBG_PRINT(F(" P=")); DBG_PRINTLN(data.pressure);
  
  return true;
}

// ===== FONA =====

bool initFONA() {
  DBG_PRINTLN(F("\n=== Init FONA ==="));
  fonaSS.begin(4800);
  if (!fona.begin(fonaSS)) return false;
  for (uint8_t i = 0; i < 30; i++) {
    wdt_reset();
    uint8_t n = fona.getNetworkStatus();
    if (n == 1 || n == 5) {
      DBG_PRINTLN(F("Network OK"));
      return true;
    }
    delay(1000);
  }
  return false;
}

bool enableGPRS() {
  DBG_PRINTLN(F("\n=== GPRS ==="));
  fonaSS.begin(4800);
  fona.enableGPRS(false);
  wdtDelay(3000);
  fona.setGPRSNetworkSettings(F(GPRS_APN), F(""), F(""));
  if (!fona.enableGPRS(true)) {
    DBG_PRINTLN(F("GPRS FAIL"));
    fonaSS.end();
    return false;
  }
  DBG_PRINTLN(F("GPRS OK"));
  delay(1000);
  return true;
}

bool upload() {
  DBG_PRINTLN(F("\n=== Upload ==="));
  
  char url[256];
  char t[10], p[10], h[10], a[10];
  
  dtostrf(data.temp, 1, 1, t);
  dtostrf(data.pressure, 1, 0, p);
  dtostrf(data.humidity, 1, 0, h);
  dtostrf(data.altitude, 1, 1, a);
  
  int written = snprintf(url, sizeof(url),
    "%s/update?api_key=%s&field1=%u&field2=%u&field3=%u&field4=%s&field5=%s&field6=%s&field7=%s",
    THINGSPEAK_HOST, THINGSPEAK_KEY,
    data.pm1_0, data.pm2_5, data.pm10_0, t, p, h, a);
  
  DBG_PRINT(F("URL: ")); DBG_PRINTLN(url);
  
  if (written < 0 || (size_t)written >= sizeof(url)) {
    DBG_PRINTLN(F("URL TRUNCATED!"));
    return false;
  }
  
  uint16_t status, length;
  fonaSS.begin(4800);
  
  if (!fona.HTTP_GET_start(url, &status, &length)) {
    DBG_PRINTLN(F("HTTP FAIL"));
    fona.HTTP_GET_end();  // safety: close session even on failure
    fonaSS.end();
    return false;
  }
  
  // Read full response with pacing to reduce CPU load
  char response[16] = {0};
  uint8_t i = 0;
  unsigned long readStart = millis();
  while (millis() - readStart < 2000 && i < sizeof(response) - 1) {
    wdt_reset();
    if (fona.available()) {
      response[i++] = fona.read();
    } else {
      delay(5);
    }
  }
  
  fona.HTTP_GET_end();
  
  DBG_PRINT(F("Status=")); DBG_PRINT(status);
  DBG_PRINT(F(" entry=")); DBG_PRINTLN(response);
  
  return (status == 200 && response[0] != '0' && response[0] != 0);
}

void cleanupGPRS() {
  fonaSS.begin(4800);
  fona.HTTP_GET_end();
  delay(500);
  fona.enableGPRS(false);
  wdtDelay(2000);
  fonaSS.end();
  delay(100);
}

// ===== SETUP =====

void setup() {
  // Disable watchdog first - might be running from previous reset
  wdt_disable();
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // PMS SET pin - sleep on boot
  pinMode(PMS_SET, OUTPUT);
  digitalWrite(PMS_SET, PMS_SLEEP);
  
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  
  DBG_PRINTLN(F("\n=== Air Quality Logger v2 ==="));
  DBG_PRINTLN(F("PMS sleep mode: enabled"));
  DBG_PRINTLN(F("Watchdog: 8s"));
  
  // Enable watchdog now - 8s timeout protects against hangs
  wdt_enable(WDTO_8S);
  
  Wire.begin();
  if (!bme.begin(BME_ADDR) && !bme.begin(0x77)) {
    DBG_PRINTLN(F("BME280 ERR - reset in 30s"));
    for (uint8_t i = 0; i < 30; i++) {
      wdt_reset();
      setLed(LED_BLINK_FAST);
      delay(1000);
    }
    softReset();
  }
  DBG_PRINTLN(F("BME280 OK"));
  
  // FONA init with longer retry window: 5 attempts x 30s
  bool fonaOk = false;
  for (uint8_t i = 0; i < FONA_RETRY_COUNT; i++) {
    DBG_PRINT(F("FONA attempt ")); DBG_PRINT(i + 1);
    DBG_PRINT(F("/")); DBG_PRINTLN(FONA_RETRY_COUNT);
    if (initFONA()) { fonaOk = true; break; }
    if (i < FONA_RETRY_COUNT - 1) {
      DBG_PRINTLN(F("Retry in 30s..."));
      wdtDelay(FONA_RETRY_DELAY);
    }
  }
  
  if (!fonaOk) {
    DBG_PRINTLN(F("FONA ERR - reset in 60s"));
    // Slow blink for 60s, then soft reset to try again
    for (uint8_t i = 0; i < 60; i++) {
      wdt_reset();
      setLed(LED_BLINK_MEDIUM);
      delay(1000);
    }
    softReset();
  }
  
  fonaSS.end();
  DBG_PRINTLN(F("Setup OK\n"));
  setLed(LED_PULSE);
}

// ===== LOOP =====

void loop() {
  wdt_reset();
  DBG_PRINTLN(F("\n========== CYCLE =========="));
  setLed(LED_ON);
  
  if (collectData()) {
    if (enableGPRS()) {
      if (upload()) {
        DBG_PRINTLN(F("OK"));
        setLed(LED_PULSE);
      } else {
        DBG_PRINTLN(F("Upload FAIL"));
      }
      cleanupGPRS();
    } else {
      DBG_PRINTLN(F("GPRS FAIL"));
    }
  } else {
    DBG_PRINTLN(F("Data FAIL"));
  }
  
  setLed(LED_OFF);
  
  // Sleep with watchdog feeding
  DBG_PRINT(F("Sleep ")); DBG_PRINT(UPLOAD_INTERVAL / 1000); DBG_PRINTLN(F("s"));
  wdtDelay(UPLOAD_INTERVAL);
}
