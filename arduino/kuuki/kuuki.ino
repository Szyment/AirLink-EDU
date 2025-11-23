/*
  Air Quality Logger - Production
  Arduino UNO R4 / Feather 32u4 + FONA + BME280 + PMS7003
  
  Upload to ThingSpeak every 15 seconds via GPRS
  
  Debug mode: Set DEBUG to 1 (requires Serial Monitor @ 115200)
  Production: Set DEBUG to 0 (no serial output, LED only)
*/

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"

// ===== DEBUG MODE =====
#define DEBUG 0  // 1 = debug logs, 0 = production (no serial)

#if DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_BEGIN(x)    Serial.begin(x); while(!Serial) delay(10)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_BEGIN(x)
#endif

// ===== CONFIG =====
#define THINGSPEAK_KEY  "6X1TNE0ORF0JMW0Q"
#define THINGSPEAK_HOST "api.thingspeak.com"
#define GPRS_APN        "internet"
#define UPLOAD_INTERVAL 15000 //900000      // 15 min
#define BME_ADDR        0x76
#define SEA_LEVEL_HPA   1013.25
#define NUM_SAMPLES     5
#define PMS_TIMEOUT     2000
#define MIN_SAMPLES     3

// ===== PINS =====
#define FONA_RX   9
#define FONA_TX   8
#define FONA_RST  4
#define PMS_RX    10
#define PMS_TX    11

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
  int pm1_0, pm2_5, pm10_0;
  float temp, pressure, humidity, altitude;
} data;

// ===== LED PATTERNS =====
enum LedPattern {
  LED_OFF,
  LED_ON,
  LED_BLINK_FAST,    // 100ms - BME280 error
  LED_BLINK_MEDIUM,  // 200ms - FONA error
  LED_BLINK_SLOW,    // 500ms - working
  LED_PULSE          // success
};

void setLed(LedPattern pattern) {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  switch (pattern) {
    case LED_OFF:
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case LED_ON:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case LED_BLINK_FAST:
    case LED_BLINK_MEDIUM:
    case LED_BLINK_SLOW: {
      unsigned long interval = (pattern == LED_BLINK_FAST) ? 100 : 
                               (pattern == LED_BLINK_MEDIUM) ? 200 : 500;
      if (millis() - lastBlink > interval) {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
        lastBlink = millis();
      }
      break;
    }
    case LED_PULSE:
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      break;
  }
}

// ===== PMS7003 =====

bool readPMS() {
  static uint8_t buffer[32];
  
  unsigned long start = millis();
  while (millis() - start < PMS_TIMEOUT) {
    if (pmsSerial.available() && pmsSerial.read() == 0x42) {
      if (pmsSerial.available() && pmsSerial.read() == 0x4D) {
        if (pmsSerial.readBytes(buffer, 30) != 30) return false;
        
        uint16_t sum = 0x42 + 0x4D;
        for (uint8_t i = 0; i < 28; i++) sum += buffer[i];
        if (sum != ((buffer[28] << 8) | buffer[29])) return false;
        
        pmsData.pm1_0  = (buffer[2] << 8) | buffer[3];
        pmsData.pm2_5  = (buffer[4] << 8) | buffer[5];
        pmsData.pm10_0 = (buffer[6] << 8) | buffer[7];
        return true;
      }
    }
  }
  return false;
}

bool collectData() {
  DEBUG_PRINTLN(F("\n=== Collecting data ==="));
  
  uint32_t sum_pm1 = 0, sum_pm25 = 0, sum_pm10 = 0;
  uint8_t valid = 0;
  
  pmsSerial.begin(9600);
  while (pmsSerial.available()) pmsSerial.read();
  
  for (uint8_t i = 0; i < NUM_SAMPLES * 2 && valid < NUM_SAMPLES; i++) {
    if (readPMS()) {
      sum_pm1  += pmsData.pm1_0;
      sum_pm25 += pmsData.pm2_5;
      sum_pm10 += pmsData.pm10_0;
      valid++;
      DEBUG_PRINT(F("."));
    } else {
      DEBUG_PRINT(F("x"));
    }
    delay(1000);
  }
  DEBUG_PRINTLN();
  
  pmsSerial.end();
  
  if (valid < MIN_SAMPLES) {
    DEBUG_PRINTLN(F("Insufficient PMS samples"));
    return false;
  }
  
  data.pm1_0  = sum_pm1  / valid;
  data.pm2_5  = sum_pm25 / valid;
  data.pm10_0 = sum_pm10 / valid;
  
  data.temp     = bme.readTemperature();
  data.pressure = bme.readPressure();
  data.humidity = bme.readHumidity();
  data.altitude = bme.readAltitude(SEA_LEVEL_HPA);
  
  #if DEBUG
    DEBUG_PRINT(F("PM: ")); DEBUG_PRINT(data.pm1_0); DEBUG_PRINT(F(",")); 
    DEBUG_PRINT(data.pm2_5); DEBUG_PRINT(F(",")); DEBUG_PRINTLN(data.pm10_0);
    DEBUG_PRINT(F("T=")); DEBUG_PRINT(data.temp); 
    DEBUG_PRINT(F(" RH=")); DEBUG_PRINT(data.humidity);
    DEBUG_PRINT(F(" P=")); DEBUG_PRINTLN(data.pressure);
  #endif
  
  return true;
}

// ===== FONA =====

bool initFONA() {
  DEBUG_PRINTLN(F("\n=== Init FONA ==="));
  
  fonaSS.begin(4800);
  if (!fona.begin(fonaSS)) return false;
  
  DEBUG_PRINTLN(F("FONA OK"));
  
  for (uint8_t i = 0; i < 30; i++) {
    uint8_t n = fona.getNetworkStatus();
    if (n == 1 || n == 5) {
      DEBUG_PRINTLN(F("Network OK"));
      return true;
    }
    delay(1000);
  }
  return false;
}

bool enableGPRS() {
  DEBUG_PRINTLN(F("\n=== Enable GPRS ==="));
  
  fonaSS.begin(4800);
  
  // Clean shutdown of old connection
  fona.enableGPRS(false);
  delay(3000);
  
  fona.setGPRSNetworkSettings(F(GPRS_APN), F(""), F(""));
  
  if (!fona.enableGPRS(true)) {
    DEBUG_PRINTLN(F("GPRS failed"));
    fonaSS.end();
    return false;
  }
  
  DEBUG_PRINTLN(F("GPRS OK"));
  delay(1000);
  return true;
}

bool upload() {
  DEBUG_PRINTLN(F("\n=== Upload ==="));
  
  char url[256];
  char t[10], p[10], h[10], a[10];
  
  dtostrf(data.temp, 1, 1, t);
  dtostrf(data.pressure, 1, 0, p);
  dtostrf(data.humidity, 1, 0, h);
  dtostrf(data.altitude, 1, 1, a);
  
  snprintf(url, sizeof(url),
    "%s/update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%s&field5=%s&field6=%s&field7=%s",
    THINGSPEAK_HOST, THINGSPEAK_KEY,
    data.pm1_0, data.pm2_5, data.pm10_0, t, p, h, a);
  
  uint16_t status;
  int16_t length;
  
  fonaSS.begin(4800);
  
  if (!fona.HTTP_GET_start(url, &status, (uint16_t*)&length)) {
    DEBUG_PRINTLN(F("HTTP failed"));
    fonaSS.end();
    return false;
  }
  
  char response[2] = {0};
  if (fona.available()) response[0] = fona.read();
  
  fona.HTTP_GET_end();
  
  #if DEBUG
    DEBUG_PRINT(F("Status: ")); DEBUG_PRINTLN(status);
    DEBUG_PRINT(F("Response: ")); DEBUG_PRINTLN(response);
  #endif
  
  return (status == 200 && response[0] != '0');
}

void cleanupGPRS() {
  DEBUG_PRINTLN(F("Cleanup GPRS"));
  
  fonaSS.begin(4800);
  fona.HTTP_GET_end();
  delay(500);
  fona.enableGPRS(false);
  delay(2000);
  fonaSS.end();
}

// ===== SETUP =====

void setup() {
  DEBUG_BEGIN(115200);
  DEBUG_PRINTLN(F("\n=== Air Quality Logger ==="));
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Init BME280
  Wire.begin();
  if (!bme.begin(BME_ADDR) && !bme.begin(0x77)) {
    DEBUG_PRINTLN(F("BME280 error"));
    while (1) setLed(LED_BLINK_FAST);
  }
  DEBUG_PRINTLN(F("BME280 OK"));
  
  // Init FONA
  for (uint8_t i = 0; i < 3; i++) {
    if (initFONA()) goto ok;
    delay(5000);
  }
  DEBUG_PRINTLN(F("FONA error"));
  while (1) setLed(LED_BLINK_MEDIUM);
  
ok:
  fonaSS.end();
  DEBUG_PRINTLN(F("Setup OK\n"));
  setLed(LED_PULSE);
}

// ===== LOOP =====

void loop() {
  DEBUG_PRINTLN(F("\n========== CYCLE =========="));
  
  setLed(LED_ON);
  
  if (collectData()) {
    if (enableGPRS()) {
      if (upload()) {
        DEBUG_PRINTLN(F("✓ Success"));
        setLed(LED_PULSE);
      } else {
        DEBUG_PRINTLN(F("✗ Upload failed"));
      }
      cleanupGPRS();
    } else {
      DEBUG_PRINTLN(F("✗ GPRS failed"));
    }
  } else {
    DEBUG_PRINTLN(F("✗ Data collection failed"));
  }
  
  setLed(LED_OFF);
  
  DEBUG_PRINT(F("Sleep ")); DEBUG_PRINT(UPLOAD_INTERVAL/1000); DEBUG_PRINTLN(F("s"));
  delay(UPLOAD_INTERVAL);
}
