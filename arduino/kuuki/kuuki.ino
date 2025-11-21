/*
  Air Quality Logger - Production Version
  Hardware: Arduino UNO R4 + BME280 (I2C) + PMS7003 (UART)
  
  CSV Output Format (9600 baud):
    pm1_0,pm2_5,pm10_0,temp_c,press_pa,humidity_pct,alt_m,checksum
  
  Checksum: XOR of all characters (excluding commas)
*/

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>

// Configuration
#define BME_ADDR          0x76
#define SEA_LEVEL_HPA     1013.25
#define NUM_SAMPLES       5
#define SAMPLE_INTERVAL   15000
#define PMS_TIMEOUT       2000
#define MIN_VALID_SAMPLES 3

// Hardware
Adafruit_BME280 bme;
SoftwareSerial pmsSerial(8, 9);

// PMS7003 Data
struct PMSData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10_0;
} pmsData;

// Read PMS7003 frame with validation
bool readPMS() {
  static uint8_t buffer[32];
  
  // Wait for start frame with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < PMS_TIMEOUT) {
    if (pmsSerial.available() && pmsSerial.read() == 0x42) {
      if (pmsSerial.available() && pmsSerial.read() == 0x4D) {
        goto frame_found;
      }
    }
  }
  return false;
  
frame_found:
  // Read frame data
  if (pmsSerial.readBytes(buffer, 30) != 30) return false;
  
  // Verify checksum
  uint16_t sum = 0x42 + 0x4D;
  for (uint8_t i = 0; i < 28; i++) sum += buffer[i];
  uint16_t checksum = (buffer[28] << 8) | buffer[29];
  if (sum != checksum) return false;
  
  // Extract PM data (CF=1 mode)
  pmsData.pm1_0  = (buffer[2] << 8) | buffer[3];
  pmsData.pm2_5  = (buffer[4] << 8) | buffer[5];
  pmsData.pm10_0 = (buffer[6] << 8) | buffer[7];
  
  return true;
}

// Calculate XOR checksum
uint8_t calculateChecksum(const char* str) {
  uint8_t checksum = 0;
  while (*str) {
    if (*str != ',') checksum ^= *str;
    str++;
  }
  return checksum;
}

void setup() {
  Serial.begin(9600);
  pmsSerial.begin(9600);
  
  delay(1000);
  
  // Initialize BME280
  if (!bme.begin(BME_ADDR)) {
    while (1) delay(1000); // Halt on critical error
  }
  
  // Output CSV header
  Serial.println(F("pm1_0,pm2_5,pm10_0,temp_c,press_pa,humidity_pct,alt_m,checksum"));
  
  // Clear PMS buffer
  while (pmsSerial.available()) pmsSerial.read();
}

void loop() {
  uint32_t sum_pm1 = 0, sum_pm25 = 0, sum_pm10 = 0;
  uint8_t validSamples = 0;
  
  // Collect valid samples
  for (uint8_t i = 0; i < NUM_SAMPLES * 2 && validSamples < NUM_SAMPLES; i++) {
    if (readPMS()) {
      sum_pm1  += pmsData.pm1_0;
      sum_pm25 += pmsData.pm2_5;
      sum_pm10 += pmsData.pm10_0;
      validSamples++;
    }
    delay(1000);
  }
  
  // Skip cycle if insufficient valid samples
  if (validSamples < MIN_VALID_SAMPLES) {
    delay(SAMPLE_INTERVAL);
    return;
  }
  
  // Calculate averages
  uint16_t pm1_avg  = sum_pm1  / validSamples;
  uint16_t pm25_avg = sum_pm25 / validSamples;
  uint16_t pm10_avg = sum_pm10 / validSamples;
  
  // Read environmental data
  float temp = bme.readTemperature();
  float pres = bme.readPressure();
  float humi = bme.readHumidity();
  float alti = bme.readAltitude(SEA_LEVEL_HPA);
  
  // Build CSV line
  char csvLine[128];
  snprintf(csvLine, sizeof(csvLine), 
           "%u,%u,%u,%.2f,%.2f,%.2f,%.2f",
           pm1_avg, pm25_avg, pm10_avg, temp, pres, humi, alti);
  
  // Calculate and append checksum
  uint8_t checksum = calculateChecksum(csvLine);
  
  Serial.print(csvLine);
  Serial.print(',');
  Serial.println(checksum, HEX);
  
  delay(SAMPLE_INTERVAL);
}
