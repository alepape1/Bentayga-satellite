// Includes the required libraries
#include <Wire.h>
#include <MS5x.h>
#include <MKRIMU.h>
#include <LoRa.h>
#include "Seeed_BME280.h"
#include <Arduino_MKRGPS.h>

// Strucuture of data for Sensor data from the satellite
struct SensorData {

  float roll;
  float pitch;
  float heading;
  float battTemp;
  float battTemp2;
  float temperature;
  float pressure;
  float humidity;
  float GpsAltitude;
  float latitude;
  float longitude;
  float speed;
  uint8_t numSatellites;
  int16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

// Initializes the serial communication and the LoRa module
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Init LoRa RF
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTimeout(1000);
  LoRa.setSpreadingFactor(7);

  Serial.println("LoRa Receiver");
}

void loop() {
  if (LoRa.parsePacket()) {
    SensorData sensorData;

    // Read the received data into the sensorData structure
    if (LoRa.readBytes((uint8_t*)&sensorData, sizeof(SensorData)) == sizeof(SensorData)) {
      // Process the received sensor data
      Serial.print(sensorData.year);
      Serial.print(",");
      Serial.print(sensorData.month);
      Serial.print(",");
      Serial.print(sensorData.day);
      Serial.print(",");
      Serial.print(sensorData.hour);
      Serial.print(",");
      Serial.print(sensorData.minute);
      Serial.print(",");
      Serial.print(sensorData.second);
      Serial.print(",");
      Serial.print(sensorData.pitch);
      Serial.print(",");
      Serial.print(sensorData.heading);
      Serial.print(",");
      Serial.print(sensorData.battTemp);
      Serial.print(",");
      Serial.print(sensorData.battTemp2);
      Serial.print(",");
      Serial.print(sensorData.temperature);
      Serial.print(",");
      Serial.print(sensorData.humidity);
      Serial.print(",");
      Serial.print(sensorData.pressure);
      Serial.print(",");
      Serial.print(sensorData.GpsAltitude);
      Serial.print(",");
      Serial.print(sensorData.latitude,6);
      Serial.print(",");
      Serial.print(sensorData.longitude,6);
      Serial.print(",");
      Serial.print(sensorData.speed,2);
      Serial.print(",");
      Serial.print(sensorData.numSatellites);
      Serial.print(",");
      Serial.print(LoRa.packetRssi());
      Serial.print(",");
      Serial.print(LoRa.packetSnr());
      Serial.println();
    }
  }
}
