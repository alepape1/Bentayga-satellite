// Includes the required libraries
#include <Wire.h>
#include <MS5x.h>
#include <MKRIMU.h>
#include <LoRa.h>
#include "Seeed_BME280.h"
#include <Arduino_MKRGPS.h>
//#include "CRC32.h" // CRC32 by Christopher Baker

// Strucuture of data for Sensor data from the satellite
struct SensorData {

  float roll;
  float pitch;
  float heading;
  // 5 DS18B20 temp sensors
  float battTempLeft; // Temperature of batteries inside heatmat, at the left, looking from the camera
  float battTempRight; // Temperature of batteries inside heatmat, at the right, looking from the camera
  float battTempDown; // Temperature of batteries inside heatmat, at the bottom, opposite to the battery box openning
  float battTempBox; // Temperature of batteries inside the box, outside the heatmats
  float TempCubesatDS18; // Temperature of the cubesat, taken from the DS18B20 sensor
  // the other temperature sensor, not DS18B20
  float temperature;
  float pressure;
  float humidity;
  float GpsAltitude;
  float latitude;
  float longitude;
  float speed;
  uint8_t numSatellites;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t camera_info;
//  uint32_t checksum; // CRC32
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
    uint32_t checksum;

    // Read the received data into the sensorData structure
    if (LoRa.readBytes((uint8_t*)&sensorData, sizeof(SensorData)) == sizeof(SensorData)) 
    {
      // Process the received sensor data
//      Serial.print(sensorData.roll);
//      Serial.print(",");
//      Serial.print(sensorData.pitch);
//      Serial.print(",");
//      Serial.print(sensorData.heading);
//      Serial.print(",");
//      Serial.print(sensorData.battTempLeft);
//      Serial.print(",");
//      Serial.print(sensorData.battTempRight);
//      Serial.print(",");
//      Serial.print(sensorData.battTempDown);
//      Serial.print(",");
//      Serial.print(sensorData.battTempBox);
//      Serial.print(",");
//      Serial.print(sensorData.TempCubesatDS18);
//      Serial.print(",");
//      Serial.print(sensorData.temperature);
//      Serial.print(",");
//      Serial.print(sensorData.pressure);
//      Serial.print(",");
//      Serial.print(sensorData.humidity);
//      Serial.print(",");
//      Serial.print(sensorData.GpsAltitude);
//      Serial.print(",");
//      Serial.print(sensorData.latitude);
//      Serial.print(",");
//      Serial.print(sensorData.longitude);
//      Serial.print(",");
//      Serial.print(sensorData.speed);
//      Serial.print(",");
//      Serial.print(sensorData.numSatellites);
//      Serial.print(",");
//      Serial.print(sensorData.year);
//      Serial.print(",");
//      Serial.print(sensorData.month);
//      Serial.print(",");
//      Serial.print(sensorData.day);
//      Serial.print(",");
//      Serial.print(sensorData.hour);
//      Serial.print(",");
//      Serial.print(sensorData.minute);
//      Serial.print(",");
//      Serial.print(sensorData.second);
//      Serial.print(",");
//      Serial.print(sensorData.camera_info);
//      Serial.print(",");
//      Serial.print(sensorData.checksum);
//      Serial.print(",");

      Serial.print("Roll: "); Serial.print(sensorData.roll);
      Serial.print("\tPitch: "); Serial.print(sensorData.pitch);
      Serial.print("\tHeading: "); Serial.println(sensorData.heading);
      
      Serial.print("Battery Temp Left: "); Serial.print(sensorData.battTempLeft);
      Serial.print("  Battery Temp Right: "); Serial.print(sensorData.battTempRight);
      Serial.print("  Battery Temp Down: "); Serial.print(sensorData.battTempDown);
      Serial.print("  Battery Temp Box: "); Serial.print(sensorData.battTempBox);
      Serial.print("  Temp Cubesat DS18: "); Serial.println(sensorData.TempCubesatDS18);
      
      Serial.print("Internal Temp: "); Serial.print(sensorData.temperature);
      Serial.print("\tPressure: "); Serial.print(sensorData.pressure);
      Serial.print("\tHumidity: "); Serial.println(sensorData.humidity);
      
      Serial.print("GPS Altitude: "); Serial.print(sensorData.GpsAltitude);
      Serial.print("\tLatitude: "); Serial.print(sensorData.latitude);
      Serial.print("\tLongitude: "); Serial.print(sensorData.longitude);
      Serial.print("\tSpeed: "); Serial.print(sensorData.speed);
      Serial.print("\tSatellites: "); Serial.println(sensorData.numSatellites);
      
      Serial.print("Date-Time: ");
      Serial.print(sensorData.year); Serial.print("-"); Serial.print(sensorData.month); Serial.print("-");
      Serial.print(sensorData.day); Serial.print(" ");
      Serial.print(sensorData.hour); Serial.print(":"); Serial.print(sensorData.minute); Serial.print(":");
      Serial.print(sensorData.second);
      
      Serial.print("\tCamera Info: "); Serial.print(sensorData.camera_info);

      Serial.print("\tRssi: ");  Serial.print(LoRa.packetRssi());
      Serial.print("\tSNR: ");  Serial.println(LoRa.packetSnr());
      
      Serial.println();

//      checksum = CRC32::calculate((uint8_t *)&sensorData, sizeof(sensorData) - sizeof(sensorData.checksum));
      
//      if (checksum == sensorData.checksum)
//      {Serial.println("Data Integrity PASSED");}
//      else {Serial.println("Data Integrity FAILED");}

      Serial.println();
    }
  }
}
