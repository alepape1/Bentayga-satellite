#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <LoRa.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define SAMPLE_SENSOR_TIME 5000

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;

SFE_UBLOX_GNSS myGNSS;


struct SensorData {

  float roll;
  float pitch;
  float heading;
  float temperature;
  float pressure;
  float humidity;
  float GpsAltitude;
  float latitude;
  float longitude;
  float speed;
  uint8_t numSatellites;

};

SensorData sensorData;

// Definir el contador para el sampling del barometro y GPS
uint32_t sample_counter = 0;

void IMU_calibration();
void IMU_get_values(SensorData &sensorData);


void setup() {
  Serial.begin(115200);
  delay(2000);

  if (!bno.begin())
  {
    Serial.println("ERROR: BNO055 NOT detected");
    
  }else{
    Serial.println("BNO55 IMU sensor connected. Status: OK");
    IMU_calibration();  
  
  }

  if (!myGNSS.begin()) {
    Serial.println("Failed to initialize GPS!");
  }else{
    Serial.println("GPS System init. Status : OK");
  }

   if (!bme.begin(0x76)) { // la dirección puede ser 0x77 dependiendo de tu módulo
    Serial.println("ERROR: Failed to initialize the barometer BME280");
  } else {
    Serial.println("BME280 Baromete Sensor init for pressure and temperature external. Status: OK");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("ERROR: Starting LoRa failed!");
  }else{
    Serial.println("LoRa Transceiver. Status: OK");
  }

  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(7);

  delay(2000);

  sample_counter = millis();
}

void loop() {

  
  IMU_get_values(sensorData);
  
  if(millis() - sample_counter >= SAMPLE_SENSOR_TIME){

  sensorData.temperature = bme.readTemperature();
  sensorData.humidity = bme.readHumidity();
  sensorData.pressure = bme.readPressure() / 100.0F;

  // if (myGNSS.getPVT()) {
    sensorData.latitude = myGNSS.getLatitude() / 10000000.0;
    sensorData.longitude = myGNSS.getLongitude() / 10000000.0;
    sensorData.GpsAltitude = myGNSS.getAltitude() / 1000.0;
    sensorData.speed = myGNSS.getGroundSpeed() / 1000.0;
    sensorData.numSatellites = myGNSS.getSIV();
  // }

  sample_counter = millis();

  }

  String data = "Roll:" + String(sensorData.roll) + ","
                + "Pitch:" + String(sensorData.pitch) + ","
                + "Heading:" + String(sensorData.heading) + ","
                + "Temperature:" + String(sensorData.temperature) + ","
                + "Humidity:" + String(sensorData.humidity) + ","
                + "Pressure:" + String(sensorData.pressure) + ","
                + "GpsAltitude:" + String(sensorData.GpsAltitude)+ ","
                + "Latitude:" + String(sensorData.latitude, 6) + ","
                + "Longitude:" + String(sensorData.longitude, 6) + ","
                + "Speed:" + String(sensorData.speed) + ","
                + "NumSatellites:" + String(sensorData.numSatellites);

Serial.println(data);
packData_and_send(sensorData);


}

void packData_and_send(SensorData sensorData) {
LoRa.beginPacket();
LoRa.write((uint8_t*)&sensorData, sizeof(SensorData));
LoRa.endPacket();
}

void IMU_get_values(SensorData &sensorData){

    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  sensorData.roll = orientationData.orientation.y;
  sensorData.pitch = orientationData.orientation.z;
  sensorData.heading = orientationData.orientation.x;


}


void IMU_calibration(){

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

}
