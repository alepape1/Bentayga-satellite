// Includes the required libraries
#include <Wire.h>
#include <MS5x.h>
#include <MKRIMU.h>
#include <LoRa.h>
#include "Seeed_BME280.h"
#include <Arduino_MKRGPS.h>


// Initializes the barometer sensor and sets up the connection status variable
MS5x barometer(&Wire);

// Initizes the barometer sensor and Temperature for batteries
//BME280 bme280;

// Strucuture of data for Sensor data from the satellite
struct SensorData {
  float roll;
  float pitch;
  float heading;
  float temperature;
  float pressure;
  float seaLevelPressure;
  float altitude;
  float correctedAltitude;
  float latitude;
  float longitude;
  float speed;
  uint8_t numSatellites;
};


SensorData sensorData;

// Flag barometer status
bool barometerConnected = false;

// Stores the time of the last connection attempt, delay time between attempts, and the last time the device was polled
uint32_t prevConnectionAttempt = 0;
uint32_t connectionAttemptDelay = 500;  // Time in ms to wait between connection attempts to the sensor
uint32_t prevTime = 0;                  // The time, in MS the device was last polled

// Stores the value of the pressure and temperature the last time the sensor was polled
double prevPressure = 0;
double prevTemperature = 0;

// Stores the sea level pressure, used for altitude calculations
double seaLevelPressure = 0;

// Stores data IMU parameters
float heading, roll, pitch;
float pressure_batt = 0;

bool barometer_init(void);

//void packData_and_send(float roll, float pitch, float heading, float temperature, float pressure, float seaLevelPressure, float altitude, float correctedAltitude);
void packData_and_send(SensorData sensorData);

// Initializes the serial communication and the LoRa module
void setup() {

  Serial.begin(115200);

  delay(1000);
  // Info init about SW
  Serial.println("Initializing Bentayga computer onboard");

  // GPS system init

  if (!GPS.begin()) {
    Serial.println("Failed to initialize GPS!");
  }

  // Init Pressure and temperature sensor BME280 ADDRESS (0x76)
  //if(!bme280.init()){
  //    Serial.println("Device error!");
  //}

  // Initializes the barometer MS5607 sensor and sets up its settings
  barometerConnected = barometer_init();

  // Init IMU inertial sensor
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }

  // Init LoRa RF
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
  }

  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(7);

  Serial.println("LoRa Sender");
}

void loop() {


  double pressure = 0;
  double temperature = 0;
  double altitude = 0;
  double correctedAltitude = 0;

  // Get the IMU absolute values on Euler angles
//  if (IMU.eulerAnglesAvailable()) {
//    IMU.readEulerAngles(sensorData.heading, sensorData.roll, sensorData.pitch);
//  } else {
//
//    sensorData.heading = -1.0;
//    sensorData.roll = -1.0;
//    sensorData.pitch = -1.0;
//
//  }

   if (GPS.available()) {
    // read GPS values
    sensorData.latitude   = GPS.latitude();
    sensorData.longitude  = GPS.longitude();
    sensorData.altitude   = GPS.altitude();
    sensorData.speed      = GPS.speed();
    sensorData.numSatellites = GPS.satellites();

    // Print GPS values
    Serial.print("Latitude: ");
    Serial.println(sensorData.latitude, 6);  // Imprime con 6 decimales de precisión
    Serial.print("Longitude: ");
    Serial.println(sensorData.longitude, 6);
    Serial.print("Altitude: ");
    Serial.println(sensorData.altitude);
    Serial.print("Speed: ");
    Serial.println(sensorData.speed);
    Serial.print("Number of Satellites: ");
    Serial.println(sensorData.numSatellites);
    
   }
   delay(100);
  // Attempt to connect to barometer if not already connected
  if (!barometerConnected) {
    if (millis() - prevConnectionAttempt >= connectionAttemptDelay) {
      if (barometer.connect() > 0) {
        Serial.println(F("Error connecting to barometer..."));
        prevConnectionAttempt = millis();
      } else {
        Serial.println(F("Connected to barometer!"));
        barometerConnected = true;
      }
    }
  }else {
    // Check if the barometer has new data available
//    barometer.checkUpdates();
    if (barometer.isReady()) {
      sensorData.temperature = barometer.GetTemp();
      sensorData.pressure = barometer.GetPres();

      // If the temperature or pressure have changed, update the variables and calculate new altitude data
      if ((sensorData.temperature != prevTemperature) || (sensorData.pressure != prevPressure)) {
        if (sensorData.seaLevelPressure == 0) sensorData.seaLevelPressure = barometer.getSeaLevel(sensorData.pressure);
        sensorData.altitude = barometer.getAltitude();
        sensorData.correctedAltitude = barometer.getAltitude(true);
        }
      }
     }

     delay(1000);
        // Create a string data to see on SerialPort
        String data = "Roll:" + String(sensorData.roll) + ","
//                      + "Pitch:" + String(sensorData.pitch) + ","
//                      + "Heading:" + String(sensorData.heading) + ","
//                      + "Temperature:" + String(sensorData.temperature) + ","
                      + "Pressure:" + String(sensorData.pressure) + ","
//                      + "SLp:" + String(sensorData.seaLevelPressure) + ","
//                      + "Altitude:" + String(sensorData.altitude) + ","
//                      + "C. Altitude:" + String(sensorData.correctedAltitude) + ","
//                      + "Frame length:" + sizeof(sensorData)
                      + "Latitude:" + String(sensorData.latitude, 6) + ","
                      + "Longitude:" + String(sensorData.longitude, 6) + ","
                      + "Speed:" + String(sensorData.speed) + ","
                      + "NumSatellites:" + String(sensorData.numSatellites);


        // Send sensor data
        packData_and_send(sensorData);

        // Print the formatted sensor data to the serial monitor
//        Serial.print("Flight data: ");
        Serial.print(data);
        Serial.println();
      
  delay(10);
}

bool barometer_init() {

  barometer.setI2Caddr(I2C_LOW);
  barometer.setSamples(MS5xxx_CMD_ADC_4096);
  barometer.setDelay(1000);
  barometer.setPressMbar();  //GetPress() return Pressure in Milibars (default preassure units)
  barometer.setTempC();      //GetTemp() return temperature in Celcius (default temperature units)
  //    barometer.setTOffset(-200);
  barometer.setPOffset(5);

  // Attempts to connect to the barometer sensor
  if (barometer.connect() > 0) {
    Serial.println(F("Error connecting..."));

    return false;

  } else {
    Serial.println(F("Connected to Baromete Sensor"));

    return true;
  }
}

void packData_and_send(SensorData sensorData) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&sensorData, sizeof(SensorData));
  LoRa.endPacket();
//  Serial.println("Packet sent!");
}

//void packData_and_send(float roll, float pitch, float heading, float temperature, float pressure, float seaLevelPressure, float altitude, float correctedAltitude) {
//
//  // Convert roll and pitch to int16_t
//  int16_t rollInt = (int16_t)(roll * 10);
//  int16_t pitchInt = (int16_t)(pitch * 10);
//
//  // Convert heading to int16_t
//  int16_t headingInt = (int16_t)heading;
//
//  // Convert temperature to int16_t
//  int16_t temperatureInt = (int16_t)(temperature * 10);
//
//  // Convert pressure to int32_t
//  int32_t pressureInt = (int32_t)(pressure * 100);
//
//  // Convert sea level pressure, altitude, and corrected altitude to int32_t
//  int32_t seaLevelPressureInt = (int32_t)(seaLevelPressure * 100);
//  int32_t altitudeInt = (int32_t)(altitude * 100);
//  int32_t correctedAltitudeInt = (int32_t)(correctedAltitude * 100);
//
//  // Create packet buffer with start and stop bytes, and add the formatted sensor data
//  unsigned char buffer[] = {
//    0xFF,  // start byte
//    //    latitude >> 24, latitude >> 16, latitude >> 8, latitude & 0xFF,
//    //    longitude >> 24, longitude >> 16, longitude >> 8, longitude & 0xFF,
//    //    altitude >> 24, altitude >> 16, altitude >> 8, altitude & 0xFF,
//    //    speed >> 24, speed >> 16, speed >> 8, speed & 0xFF,
//    //    numSatellites,
//    rollInt >> 8, rollInt & 0xFF,
//    pitchInt >> 8, pitchInt & 0xFF,
//    headingInt >> 8, headingInt & 0xFF,
//    temperatureInt >> 8, temperatureInt & 0xFF,
//    pressureInt >> 24, pressureInt >> 16, pressureInt >> 8, pressureInt & 0xFF,
//    seaLevelPressureInt >> 24, seaLevelPressureInt >> 16, seaLevelPressureInt >> 8, seaLevelPressureInt & 0xFF,
//    altitudeInt >> 24, altitudeInt >> 16, altitudeInt >> 8, altitudeInt & 0xFF,
//    correctedAltitudeInt >> 24, correctedAltitudeInt >> 16, correctedAltitudeInt >> 8, correctedAltitudeInt & 0xFF,
//    0xFE  // stop byte
//  };
//
//  // Send packet via LoRa
//  LoRa.beginPacket();
//  LoRa.write(buffer, sizeof(buffer));
//  LoRa.endPacket();
//  Serial.println("Packet sent!");
//}
