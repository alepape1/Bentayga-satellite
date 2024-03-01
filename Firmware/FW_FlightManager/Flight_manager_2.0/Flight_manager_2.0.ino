#include <Wire.h>
#include <Adafruit_Sensor.h>  // Adafruit Unified Sensor
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include "DS1307.h"  // Grove RTC DS1307 - Seeed Studio
#include <LoRa.h> //  LoRa by Sandeep Mistry
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>
// #include "Arduino_CRC32.h"
// #include <PID_v1_bc.h>
#include <PID_v1.h>  // PID by Brett Beauregard
// #include <Arduino_MKRGPS.h>
// #include "ArduPID.h"

// COMMENT IN REAL MISSION
// UNCOMMENT FOR DEBUGGIN
//#define DEBUG

#ifdef DEBUG
  #define SERIAL_DEBUG(x) Serial.print (x)
  #define SERIAL_DEBUG_LN(x) Serial.println (x)
#else  // do not use serial print: do nothing:
  #define SERIAL_DEBUG(x)
  #define SERIAL_DEBUG_LN(x)
#endif

// -----------------------------------------
// COMMENT define IN REAL MISSION OR IF YOU DONT WANT TO TEST AVAILABLE SRAM
// UNCOMMENT define IF YOU WANT TO TEST AVAILABLE SRAM
#define CHECK_FREE_SRAM 

// You need to install https://github.com/mpflaga/Arduino-MemoryFree
#ifdef CHECK_FREE_SRAM
#include <MemoryFree.h>
#include <pgmStrToRAM.h>
#endif


// --------------------------------------------------------------------
// ---- CHANGE HERE THE BEHAVIOUR OF THE HEATPAD CONTROL WITH TEMPERATURE
// if you are testing at room temperature, outside a fridge
#define TESTING_ROOM_TEMP  // COMMENT IN REAL MISSION OR TESTING INSIDE A FREEZER

// Below this temperature the heatpad will be ON. Should be low
#ifdef TESTING_ROOM_TEMP
const int MIN_TEMP_START = 35;  // but this is for testing at room temperature
#else                           // Real mission or inside a freezer
const int MIN_TEMP_START = 5;  // Below 5 the heatpad will be on
#endif


#define SAMPLE_SENSOR_TIME 100
#define SAMPLE_GPS_TIME 6000
#define SEND_DATA_DELAY 1000
#define WRITE_SD_DATA_DELAY 2000
#define WATCHDOG_TIMEOUT 20000

#define fileLog "tLog.txt"  //Less 8 character plus extension (.txt) for the file name
// if any sensor is above this is the temperature, heatmats should be off
//const int LIMIT_TEMP_HIGH = 45;

const int TEMP_STEP = 5;  // Temperature steps that define the levels
// Half way between the starting temperature and the extreme
const int MIN_TEMP_2 = MIN_TEMP_START - TEMP_STEP;  // More levels could be added
const int MIN_TEMP_3 = MIN_TEMP_2 - TEMP_STEP;      // More levels could be added
// If this low temperature is reached, the heatpads will be at maximum temperature
const int MIN_TEMP_EXTREM = MIN_TEMP_3 - TEMP_STEP;  // More levels could be added
// We have defined 4 levels
//                   Mission/Freezer  -- Room temperature experiment
// MIN_TEMP_START  =   5 C                40   -- below this, heatpad are partially ON
// MIN_TEMP_2      =   0 C                35
// MIN_TEMP_3      =  -5 C                30
// MIN_TEMP_EXTREM = -10 C                25   -- below this, heatpads are FULL-ON

// There are 5 DS18B20 temperature sensors and another one of a different kind
//
// The temperature sensors inside the battery box should have a similar temperature
// Especially the 3 sensors inside the mats
// If one of them is off, it should be discarded. Double check with the other sensor
// inside the box.
// This will be the maximum allowable temperature difference.
// change it if it is too high or low
// This is for the difference with 3 the sensor temperatures, comparing the temperature of the
// middle with the other two. If any is higher than this difference, it could be a problem in a sensor
const int MAX_DIFF_SENS_TEMP = 5;  // if sensors temperature is larger than this, it might be a problem



// --------- END OF TEMPERATURE TESTING CHANGES -----------------------------------

// Since the heatpads are rated for 12V and their batteries are 14.4V,
// The maximum value for the PWM should be 212 = 255*12/14.4

const int HEAT_FULL = 170;
const int HEAT_START = (int)HEAT_FULL / 4;      // 53
const int HEAT_2 = (int)HEAT_FULL / 2;          // 106
const int HEAT_3 = (int)((3 * HEAT_FULL) / 4);  // 159
const int MAX_OUTPUT_PWM = 220;



// Pin were the PWM output to the MOSFET for the heatmats:
const int PIN_HEATMATS = 1;  // cannot be 0, because it is used by the IMU

// Pin were the 1-Wire bus with the temp sensors DS18B20
const int PIN_ONEWIRE = 2;

//Information camera capturing and memory size available
const int GPIO_C = 5;    // GPIO pin for the camera capturing signal
const int GPIO_LSB = 6;  // GPIO pin for the less significative bit of the memory capacity signal
const int GPIO_MSB = 3;  // GPIO pin for the less significative bit of the memory capacity signal. CHANGED, before was pin 7, but there was conflict


// These are the address of the 5 DS18B20 temperature sensors, their code has been requested
// with the sketch: detect_ds1820_tempsens_id.ino
DeviceAddress TEMP_CUBESAT_ADDR = { 0x28, 0xFF, 0x10, 0x4B, 0x20, 0x18, 0x01, 0x10 };    // inside cubesat
DeviceAddress TEMP_BAT_BOX_ADDR = { 0x28, 0x55, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0xEE };    // inside battery box
DeviceAddress TEMP_BAT_LEFT_ADDR = { 0x28, 0x37, 0x50, 0x95, 0xF0, 0x01, 0x3C, 0xD7 };   // inside battery box, in left heatmat
DeviceAddress TEMP_BAT_RIGHT_ADDR = { 0x28, 0x53, 0x6E, 0x95, 0xF0, 0x01, 0x3C, 0xEE };  // inside battery box, in right heatmat
DeviceAddress TEMP_BAT_DOWN_ADDR = { 0x28, 0x7A, 0xEF, 0x95, 0xF0, 0x01, 0x3C, 0xC8 };   // inside battery box, in bottom heatmat
char DS18_NR = 5;                                                                        // Number of DS18 sensors
// variables that check if the DS18B20 temperature sensors have been found
bool temp_cubesat_found = false;
bool temp_bat_box_found = false;
bool temp_bat_left_found = false;
bool temp_bat_right_found = false;
bool temp_bat_down_found = false;



// Resolution can be 9,10,11,12, the higher the slower
// default seems to be the last that has been set.
// with  9 it seems that it is 0.50 C resolution
// with 10 it seems that it is 0.25 C resolution
const uint8_t SENSOR_BIT_RESOL = 11;

// Instance to classes OneWire y DallasTemperature
OneWire OneWireObj(PIN_ONEWIRE);
DallasTemperature sensor_DS18(&OneWireObj);

// ArduPID myController;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;

SFE_UBLOX_GNSS myGNSS;
DS1307 clock;  //define a object of DS1307 class

// Arduino_CRC32 crc32;

struct SensorData {

  float roll;
  float pitch;
  float heading;
  // 5 DS18B20 temp sensors
  float battTempLeft;     // Temperature of batteries inside heatmat, at the left, looking from the camera
  float battTempRight;    // Temperature of batteries inside heatmat, at the right, looking from the camera
  float battTempDown;     // Temperature of batteries inside heatmat, at the bottom, opposite to the battery box openning
  float battTempBox;      // Temperature of batteries inside the box, outside the heatmats
  float TempCubesatDS18;  // Temperature of the cubesat, taken from the DS18B20 sensor
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
  uint8_t camera_info; // info of Camera Flag (bit2), and Jetson memory usage (bits 1 and 0)
  uint8_t camera_flag; // Remove when ground station is updated
  // uint32_t checksum;
};


SensorData sensorData;

// Archivo en la tarjeta SD
File dataFile;

// dataFile = SD.open(fileLog, FILE_WRITE);

// Definir el contador para el sampling del barometro , IMU y sensores de temperatura
uint32_t sample_counter = 0;

//Definir el contador para el sampling del GPS
uint32_t GPS_sample_counter = 0;

// Definir el contador para el envio de datos
uint32_t send_data_timer = 0;

// Definir el contador de tiempo para la escritura en la SD de la trama de datos
uint32_t write_sd_data_timer = 0;

bool update_time = true;

File openDataFile(const char *filename);

void saveDataToSDCard(File dataFile, SensorData &sensorData);
bool temperature_sensor_init();
void IMU_calibration();
void IMU_get_values(SensorData &sensorData);
void print_dev_addr(DeviceAddress addr);
bool comp_dev_addr(DeviceAddress addr1, DeviceAddress addr2);
int temp_ctrl(int temp_left, int temp_down, int temp_right);

// Constantes de configuración del PID
double setpoint = 20.0;  // Temperatura deseada en grados Celsius
double kp = 0.9;         // Coeficiente proporcional
double ki = 0.001;       // Coeficiente integral
double kd = 0.4;         // Coeficiente derivativo

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

// Variables globales para el controlador PID
double input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);



void setup() {

  Watchdog.disable();
#ifdef DEBUG
  Serial.begin(115200);
#endif
  delay(2000);

  //GPIO configuration for camera info flags
  pinMode(GPIO_C, INPUT_PULLUP);
  pinMode(GPIO_LSB, INPUT_PULLUP);
  pinMode(GPIO_MSB, INPUT_PULLUP);


  // Setup the output to control the heatpad, which goes to the MOSFET
  pinMode(PIN_HEATMATS, OUTPUT);    // it has to be a PWM pin
  digitalWrite(PIN_HEATMATS, LOW);  // turn-off the heatmats, until we have temperature values

  // Inicialización del controlador PID
  myPID.SetMode(AUTOMATIC);

  clock.begin();

  SERIAL_DEBUG_LN("Bentayga I. System init");

  temperature_sensor_init();  // Initializacion of the 5 DS18B20 temperature sensors

  // Inicializar la tarjeta SD
  if (!SD.begin(4)) {
    SERIAL_DEBUG_LN("ERROR: Failed to initialize SD card!");

  } else {

    SERIAL_DEBUG_LN("SD card initialized.      Status: OK");
    dataFile = openDataFile(fileLog);
    dataFile.println("Reset WatchDog");
    dataFile.close();
  }

  if (!bno.begin()) {
    SERIAL_DEBUG_LN("ERROR: BNO055 NOT detected");

  } else {
    SERIAL_DEBUG_LN("BNO55 IMU sensor connected.Status: OK");
    IMU_calibration();
  }

  //  if (!GPS.begin()) {
  //   SERIAL_DEBUG_LN("Failed to initialize GPS!");
  // } else {
  //   SERIAL_DEBUG_LN("GPS System init.          Status: OK");
  // }

  if (!myGNSS.begin()) {
    SERIAL_DEBUG_LN("Failed to initialize GPS!");
  } else {
    SERIAL_DEBUG_LN("GPS System init.          Status : OK");
  }

  if (!bme.begin(0x76)) {  // la dirección puede ser 0x77 dependiendo de tu módulo
    SERIAL_DEBUG_LN("ERROR: Failed to initialize the barometer BME280");
  } else {
    SERIAL_DEBUG_LN("BME280 Sensor.            Status: OK");
  }

  if (!LoRa.begin(868E6)) {
    SERIAL_DEBUG_LN("ERROR: Starting LoRa failed!");
  } else {
    SERIAL_DEBUG_LN("LoRa Transceiver.         Status: OK");
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(12);

  delay(4000);

  //Watchdog.enable for the setup and loop
  Watchdog.enable(WATCHDOG_TIMEOUT);

  sample_counter = millis();
}

void loop() {

  IMU_get_values(sensorData);

  // if (millis() - sample_counter >= SAMPLE_SENSOR_TIME) {

#ifdef CHECK_FREE_SRAM
  // Check free memory:
  SERIAL_DEBUG("Free Memory: ");
  SERIAL_DEBUG_LN(freeMemory());
#endif

  // Abrir el archivo en modo de escritura
  SERIAL_DEBUG_LN("Before barometer measure");
  sensorData.temperature = bme.readTemperature();
  sensorData.humidity = bme.readHumidity();
  sensorData.pressure = bme.readPressure() / 100.0;
  sensor_DS18.requestTemperatures();  //Se envia el comando para leer la temperatura

  // Reading the 5 DS18B20 sensors
  sensorData.TempCubesatDS18 = sensor_DS18.getTempC(TEMP_CUBESAT_ADDR);
  sensorData.battTempBox = sensor_DS18.getTempC(TEMP_BAT_BOX_ADDR);
  sensorData.battTempLeft = sensor_DS18.getTempC(TEMP_BAT_LEFT_ADDR);
  sensorData.battTempRight = sensor_DS18.getTempC(TEMP_BAT_RIGHT_ADDR);
  sensorData.battTempDown = sensor_DS18.getTempC(TEMP_BAT_DOWN_ADDR);

  // int temp_left = (int)round(sensorData.battTempLeft);
  // int temp_down = (int)round(sensorData.battTempDown);
  // int temp_right = (int)round(sensorData.battTempRight);
  // int temp_box = (int)round(sensorData.battTempBox);

  // heatmat temperature control
  temp_ctrl(sensorData.battTempLeft, sensorData.battTempDown, sensorData.battTempRight);


  // if(millis() - GPS_sample_counter >SAMPLE_GPS_TIME){

  //   SERIAL_DEBUG_LN("Before GPS measure");
  //   sensorData.latitude = GPS.latitude();
  //   sensorData.longitude = GPS.longitude();
  //   sensorData.GpsAltitude = GPS.altitude();
  //   sensorData.speed = GPS.speed();
  //   sensorData.numSatellites = GPS.satellites();
  //   }

  if(millis() - GPS_sample_counter >SAMPLE_GPS_TIME){
   SERIAL_DEBUG_LN("Before GPS measure");
   sensorData.latitude = myGNSS.getLatitude() / 10000000.0;
   sensorData.longitude = myGNSS.getLongitude() / 10000000.0;
   sensorData.GpsAltitude = myGNSS.getAltitude() / 1000.0;
   sensorData.speed = myGNSS.getGroundSpeed() / 1000.0;
   sensorData.numSatellites = myGNSS.getSIV();
   GPS_sample_counter = millis();
   delay(10);
  }

  sensorData.camera_info = get_camera_info();

  // sensorData.checksum = crc32.calc((uint8_t *)&sensorData, sizeof(sensorData) - sizeof(sensorData.checksum));

  // if (update_time) {
  //   if (myGNSS.getTimeValid()) {

  //     clock.fillByYMD(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());      //Jan 19,2013
  //     clock.fillByHMS(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());  //15:28 30"
  //     // clock.fillDayOfWeek(SAT);//Saturday
  //     clock.setTime();  //write time to the RTC chip
  //     SERIAL_DEBUG_LN("TIME GPS SYNC OK");
  //     update_time = false;
  //   }
  // }

  //   sample_counter = millis();
  // }

  SERIAL_DEBUG_LN("Before clock get time");
  clock.getTime();
  sensorData.year = clock.year + 2000;
  sensorData.month = clock.month;
  sensorData.day = clock.dayOfMonth;
  sensorData.hour = clock.hour;
  sensorData.minute = clock.minute;
  sensorData.second = clock.second;


#ifdef DEBUG
  // string_data is sent to serial port
  String string_data = String(sensorData.day) + "/" + String(sensorData.month) + "/" + String(sensorData.year) + " " + String(sensorData.hour) + ":" + String(sensorData.minute) + ":" + String(sensorData.second) + "\t" + "Roll:" + String(sensorData.roll) + "\t" + "Pitch:" + String(sensorData.pitch) + "\t" + "Heading:" + String(sensorData.heading) + "\t" + "Temp-batt-Left:" + String(sensorData.battTempLeft) + "\t" + "Temp-batt-Right:" + String(sensorData.battTempRight) + "\t" + "Temp-batt-Down:" + String(sensorData.battTempDown) + "\t" + "Temp-batt-Box:" + String(sensorData.battTempBox) + "\t" + "Temp-Cubesat-DS18:" + String(sensorData.TempCubesatDS18) + "\n" + "Temperature:" + String(sensorData.temperature) + "\t" + "Humidity:" + String(sensorData.humidity) + "\t" + "Pressure:" + String(sensorData.pressure) + "\t" + "GpsAltitude:" + String(sensorData.GpsAltitude) + "\t" + "Latitude:" + String(sensorData.latitude, 6) + "\t" + "Longitude:" + String(sensorData.longitude, 6) + "\t" + "Speed:" + String(sensorData.speed) + "\t" + "NumSatellites:" + String(sensorData.numSatellites) + "\t" + "Camera:" + String(sensorData.camera_info) + "\t" + "Capturing:" + String(sensorData.camera_flag);
  //  + "," + "CheckSum:" + String(sensorData.checksum, HEX);
#endif

  if (millis() - send_data_timer >= SEND_DATA_DELAY) {

#ifdef DEBUG
    SERIAL_DEBUG_LN(string_data);
#endif
    packData_and_send(sensorData);
    send_data_timer = millis();
  }

  if (millis() - write_sd_data_timer >= WRITE_SD_DATA_DELAY) {

    SERIAL_DEBUG_LN("Before write SD");
    saveDataToSDCard(openDataFile(fileLog), sensorData);
    write_sd_data_timer = millis();
    delay(5);
  }

  Watchdog.reset();
}

// print a One Wire device address a DeviceAddress is a 8 byte array
void prnt_dev_addr(DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    // If only one digit, fill it wit a zero on the left
    if (addr[i] < 16) SERIAL_DEBUG("0");
    // show in HEX
#ifdef DEBUG    
    Serial.print(addr[i], HEX);
#endif
 
  }
  SERIAL_DEBUG_LN("");
}

void packData_and_send(SensorData sensorData) {
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&sensorData, sizeof(SensorData));
  LoRa.endPacket();
}

void IMU_get_values(SensorData &sensorData) {

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

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

void IMU_calibration() {

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  SERIAL_DEBUG_LN();
  SERIAL_DEBUG("Calibration: Sys=");
  SERIAL_DEBUG(system);
  SERIAL_DEBUG(" Gyro=");
  SERIAL_DEBUG(gyro);
  SERIAL_DEBUG(" Accel=");
  SERIAL_DEBUG(accel);
  SERIAL_DEBUG(" Mag=");
  SERIAL_DEBUG_LN(mag);
}

bool temperature_sensor_init() {

  sensor_DS18.begin();

  uint8_t resolution = sensor_DS18.getResolution();

  sensor_DS18.setResolution(SENSOR_BIT_RESOL);
  resolution = sensor_DS18.getResolution();
  SERIAL_DEBUG("Setting DS18B20 temperature sensor resolution to 0.50 C: ");
  SERIAL_DEBUG(resolution);
  SERIAL_DEBUG_LN("bits");

  // Searching sensors
  SERIAL_DEBUG_LN("Searching temp sensors...");
  SERIAL_DEBUG("Found: ");
  byte numSensorsFound = sensor_DS18.getDeviceCount();
  SERIAL_DEBUG(numSensorsFound);
  SERIAL_DEBUG_LN(" sensors");
  if (numSensorsFound < DS18_NR) {
    SERIAL_DEBUG(" Missing ");
    int miss_ds18 = DS18_NR - numSensorsFound;
    SERIAL_DEBUG(miss_ds18);
    SERIAL_DEBUG_LN(" DS18B20 temperature sensors");
  } else {
    SERIAL_DEBUG_LN("All DS18B20 temperature sensors found");
  }
  // If found any, show address
  if (numSensorsFound >= 1) {
    for (byte sens_i = 0; sens_i < numSensorsFound; sens_i++) {
      DeviceAddress sens_temp_addr;  // 8 byte array (uint8_t)
      // get adddres of the sensor
      sensor_DS18.getAddress(sens_temp_addr, sens_i);
      // compare the address with the sensors
      if (comp_dev_addr(sens_temp_addr, TEMP_CUBESAT_ADDR)) {
        temp_cubesat_found = true;
        SERIAL_DEBUG("General cubesat sensor address found: ");  // inside cubesat, outside battery box
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_BOX_ADDR)) {
        temp_bat_box_found = true;
        SERIAL_DEBUG("Battery box sensor address found:     ");
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_LEFT_ADDR)) {
        SERIAL_DEBUG("Left heatmat sensor address found:    ");
        temp_bat_left_found = true;
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_RIGHT_ADDR)) {
        SERIAL_DEBUG("Right heatmat sensor address found:   ");
        temp_bat_right_found = true;
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_DOWN_ADDR)) {
        SERIAL_DEBUG("Down heatmat sensor address found:    ");
        temp_bat_down_found = true;
      } else {
        SERIAL_DEBUG("Unkown temperature sensor address found:    ");
      }
      // print the address
      prnt_dev_addr(sens_temp_addr);  // print the address
    }

    // after the loop, check if there is a missing temperature sensor:
    if (!temp_cubesat_found) {
      SERIAL_DEBUG("WARNING: General cubesat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr(TEMP_CUBESAT_ADDR);
    }
    if (!temp_bat_box_found) {
      SERIAL_DEBUG("WARNING: Battery box DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr(TEMP_BAT_BOX_ADDR);
    }
    if (!temp_bat_right_found) {
      SERIAL_DEBUG("WARNING: Rigth heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr(TEMP_BAT_RIGHT_ADDR);
    }
    if (!temp_bat_left_found) {
      SERIAL_DEBUG("WARNING: Left heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr(TEMP_BAT_LEFT_ADDR);
    }
    if (!temp_bat_down_found) {
      SERIAL_DEBUG("WARNING: Down heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr(TEMP_BAT_DOWN_ADDR);
    }

    return true;
  } else {
    SERIAL_DEBUG_LN("No DS18B20 temperature sensors found");
    return false;
  }
}

File openDataFile(const char *filename) {
  File dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    SERIAL_DEBUG("ERROR: Failed to open data file: ");
    SERIAL_DEBUG_LN(filename);
  }
  return dataFile;
}

void saveDataToSDCard(File dataFile, SensorData &sensorData) {

  dataFile.print(sensorData.year);
  dataFile.print("-");
  dataFile.print(sensorData.month);
  dataFile.print("-");
  dataFile.print(sensorData.day);
  dataFile.print(" ");
  dataFile.print(sensorData.hour);
  dataFile.print(":");
  dataFile.print(sensorData.minute);
  dataFile.print(":");
  dataFile.print(sensorData.second);
  dataFile.print("\t");
  dataFile.print(sensorData.roll);
  dataFile.print("\t");
  dataFile.print(sensorData.pitch);
  dataFile.print("\t");
  dataFile.print(sensorData.heading);
  dataFile.print("\t");
  dataFile.print(sensorData.battTempLeft);
  dataFile.print("\t");
  dataFile.print(sensorData.battTempRight);
  dataFile.print("\t");
  dataFile.print(sensorData.battTempDown);
  dataFile.print("\t");
  dataFile.print(sensorData.battTempBox);
  dataFile.print("\t");
  dataFile.print(sensorData.TempCubesatDS18);
  dataFile.print("\t");
  dataFile.print(sensorData.temperature);
  dataFile.print("\t");
  dataFile.print(sensorData.humidity);
  dataFile.print("\t");
  dataFile.print(sensorData.pressure);
  dataFile.print("\t");
  dataFile.print(sensorData.GpsAltitude);
  dataFile.print("\t");
  dataFile.print(sensorData.latitude, 6);
  dataFile.print("\t");
  dataFile.print(sensorData.longitude, 6);
  dataFile.print("\t");
  dataFile.print(sensorData.speed);
  dataFile.print("\t");
  dataFile.print(sensorData.numSatellites);
  dataFile.print("\t");
  dataFile.println(sensorData.camera_info);


  dataFile.close();
}

uint8_t get_camera_info() {
// Read values from jetson nano pins GPIO. We will use a byte to encode this data
// bit 2: 1 if camera flag is ON
// bit 1: 1 if MSB of Jetson memory capacity is 1 -> above 50%
// bit 0: 1 if LSB of Jetson memory capacity is 1
// So we will have:
//           Camera ON  : Capacity
// 7 (111):      1           +75%
// 6 (110):      1           +50%
// 5 (101):      1           +25%
// 4 (100):      1           -25%
// 3 (011):      0           +75%
// 2 (010):      0           +50%
// 1 (001):      0           +25%
// 0 (000):      0           -25%

  // Keep this for legacy, but remove when changed in ground station
  sensorData.camera_flag = digitalRead(GPIO_C);

  byte encodedData = 0x00; // used to encode the 3 bits as explained above
  if (digitalRead(GPIO_C) == HIGH) {
    encodedData = encodedData | 0x04; // write 1 in bit 2
  } 
  if (digitalRead(GPIO_MSB) == HIGH) {
    encodedData = encodedData | 0x02; // write 1 in bit 1
  } 
  if (digitalRead(GPIO_LSB) == HIGH) {
    encodedData = encodedData | 0x01; // write 1 in bit 0
  } 

  // Imprimir el valor codificado en binario
  //SERIAL_DEBUG_LN(encodedData, HEX);
  return encodedData;
}

// compare two One Wire device addressess
// a DeviceAddress is a 8 byte array
bool comp_dev_addr(DeviceAddress addr1, DeviceAddress addr2) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr1[i] != addr2[i]) {
      return false;
    }
  }
  return true;  // if here, all are equal
}

//Test PID function to control the thermal Pad
int temp_ctrl(float temp_left, float temp_down, float temp_right) {
  double current_temp = (double)(temp_left + temp_down + temp_right) / 3.0;

  // Actualización de las variables para el controlador PID
  input = current_temp;

  double gap = abs(setpoint - input);  //distance away from setpoint
  if (gap < 2) {                       //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }


  // Cálculo de la salida del controlador PID
  myPID.Compute();
  // myController.compute();
  // Ajuste del PWM en el MOSFET para controlar la potencia
  // int pwm_value = map(output, 0, HEAT_FULL, 0, 100);  // Ajusta los valores si es necesario

  // Limitar el valor de salida
  // if (output > HEAT_FULL) {
  //   output = HEAT_FULL;
  // } else if (output < 0) {
  //   output = 0;
  // }

  // if(input>(setpoint-2.0)){
  //   output=0;
  // }

  SERIAL_DEBUG("PID Setpoint");
  SERIAL_DEBUG(setpoint);
  SERIAL_DEBUG("\t");
  SERIAL_DEBUG("PID Output");
  SERIAL_DEBUG(output);
  SERIAL_DEBUG("\t");
  SERIAL_DEBUG("Input PID:");
  SERIAL_DEBUG(input);
  SERIAL_DEBUG("\t");
  SERIAL_DEBUG_LN();

  // Write the value to PWM port
  analogWrite(PIN_HEATMATS, output);


  return 0;  // No hay error
}


//// temperature control of the heatmats, returns 1 if there is a problem, 0 if ok
//int temp_ctrl(int temp_left, int temp_down, int temp_right, int temp_box) {
//
//  int temp_error = 0;
//
//  // order the heatmats temperatures
//  int highest_temp, mid_temp, lowest_temp;
//  if (temp_left > temp_down) {
//    if (temp_left > temp_right) {
//      highest_temp = temp_left;
//      if (temp_right > temp_down) {
//        mid_temp = temp_right;
//        lowest_temp = temp_down;
//      } else {
//        mid_temp = temp_down;
//        lowest_temp = temp_right;
//      }
//    } else { // right > left > down
//      highest_temp = temp_right;
//      mid_temp = temp_left;
//      lowest_temp = temp_down;
//    }
//  } else { // down > left
//    if (temp_down > temp_right) {
//      highest_temp = temp_down;
//      if (temp_right > temp_left) {
//        mid_temp = temp_right;
//        lowest_temp = temp_left;
//      } else {
//        mid_temp = temp_left;
//        lowest_temp = temp_right;
//      }
//    } else { // right > down > left
//      highest_temp = temp_right;
//      mid_temp = temp_down;
//      lowest_temp = temp_left;
//    }
//  }
//
//  int temp_diff_high = highest_temp - mid_temp;
//  int temp_diff_low  = mid_temp - lowest_temp;
//
//  int comm_temp; // command temperature
//  if (temp_diff_high > MAX_DIFF_SENS_TEMP) {
//    if (temp_diff_low > MAX_DIFF_SENS_TEMP) {
//      // the 3 sensors give very different temperatures
//      // to be safe, take the highest of them, to avoid overheat
//      comm_temp = highest_temp;
//      SERIAL_DEBUG_LN("The 3 heatmat sensors give very diff temperatures");
//      temp_error = 1;
//    } else { // temp_high is much larger than the two low temperatures
//      // check with the battery box temperature
//      if ((highest_temp - MAX_DIFF_SENS_TEMP) > temp_box) {
//        // high temp abnormally high, remove it, and get average of other 2
//        comm_temp = (int) (mid_temp + lowest_temp) / 2;
//        SERIAL_DEBUG_LN("One heatmat sensor gives higher temperatures: discarded");
//        temp_error = 1;
//      } else {
//        // strange, two temps are high, and two are low, take the high temp to be safe
//        comm_temp = highest_temp;
//        SERIAL_DEBUG_LN("2 heatmat sensor give lower temperatures");
//        temp_error = 1;
//      }
//    }
//  } else if (temp_diff_low > MAX_DIFF_SENS_TEMP) { // one sensor give much lower temperatures
//    // check with the battery box temperature
//    if (temp_box > (lowest_temp + MAX_DIFF_SENS_TEMP))  { // lowest is outlier, remove it
//      comm_temp = (int) (mid_temp + highest_temp) / 2;
//      SERIAL_DEBUG_LN("One heatmat sensor gives lower temperatures: discarded");
//      temp_error = 1;
//    } else {
//      // strange, two temps are high, and two are low, take the high temp to be safe
//      comm_temp = (int) (mid_temp + highest_temp) / 2;
//      SERIAL_DEBUG_LN("2 heatmat sensor give higher temperatures");
//      temp_error = 1;
//    }
//  } else { // temperatures in range
//    // get the average temperature of the three heatmat sensors
//    comm_temp = (temp_left + temp_down + temp_right)/3;
//    temp_error = 0;
//  }
//
//  SERIAL_DEBUG("Avg Bat temp: ");
//  SERIAL_DEBUG_LN(comm_temp);
//  if (comm_temp > MIN_TEMP_START) { // batteries are warm, heatmats off
//    digitalWrite(PIN_HEATMATS, LOW);
//    SERIAL_DEBUG_LN("Heat OFF");
//  } else if (comm_temp > MIN_TEMP_2) { // batteries starting to be cold
//    analogWrite(PIN_HEATMATS, HEAT_START);
//    SERIAL_DEBUG("Heat level 1, PWM value: ");
//    SERIAL_DEBUG_LN(HEAT_START);
//  } else if (comm_temp > MIN_TEMP_3) { // batteries are cold
//    analogWrite(PIN_HEATMATS, HEAT_2);
//    SERIAL_DEBUG("Heat level 2, PWM value: ");
//    SERIAL_DEBUG_LN(HEAT_2);
//  } else if (comm_temp > MIN_TEMP_EXTREM) { // batteries even colder
//    analogWrite(PIN_HEATMATS, HEAT_3);
//    SERIAL_DEBUG("Heat level 3, PWM value: ");
//    SERIAL_DEBUG_LN(HEAT_3);
//  } else { // batteries beyond the limit
//    analogWrite(PIN_HEATMATS, HEAT_FULL); // heat mats at full power
//    SERIAL_DEBUG("Heat level FULL, PWM value: ");
//    SERIAL_DEBUG_LN(HEAT_FULL);
//  }
//  return temp_error;
// }
 
