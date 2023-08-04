#include <Wire.h>
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include "DS1307.h"  // Grove RTC DS1307 - Seeed Studio
#include <LoRa.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>
#include "Arduino_CRC32.h" 

// --------------------------------------------------------------------
// ---- CHANGE HERE THE BEHAVIOUR OF THE HEATPAD CONTROL WITH TEMPERATURE
// if you are testing at room temperature, outside a fridge
#define TESTING_ROOM_TEMP  // COMMENT IN REAL MISSION OR TESTING INSIDE A FREEZER

// Below this temperature the heatpad will be ON. Should be low
#ifdef TESTING_ROOM_TEMP
  const int MIN_TEMP_START = 35; // but this is for testing at room temperature
#else // Real mission or inside a freezer
  const int MIN_TEMP_START = 5; // Below 5 the heatpad will be on
#endif

// if any sensor is above this is the temperature, heatmats should be off
//const int LIMIT_TEMP_HIGH = 45;

const int TEMP_STEP = 5; // Temperature steps that define the levels
// Half way between the starting temperature and the extreme
const int MIN_TEMP_2       = MIN_TEMP_START - TEMP_STEP; // More levels could be added
const int MIN_TEMP_3       = MIN_TEMP_2 - TEMP_STEP; // More levels could be added
// If this low temperature is reached, the heatpads will be at maximum temperature
const int MIN_TEMP_EXTREM = MIN_TEMP_3 - TEMP_STEP; // More levels could be added
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
const int MAX_DIFF_SENS_TEMP = 5; // if sensors temperature is larger than this, it might be a problem


// --------- END OF TEMPERATURE TESTING CHANGES -----------------------------------

// Since the heatpads are rated for 12V and their batteries are 14.4V, 
// The maximum value for the PWM should be 212 = 255*12/14.4

const int HEAT_FULL  = 212;
const int HEAT_START = (int) HEAT_FULL/4;  // 53
const int HEAT_2     = (int) HEAT_FULL/2;  // 106
const int HEAT_3     = (int) ((3*HEAT_FULL)/4);  // 159


#define SAMPLE_SENSOR_TIME 2000
#define SEND_DATA_DELAY 10000
#define WRITE_SD_DATA_DELAY 10000
#define WATCHDOG_TIMEOUT 12000

#define fileLog "lab.txt"         //Less 8 character plus extension (.txt) for the file name

// Pin were the PWM output to the MOSFET for the heatmats:
const int PIN_HEATMATS = 1; // cannot be 0, because it is used by the IMU

// Pin were the 1-Wire bus with the temp sensors DS18B20
const int PIN_ONEWIRE = 2;

//Information camera capturing and memory size available
const int GPIO_C = 5;  // GPIO pin for the camera capturing signal
const int GPIO_LSB = 6;  // GPIO pin for the less significative bit of the memory capacity signal
const int GPIO_MSB = 7;  // GPIO pin for the less significative bit of the memory capacity signal


// These are the address of the 5 DS18B20 temperature sensors, their code has been requested 
// with the sketch: detect_ds1820_tempsens_id.ino
DeviceAddress TEMP_CUBESAT_ADDR   = {0x28, 0xFF, 0x10, 0x4B, 0x20, 0x18, 0x01, 0x10}; // inside cubesat
DeviceAddress TEMP_BAT_BOX_ADDR   = {0x28, 0x55, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0xEE};  // inside battery box
DeviceAddress TEMP_BAT_LEFT_ADDR  = {0x28, 0x37, 0x50, 0x95, 0xF0, 0x01, 0x3C, 0xD7}; // inside battery box, in left heatmat
DeviceAddress TEMP_BAT_RIGHT_ADDR = {0x28, 0x53, 0x6E, 0x95, 0xF0, 0x01, 0x3C, 0xEE}; // inside battery box, in right heatmat
DeviceAddress TEMP_BAT_DOWN_ADDR  = {0x28, 0x7A, 0xEF, 0x95, 0xF0, 0x01, 0x3C, 0xC8}; // inside battery box, in bottom heatmat
char DS18_NR = 5; // Number of DS18 sensors
// variables that check if the DS18B20 temperature sensors have been found
bool temp_cubesat_found   = false;
bool temp_bat_box_found   = false;
bool temp_bat_left_found  = false;
bool temp_bat_right_found = false;
bool temp_bat_down_found  = false;



// Resolution can be 9,10,11,12, the higher the slower
// default seems to be the last that has been set.
// with  9 it seems that it is 0.50 C resolution
// with 10 it seems that it is 0.25 C resolution
const uint8_t SENSOR_BIT_RESOL = 9;

// Instance to classes OneWire y DallasTemperature
OneWire OneWireObj(PIN_ONEWIRE);
DallasTemperature sensor_DS18(&OneWireObj);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BME280 bme;

SFE_UBLOX_GNSS myGNSS;
DS1307 clock;               //define a object of DS1307 class

Arduino_CRC32 crc32;

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
  uint32_t checksum;
};


SensorData sensorData;

// Archivo en la tarjeta SD
File dataFile;

//dataFile = SD.open("sensor_data.txt", FILE_WRITE);

// Definir el contador para el sampling del barometro y GPS
uint32_t sample_counter = 0;

// Definir el contador para el envio de datos 
uint32_t send_data_timer = 0;

// Definir el contador de tiempo para la escritura en la SD de la trama de datos
uint32_t write_sd_data_timer = 0;

bool update_time = true;

File openDataFile(const char* filename);

void saveDataToSDCard(File dataFile, SensorData &sensorData);
bool temperature_sensor_init();
void IMU_calibration();
void IMU_get_values(SensorData &sensorData);
void print_dev_addr(DeviceAddress addr);
bool comp_dev_addr(DeviceAddress addr1, DeviceAddress addr2);
int temp_ctrl(int temp_left, int temp_down, int temp_right, int temp_box);

void setup() {

  Watchdog.disable(); 
  Serial.begin(115200);
  delay(2000);

  //GPIO configuration for camera info flags
  pinMode(GPIO_C, INPUT_PULLUP);
  pinMode(GPIO_LSB, INPUT_PULLUP);
  pinMode(GPIO_MSB, INPUT_PULLUP);

 
  // Setup the output to control the heatpad, which goes to the MOSFET
  pinMode(PIN_HEATMATS, OUTPUT);  // it has to be a PWM pin
  digitalWrite(PIN_HEATMATS, LOW); // turn-off the heatmats, until we have temperature values

  clock.begin();

  Serial.println("Bentayga I. System init");

  temperature_sensor_init(); // Initializacion of the 5 DS18B20 temperature sensors

  // Inicializar la tarjeta SD
  if (!SD.begin(4)) {
    Serial.println("ERROR: Failed to initialize SD card!");

  } else {

    Serial.println("SD card initialized.      Status: OK");
    dataFile=openDataFile(fileLog);
    dataFile.println("Reset WatchDog");
    dataFile.close();

  }
  if (!bno.begin())
  {
    Serial.println("ERROR: BNO055 NOT detected");

  } else {
    Serial.println("BNO55 IMU sensor connected.Status: OK");
    IMU_calibration();

  }

  if (!myGNSS.begin()) {
    Serial.println("Failed to initialize GPS!");
  } else {
    Serial.println("GPS System init.          Status : OK");
  }

  if (!bme.begin(0x76)) { // la dirección puede ser 0x77 dependiendo de tu módulo
    Serial.println("ERROR: Failed to initialize the barometer BME280");
  } else {
    Serial.println("BME280 Sensor.            Status: OK");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("ERROR: Starting LoRa failed!");
  } else {
    Serial.println("LoRa Transceiver.         Status: OK");
  }

  LoRa.setTxPower(10);
  LoRa.setSpreadingFactor(7);
  
  delay(4000);
  
  //Watchdog.enable for the setup and loop
  Watchdog.enable(WATCHDOG_TIMEOUT);
  
  sample_counter = millis();

}

void loop() {

  
  IMU_get_values(sensorData);

  if (millis() - sample_counter >= SAMPLE_SENSOR_TIME) {

    // Abrir el archivo en modo de escritura

    sensorData.temperature = bme.readTemperature();
    sensorData.humidity = bme.readHumidity();
    sensorData.pressure = bme.readPressure() / 100.0;
    sensor_DS18.requestTemperatures();   //Se envia el comando para leer la temperatura

    // Reading the 5 DS18B20 sensors
    sensorData.TempCubesatDS18 = sensor_DS18.getTempC(TEMP_CUBESAT_ADDR);
    sensorData.battTempBox     = sensor_DS18.getTempC(TEMP_BAT_BOX_ADDR);
    sensorData.battTempLeft    = sensor_DS18.getTempC(TEMP_BAT_LEFT_ADDR);
    sensorData.battTempRight   = sensor_DS18.getTempC(TEMP_BAT_RIGHT_ADDR);
    sensorData.battTempDown    = sensor_DS18.getTempC(TEMP_BAT_DOWN_ADDR);

    int temp_left  = (int) round(sensorData.battTempLeft);
    int temp_down  = (int) round(sensorData.battTempDown);
    int temp_right = (int) round(sensorData.battTempRight);
    int temp_box   = (int) round(sensorData.battTempBox);

    // heatmat temperature control
    temp_ctrl (temp_left, temp_down, temp_right, temp_box);  

    sensorData.latitude = myGNSS.getLatitude() / 10000000.0;
    sensorData.longitude = myGNSS.getLongitude() / 10000000.0;
    sensorData.GpsAltitude = myGNSS.getAltitude() / 1000.0;
    sensorData.speed = myGNSS.getGroundSpeed() / 1000.0;
    sensorData.numSatellites = myGNSS.getSIV();

    sensorData.camera_info = get_camera_info();

    sensorData.checksum = crc32.calc((uint8_t *)&sensorData, sizeof(sensorData) - sizeof(sensorData.checksum));

    if (update_time){
      if (myGNSS.getTimeValid()) {
      
      clock.fillByYMD(myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay()); //Jan 19,2013
      clock.fillByHMS(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond()); //15:28 30"
      // clock.fillDayOfWeek(SAT);//Saturday
      clock.setTime();//write time to the RTC chip
      Serial.println("TIME GPS SYNC OK");
      update_time= false;
      }
    }

    sample_counter = millis();

  }

  clock.getTime();
  sensorData.year = clock.year +  2000;
  sensorData.month = clock.month;
  sensorData.day = clock.dayOfMonth;
  sensorData.hour = clock.hour;
  sensorData.minute = clock.minute;
  sensorData.second = clock.second;


  String data =
    String(sensorData.day) + "/" + String(sensorData.month) + "/" + String(sensorData.year) + 
    " " + String(sensorData.hour) + ":" + String(sensorData.minute) + ":" + String(sensorData.second) + "\t" +
    "Roll:" + String(sensorData.roll) + "," +
    "Pitch:" + String(sensorData.pitch) + "," +
    "Heading:" + String(sensorData.heading) + "," +
    "Temp-batt-Left:" + String(sensorData.battTempLeft) + "," +
    "Temp-batt-Right:" + String(sensorData.battTempRight) + "," +
    "Temp-batt-Down:" + String(sensorData.battTempDown) + "," +
    "Temp-batt-Box:" + String(sensorData.battTempBox) + "," +
    "Temp-Cubesat-DS18:" + String(sensorData.TempCubesatDS18) + "," +    
    "Temperature:" + String(sensorData.temperature) + "," +
    "Humidity:" + String(sensorData.humidity) + "," +
    "Pressure:" + String(sensorData.pressure) + "," +
    "GpsAltitude:" + String(sensorData.GpsAltitude) + "," +
    "Latitude:" + String(sensorData.latitude, 6) + "," +
    "Longitude:" + String(sensorData.longitude, 6) + "," +
    "Speed:" + String(sensorData.speed) + "," +
    "NumSatellites:" + String(sensorData.numSatellites) + "," +
    "Camera:" + String(sensorData.camera_info) + "," +
    "CheckSum:" + String(sensorData.checksum, HEX); 

  if(millis() - send_data_timer >= SEND_DATA_DELAY){
  
    Serial.println(data);
    packData_and_send(sensorData);
    send_data_timer = millis();

  }  

  if(millis() - write_sd_data_timer >= WRITE_SD_DATA_DELAY){
    saveDataToSDCard(openDataFile(fileLog), sensorData);
    write_sd_data_timer = millis();
  }
  
  Watchdog.reset();

}

// print a One Wire device address a DeviceAddress is a 8 byte array
void prnt_dev_addr(DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    // If only one digit, fill it wit a zero on the left
    if (addr[i] < 16) Serial.print("0");
    // show in HEX
    Serial.print(addr[i], HEX);
  }
  Serial.println("");
}

void packData_and_send(SensorData sensorData) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&sensorData, sizeof(SensorData));
  LoRa.endPacket();
}

void IMU_get_values(SensorData &sensorData) {

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

void IMU_calibration() {

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

bool temperature_sensor_init() {

  sensor_DS18.begin();

  uint8_t resolution = sensor_DS18.getResolution();

  sensor_DS18.setResolution(SENSOR_BIT_RESOL);
  resolution = sensor_DS18.getResolution();
  Serial.print("Setting DS18B20 temperature sensor resolution to 0.50 C: ");
  Serial.print(resolution);
  Serial.println("bits");

  // Searching sensors
  Serial.println("Searching temp sensors...");
  Serial.print("Found: ");
  byte numSensorsFound = sensor_DS18.getDeviceCount();
  Serial.print(numSensorsFound);
  Serial.println(" sensors");
  if (numSensorsFound < DS18_NR) {
    Serial.print(" Missing ");
    int miss_ds18 = DS18_NR - numSensorsFound;
    Serial.print(miss_ds18);
    Serial.println(" DS18B20 temperature sensors");
  } else {
    Serial.println("All DS18B20 temperature sensors found");
  }
  // If found any, show address
  if (numSensorsFound >= 1) {
    for (byte sens_i = 0; sens_i < numSensorsFound; sens_i++) {
      DeviceAddress sens_temp_addr; // 8 byte array (uint8_t)
      // get adddres of the sensor
      sensor_DS18.getAddress(sens_temp_addr, sens_i);
      // compare the address with the sensors
      if        (comp_dev_addr(sens_temp_addr, TEMP_CUBESAT_ADDR)) {
        temp_cubesat_found = true;
        Serial.print("General cubesat sensor address found: "); // inside cubesat, outside battery box
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_BOX_ADDR)) {
        temp_bat_box_found = true;
        Serial.print("Battery box sensor address found:     ");
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_LEFT_ADDR)) {
        Serial.print("Left heatmat sensor address found:    ");
        temp_bat_left_found = true;
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_RIGHT_ADDR)) {
        Serial.print("Right heatmat sensor address found:   ");
        temp_bat_right_found = true;
      } else if (comp_dev_addr(sens_temp_addr, TEMP_BAT_DOWN_ADDR)) {
        Serial.print("Down heatmat sensor address found:    ");
        temp_bat_down_found = true;
      } else {
        Serial.print("Unkown temperature sensor address found:    ");
      }
      // print the address
      prnt_dev_addr (sens_temp_addr); // print the address
    }

    // after the loop, check if there is a missing temperature sensor:
    if (!temp_cubesat_found) {
      Serial.print("WARNING: General cubesat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr (TEMP_CUBESAT_ADDR);
    }
    if (!temp_bat_box_found) {
      Serial.print("WARNING: Battery box DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr (TEMP_BAT_BOX_ADDR);
    }
    if (!temp_bat_right_found) {
      Serial.print("WARNING: Rigth heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr (TEMP_BAT_RIGHT_ADDR);
    }
    if (!temp_bat_left_found) {
      Serial.print("WARNING: Left heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr (TEMP_BAT_LEFT_ADDR);
    }
    if (!temp_bat_down_found) {
      Serial.print("WARNING: Down heatmat DS18B20 temperature sensor address not found, address:  ");
      prnt_dev_addr (TEMP_BAT_DOWN_ADDR);
    }

    return true;
  } else {
    Serial.println("No DS18B20 temperature sensors found");
    return false;
  }

}

File openDataFile(const char* filename) {
  File dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.print("ERROR: Failed to open data file: ");
    Serial.println(filename);
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
  dataFile.print(",");
  dataFile.print(sensorData.roll);
  dataFile.print(",");
  dataFile.print(sensorData.pitch);
  dataFile.print(",");
  dataFile.print(sensorData.heading);
  dataFile.print(",");
  dataFile.print(sensorData.battTempLeft);
  dataFile.print(",");
  dataFile.print(sensorData.battTempRight);
  dataFile.print(",");
  dataFile.print(sensorData.battTempDown);
  dataFile.print(",");
  dataFile.print(sensorData.battTempBox);
  dataFile.print(",");
  dataFile.print(sensorData.TempCubesatDS18);
  dataFile.print(",");
  dataFile.print(sensorData.temperature);
  dataFile.print(",");
  dataFile.print(sensorData.humidity);
  dataFile.print(",");
  dataFile.print(sensorData.pressure);
  dataFile.print(",");
  dataFile.print(sensorData.GpsAltitude);
  dataFile.print(",");
  dataFile.print(sensorData.latitude, 6);
  dataFile.print(",");
  dataFile.print(sensorData.longitude, 6);
  dataFile.print(",");
  dataFile.print(sensorData.speed);
  dataFile.print(",");
  dataFile.println(sensorData.numSatellites);
  dataFile.print(",");
  dataFile.println(sensorData.camera_info);


  dataFile.close();

}

uint8_t get_camera_info(){

  // Leer los valores de las señales de los puertos GPIO
  int cameraFlag = digitalRead(GPIO_C);
  int memoryCapacityLSB = digitalRead(GPIO_LSB);    
  int memoryCapacityMSB = digitalRead(GPIO_MSB);

  // Codificar los valores en un byte
  byte encodedData = 0x00;
  encodedData =  encodedData | (memoryCapacityMSB << 5) | (memoryCapacityLSB << 4) | cameraFlag;

  // Imprimir el valor codificado en binario
  //Serial.println(encodedData, HEX);
  return encodedData;

}

// compare two One Wire device addressess
// a DeviceAddress is a 8 byte array
bool comp_dev_addr(DeviceAddress addr1, DeviceAddress addr2){
 for (uint8_t i = 0; i < 8; i++){
    if (addr1[i] != addr2[i]) {
      return false;
    }
 }
 return true; // if here, all are equal
}

// temperature control of the heatmats, returns 1 if there is a problem, 0 if ok
int temp_ctrl(int temp_left, int temp_down, int temp_right, int temp_box) {

  int temp_error = 0;

  // order the heatmats temperatures
  int highest_temp, mid_temp, lowest_temp;
  if (temp_left > temp_down) {
    if (temp_left > temp_right) {
      highest_temp = temp_left;
      if (temp_right > temp_down) {
        mid_temp = temp_right;
        lowest_temp = temp_down;
      } else {
        mid_temp = temp_down;
        lowest_temp = temp_right;
      }
    } else { // right > left > down
      highest_temp = temp_right;
      mid_temp = temp_left;
      lowest_temp = temp_down;
    }
  } else { // down > left
    if (temp_down > temp_right) {
      highest_temp = temp_down;
      if (temp_right > temp_left) {
        mid_temp = temp_right;
        lowest_temp = temp_left;
      } else {
        mid_temp = temp_left;
        lowest_temp = temp_right;
      }
    } else { // right > down > left
      highest_temp = temp_right;
      mid_temp = temp_down;
      lowest_temp = temp_left;
    }
  }

  int temp_diff_high = highest_temp - mid_temp;
  int temp_diff_low  = mid_temp - lowest_temp;
  
  int comm_temp; // command temperature
  if (temp_diff_high > MAX_DIFF_SENS_TEMP) {
    if (temp_diff_low > MAX_DIFF_SENS_TEMP) {
      // the 3 sensors give very different temperatures
      // to be safe, take the highest of them, to avoid overheat
      comm_temp = highest_temp;
      Serial.println("The 3 heatmat sensors give very diff temperatures");
      temp_error = 1;
    } else { // temp_high is much larger than the two low temperatures
      // check with the battery box temperature
      if ((highest_temp - MAX_DIFF_SENS_TEMP) > temp_box) {
        // high temp abnormally high, remove it, and get average of other 2
        comm_temp = (int) (mid_temp + lowest_temp) / 2;
        Serial.println("One heatmat sensor gives higher temperatures: discarded");
        temp_error = 1;
      } else {
        // strange, two temps are high, and two are low, take the high temp to be safe
        comm_temp = highest_temp;
        Serial.println("2 heatmat sensor give lower temperatures");
        temp_error = 1;
      }
    }
  } else if (temp_diff_low > MAX_DIFF_SENS_TEMP) { // one sensor give much lower temperatures
    // check with the battery box temperature
    if (temp_box > (lowest_temp + MAX_DIFF_SENS_TEMP))  { // lowest is outlier, remove it
      comm_temp = (int) (mid_temp + highest_temp) / 2;
      Serial.println("One heatmat sensor gives lower temperatures: discarded");
      temp_error = 1;
    } else {
      // strange, two temps are high, and two are low, take the high temp to be safe
      comm_temp = (int) (mid_temp + highest_temp) / 2;
      Serial.println("2 heatmat sensor give higher temperatures");
      temp_error = 1;
    }
  } else { // temperatures in range
    // get the average temperature of the three heatmat sensors
    comm_temp = (temp_left + temp_down + temp_right)/3;
    temp_error = 0;
  }
    
  Serial.print("Avg Bat temp: ");
  Serial.println(comm_temp);
  if (comm_temp > MIN_TEMP_START) { // batteries are warm, heatmats off
    digitalWrite(PIN_HEATMATS, LOW);
    Serial.println("Heat OFF");
  } else if (comm_temp > MIN_TEMP_2) { // batteries starting to be cold
    analogWrite(PIN_HEATMATS, HEAT_START);
    Serial.print("Heat level 1, PWM value: ");
    Serial.println(HEAT_START);
  } else if (comm_temp > MIN_TEMP_3) { // batteries are cold
    analogWrite(PIN_HEATMATS, HEAT_2);
    Serial.print("Heat level 2, PWM value: ");
    Serial.println(HEAT_2);
  } else if (comm_temp > MIN_TEMP_EXTREM) { // batteries even colder
    analogWrite(PIN_HEATMATS, HEAT_3);
    Serial.print("Heat level 3, PWM value: ");
    Serial.println(HEAT_3);
  } else { // batteries beyond the limit
    analogWrite(PIN_HEATMATS, HEAT_FULL); // heat mats at full power
    Serial.print("Heat level FULL, PWM value: ");
    Serial.println(HEAT_FULL);    
  }
  return temp_error;
 }

