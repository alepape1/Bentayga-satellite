/*
* This script is intended to be a test station for the process of saving to the SD card.
* SD card. It will generate satellite data to test if the Arduino saves it correctly.
*/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_SleepyDog.h>

// This value is set using this constant
// in the example of the SD card SHILED 
// https://docs.arduino.cc/tutorials/mkr-sd-proto-shield/mkr-sd-proto-shield-data-logger
// It seems that you can use any unused pin to do this
#define SDCARD_SS_PIN 4 
const int chipSelect = SDCARD_SS_PIN;

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
  uint8_t camera_info;
  uint8_t camera_flag;
  // uint32_t checksum;
};

struct DummyClock {
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
};

// Data struct to save random generated time
DummyClock clock;

// Data struct to save the random generated data
SensorData sensorData;

// Archivo en la tarjeta SD
char folderName[10];
char fileName[10];
char filePath[22];

File dataFile;

#define SAMPLE_SENSOR_TIME 100
#define SAMPLE_GPS_TIME 6000
#define SEND_DATA_DELAY 1000
#define WRITE_SD_DATA_DELAY 2000
#define WATCHDOG_TIMEOUT 20000
#define NEW_SD_FILE 20

// Definir el contador para el sampling del barometro , IMU y sensores de temperatura
uint32_t sample_counter = 0;

//Definir el contador para el sampling del GPS
uint32_t GPS_sample_counter = 0;

// Definir el contador para el envio de datos
uint32_t send_data_timer = 0;

// Definir el contador de tiempo para la escritura en la SD de la trama de datos
uint32_t write_sd_data_timer = 0;

// Counter that is used to monitor the numbers of lines since the last new file
uint32_t sd_writes_counter = 0;

File openDataFile(const char *filename);
void saveDataToSDCard(File dataFile, SensorData &sensorData);
void generateFolderName(char * outName, DummyClock &clockData);
void generateFileName(char * outName, DummyClock &clockData);
// Fuzzing the script
void randomIMUData(SensorData &sensorData);
void randomBMEData(SensorData &sensorData);
void randomTemperatureData(SensorData &sensorData);
void randomCameraInfo(SensorData &sensorData);
void randomClockData(SensorData &sensorData);
void randomGPSData(SensorData &sensorData);


void setup() {
  // Initialize serial comunication to debug the code
  //Watchdog.disable();
  Serial.begin(115200);
  delay(2000);

  // Random data generator
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  //Initialize Random Clock

  Serial.println("Bentayga I. System init");
  // Initializing DummyClock
  clock.month = random(1, 12);
  clock.day = random(1, 30);
  clock.hour = random(0, 23);
  clock.minute = random(0, 59);

  // Inicializar la tarjeta SD
  if (!SD.begin(chipSelect)) {
    //Serial.println(SD.begin(4));
    Serial.println("ERROR: Failed to initialize SD card!");
  } else {
    Serial.print("SD card initialized on pin ");
    Serial.print(chipSelect, DEC);
    Serial.println(".      Status: OK");
    generateFolderName(folderName, clock);
    // Checking if folder exists or the script needs to create it
    if(!SD.exists(folderName)) {
      Serial.println("Creating directory...");
      SD.mkdir(folderName);
    }
    // Checks if the directory was correctly created
    if(SD.exists(folderName)) {
      Serial.print("Using directory ");
      Serial.println(folderName);
      generateFileName(fileName, clock);
      strcpy(filePath, folderName);
      strcat(filePath, "/");
      strcat(filePath, fileName);
      dataFile = openDataFile(filePath);
      dataFile.println("Reset WatchDog");
      dataFile.close();
      Serial.print("Using path ");
      Serial.println(filePath);

    } else {
      Serial.println("ERROR: Failed to create folder!");
    }
  }

  delay(4000);

  //Watchdog.enable for the setup and loop
  Watchdog.enable(WATCHDOG_TIMEOUT);

  sample_counter = millis();

}

void loop() {

  randomIMUData(sensorData);
  delay(100);

  randomBMEData(sensorData);
  delay(100);

  randomTemperatureData(sensorData);
  delay(100);

  randomCameraInfo(sensorData);
  delay(100);

  randomClockData(sensorData);
  delay(100);


  if(millis() - GPS_sample_counter > SAMPLE_GPS_TIME) {
    randomGPSData(sensorData);
    GPS_sample_counter = millis();
    delay(10);
  }

  String data = String(sensorData.day) + "/" + String(sensorData.month) + "/" + String(sensorData.year) + " " + String(sensorData.hour) + ":" + String(sensorData.minute) + ":" + String(sensorData.second) + "\t" + "Roll:" + String(sensorData.roll) + "\t" + "Pitch:" + String(sensorData.pitch) + "\t" + "Heading:" + String(sensorData.heading) + "\t" + "Temp-batt-Left:" + String(sensorData.battTempLeft) + "\t" + "Temp-batt-Right:" + String(sensorData.battTempRight) + "\t" + "Temp-batt-Down:" + String(sensorData.battTempDown) + "\t" + "Temp-batt-Box:" + String(sensorData.battTempBox) + "\t" + "Temp-Cubesat-DS18:" + String(sensorData.TempCubesatDS18) + "\n" + "Temperature:" + String(sensorData.temperature) + "\t" + "Humidity:" + String(sensorData.humidity) + "\t" + "Pressure:" + String(sensorData.pressure) + "\t" + "GpsAltitude:" + String(sensorData.GpsAltitude) + "\t" + "Latitude:" + String(sensorData.latitude, 6) + "\t" + "Longitude:" + String(sensorData.longitude, 6) + "\t" + "Speed:" + String(sensorData.speed) + "\t" + "NumSatellites:" + String(sensorData.numSatellites) + "\t" + "Camera:" + String(sensorData.camera_info) + "\t" + "Capturing:" + String(sensorData.camera_flag);

  if (millis() - send_data_timer >= SEND_DATA_DELAY) {
    Serial.println(data);
    send_data_timer = millis();
  }

  if (millis() - write_sd_data_timer >= WRITE_SD_DATA_DELAY) {
    Serial.println("Before write SD");
    saveDataToSDCard(openDataFile(filePath), sensorData);
    write_sd_data_timer = millis();
    sd_writes_counter++;
    if (sd_writes_counter > NEW_SD_FILE) {
      clock.hour++;

      if (clock.hour > 23) {
        clock.hour = 0;
      }

      generateFileName(fileName, clock);
      strcpy(filePath, folderName);
      strcat(filePath, "/");
      strcat(filePath, fileName);
      dataFile = openDataFile(filePath);
      dataFile.println("Reset Filesize");
      dataFile.close();
      Serial.print("Using path ");
      Serial.println(filePath);
      sd_writes_counter = 0;
    }

    delay(5);
  }

  Watchdog.reset();
}

/* @brief Generate random information that could be possible data 
*         read from the IMU and save it in sensorData 
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomIMUData(SensorData &sensorData)
{
  sensorData.roll = random(0, 35999)/100.0;
  sensorData.pitch = random(0, 35999)/100.0;
  sensorData.heading = random(0, 35999)/100.0;
}

/* @brief Generate random information that could be possible data 
*         read from the BME (pressure, humidity, temperature) sensor
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomBMEData(SensorData &sensorData)
{
  sensorData.temperature = random(-2000, 15000)/100.0;
  sensorData.humidity = random(0, 10000)/100.0;
  sensorData.pressure = random(0, 100000)/100.0;
}

/* @brief Generate random information that could be possible data 
*         read from the temperature sensors
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomTemperatureData(SensorData &sensorData)
{
  sensorData.TempCubesatDS18 = random(-2000, 15000)/100.0;
  sensorData.battTempBox = random(-2000, 15000)/100.0;
  sensorData.battTempLeft = random(-2000, 15000)/100.0;
  sensorData.battTempRight = random(-2000, 15000)/100.0;
  sensorData.battTempDown = random(-2000, 15000)/100.0;
}

/* @brief Generate random possible state of the camera
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomCameraInfo(SensorData &sensorData)
{
  uint8_t capturing = random(0,1);
  uint8_t memoryLSB = random(0,1);
  uint8_t memoryMSB = random(0,1);
  sensorData.camera_info = (memoryMSB << 2) | (memoryLSB << 1) | capturing;
}
/* @brief Generate random possible data ftom the RTC
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomClockData(SensorData &sensorData)
{
  sensorData.year = random(0, 99) + 2000;
  sensorData.month = random(0, 12);
  sensorData.day = random(0, 30);
  sensorData.hour = random(0, 23);
  sensorData.minute = random(0, 59);
  sensorData.second = random(0, 59);
}

/* @brief Generate random possible data ftom the GPS
*  @param [out] sensorData - Struct with all the data adquired by the sensors
*/
void randomGPSData(SensorData &sensorData)
{
  sensorData.latitude = random(-45000000, 45000000) / 1000000.0;
  sensorData.longitude = random(-45000000, 45000000) / 1000000.0;
  sensorData.GpsAltitude = random(0, 40000000) / 1000.0;
  sensorData.speed = random(0, 50000) / 1000.0;
  sensorData.numSatellites = random(3, 30);
}

/* @brief Store the sensors data in the current file of the SDand close the file
*  @param [in] dataFile - File object in which the data will be stored 
*  @param [in] sensorData - Struct with all the data adquired by the sensors
*/
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

/* @brief Open a file in the SD and checks if it exist
*  @param [in] filename - Path of the file to be opened
*  @return File - The SD file object correctly opened
*/
File openDataFile(const char *filename) {
  File dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.print("ERROR: Failed to open data file: ");
    Serial.println(filename);
  }
  return dataFile;
}

/* @brief Read the clock data and create a folder name with it
*  @param [out] outName - The name of the folder 
*  @param [in] clockData - The data extracted to the RTC
*/
void generateFolderName(char *outName, DummyClock &clockData)
{
  char strMonth[5];
  char strDay[5];
  itoa(clockData.month, strMonth, 10);
  itoa(clockData.day, strDay, 10);
  strcpy(outName, strMonth);
  strcat(outName, "_");
  strcat(outName, strDay);
}

/* @brief Read the clock data and create a file name with it
*  @param [out] outName - The name of the file 
*  @param [in] clockData - The data extracted to the RTC
*/
void generateFileName(char *outName, DummyClock &clockData)
{
  char strHour[5];
  char strMinute[5];
  itoa(clockData.hour, strHour, 10);
  itoa(clockData.minute, strMinute, 10);
  strcpy(outName, strHour);
  strcat(outName, strMinute);
  strcat(outName, ".txt");
}
