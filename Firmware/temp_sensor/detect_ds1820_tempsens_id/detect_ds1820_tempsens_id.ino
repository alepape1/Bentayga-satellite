// Get the ID (address) of a temperature sensor
// To identify one sensor address, better to use with
// only one sensor

#include <OneWire.h>
#include <DallasTemperature.h>

// Pin were the 1-Wire bus with the temp sensors DS18B20
const int PIN_ONEWIRE = 9;

// Instance to classes OneWire y DallasTemperature
OneWire OneWireObj(PIN_ONEWIRE);
DallasTemperature sensorDS18B20(&OneWireObj);

// print a One Wire device address
// a DeviceAddress is a 8 byte array
void prnt_dev_addr(DeviceAddress addr){
  for (uint8_t i = 0; i < 8; i++){
    // If only one digit, fill it wit a zero on the left
    if (addr[i] < 16) Serial.print("0");
      // show in HEX
    Serial.print(addr[i], HEX);
  }
  Serial.println("");
}

void setup() {
  // Init serial monitor y and temperature sensor DS18B20
  Serial.begin(9600);
  sensorDS18B20.begin();

  // Searching sensors
  Serial.println("Searching temp sensors...");
  Serial.print("Found: ");
  int numSensorsFound = sensorDS18B20.getDeviceCount();
  Serial.print(numSensorsFound);
  Serial.println(" sensors");
  // If found any, show address
  if(numSensorsFound>=1){
    for (int sens_i=0; sens_i<numSensorsFound; sens_i++) { 
      DeviceAddress sens_temp_addr; // 8 byte array (uint8_t)
      // get adddres of the sensor
      sensorDS18B20.getAddress(sens_temp_addr, sens_i);
      Serial.print("Sensor address: ");
      prnt_dev_addr (sens_temp_addr); // print the address
    }
  }   
}
 
void loop() {
}