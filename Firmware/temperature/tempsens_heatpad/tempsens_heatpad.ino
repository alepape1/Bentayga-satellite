// Control the heatpad with the temperature sensor
// Since there are 2 sensors, it will take the average
// of them, and if the difference is larger than a value it will stop

#include <OneWire.h>
#include <DallasTemperature.h>

// Pin were the 1-Wire bus with the temp sensors DS18B20
const int PIN_ONEWIRE = 9;
const int PIN_HEATPAD = 10; // Pin where the MOSFET of the heatpad will be. Should be PWM

const int MAX_DIFF_SENS_TEMP = 5; // if temperature is larger than this, it might be a problem
// temperature from which the heatpad will be ON. Should be low, but this is for testing at room temperature
const int MIN_TEMP_START = 40;
// If this low temperature is reached, the heatpads will be at maximum temperature
const int MIN_TEMP_EXTREME = MIN_TEMP_START - 10;

// Resolution can be 9,10,11,12, the higher the slower
// default seems to be the last that has been set.
// with  9 it seems that it is 0.50 C resolution
// with 10 it seems that it is 0.25 C resolution
const uint8_t SENSOR_BIT_RESOL = 9;

// Instance to classes OneWire y DallasTemperature
OneWire OneWireObj(PIN_ONEWIRE);
DallasTemperature sensor_DS18(&OneWireObj);

// These are the address of the 2 sensors, the one on the board and the other waterproof
DeviceAddress board_tsens_addr  = {0x28, 0xFF, 0x10, 0x4B, 0x20, 0x18, 0x01, 0x10};
DeviceAddress waterp_tsens_addr = {0x28, 0x55, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0xEE};

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
    // Setup the output to control the heatpad
    pinMode(PIN_HEATPAD, OUTPUT);
    // Init serial monitor y and temperature sensor DS18B20
    Serial.begin(9600);
    sensor_DS18.begin();
    uint8_t resolution = sensor_DS18.getResolution();
    Serial.print("Resolution: ");
    Serial.print(resolution);
    Serial.println("bits");
    sensor_DS18.setResolution(SENSOR_BIT_RESOL);
    resolution = sensor_DS18.getResolution();
    Serial.print("Resolution: ");
    Serial.print(resolution);
    Serial.println("bits");

    // Searching sensors
    Serial.println("Searching temp sensors...");
    Serial.print("Found: ");
    int numSensorsFound = sensor_DS18.getDeviceCount();
    Serial.print(numSensorsFound);
    Serial.println(" sensors");

    // If found show address
    if(numSensorsFound>=1){
      for (int sens_i=0; sens_i<numSensorsFound; sens_i++) { 
        DeviceAddress sens_temp_addr; // 8 byte array (uint8_t)
        // get adddres of the sensor
        sensor_DS18.getAddress(sens_temp_addr, sens_i);

        if (comp_dev_addr(sens_temp_addr, waterp_tsens_addr)) {
          Serial.print("Waterproof sensor address: ");
        }
        if (comp_dev_addr(sens_temp_addr, board_tsens_addr)) {
          Serial.print("Board sensor address:      ");
        }
        // print the address
        prnt_dev_addr (sens_temp_addr);
      }  
   }    
}
 
void loop() {

  Serial.println("Requesting temperatures to sensors");
  sensor_DS18.requestTemperatures();
 
  float board_tsens_val = sensor_DS18.getTempC(board_tsens_addr);
  Serial.print("Board sensor:     ");
  Serial.print(board_tsens_val);
  Serial.println(" C");
  float waterp_tsens_val = sensor_DS18.getTempC(waterp_tsens_addr);
  Serial.print("Waterprof sensor: ");
  Serial.print(waterp_tsens_val);
  Serial.println(" C\n");

  // temperature difference between the two sensors
  float diff_sens_temp = abs(board_tsens_val -waterp_tsens_val);
  if (diff_sens_temp > MAX_DIFF_SENS_TEMP) {
    Serial.print("Temperature sensor difference larger than ");
    Serial.println(diff_sens_temp);
    // If temperature difference is too large: do nothing, maybe is an error
  } else {
    float avg_temp = (board_tsens_val + waterp_tsens_val)/2;
    Serial.print("Average temperature ");
    Serial.println(avg_temp);
    // Now it can be set the proportional control, but for now, just 2 limits
    if (avg_temp < MIN_TEMP_EXTREME) {
      digitalWrite (PIN_HEATPAD, HIGH); // heatpad at full ON
    } else if (avg_temp < MIN_TEMP_START) {
      analogWrite (PIN_HEATPAD, 128); // heatpad at half
    } else {
      digitalWrite (PIN_HEATPAD, LOW); // heatpad OFF
    }

  }
 
  delay(4000);
}