// Includes the required libraries
#include <Wire.h>
#include <MS5x.h>
#include <MKRIMU.h>
#include <LoRa.h>

// Initializes the barometer sensor and sets up the connection status variable
MS5x barometer(&Wire);
bool barometerConnected = false;

// Stores the time of the last connection attempt, delay time between attempts, and the last time the device was polled
uint32_t prevConnectionAttempt = 0;
uint32_t connectionAttemptDelay = 500; // Time in ms to wait between connection attempts to the sensor
uint32_t prevTime = 0; // The time, in MS the device was last polled

// Stores the value of the pressure and temperature the last time the sensor was polled
double prevPressure = 0;
double prevTemperature = 0;

// Stores the sea level pressure, used for altitude calculations
double seaLevelPressure = 0;

// Initializes the serial communication and the LoRa module
void setup() {
Serial.begin(9600);
while (!Serial);

if (!LoRa.begin(868E6)) {
Serial.println("Starting LoRa failed!");
while (1);
}

// Initializes the barometer sensor and sets up its settings
barometer.setI2Caddr(I2C_LOW);
barometer.setSamples(MS5xxx_CMD_ADC_2048);
barometer.setDelay(1000);
barometer.setPressMbar();
barometer.setTOffset(-200);
barometer.setPOffset(5);

// Attempts to connect to the barometer sensor
if (barometer.connect() > 0) {
Serial.println(F("Error connecting..."));
} else {
Serial.println(F("Connected to Sensor"));
barometerConnected = true;
}

Serial.println("LoRa Sender");
}

void loop() {
  // Read sensor data
  float heading, roll, pitch, temperature, pressure, seaLevelPressure, altitude, correctedAltitude;
  if (IMU.eulerAnglesAvailable()) {
    IMU.readEulerAngles(heading, roll, pitch);

    // Attempt to connect to barometer if not already connected
    if (!barometerConnected) {
      if (millis() - prevConnectionAttempt >= connectionAttemptDelay) {
        if (barometer.connect()>0) {
          Serial.println(F("Error connecting to barometer..."));
          prevConnectionAttempt = millis();
        } else {
          Serial.println(F("Connected to barometer!"));
          barometerConnected = true;
        }
      }
    } else {
      // Check if the barometer has new data available
      barometer.checkUpdates();
      if (barometer.isReady()) { 
        temperature = barometer.GetTemp(); 
        pressure = barometer.GetPres();

        // If the temperature or pressure have changed, update the variables and calculate new altitude data
        if ((temperature != prevTemperature) || (pressure != prevPressure)) {
          if (seaLevelPressure == 0) seaLevelPressure = barometer.getSeaLevel(pressure);
          altitude = barometer.getAltitude();
          correctedAltitude = barometer.getAltitude(true);

          // Format sensor data into a string
          String data = String(roll) + "," + String(pitch) + "," + String(heading) + "," + String(temperature) + "," + String(pressure) + "," + String(seaLevelPressure) + "," + String(altitude) + "," + String(correctedAltitude);
         
          // Create packet buffer with start and stop bytes, and add the formatted sensor data
          unsigned char buffer[] = {
            0xFF, // start byte
            (unsigned char)(roll * 10),
            (unsigned char)(pitch * 10),
            (unsigned char)(heading * 10),
            (unsigned char)(temperature + 128),
            (unsigned char)(fmod(pressure, 256)),
            (unsigned char)(fmod(seaLevelPressure, 256)),
            (unsigned char)(fmod(altitude, 256)),
            (unsigned char)(fmod(correctedAltitude, 256)),
            (unsigned char)(altitude / 256),
            (unsigned char)((int)altitude % 256),
            (unsigned char)(correctedAltitude / 256),
            (unsigned char)((int)correctedAltitude % 256),
            0xFE // stop byte
          };
          
          // Send packet via LoRa
          LoRa.beginPacket();
          LoRa.write(buffer, sizeof(buffer));
          LoRa.endPacket();
          Serial.println("Packet sent!");

          // Print the formatted sensor data to the serial monitor
          Serial.print("Flight data: ");
          Serial.print(data);
          Serial.println();
        }
      }
    }
  }
  delay(200);
}
