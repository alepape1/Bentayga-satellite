/*
    AdvancedMS5x.ino
	
	Shows all available function in the MS5X library
    
	Created 2021-05-25 
	By Matthew Bennett
	Modified -
	By -

    This file is part of arduino-MS5x.

    arduino-MS5x is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    arduino-MS5x is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with arduino-MS5x.  If not, see <http://www.gnu.org/licenses/>.

*/

//**************************************************************************************************************
//!
//! If you are using a MS5607 you need to open the header file and comment out line 24 (#define MS5611)
//! Failing to do this will result in the wrong math calculations being performed for temperature and pressure
//!
//**************************************************************************************************************

#include <Wire.h>
#include <MS5x.h>

MS5x barometer(&Wire);

bool barometerConnected = false;

uint32_t prevConnectionAttempt = 0;
uint32_t connectionAttemptDelay = 500; // Time in ms to wait between connection attempts to the sensor
uint32_t prevTime = 0;; // The time, in MS the device was last polled

double prevPressure=0; // The value of the pressure the last time the sensor was polled
double prevTemperature=0; // The value of the temperature the last time the sensor was polled
double seaLevelPressure=0;

void setup() {
	Serial.begin(115200);
	
	/* This will set the I2C address of the sensor, acceptable values are:
	   I2C_LOW		Sets address to 0x76
	   I2C_HIGH		Sets address to 0x77 <- Default
	*/
	barometer.setI2Caddr(I2C_HIGH); 
	
	/* This will set oversampling ratio, acceptable values are:
	   MS5xxx_CMD_ADC_256 (Resolution RMS Temp=0.012°C/Press=0.065mbar)
	   MS5xxx_CMD_ADC_512 (Resolution RMS Temp=0.008°C/Press=0.042mbar)
	   MS5xxx_CMD_ADC_1024 (Resolution RMS Temp=0.005°C/Press=0.027mbar)
	   MS5xxx_CMD_ADC_2048 (Resolution RMS Temp=0.003°C/Press=0.018mbar)
	   MS5xxx_CMD_ADC_4096 (Resolution RMS Temp=0.002°C/Press=0.012mbar) <- Default
	*/
	barometer.setSamples(MS5xxx_CMD_ADC_2048);
	/*
	Set the time between readings.  Make sure that as you do testing with the sensor you check for
	it self heating from reading too quickly.  Once a second polls and maximum oversampling didn't create
	any observable issues when used indoors during testing, but make sure to test for yourself and adjust these
	two settings as required.
	*/
	barometer.setDelay(1000); 

  //These three lines will set the temperature units returned by GetTemp().

  //barometer.setTempC(); // Uncommenting this line will have GetTemp() return temperature in Celcius (default temperature units)
  //barometer.setTempF(); // Uncommenting this line will have GetTemp() return temperature in Fahrenheit 
  //barometer.setTempK(); // Uncommenting this line will have GetTemp() return temperature in Kelvin 
  
  //These three lines will set the pressure units returned by GetPress().
  
  //barometer.setPressMbar(); // Uncommenting this line will have GetPress() return Pressure in Millibars (default pressure units)
  //barometer.setPressHg(); // Uncommenting this line will have GetPress() return Pressure in Inches Mercury (Inches Hg)
  //barometer.setPressPa(); // Uncommenting this line will have GetPress() return Pressure in Pascals
  
  // Temperature and Pressure offsets will always be in degrees Celcius and Pascals regardless of what temperature and pressure units are selected
  // This is intentional based on the way temperature and pressure calculations are performed.
  barometer.setTOffset(-200); // Decreases temperature reading by 2.00°C
  barometer.setPOffset(5); // Increases pressure reading by 5 Pascals
  
  
	if(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
		Serial.println(F("Error connecting..."));
	} else {
		Serial.println(F("Connected to Sensor"));
		barometerConnected = true;
	}
}

void loop() {
	
	double pressure = 0;
	double temperature = 0;
	double altitude = 0;
	
	if (!barometerConnected) {
		if (millis() - prevConnectionAttempt >= connectionAttemptDelay) {
			// Retry connection attemp
			if(barometer.connect()>0) {
				Serial.println(F("Error connecting..."));
				prevConnectionAttempt = millis();
			} else {
				Serial.println(F("Connected!"));
				barometerConnected = true;
			}
		}
	} else {
		/* In order to not have any delays used in code, checkUpdates cycles through sensor read process
			Step 1: Ask for raw temperature calculation to be performed
			Step 2: Once enough time has passed for calculation ask sensor to send results
			Step 3: Ask for raw pressure calculation to be performed
			Step 4: Once enough time has passed for calculation ask sensor to send results
			At this point checkUpdates returns true, but no new sensor readings will be performed until Readout function is called. */
		barometer.checkUpdates();
		if (barometer.isReady()) { // Barometer has performed inital calculations and data exists for  Temperature and Pressure 	
			temperature = barometer.GetTemp(); // Returns temperature in selected units
			pressure = barometer.GetPres(); // Returns pressure in selected units
			
			// New temperature and/or pressure values exist
			if ((temperature != prevTemperature) || (pressure != prevPressure)) {
				
				// If Sea Level Pressure has not been calculated, do so now.
				if (seaLevelPressure == 0) seaLevelPressure = barometer.getSeaLevel(217.3);
				
				Serial.print(F("The current pressure is: "));
				Serial.print(pressure);
				Serial.println(F(" Pascals"));
	
				Serial.print(F("The current temperature is: "));
				Serial.print(temperature);
				Serial.println(F("°C"));
			
				Serial.print(F("The calculated pressure at seaLevel is: "));
				Serial.print(seaLevelPressure);
				Serial.println(F(" Pascals"));
			
				// Calculate current altitude based on pressure reading
				altitude = barometer.getAltitude();
				Serial.print(F("The calculated altitude is: "));
				Serial.print(altitude);
				Serial.println(F(" meters"));
				
				// Calculate current altitude performing temperature corrections
				altitude = barometer.getAltitude(true);
				Serial.print(F("The calculated altitude with temperature correction is: "));
				Serial.print(altitude);
				Serial.println(F(" meters"));
				
				Serial.println();
				Serial.println();
				
				prevTemperature = temperature;
				prevPressure = pressure;
				
			}
		}
	}
}
