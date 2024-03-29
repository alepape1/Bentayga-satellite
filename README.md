# Bentayga Satellite - Flight Manager

## Description

This code is part of the Bentayga Satellite project, developed by the IUMA at the University of Las Palmas de Gran Canaria. The Flight Manager is responsible for gathering flight data from the IMU and barometric pressure sensor and transmitting it via LoRa to a ground station.

## Requirements

This code requires the following libraries:
- Wire.h
- MS5x.h
- MKRIMU.h
- LoRa.h

## Setup

The Flight Manager should be connected to an IMU and a barometric pressure sensor. The LoRa module should also be properly connected.

## Usage

Upon startup, the Flight Manager will attempt to connect to the barometric pressure sensor. Once connected, it will continuously poll the sensor for data and transmit it via LoRa. Flight data is formatted into a byte array that includes start and stop bytes, as well as the following data:
- Roll
- Pitch
- Heading
- Temperature
- Pressure
- Sea level pressure
- Altitude
- Corrected altitude

In addition to transmitting the data, the Flight Manager also prints it to the serial monitor. 

## Contributing

If you wish to contribute to this code, please create a new branch from the develop branch and submit a pull request. 

## License

This code is licensed under the MIT License. 
