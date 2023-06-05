
## [Temperature sensors](./temp_sensor)

Some Arduino code to identify and request the OneWire [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) temperature sensor.

Use libraries:

- https://playground.arduino.cc/Learning/OneWire/
- https://www.milesburton.com/w/index.php/Dallas_Temperature_Control_Library

---

### [Detect sensor ID](./detect_ds1820_tempsens_id/)

Get the sensor ID, better to have just one sensor to know the ID, because it will print all the sensors ID found


### [Get sensor temperature](./get_temp_ds18b20_wid/)

Having the IDs of the sensor, it will request their temperatures


