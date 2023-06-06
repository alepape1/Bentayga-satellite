

## [Temperature sensors](./temp_sensor)

Arduino code to identify and request temperature from the OneWire
 [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) temperature sensor, and to control the heatmats

Use libraries:

- https://playground.arduino.cc/Learning/OneWire
- https://www.milesburton.com/w/index.php/Dallas\_Temperature\_Control\_Library

---

### [Detect sensor ID](./temperature/detect_ds1820_tempsens_id/)

Get the sensor ID, better to have just one sensor to know the ID, because it will print all the sensors ID found.

As explained in the datasheet, there are two options for the sensor connection, we have used the one with external supply, using as VDD the 5V of the Arduino. 
A 4.7 Ohm resistor is needed, but the board sensor already has one. In the other cases, you will need to connect it.

Various sensor can be connected in multidrop, with multiple slaves.

![DS18D20 wiring](ds18bd20_wiring.png)


### [Get sensor temperature](./temperature/get_temp_ds18b20_wid/)

Having the IDs of the sensor, it will request their temperatures.

Since one of the sensors is in a board with a resistor, an additional resistor is not needed.

### [Temperature control](./temperature/tempsens_heatpad)

It has been tested with two sensors and two heatmats. The two heatmats are connected in parallel.
Since one of the sensors is in a board with a resistor, an additional resistor is not needed.

Using the MOSFET [IRLB8721](https://www.infineon.com/cms/en/product/power/mosfet/n-channel/irlb8721/) to control the heatmats.

The connections of the MOSFET are the following:
 - Gate (G) goes directly to a PWM output pin of the Arduino.
 - Source (S) goes to the GND
 - Drain (D) goes to one of the wires of the heatpad, the other wire of the heatpad goes to the external battery. We have connected two heatpads in parallel, since the MOSFET has an absolute maximum continous drain current of 62A at VGS=10V and T=25C

The battery GND is connected with the Arduino GND.


![IRLB8721](irlb8721.png)

Heatpads: https://es.rs-online.com/web/p/alfombrillas-calefactoras/0245512




