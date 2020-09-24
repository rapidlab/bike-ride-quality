# :mountain_bicyclist: Ride Quality Firmware  :bicyclist:

This program has been designed for bikers. 

It combines the functions of a GPS tracker,
while allowing you to determine the unevenness of the terrain on bicycle routes.

Its operation is based on collecting data from the GPS module (goegreaphic coordinates) and from the accelerometer (x-axis, y-axis, z-axis) 
and saving them as a .csv file on the SD card. 

The collected data can then be presented on a map in the form of points with a color intensity depending on the amount of vibration 
(this can be done by map_generator.py script).

## Hardware and Technologies used 

To be able to fully enjoy the functionality of the project, you must have:
- Nordic development board nrf52840dk
- GPS module (GPS module used during software testing: NEO-7M-C)
- accelerometer module (accelerometer module used during software tests: LIS2DW12 on STMicroelectronics STEVAL-MKI179V1 adapter board )

![nrf__1_](/uploads/f070be0eb40969fc2ebd08b5d6698fa9/nrf__1_.png)

Additionally, you must have the following software installed:
- Zephyr RTOS version 2.3.0.

## Usage

### Build

To build this firmawe, type the following on the command line:
```
    west build -b nrf52840dk_nrf52840
```

### Flashing

Then flash this program into your development board, by typing:
```
    west flash
```


