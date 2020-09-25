# :mountain_bicyclist: Ride Quality Firmware  :bicyclist:

This tool has been designed for bikers.

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

<img width="294" alt="nRF52840 DK promo" src="https://user-images.githubusercontent.com/33400631/94154967-bab54e00-fe7e-11ea-8614-2f31241ba4f8.png">


Additionally, you must have the following software installed:
- Zephyr RTOS version 2.3.0.

An example of device housing created in Autodesk Fusion 360:

<img width="361" alt="Sample Enclosure" src="https://user-images.githubusercontent.com/33400631/94154606-601bf200-fe7e-11ea-93e5-69cbd3bb7617.PNG">


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

For more info, contact [RapidLab](https://www.rapidlab.io)