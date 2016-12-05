# groundGuard
Helps keep FPV drones off the ground.

Uses a lidarLiteV3 in conjunction with pitch angle from an adafruit 10DOF IMU to calculate the height of a drone.
Reports the height via a HC-12 433Mhz serial bridge, to a user-worn tactile feedback solution that helps keep drones off the ground.

![Concept](https://rawgit.com/Robotto/groundGuard/master/anglesSketch.svg "Concept")

## Transmitter:

![TX](https://rawgit.com/Robotto/groundGuard/master/tx_hw.png "Transmitter hardware")
### External libraries:

Uses the Adafruit unified sensor driver from [here](https://github.com/adafruit/Adafruit_Sensor)

Gyroscope library from [here](https://github.com/adafruit/Adafruit_L3GD20_U)

Accelerometer library from [here](https://github.com/adafruit/Adafruit_LSM303DLHC)

The lidarLiteV3 distanceFast() function is from [here](https://github.com/garmin/LIDARLite_v3_Arduino_Library/)


## Reciever:

![RX](https://rawgit.com/Robotto/groundGuard/master/rx_hw.png "Reciever hardware")

Uses the Adafruit SSD 1306 OLED library from [here](https://github.com/adafruit/Adafruit_SSD1306)

Which requires the adafruit GFX library found [here](https://github.com/adafruit/Adafruit-GFX-Library)


Both serial connections run at 115200 Baud, and the I2C connection with the lidarLite runs at 400Khz, so this should be pretty damn fast.
Setup the HC-12 to FU1 and 115200 baud (AT+FU1, AT+B115200) by pulling the "SET" pin low and issuing AT commands.

