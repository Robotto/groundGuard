# groundGuard
Helps keep FPV drones off the ground.

Uses a lidarLiteV3 in conjunction with pitch angle from an adafruit 10DOF IMU to calculate the height of a drone.
Reports the height via a HC-12 433Mhz serial bridge, to a user-worn tactile feedback solution that helps keep drones off the ground.

![Concept](https://rawgit.com/Robotto/groundGuard/master/anglesSketch.svg "Concept")

## Transmitter:

### External libraries:

Uses the Adafruit unified sensor driver from [here](https://github.com/adafruit/Adafruit_Sensor)

Gyroscope library from [here](https://github.com/adafruit/Adafruit_L3GD20_U)

Accelerometer library from [here](https://github.com/adafruit/Adafruit_LSM303DLHC)

The lidarLiteV3 distanceFast() function is from [here](https://github.com/garmin/LIDARLite_v3_Arduino_Library/)

![TX](https://rawgit.com/Robotto/groundGuard/master/tx_hw.png "Transmitter hardware")

## Reciever:

### External libraries:

Uses Paul Stoffregens quadrature encoder library from [here](https://github.com/PaulStoffregen/Encoder)

Uses the Adafruit SSD 1306 OLED library from [here](https://github.com/adafruit/Adafruit_SSD1306), which requires the adafruit GFX library found [here](https://github.com/adafruit/Adafruit-GFX-Library)

![RX](https://rawgit.com/Robotto/groundGuard/master/rx_hw.png "Reciever hardware")

The I2C connection with the lidarLite runs at 400Khz, which yields individual filtered measurements at about 250Hz. Both serial connections run at 4800 Baud, so this should be pretty damn fast.

Use the HC12 configurator sketch in the utility folder to setup the HC-12 to FU1 and 4800 baud (AT+FU1, AT+4800) (by pulling the "SET" pin low and issuing AT commands.)

