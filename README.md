# groundGuard
Helps keep FPV drones off the ground.

Uses a lidarLiteV3 in conjunction with pitch angle from LightweightTeLemetry attitude packets from a Naze32 flightcontroller to calculate the height of a drone.
Reports the height via a HC-12 433Mhz serial bridge, to a user-worn tactile feedback solution that helps keep the drone off the ground.

![Concept](https://rawgit.com/Robotto/groundGuard/master/anglesSketch.svg "Concept")

Both serial connections run at 115200 Baud, and the I2C connection with the lidarLite runs at 400Khz, so this should be pretty damn fast.

The lidarLiteV3 distanceFast() function is from [here](https://github.com/garmin/LIDARLite_v3_Arduino_Library/)

Setup the HC-12 to FU1 and 115200 (AT+FU1, AT+B115200) by pulling the "SET" pin low and issuing AT commands.