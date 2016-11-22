//groundGuardTX.ino - By Mark Moore - October 2016
#define packetLength 16
const float pi = 3.14159265F;

int printdelay = 50;

//lidar:
#include <Wire.h>
#include <LIDARLite.h>
int distance=0;
int lastDistance=0;
LIDARLite Lidar;	

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float pitchRad;
float height;


void setup() {
  //using the same UART for RXing from flight controller and TXing to HC-12:
	Serial1.begin(115200); //RX: GPIO3, TX: GPIO1
  Serial.begin(115200);
	
  Lidar.begin(0, true); // Set configuration to default and I2C to 400 kHz
	Lidar.write(0x02, 0x0d);       //Maximum acquisition count of 0x0d. (default is 0x80) (limits effective range to about half of max)
	Lidar.write(0x04, 0b00000100); //Use non-default reference acquisition count
	Lidar.write(0x12, 0x03);       //Reference acquisition count of 3 (default is 5)
  delay(500);
  Serial.println("Lidar initialized.");
  
  delay(500);
  accel.begin();
  mag.begin();
  
  Serial.println("10dof initialized.");

	Serial1.println("Bird up."); //Transmit
	}

void loop() {
  
  delay(10);

  //Serial.println();
  
  sensors_vec_t   orientation;  

  if (ahrs.getOrientation(&orientation))
  {
    //distance = 100; //testdata
    distance = distanceFast(true); //pull a distance measurement
   
    pitchRad = (float)orientation.roll*(pi/180);
    height = (float)distance*(float)cos(pitchRad);
    
    /*
    Serial.print("pitch : "); Serial.println(-1*orientation.roll); //drone pitch is orientation.roll because the 9dof board is sideways.
    Serial.print("LIDAR : "); Serial.println(distance);   
    Serial.print("Height: "); Serial.println(height);*/

  	/*if(Serial1.availableForWrite()>=packetLength) {*/ Serial1.println(height); // }  //Transmit if there is room in the serial TX buffer  	
    
  }

}

// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int distanceFast(bool biasCorrection)
{
  byte isBusy = 1;
  int distance;
  int loopCount;

  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
    Wire.write(0X03); // Perform measurement without receiver bias correction
  }
  Wire.endTransmission();

  // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  // The I2C transaction finishes before new distance measurement data is acquired.
  // Prepare 2 byte read from registers 0x0f and 0x10
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0x8f);
  Wire.endTransmission();

  // Perform the read and repack the 2 bytes into 16-bit word
  Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
  distance = Wire.read();
  distance <<= 8;
  distance |= Wire.read();

  // Return the measured distance (in cm)
  return distance;
}