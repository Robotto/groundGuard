//GGG.ino - GroundGuardGyro

//groundGuardTX.ino - By Mark Moore - October 2016
//unused:
#define forceMagnitudeMIN 0.5
#define forceMagnitudeMAX 10

const float pi = 3.14159265F;

int printdelay = 50;

//lidar:
#include <Wire.h>
#include <LIDARLite.h>

int packetLength=8;

int distance=0;
LIDARLite Lidar;  

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h> //Gyroscope
#include <Adafruit_LSM303_U.h> //Accelerometer

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

float pitchAccRad;  //based on accelerometer data
float pitchGyroRad; //based on gyroscope data
float pitchMasterRad; //combined using 98% gyro and 2% acc 

float height;   //calculated from pitch and lidar distance using cosine relation
unsigned long time=0;


void setup() {
  	
	Serial1.begin(115200); //HC-12
  	Serial.begin(115200);  //Debug output
	
	//Setup lidar sensor:
  	Lidar.begin(0, true); // Set configuration to default and I2C to 400 kHz
	Lidar.write(0x02, 0x0d);       //Maximum acquisition count of 0x0d. (default is 0x80) (limits effective range to about half of max)
	Lidar.write(0x04, 0b00000100); //Use non-default reference acquisition count
	Lidar.write(0x12, 0x03);       //Reference acquisition count of 3 (default is 5)
  	Serial.println("Lidar initialized.");
  
	if(!accel.begin()) { Serial.println("Acc setup failed. Endless loop."); while(1); }
  	Serial.println("Accelerometer initialized.");
	
  	gyro.enableAutoRange(true);
  	if(!gyro.begin()) { Serial.println("gyro setup failed. Endless loop."); while(1); }
    Serial.println("Gyroscope initialized.");

	Serial1.println("Bird up."); //Transmit

	time=millis(); //get initial time

	}


void loop() {
  
  sensors_event_t event; 
  
  
  if(gyro.getEvent(&event))
  {
  	accel.getEvent(&event); //update accelerometer data

    //Convert from rad/sec to rad:
    float dt = ((float)millis()-(float)time)/1000; //time elapsed since last measurement
    time=millis(); //update time
    pitchGyroRad+=(float)event.gyro.x*dt; //rad/sec * sec;

    //Get bulk ACC data to filter out noisy data: (unused)
    //int forceMagnitudeApprox = abs(event.acceleration.x) + abs(event.acceleration.y) + abs(event.acceleration.z);
    //if(forceMagnitudeApprox>forceMagnitudeMAX || forceMagnitudeApprox<forceMagnitudeMIN) {Serial.print("forceMagnitudeApprox: "); Serial.println(forceMagnitudeApprox); return;}

    // Turning around the X axis results in a vector on the Y-axis .. i think...
    pitchAccRad = atan2f((float)event.acceleration.y, (float)event.acceleration.z);

    //filter:
    pitchMasterRad = (pitchMasterRad+(float)event.gyro.x*dt) * 0.98 + pitchAccRad * 0.02; 
    
    //rad -> deg
    float pitchMasterDeg = pitchMasterRad*(180/pi);//acceptable values are between 0 and 85 degrees.
    if(pitchMasterDeg>85 || pitchMasterDeg<0) { /*Serial.print("Pitch out of range: "); Serial.println(pitchMasterDeg);*/ return;} 
    
   	//Do a lidar measurement and calculate height:
    distance = distanceFast(false); //pull a distance measurement
    height = (float)distance*(float)cos(pitchMasterRad);
    
    //DEBUG PRINTOUT:
    //Serial.println();
    //Serial.print("forceMagnitudeApprox: "); Serial.println(forceMagnitudeApprox);
    //Serial.print("rawAcc : x: "); Serial.print(event.acceleration.x); Serial.print(" y: "); Serial.print(event.acceleration.y); Serial.print(" z: "); Serial.println(event.acceleration.z);
    //Serial.print("pitchAccRad : "); Serial.println(pitchAccRad); //drone pitch is orientation.roll because the 9dof board is sideways
   	//	Serial.print("pitchAccDeg : "); Serial.println(pitchAccRad*(180/pi));
    
    //Serial.print("dt : "); Serial.println(dt); 
    //Serial.print("rawGyro x: "); Serial.print(event.gyro.x); Serial.println("rad/s");
    //Serial.print("pitchGyroRad : "); Serial.println(pitchGyroRad); 
    //	Serial.print("pitchGyroDeg : "); Serial.println(pitchGyroRad*(180/pi));

    //Serial.print("pitchMasterRad : "); Serial.println(pitchMasterRad); //drone pitch is orientation.roll because the 9dof board is sideways.
    
    //	Serial.print("Pitch : "); Serial.println(pitchMasterDeg); //drone pitch is orientation.roll because the 9dof board is sideways.
    //	Serial.print("LIDAR : "); Serial.println(distance); 
    	Serial.print("Height float: ");
    		Serial.println(height);

    	Serial.print("Height uint8_t: "); 
    		Serial.println((uint8_t)height);
	
    if((uint8_t)height>255) return;
    //Report wirelessly via HC-12 on Serial1:
  	/*if(Serial1.availableForWrite()>=packetLength)*/ Serial1.write((uint8_t)height); //  //Transmit if there is room in the serial TX buffer  	
    
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