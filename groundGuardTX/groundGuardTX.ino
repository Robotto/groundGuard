//groundGuardTX.ino - By Mark Moore - October 2016
#define packetLength 16
const float pi = 3.14159;

//lidar:
#include <Wire.h>
#include <LIDARLite.h>
int distance=0;
int lastDistance=0;
LIDARLite Lidar;	


//FC:
byte fifo0=0, fifo1=0, fifo2=0;

int16_t pitchAngle=45/*, rollAngle, headingAngle*/;
int16_t lastPitchAngle;
float pitchRadians;
byte fcRX;
unsigned long attitudeFrameCounter=0;


void setup() {
  //using the same UART for RXing from flight controller and TXing to HC-12:
	Serial1.begin(115200); //RX: GPIO3, TX: GPIO1
  Serial.begin(115200);
	Lidar.begin(0, true); // Set configuration to default and I2C to 400 kHz

	Lidar.write(0x02, 0x0d);       //Maximum acquisition count of 0x0d. (default is 0x80) (limits effective range to about half of max)
	Lidar.write(0x04, 0b00000100); //Use non-default reference acquisition count
	Lidar.write(0x12, 0x03);       //Reference acquisition count of 3 (default is 5)

	delay(500);

	Serial1.println("groundGuard online.");
	Serial1.println("Hello basestation my old friend,");
	Serial1.println("I've come online to talk to you again.");
}

void loop() {



  distance = distanceFast(true); // Take a measurement with receiver bias correction and print to serial terminal
  //distance = 100; //testdata
   // Take 99 measurements without receiver bias correction and print to serial terminal
  //for(int i = 0; i < 99; i++) Serial1.println(distanceFast(false));
  
  //recalculate height if either pitchAngle or distance measurement has changed:
  if(pitchAngle!=lastPitchAngle || distance!=lastDistance) {
  	
    lastDistance=distance;
    lastPitchAngle=pitchAngle;

  	pitchRadians = (float)pitchAngle*pi/180;
  	float height = distance*cos(pitchRadians);
  	
  	/*if(Serial1.availableForWrite()>=packetLength) {*/ Serial1.println(height); // }  //Transmit if there is room in the serial TX buffer  	
    /*if(Serial.availableForWrite()>=packetLength)  {*/ Serial.print("Frame#: "); Serial.println(attitudeFrameCounter); Serial.println(height);   // } //Transmit if there is room in the serial TX buffer   
  }


if(Serial1.available()){
	//3 byte fifo buffer:
	fifo0 = fifo1;
	fifo1 = fifo2;
	fifo2 = Serial1.read();

	if(fifo0=='$' && fifo1=='T' && fifo2=='A') {//the next 6 rx bytes are the attitude frame

	//0,1: Pitch
	//2,3: Roll
	//4,5: Heading

	//recieve and 'decode' the first 2 bytes only, into the pitch uint16_t with little endianness:
		for(int i = 0; i<2 ;i++) {

			while(!Serial1.available()); //wait for byte

			fcRX=Serial1.read();

			switch (i) {
		    	case 0: //first pitch byte (binary: xxxx xxxx)
		     		pitchAngle = fcRX; //pitch = 0000 0000 xxxx xxxx
		     		break;

		    	case 1:	//second pitch byte (binary: yyyy yyyy)
		      		pitchAngle |= fcRX<<8; //pitch = yyyy yyyy xxxx xxxx 
		      		break;

			// case 2:	//first roll byte (binary: zzzz zzzz)
		 //      rollAngle = rx; //roll = 0000 0000 zzzz zzzz
		 //      break;

		 //    case 3:	//second roll byte (binary: ææææ ææææ)
		 //      rollAngle |= rx<<8; //roll = ææææ ææææ zzzz zzzz
		 //      break;

		 //    case 4:	//first heading byte (binary: øøøø øøøø)
		 //      headingAngle = rx; //heading = 0000 0000 øøøø øøøø
		 //      break;

		 //    case 5: //second heading byte (binary: åååå åååå)
		 //      headingAngle |= rx<<8; //heading = åååå åååå øøøø øøøø
		 //      break;

		    default: //shouldn't happen...
		      break;
				}
			}	
		attitudeFrameCounter++;
		}
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