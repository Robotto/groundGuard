<<<<<<< HEAD
const unsigned int vibratorPin = 5;
void setup()
{

	//while(!Serial);
	Serial.begin(115200);
	Serial1.begin(115200);

	Serial1.setTimeout(5000); //ms

	pinMode(vibratorPin,OUTPUT);

=======
//groundGuardRX.ino
#include "ESP8266WiFi.h" //ironically, this is included to allow for disabling wifi..

static int vibroPin = D3;

void setup() {
	WiFi.forceSleepBegin();                  // turn off ESP8266 RF
	delay(1);  

	//while(!Serial); //wait for PC connection

	Serial.begin(115200);
	//Serial.swap(); //RX is now on D7

	Serial.setTimeout(100); //how long to wait when parsing data from serial port
>>>>>>> master
}

float height=100;

<<<<<<< HEAD
void loop()
{
	height = Serial1.parseFloat();
	Serial.println(height);

	if(height<10) digitalWrite(vibratorPin, HIGH);
	else digitalWrite(vibratorPin, LOW);
=======
void loop() {

	//height = Serial.parseFloat();

	if(Serial.available()) Serial.write(Serial.read()); //TEST CODE

>>>>>>> master
}