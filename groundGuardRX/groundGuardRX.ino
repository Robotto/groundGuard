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
}

float height=100;

void loop() {

	//height = Serial.parseFloat();

	if(Serial.available()) Serial.write(Serial.read()); //TEST CODE

}