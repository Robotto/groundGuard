//groundGuardRX.ino

static int vibroPin = D1;

void setup() {
	Serial.begin(115200);
	Serial.swap(); //RX is now on D7

	Serial.setTimeout(100); //how long to wait when parsing data from serial port
}

float height=100;

void loop() {

	//height = Serial.parseFloat();

	if(Serial.available()) Serial.write(Serial.read()); //TEST CODE

}