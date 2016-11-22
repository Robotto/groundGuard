//groundGuardRX.ino
const unsigned int vibratorPin = 5;
void setup()
{

	//while(!Serial);
	Serial.begin(115200);
	Serial1.begin(115200);

	Serial1.setTimeout(5000); //ms

	pinMode(vibratorPin,OUTPUT);

}

float height=100;

void loop()
{
	height = Serial1.parseFloat();
	Serial.println(height);

	if(height<10) digitalWrite(vibratorPin, HIGH);
	else digitalWrite(vibratorPin, LOW);
}