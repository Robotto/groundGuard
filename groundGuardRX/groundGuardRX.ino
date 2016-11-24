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
bool buzz=false;

void loop()
{
	height = Serial1.parseFloat();
	Serial.println(height);

	if(height<50) buzz=true;
	else buzz=false;

	if(height==0) buzz=false; //special case: data timeout.
	
	digitalWrite(vibratorPin, buzz);

}