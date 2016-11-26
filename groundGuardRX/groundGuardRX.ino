//groundGuardRX.ino
const unsigned int vibratorPin = 5;
void setup()
{

	//while(!Serial);
	Serial.begin(115200);
	Serial1.begin(115200);

	//Serial1.setTimeout(5000); //ms

	pinMode(vibratorPin,OUTPUT);

}

int height=100;
bool buzz=false;

void loop()
{
	if(Serial1.available()){
		height = (int)Serial1.read();
		Serial.println(height);
	}

	if(height<50) buzz=true;
	else buzz=false;

	if(height==0) buzz=false; //special case: data timeout.
	
	digitalWrite(vibratorPin, buzz);

}