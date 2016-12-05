//HC-12 AT Commander
//The HC-12 default baud rate is 9600

/*
while energized, pull Pin 5 (“SET”) low,
wait 40ms for command mode to engage
*/

const int SETpin = 2; //command pin
bool commandMode = false;
void setup() {
	while(!Serial); //wait for serial connection from PC
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(SETpin, OUTPUT);
  digitalWrite(SETpin, !commandMode);

  Serial.println("Now in RF mode. Switch modes with: \\");

///AT+FU1
///AT+B115200

///AT+RF
///AT+RB

}

char rx;

void loop(){
  if (Serial1.available()) Serial.write(Serial1.read()); //dump data directly from HC-12 to PC

  if (Serial.available()){
  	rx=Serial.read();

  	if(rx=='\\') {// \\ means '\'', since \ is the escape char
  		commandMode=!commandMode; //flip the bool
  		digitalWrite(SETpin,!commandMode); //set the pin (command mode is active low)
  		if(commandMode) Serial.println("Command mode set.");
  		else Serial.println("RF mode set.");
  		delay(50);
  	}
   else Serial1.write(rx);
  }
}
