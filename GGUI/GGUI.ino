//GGUI.ino - GroundGuard User Interface
//Mark Moore - November 2016



//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
//bool redraw=false;

//ENCODER
#include <Encoder.h>
static unsigned blueLED=9, purpleLED=6, orangeLed=5;
static unsigned encA=7, encB=8, encBTN=4;
Encoder myEnc(encA, encB);
int clickCount=0;

//Groundguard:
const unsigned int vibratorPin = 10;
bool buzz=false;
uint8_t triggerHeight=64;
uint8_t height=64;

void setup(){
	oledInit();
	
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial1.setTimeout(5000); //ms
	
	pinMode(vibratorPin,OUTPUT);
	pinMode(purpleLED, OUTPUT);
	pinMode(orangeLed, OUTPUT);
	pinMode(blueLED, OUTPUT);

	digitalWrite(purpleLED, LOW);
	digitalWrite(orangeLed, LOW);	
	digitalWrite(blueLED, HIGH);

	Wire.begin();
	pinMode(encBTN,INPUT); //encoder channel A and B pin states are handled by the encoder library
}


void loop(){

if(Serial.available()) 
{
height=(uint8_t)Serial.read();
refreshRXHeight();
}

if(height<triggerHeight) buzz=true;
else buzz=false;
if(height==0) buzz=false; //special case: data timeout.

digitalWrite(vibratorPin, buzz);

if(buzz) { digitalWrite(blueLED, LOW); OLEDalarm(); }
else { display.invertDisplay(false); digitalWrite(blueLED, HIGH); digitalWrite(orangeLed,LOW); digitalWrite(purpleLED, LOW); }

int delta=readEncoder();

if(digitalRead(encBTN)) { 
	clickCount++;
	if 		(triggerHeight==0) 	triggerHeight=64; 
	else if (triggerHeight==64) triggerHeight=128;
	else 						triggerHeight=0;
	refreshTriggerHeight();
	delay(50); //debounce
	while(digitalRead(encBTN));
	if(clickCount>10) easteregg(true);
}


if(delta)
    {
    	if(clickCount>10)  easteregg(false); //clean up after easter egg.
 		clickCount=0;
    	triggerHeight-=delta;
    	if(triggerHeight<0) triggerHeight=0;
    	refreshTriggerHeight();
    	delta=0;
    }
	
}

void oledInit()
{
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

	setContrast(&display, 255);  //contrast is a number between 0 and 255. Use a lower number for lower contrast

	display.clearDisplay();
	display.setTextSize(3);
	display.setCursor(0,20);
	display.setTextColor(WHITE);
	display.print("OMGWTF!");
	display.display();
	//crazy_draw();
	display.clearDisplay();
	display.display();

	drawUIframe();

	

}

void drawUIframe()
{
	display.setTextColor(WHITE);
	
	display.setTextSize(2);
	display.setCursor(0,40);
	display.print("RX: ");
	refreshRXHeight();
	
	display.setTextSize(1);
	display.setCursor(0,0);
	display.print("Trigger height: ");
	refreshTriggerHeight();

	display.display();
}

void easteregg(bool active)
{
	if(active){
		display.clearDisplay();
		display.setTextSize(3);
		display.setCursor(0,20);
		display.setTextColor(WHITE);
		display.print(" STOP NU ");
		display.display();
		display.startscrolldiagright(0x00, 0x0F);
		}
	else{
		display.stopscroll(); 
		display.clearDisplay(); 
		drawUIframe();
		}
}

void refreshTriggerHeight()
{
	display.fillRect(0,10,128,6,BLACK);
	for(int j=0;j<triggerHeight;j++)
	{
	for(int i=0; i<6;i++)
    {
    display.drawPixel(j, i+10, WHITE);
    }
}

	display.fillRect(87,0,128,8,BLACK);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(90,0);
	display.print(triggerHeight);
	display.print(" cm");
	display.display();
}

void refreshRXHeight()
{
	display.fillRect(0,18,128,8,BLACK); //clear bar
	for(int j=128;j>=height;j--)
	{
	for(int i=0; i<8;i++)
    {
    	display.drawPixel(j, i+18, WHITE);
    	}
    }
	
	display.fillRect(0,56,128,8,BLACK); //clear under "RX"
    display.fillRect(36,32,128,32,BLACK); //clear numbers
	display.setTextSize(3);
	display.setTextColor(WHITE);
	display.setCursor(36,36);
	display.print(height);
	display.setTextSize(2);

	display.print(" cm");
	display.display();


}

void OLEDalarm()
{	
	static bool inverted;
	static long invertTime;
	if(millis()>invertTime+125){
	display.invertDisplay(inverted);
	digitalWrite(orangeLed, inverted);
	digitalWrite(purpleLED, !inverted);
	inverted=!inverted;
	invertTime=millis();
	}
}

void setContrast(Adafruit_SSD1306 *display, uint8_t contrast)
{
    display->ssd1306_command(SSD1306_SETCONTRAST);
    display->ssd1306_command(contrast);
}

int readEncoder()
{
	static long oldPosition;
	static int change;
	int returnVal=0;
	unsigned long newPosition = myEnc.read();

	if (newPosition != oldPosition)
  	{
  		change += newPosition-oldPosition;
		oldPosition = newPosition;
  	}

  	//if(change) redraw=true;
  	if(change<-3) { returnVal=-1; change=0;}
  	if(change>3) { returnVal=1; change=0;} //divide ticks by four so the input matches the mechanical feedback
  	return returnVal;
}

void crazy_draw()
{
    for(int j=4; j<128; j+=2)
    {
    for(int i=0; i<8;i++)
    {
    display.drawPixel(j, i+8, WHITE);
    display.drawPixel(j, i+48, WHITE);
    //delay(1);
  }
    display.display();
  }
  for(int j=4; j<128; j+=2)
    {
    for(int i=0; i<8;i++)
    {
    display.drawPixel(j, i+8, BLACK);
    display.drawPixel(j, i+48, BLACK);
    //delay(1);
  }
    display.display();
  }
}