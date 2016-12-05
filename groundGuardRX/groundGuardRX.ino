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

//States enum:
enum states { //default state should be "get_data"
	min,
	max,
	insideRange,
	outsideRange
};

states state; //init the enum with the pass states.

const unsigned int vibratorPin = 10;
bool buzz=false;

uint8_t maxHeight;
uint8_t minHeight;
uint8_t rx_height=0;

void setup(){
	oledInit();
	
	//Serial.begin(115200);
	Serial1.begin(115200);
	//Serial1.setTimeout(5000); //ms
	
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

if(Serial1.available()) rx_height=(uint8_t)Serial1.read();

switch (state) {
    case min:
	  if(rx_height<minHeight) buzz=true;
	  else buzz=false;
      break;
    case max:
      if(rx_height>maxHeight) buzz=true;
      else buzz=false;
      break;
    case insideRange:
	  if(rx_height>minHeight && rx_height<maxHeight) buzz=true;
      else buzz=false;
      break;
    case outsideRange:
	  if(rx_height<minHeight || rx_height>maxHeight) buzz=true;
      else buzz=false;
      break; 
    
    default:
	  buzz=false;
	  break;
}

if(rx_height==0) buzz=false;     //special case: data timeout.
digitalWrite(vibratorPin, buzz);

int delta=readEncoder();

if(digitalRead(encBTN)) { 
	
	switch (state) {
    case min:
	  state=max;
      break;

    case max:
      state=insideRange;
      break;

    case insideRange:
	  state=outsideRange;
      break; 

    case outsideRange:
	  state=min;
      break; 

    default:
      state=min;
      break;
}
	
	display.clearDisplay();

	drawUIframe();

	refreshTriggerHeight();

	delay(30); //debounce
	while(digitalRead(encBTN));
	delay(20); //debounce
	}


if(delta)
    {
    	
	switch (state) {
	    case min:
	    	minHeight-=delta;	
	    	//if(minHeight<0) minHeight=255;      //uint8_t should wrap automagically
    		//else if(minHeight>255) minHeight=0;
    		if(minHeight>maxHeight) maxHeight=minHeight; //push values up
      		break;
    	case max:
	    	maxHeight-=delta;	
	    	//if(maxHeight<0) maxHeight=255;
    		//else if(maxHeight>255) maxHeight=0;
    		if(maxHeight<minHeight) minHeight=maxHeight;  //push values down	
      		break; 

    	default:
    		
    		if(minHeight>0 && maxHeight<255) //if range allows movement within uin8t_scope
    		{
	    	minHeight-=delta;	
	    	maxHeight-=delta;	
    		}

      		break;
		}

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
	delay(200);
	display.clearDisplay();
	display.display();

	drawUIframe();
}

void drawUIframe()
{
	display.setTextColor(WHITE);

		switch (state) {
    case min:
	  display.setTextSize(2);
	  display.setCursor(0,36);
	  display.print("Min: ");
	  refreshTriggerHeight();
      break;
    case max:
      display.setTextSize(2);
	  display.setCursor(0,36);
	  display.print("Max: ");
	  refreshTriggerHeight();
      break;
    //case range:
    default:
	  display.setTextSize(1);
	  display.setCursor(0,14);
	  if(state==insideRange) display.print("Buzz inside range:");
	  else display.print("Buzz outside range:");
	  display.setTextSize(2);

	  display.setCursor(0,28);
	  display.print("Min: ");
	  display.setCursor(0,44);
	  display.print("Max: ");
      break;
}

	display.display();
}


void refreshTriggerHeight()
{

	//display.fillRect(0,60,128,8,BLACK); //clear bottom of screen.
    display.fillRect(48,28,128,32,BLACK); //clear numbers
	
	display.setTextColor(WHITE);


		switch (state) {
    case min:

    	display.fillRect(0,0,128,8,BLACK);
		for(int j=0;j<minHeight;j++){
			for(int i=0; i<8;i++) display.drawPixel(j/2, i, WHITE);
			}
	  	display.setCursor(48,36);
		display.setTextSize(3);	
		display.print(minHeight);
		display.setTextSize(2);	
		display.print("cm");
	  	break;

    case max:

    	display.fillRect(0,0,128,8,BLACK);
		for(int j=255;j>maxHeight;j--){
			for(int i=0; i<8;i++) display.drawPixel(j/2, i, WHITE);
			}

      	display.setCursor(48,36);
		display.setTextSize(3);	
		display.print(maxHeight);
		display.setTextSize(2);	
		display.print("cm");

      	break;

    case insideRange:

        	display.fillRect(0,0,128,8,BLACK);
        
    	for(int j=minHeight;j<maxHeight;j++){
			for(int i=0; i<8;i++) display.drawPixel(j/2, i, WHITE);
			}		
		
		display.setTextSize(2);	

      	display.setCursor(48,28);
		display.print(minHeight);
		display.print("cm");

      	display.setCursor(48,44);
		display.print(maxHeight);
		display.print("cm");
	  
	  break; 

    case outsideRange:

         display.fillRect(0,0,128,8,BLACK);
        //bottom:
    	for(int j=0;j<minHeight;j++){
			for(int i=0; i<8;i++) display.drawPixel(j/2, i, WHITE);
			}		
		//top:
		for(int j=255;j>maxHeight;j--){
			for(int i=0; i<8;i++) display.drawPixel(j/2, i, WHITE);
			}

		
		display.setTextSize(2);	

      	display.setCursor(48,28);
		display.print(minHeight);
		display.print("cm");

      	display.setCursor(48,44);
		display.print(maxHeight);
		display.print("cm");
	  
	  break; 

    default:
      break;
}


	display.display();
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
