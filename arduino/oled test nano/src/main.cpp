#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define UP_BUTTON 6
#define OK_BUTTON 10
#define DOWN_BUTTON 9
#define LEFT_BUTTON 8
#define RIGHT_BUTTON 7

#define LED_X1 5
#define LED_X2 4
#define LED_X3 3
#define LED_X4 2
#define LED_Y1 14
#define LED_Y2 15
#define LED_Y3 16

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void UserInterface(void);
void OledDisplay(uint8_t, uint8_t, uint8_t, uint8_t);
void Led_matrix(uint8_t, uint8_t, uint8_t, uint8_t);

// global variables
volatile uint8_t t35 = 0; 
volatile uint8_t t15 = 0;
volatile uint8_t send = 0;
uint8_t data[50];
uint8_t dataToSend[50];
uint8_t bytesRecieved;
uint8_t error = 0;
uint16_t bytecount = 0;
uint16_t bitcount = 0;
uint8_t dummycoil = 0;
uint8_t backup_state = 0;
uint8_t efuse_1_config_error = 0;
uint8_t efuse_2_config_error = 0;
uint8_t efuse_3_config_error = 0;
uint8_t efuse_4_config_error = 0;

void UserInterface(void){
	static uint8_t redraw = 1;
	static uint8_t position = 0;
	static uint8_t fuse = 0;
	//enum menu {lcl1,lcl2,lcl3,V5,V12};

	if (!digitalRead(UP_BUTTON)){
		delay(20);
		if (!digitalRead(UP_BUTTON)){
			if(position == (uint8_t)0){
				position = 3;
			}
			else
			{
				position--;
			}
			redraw = 1;
		}
	}
	else if (!digitalRead(DOWN_BUTTON)){
		delay(20);
		if (!digitalRead(DOWN_BUTTON)){
			if(position == (uint8_t)3){
				position = 0;
			}
			else
			{
				position++;
			}
			redraw = 1;
		}
	}
	else if (!digitalRead(LEFT_BUTTON)){
		delay(20);
		if (!digitalRead(LEFT_BUTTON)){
			if(fuse == (uint8_t)0){
				fuse = 3;
			}
			else
			{
				fuse--;
			}
			redraw = 1;
		}
	}
	else if (!digitalRead(RIGHT_BUTTON)){
		delay(20);
		if (!digitalRead(RIGHT_BUTTON)){
			if(fuse == (uint8_t)3){
				fuse = 0;
			}
			else
			{
				fuse++;
			}
			redraw = 1;
		}
	}
	else if (!digitalRead(OK_BUTTON)){

	}
	else{}

	if (redraw == (uint8_t)1){
		uint8_t status = 1;
		uint8_t onoff = 0;
		OledDisplay(position, fuse, status, onoff);
		redraw = 0;
		delay(50);
		while (!digitalRead(UP_BUTTON) || !digitalRead(OK_BUTTON) || !digitalRead(DOWN_BUTTON) || !digitalRead(LEFT_BUTTON) || !digitalRead(RIGHT_BUTTON))
		{
		}
	}
}

void Led_matrix(uint8_t x1bits, uint8_t x2bits, uint8_t x3bits, uint8_t x4bits){

	digitalWrite(LED_X1, HIGH);
	digitalWrite(LED_X2, LOW);
	digitalWrite(LED_X3, LOW);
	digitalWrite(LED_X4, LOW);
	if(x1bits & 1){
		digitalWrite(LED_Y1, LOW);
	}
	if(x1bits & 2){
		digitalWrite(LED_Y2, LOW);
	}
	if(x1bits & 4){
		digitalWrite(LED_Y3, LOW);
	}
	delayMicroseconds(100);
	digitalWrite(LED_Y1, HIGH);
	digitalWrite(LED_Y2, HIGH);
	digitalWrite(LED_Y3, HIGH);


	digitalWrite(LED_X1, LOW);
	digitalWrite(LED_X2, HIGH);
	digitalWrite(LED_X3, LOW);
	digitalWrite(LED_X4, LOW);
	if(x2bits & 1){
		digitalWrite(LED_Y1, LOW);
	}
	if(x2bits & 2){
		digitalWrite(LED_Y2, LOW);
	}
	if(x2bits & 4){
		digitalWrite(LED_Y3, LOW);
	}
	delayMicroseconds(100);
	digitalWrite(LED_Y1, HIGH);
	digitalWrite(LED_Y2, HIGH);
	digitalWrite(LED_Y3, HIGH);


	digitalWrite(LED_X1, LOW);
	digitalWrite(LED_X2, LOW);
	digitalWrite(LED_X3, HIGH);
	digitalWrite(LED_X4, LOW);
	if(x3bits & 1){
		digitalWrite(LED_Y1, LOW);
	}
	if(x3bits & 2){
		digitalWrite(LED_Y2, LOW);
	}
	if(x3bits & 4){
		digitalWrite(LED_Y3, LOW);
	}
	delayMicroseconds(100);
	digitalWrite(LED_Y1, HIGH);
	digitalWrite(LED_Y2, HIGH);
	digitalWrite(LED_Y3, HIGH);


	digitalWrite(LED_X1, LOW);
	digitalWrite(LED_X2, LOW);
	digitalWrite(LED_X3, LOW);
	digitalWrite(LED_X4, HIGH);
	if(x4bits & 1){
		digitalWrite(LED_Y1, LOW);
	}
	if(x4bits & 2){
		digitalWrite(LED_Y2, LOW);
	}
	if(x4bits & 4){
		digitalWrite(LED_Y3, LOW);
	}
	delayMicroseconds(100);
	digitalWrite(LED_Y1, HIGH);
	digitalWrite(LED_Y2, HIGH);
	digitalWrite(LED_Y3, HIGH);
}

void OledDisplay(uint8_t pos, uint8_t fuse, uint8_t status, uint8_t onoff){
	pos = pos + 1; 
	oled.clearDisplay(); // clear display
	uint8_t triangle_pos_left = 5;
	uint8_t triangle_pos_right = 58;
	uint8_t left_align = 6;
  	const char * items[4] = {"ON/OFF", "LCL1", "LCL2", "LCL3"};
	//const char * fuse_text[4] = {"eFuse 1", "eFuse 2", "eFuse 3", "eFuse 4"};
  	oled.clearDisplay(); // clear display
	oled.drawLine(64,0,64,64,WHITE);
	for (uint8_t i = 0; i < ((uint8_t)8+(uint8_t)2); i++)
	{
		oled.drawLine(0,i+(pos*(uint8_t)10),64,i+(pos*(uint8_t)10),WHITE);
	}
	oled.fillTriangle(triangle_pos_left,7,triangle_pos_left,1,triangle_pos_left-3,4,WHITE);
	oled.fillTriangle(triangle_pos_right,7,triangle_pos_right,1,triangle_pos_right+3,4,WHITE);

	oled.setTextSize(1);		  // text size
	oled.setTextColor(WHITE);	  // text color

	oled.setCursor(70, 1);	// position to display
	oled.println("status:"); // text to display
	oled.setCursor(70, 11);	// position to display
	switch (status)
	{
	case 1:
		oled.println("LCL1"); // text to display
		break;
	case 2:
		oled.println("LCL2"); // text to display
		break;
	case 4:
		oled.println("LCL3"); // text to display
		break;
	default:
		break;
	}
	oled.setCursor(70, 21);	// position to display
	switch (onoff)
	{
	case 0:
		oled.println("OFF"); // text to display
		break;
	case 1:
		oled.println("ON"); // text to display
		break;
	default:
		break;
	}
	
	oled.setCursor(11, 1);	// position to display
	switch (fuse)
	{
	case 0:
		oled.println("eFuse 1"); // text to display
		break;
	case 1:
		oled.println("eFuse 2"); // text to display
		break;
	case 2:
		oled.println("eFuse 3"); // text to display
		break;
	case 3:
		oled.println("eFuse 4"); // text to display
		break;
	default:
		break;
	}
	for (uint8_t i = 1; i < (uint8_t)(sizeof(items)/sizeof(items[0]))+1; i++)
	{
		if (pos == i){
			oled.setTextColor(BLACK,WHITE);	  // text color
			oled.setCursor(left_align, (uint8_t)1+i+(i*(uint8_t)9));	// position to display
			oled.println(items[i-1]); // text to display
		}
		else
		{
			oled.setTextColor(WHITE);	  // text color
			oled.setCursor(left_align, (uint8_t)1+i+(i*(uint8_t)9));	// position to display
			oled.println(items[i-1]); // text to display
		}
	}
  	oled.display();
	Serial.println("ok2");
	return;
}

void setup() {
	Serial.begin(9600);
	pinMode(UP_BUTTON, INPUT);
	pinMode(OK_BUTTON, INPUT);
	pinMode(DOWN_BUTTON, INPUT);
	pinMode(LEFT_BUTTON, INPUT);
	pinMode(RIGHT_BUTTON, INPUT);

	// setup oled display
	if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3D))
	{
		Serial.println(F("SSD1306 allocation failed"));
		while (true){}
	}
	delay(2000);		 // wait for initializing
	oled.clearDisplay(); // clear display

	pinMode(LED_X1, OUTPUT);
	pinMode(LED_X2, OUTPUT);
	pinMode(LED_X3, OUTPUT);
	pinMode(LED_X4, OUTPUT);
	pinMode(LED_Y1, OUTPUT);
	pinMode(LED_Y2, OUTPUT);
	pinMode(LED_Y3, OUTPUT);
	OledDisplay(1, 2, 1, 1);
  	Serial.println("ok");
}

void loop() {
    UserInterface();
	Led_matrix(0b010, 0b001, 0b010, 0b010);
}