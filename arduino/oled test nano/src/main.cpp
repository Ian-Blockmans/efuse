#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {

  	//setup serial
	Serial.begin(9600,SERIAL_8N2);

  	// setup oled display
	if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3D))
	{
		Serial.println(F("SSD1306 allocation failed"));
		while (true){}
	}
	delay(2000);		 // wait for initializing
	oled.clearDisplay(); // clear display
}

void loop() {
    int pos = 1;
  const char * items[6] = {"LCL1", "LCL2", "LCL3", "12V", "5V", "toggle ON/OFF"};
  oled.clearDisplay(); // clear display

	for (uint8_t i = 0; i < ((uint8_t)8+(uint8_t)2); i++)
	{
		oled.drawLine(0,i+(pos*(uint8_t)10),128,i+(pos*(uint8_t)10),WHITE);
	}

	oled.setTextSize(1);		  // text size
	for (uint8_t i = 0; i < (uint8_t)(sizeof(items)/sizeof(items[0])); i++)
	{
		if (pos == i){
			oled.setTextColor(BLACK,WHITE);	  // text color
			oled.setCursor(20, (uint8_t)1+i+(i*(uint8_t)9));	// position to display
			oled.println(items[i]); // text to display
		}
		else
		{
			oled.setTextColor(WHITE);	  // text color
			oled.setCursor(20, (uint8_t)1+i+(i*(uint8_t)9));	// position to display
			oled.println(items[i]); // text to display
		}
	}
  oled.display();
  Serial.println("ok");
  delay(2000);
}