#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  // put your setup code here, to run once:
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  delay(2000);         // wait for initializing
  oled.clearDisplay(); // clear display

  oled.setTextSize(1);          // text size
  oled.setTextColor(WHITE);     // text color
  oled.setCursor(0, 10);        // position to display
  oled.println("Hello World!"); // text to display
  oled.display();               // show on OLED

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

}

void loop() {
  static char temp[50];
  static char temp1[50];
  static int i = 0;
  temp[0]=0;

  if (Serial.available() > 0) {
    while ((temp[i] = Serial.read()) > 1) {
      i++;
    }
    temp[i] = 0;
  }
  i = 0;
  if (strcmp(temp, "*IDN?\n") == 0) {
    Serial.write("hallo\n\r");
    oled.clearDisplay();          // clear display
    oled.setTextSize(1);          // text size
    oled.setTextColor(WHITE);     // text color
    oled.setCursor(0, 10);        // position to display
    oled.println("Data sent");    // text to display
    oled.display();       
  }
  if (strcmp(temp, "ok?\n") == 0) {
    Serial.write("fuse ok\n\r");
    oled.clearDisplay();          // clear display
    oled.setTextSize(1);          // text size
    oled.setTextColor(WHITE);     // text color
    oled.setCursor(0, 10);        // position to display
    oled.println("fuse ok");    // text to display
    oled.display();       
  }
}
