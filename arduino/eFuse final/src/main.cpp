#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DueFlashStorage.h>
#include <DueTimer.h>
#include <string.h>
#include <math.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define COILS 16 // aantal coils
#define INPUTS 12 // aantal inputs

//arduino io
#define UP_BUTTON 6
#define OK_BUTTON 4
#define DOWN_BUTTON 3
#define LEFT_BUTTON 2
#define RIGHT_BUTTON 5

#define LED_X1 10
#define LED_X2 9
#define LED_X3 8
#define LED_X4 7
#define LED_Y1 11
#define LED_Y2 12
#define LED_Y3 13

#define RELAY1 22
#define LCL3_1 24
#define LCL2_1 26
#define LCL1_1 28
#define SHDN1 30
#define FLT1 32

#define RELAY2 34
#define LCL3_2 36
#define LCL2_2 38
#define LCL1_2 40
#define SHDN2 42
#define FLT2 44

#define RELAY3 46
#define LCL3_3 48
#define LCL2_3 50
#define LCL1_3 52
#define SHDN3 53
#define FLT3 51

#define RELAY4 49
#define LCL3_4 47
#define LCL2_4 45
#define LCL1_4 43
#define SHDN4 41
#define FLT4 39

// flash storage object
DueFlashStorage dueFlashStorage;

// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);

typedef enum
{
	INITIAL_STATE = 0,
	IDLE_STATE,
	EMISSION_STATE,
	RECEPTION_STATE,
	CONTROL_AND_WAIT_STATE,
} fsm_states_t;

/* Table of CRC values for high–order byte */ static unsigned char auchCRCHi[] = {0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};
/* Table of CRC values for low–order byte */ static char auchCRCLo[] = {0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

//functions
void initial_state(void *);
void idle_state(void *);
void reception_state(void *);
void control_and_wait_state(void *);
void emission_state(void *);

void init_timer(void);
void T15_Handler(void);
void T35_Handler(void);
void Oled_Handler(void);

uint8_t ReadCoils(uint16_t,uint16_t);
uint8_t ReadReg(uint16_t, uint16_t);
uint8_t ReadInputs(uint16_t, uint16_t);
uint8_t WriteCoils(uint16_t, uint16_t, uint8_t *, uint16_t);
void Exeption(uint8_t);

void UserInterface(void);
void OledDisplay(uint8_t, uint8_t, uint8_t, uint8_t);
void Led_matrix(uint8_t, uint8_t, uint8_t, uint8_t);

// global variables
fsm_states_t fsm_current_state = INITIAL_STATE;
typedef void (*fsm_handler_t)(void *arg);
fsm_handler_t call_handler[6] = {initial_state, idle_state, emission_state, reception_state, control_and_wait_state};
volatile uint8_t t35 = 0; 
volatile uint8_t t15 = 0;
volatile uint8_t send = 0;
volatile uint8_t redraw = 1;
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
uint8_t efuse1_status = 0;
uint8_t efuse2_status = 0;
uint8_t efuse3_status = 0;
uint8_t efuse4_status = 0;

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen) /* The function returns the CRC as a unsigned short type   */
//unsigned char *puchMsg ;  message to calculate CRC upon 
//unsigned short usDataLen ;  quantity of bytes in message 
{
	unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized  */
	unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized  */
	unsigned uIndex;			   /* will index into CRC lookup table  */
	while (usDataLen-- != NULL)			   /* pass through message buffer  */
	{
		uIndex = uchCRCLo ^ *puchMsg; /* calculate the CRC  */
		puchMsg++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (((uint16_t)uchCRCHi << 8) | uchCRCLo);
}

void start_t15()
{
	Timer4.start(); //enable timer4
}

void stop_t15()
{
	Timer4.stop(); //disable timer4
	//clear counter
}

void start_t35()
{
	Timer5.start(); //enable timer5
}

void stop_t35()
{
	Timer5.stop(); //disable timer5
	//clear counter
}

void start_timers()
{
	Timer4.start(); //enable timer4
	Timer5.start(); //enable timer5
}

void stop_timers()
{
	Timer4.stop(); //disable timer4
	//clear counter
	Timer5.stop(); //disable timer5
	//clear counter
}

void initial_state(void *arg)
{
	start_t35();
	if (t35 == (uint8_t)1)
	{
		t35 = 0;
		stop_t35();
		fsm_current_state = IDLE_STATE;
	}
	else if (SerialUSB.available() > 0)
	{
		data[bytesRecieved] = SerialUSB.read();
		bytesRecieved++;
		stop_t35();
		start_t35();
	}
	else{}	
}

void idle_state(void *arg)
{
	if (SerialUSB.available() > 0)
	{
		data[bytesRecieved] = SerialUSB.read();
		bytesRecieved++;
		stop_timers();
		start_timers();
		fsm_current_state = RECEPTION_STATE;
	}
	else if (send == (uint8_t)1)
	{
		send = 0;
		fsm_current_state = EMISSION_STATE;
	}
	else {
		if (backup_state == 1)
		{
			backup_state = 0;
			for (uint8_t i = 0; i < (COILS/8); i++)
			{
				if ((ReadCoils((i*8),((i*8)+8)) & 0b01110111) != dueFlashStorage.read(i)){
					//dueFlashStorage.write(0,ReadCoils(0,8) & 0b01110111);//write byte 1 - on of bits if change
				}
			}
		}
		if( efuse_1_config_error > 1){
			digitalWrite(RELAY1, LOW);
			digitalWrite(SHDN1, LOW);
			efuse1_status |= 0b100;
			//turn on error led
		}
		else{
			efuse1_status &= (~0b100);
		}

		if( efuse_2_config_error > 1){
			digitalWrite(RELAY2, LOW);
			digitalWrite(SHDN2, LOW);
			efuse2_status |= 0b100;
			//turn on error led
		}
		else{
			efuse2_status &= (~0b010);
		}

		if( efuse_3_config_error > 1){
			digitalWrite(RELAY3, LOW);
			digitalWrite(SHDN3, LOW);
			efuse3_status |= 0b100;
			//turn on error led
		}
		else{
			efuse3_status &= (~0b010);
		}

		if( efuse_4_config_error > 1){
			digitalWrite(RELAY4, LOW);
			digitalWrite(SHDN4, LOW);
			efuse4_status |= 0b100;
			//turn on error led
		}
		else{
			efuse4_status &= (~0b010);
		}

		if (digitalRead(FLT1) == LOW)
		{
			digitalWrite(RELAY1, LOW);
			digitalWrite(SHDN1, LOW);
			efuse1_status |= 0b010;
		}
		else{}

		if (digitalRead(FLT2) == LOW)
		{
			digitalWrite(RELAY2, LOW);
			digitalWrite(SHDN2, LOW);
			efuse2_status |= 0b010;
		}
		else{}

		if (digitalRead(FLT3) == LOW)
		{
			digitalWrite(RELAY3, LOW);
			digitalWrite(SHDN3, LOW);
			efuse3_status |= 0b010;
		}
		else{}

		if (digitalRead(FLT4) == LOW)
		{
			digitalWrite(RELAY4, LOW);
			digitalWrite(SHDN4, LOW);
			efuse4_status |= 0b010;
		}
		else{}

		if (ReadCoils(0,1)>0)
		{
			efuse1_status |= 1;
			efuse1_status &= (~0b110);
		}
		else
		{
			efuse1_status &= (~1);
		}
		
		if (ReadCoils(4,1)>0)
		{
			efuse2_status |= 1;
			efuse2_status &= (~0b110);
		}
		else
		{
			efuse2_status &= (~1);
		}

		if (ReadCoils(8,1)>0)
		{
			efuse3_status |= 1;
			efuse3_status &= (~0b110);
		}
		else
		{
			efuse3_status &= (~1);
		}

		if (ReadCoils(12,1)>0)
		{
			efuse4_status |= 1;
			efuse4_status &= (~0b110);
		}
		else
		{
			efuse4_status &= (~1);
		}
		Led_matrix(efuse1_status, efuse2_status, efuse3_status, efuse4_status);
		UserInterface();
	}
}

void reception_state(void *arg)
{
	if (SerialUSB.available() > 0)
	{
		data[bytesRecieved] = SerialUSB.read();
		bytesRecieved++;
		stop_timers();
		start_timers();
		fsm_current_state = RECEPTION_STATE;
	}
	if (t15 == (uint8_t)1)
	{
		bytesRecieved = 0;
		t15 = 0;
		stop_t15();
		fsm_current_state = CONTROL_AND_WAIT_STATE;
	}
}

uint8_t ReadCoils(uint16_t starta, uint16_t bits) //COILS 1
{
	uint8_t retdata = 0;
	for (uint16_t i = starta; i < (bits+starta); i++)
	{
		int req = i;
		//efuse1
		if (i == (uint16_t)0)
		{
			retdata |= ((digitalRead(SHDN1) & digitalRead(RELAY1)) <<(i-starta));
		}
		if (i == (uint16_t)1)
		{
			retdata |= ((!digitalRead(LCL1_1))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)2)
		{
			retdata |= ((!digitalRead(LCL2_1))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)3)
		{
			retdata |= ((!digitalRead(LCL3_1))&HIGH)<<(i-starta);
		}
		//efuse2
		if (i == (uint16_t)4)
		{
			retdata |= ((digitalRead(SHDN2) & digitalRead(RELAY2))<<(i-starta));
		}
		if (i == (uint16_t)5)
		{
			retdata |= ((!digitalRead(LCL1_2))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)6)
		{
			retdata |= ((!digitalRead(LCL2_2))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)7)
		{
			retdata |= ((!digitalRead(LCL3_2))&HIGH)<<(i-starta);
		}
		//efuse3
		if (i == (uint16_t)8)
		{
			retdata |= ((digitalRead(SHDN3) & digitalRead(RELAY3))<<(i-starta));
		}
		if (i == (uint16_t)9)
		{
			retdata |= ((!digitalRead(LCL1_3))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)10)
		{
			retdata |= ((!digitalRead(LCL2_3))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)11)
		{
			retdata |= ((!digitalRead(LCL3_3))&HIGH)<<(i-starta);
		}
		//efuse4
		if (i == (uint16_t)12)
		{
			retdata |= ((digitalRead(SHDN4) & digitalRead(RELAY4))<<(i-starta));
		}
		if (i == (uint16_t)13)
		{
			retdata |= ((!digitalRead(LCL1_4))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)14)
		{
			retdata |= ((!digitalRead(LCL2_4))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)15)
		{
			retdata |= ((!digitalRead(LCL3_4))&HIGH)<<(i-starta);
		}
		//... to add coils for read
	}
	return retdata;
}

uint8_t ReadInputs(uint16_t starta, uint16_t bits)
{
	uint8_t retdata = 0;
	for (uint16_t i = starta; i < (bits+starta); i++)
	{
		int req = i;
		if (i == (uint16_t)0)
		{
			retdata |= digitalRead(RELAY1)<<(i-starta); //1 == on 
		}
		if (i == (uint16_t)1)
		{
			retdata |= ((efuse1_status & 0b010)>>1)<<(i-starta); //1 == fault
		}
		if (i == (uint16_t)2)
		{
			retdata |= ((efuse1_status & 0b100)>>2)<<(i-starta); //1 == config fault
		}
		if (i == (uint16_t)3)
		{
			retdata |= digitalRead(RELAY2)<<(i-starta); //1 == on 
		}
		if (i == (uint16_t)4)
		{
			retdata |= ((efuse2_status & 0b010)>>1)<<(i-starta); //1 == fault
		}
		if (i == (uint16_t)5)
		{
			retdata |= ((efuse2_status & 0b100)>>2)<<(i-starta);
		}
		if (i == (uint16_t)6)
		{
			retdata |= digitalRead(RELAY3)<<(i-starta); //1 == on  
		}
		if (i == (uint16_t)7)
		{
			retdata |= ((efuse3_status & 0b010)>>1)<<(i-starta); //1 == fault
		}
		if (i == (uint16_t)8)
		{
			retdata |= ((efuse3_status & 0b100)>>2)<<(i-starta);
		}
		if (i == (uint16_t)9)
		{
			retdata |= digitalRead(RELAY4)<<(i-starta); //1 == on  
		}
		if (i == (uint16_t)10)
		{
			retdata |= ((efuse4_status & 0b010)>>1)<<(i-starta); //1 == fault
		}
		if (i == (uint16_t)11)
		{
			retdata |= ((efuse4_status & 0b100)>>2)<<(i-starta);
		}
		//... to add coils for read
	}
	return retdata;
}

uint8_t WriteCoils(uint16_t starta, uint16_t byte, uint8_t* write,uint16_t bits)
{
	efuse_1_config_error = 0;
	efuse_2_config_error = 0;
	efuse_3_config_error = 0;
	efuse_4_config_error = 0;
	//efuse1
	//adress:                                                              ↓                          ↓                                ↓
	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)0)) && (starta <= (uint8_t)0) && ((bits+starta) >= (uint8_t)0)) //ON/OFF
	{
		digitalWrite(SHDN1, HIGH);
		digitalWrite(RELAY1, HIGH);
	}
	//adress:                    ↓                                ↓
	else if ((starta <= (uint8_t)0) && ((bits+starta) >= (uint8_t)0))
	{
		digitalWrite(SHDN1, LOW);
		digitalWrite(RELAY1, LOW);
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)1)) && (starta <= (uint8_t)1) && ((bits+starta) >= (uint8_t)1)) //LCL1
	{
		digitalWrite(LCL1_1, LOW);
		digitalWrite(LCL2_1, HIGH);
		digitalWrite(LCL3_1, HIGH);
		efuse_1_config_error ++;
	}
	else if((starta <= (uint8_t)1) && ((bits+starta) >= (uint8_t)1))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)2)) && (starta <= (uint8_t)2) && ((bits+starta) >= (uint8_t)2)) //LCL2
	{
		digitalWrite(LCL1_1, HIGH);
		digitalWrite(LCL2_1, LOW);
		digitalWrite(LCL3_1, HIGH);
		efuse_1_config_error ++;
	}
	else if((starta <= (uint8_t)2) && ((bits+starta) >= (uint8_t)2))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)3)) && (starta <= (uint8_t)3) && ((bits+starta) >= (uint8_t)3)) //LCL3
	{
		digitalWrite(LCL1_1, HIGH);
		digitalWrite(LCL2_1, HIGH);
		digitalWrite(LCL3_1, LOW);
		efuse_1_config_error ++;
	}
	else if((starta <= (uint8_t)3) && ((bits+starta) >= (uint8_t)3))
	{
	}
	else{}

	// efuse 2 ------------------------------------------------------------------------------------------------------------------------------------
	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)4)) && (starta <= (uint8_t)4) && ((bits+starta) >= (uint8_t)4)) //ON/OFF
	{
		digitalWrite(SHDN2, HIGH);
		digitalWrite(RELAY2, HIGH);
	}

	else if ((starta <= (uint8_t)4) && ((bits+starta) >= (uint8_t)4))
	{
		digitalWrite(SHDN2, LOW);
		digitalWrite(RELAY2, LOW);
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)5)) && (starta <= (uint8_t)5) && ((bits+starta) >= (uint8_t)5)) //LCL1
	{
		digitalWrite(LCL1_2, LOW);
		digitalWrite(LCL2_2, HIGH);
		digitalWrite(LCL3_2, HIGH);
		efuse_2_config_error ++;
	}
	else if((starta <= (uint8_t)5) && ((bits+starta) >= (uint8_t)5))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)6)) && (starta <= (uint8_t)6) && ((bits+starta) >= (uint8_t)6)) //LCL2
	{
		digitalWrite(LCL1_2, HIGH);
		digitalWrite(LCL2_2, LOW);
		digitalWrite(LCL3_2, HIGH);
		efuse_2_config_error ++;
	}
	else if((starta <= (uint8_t)6) && ((bits+starta) >= (uint8_t)6))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)7)) && (starta <= (uint8_t)7) && ((bits+starta) >= (uint8_t)7)) //LCL3
	{
		digitalWrite(LCL1_2, HIGH);
		digitalWrite(LCL2_2, HIGH);
		digitalWrite(LCL3_2, LOW);
		efuse_2_config_error ++;
	}
	else if((starta <= (uint8_t)7) && ((bits+starta) >= (uint8_t)7))
	{
	}
	else{}

	// efuse 3 ------------------------------------------------------------------------------------------------------------------------------------
	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)8)) && (starta <= (uint8_t)8) && ((bits+starta) >= (uint8_t)8)) //ON/OFF
	{
		digitalWrite(SHDN3, HIGH);
		digitalWrite(RELAY3, HIGH);
	}

	else if ((starta <= (uint8_t)8) && ((bits+starta) >= (uint8_t)8))
	{
		digitalWrite(SHDN3, LOW);
		digitalWrite(RELAY3, LOW);
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)9)) && (starta <= (uint8_t)9) && ((bits+starta) >= (uint8_t)9)) //LCL1
	{
		digitalWrite(LCL1_3, LOW);
		digitalWrite(LCL2_3, HIGH);
		digitalWrite(LCL3_3, HIGH);
		efuse_3_config_error ++;
	}
	else if((starta <= (uint8_t)9) && ((bits+starta) >= (uint8_t)9))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)10)) && (starta <= (uint8_t)10) && ((bits+starta) >= (uint8_t)10)) //LCL2
	{
		digitalWrite(LCL1_3, HIGH);
		digitalWrite(LCL2_3, LOW);
		digitalWrite(LCL3_3, HIGH);
		efuse_3_config_error ++;
	}
	else if((starta <= (uint8_t)10) && ((bits+starta) >= (uint8_t)10))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)11)) && (starta <= (uint8_t)11) && ((bits+starta) >= (uint8_t)11)) //LCL3
	{
		digitalWrite(LCL1_3, HIGH);
		digitalWrite(LCL2_3, HIGH);
		digitalWrite(LCL3_3, LOW);
		efuse_3_config_error ++;
	}
	else if((starta <= (uint8_t)11) && ((bits+starta) >= (uint8_t)11))
	{
	}
	else{}

	// efuse 4 ------------------------------------------------------------------------------------------------------------------------------------
	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)12)) && (starta <= (uint8_t)12) && ((bits+starta) >= (uint8_t)12)) //ON/OFF
	{
		digitalWrite(SHDN4, HIGH);
		digitalWrite(RELAY4, HIGH);
	}

	else if ((starta <= (uint8_t)12) && ((bits+starta) >= (uint8_t)12))
	{
		digitalWrite(SHDN4, LOW);
		digitalWrite(RELAY4, LOW);
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)13)) && (starta <= (uint8_t)13) && ((bits+starta) >= (uint8_t)13)) //LCL1
	{
		digitalWrite(LCL1_4, LOW);
		digitalWrite(LCL2_4, HIGH);
		digitalWrite(LCL3_4, HIGH);
		efuse_4_config_error ++;
	}
	else if((starta <= (uint8_t)13) && ((bits+starta) >= (uint8_t)13))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)14)) && (starta <= (uint8_t)14) && ((bits+starta) >= (uint8_t)14)) //LCL2
	{
		digitalWrite(LCL1_4, HIGH);
		digitalWrite(LCL2_4, LOW);
		digitalWrite(LCL3_4, HIGH);
		efuse_4_config_error ++;
	}
	else if((starta <= (uint8_t)14) && ((bits+starta) >= (uint8_t)14))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)15)) && (starta <= (uint8_t)15) && ((bits+starta) >= (uint8_t)15)) //LCL3
	{
		digitalWrite(LCL1_4, HIGH);
		digitalWrite(LCL2_4, HIGH);
		digitalWrite(LCL3_4, LOW);
		efuse_4_config_error ++;
	}
	else if((starta <= (uint8_t)15) && ((bits+starta) >= (uint8_t)15))
	{
	}
	else{}

	/*if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)8)) && (starta <= (uint8_t)8) && ((bits+starta) >= (uint8_t)8)) //buildin led
	{
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else if((starta <= (uint8_t)8) && ((bits+starta) >= (uint8_t)8))
	{
		digitalWrite(LED_BUILTIN, LOW);
	}
	else{}*/

	/*  adress:                                 ↓                ↓                   ↓
	if (((write[byte]<<(starta+(byte*8))) & (1<<0)) && starta <= 0 && bits+starta >= 0)
	{
		digitalWrite(LED_BUILTIN, HIGH);
	}
	  adress:          ↓                   ↓
	else if (starta <= 0 && bits+starta >= 0)
	{
		digitalWrite(LED_BUILTIN, LOW);
	}*/
}

void Exeption(uint8_t code){
	uint16_t crc16bit;
	uint8_t crc[2];
	dataToSend[0] = data[0];
	dataToSend[1] = data[1] | (uint8_t)0x80;
	dataToSend[2] = code;
	crc16bit = CRC16(dataToSend, 3);
	crc[0] = crc16bit;
	crc[1] = crc16bit >> 8;
	dataToSend[3] = crc[0];
	dataToSend[4] = crc[1];
	error = 1;
	send = 1;
}

void control_and_wait_state(void *arg)
{
	uint16_t crc16bit;
	uint8_t crc[2];
	uint16_t starta;
	uint16_t by;
	uint8_t write[10];
	static uint8_t done = 0;
	if (data[0] == (uint8_t)1) //slave adress
	{
		if (done == (uint8_t)0) //keep going until done normaly once
		{
			if (data[1] == (uint8_t)0x01) //read coil
			{
				starta = data[3];
				starta |= data[2]<<(uint8_t)8;
				bitcount = data[5];
				bitcount |= data[4]<<(uint8_t)8;
				bytecount = ceil((float)bitcount / 8);
				if ((bitcount < (uint16_t)1) || (bitcount > (uint16_t)65535))
				{
					Exeption(3);
					error = 1;
				}
				else if ((bitcount > (uint8_t)COILS) || ((starta+bitcount) > (uint8_t)COILS))
				{
					Exeption(2);
					error = 1;
				}
				else
				{
					dataToSend[0] = data[0];
					dataToSend[1] = data[1];
					dataToSend[2] = bytecount;
					uint16_t last_byte_bits = bitcount - (uint16_t)((float)bitcount / 8);
					for (uint16_t i = 0; i < bytecount; i++)
					{
						if (i == (bytecount - 1)){
							dataToSend[i+(uint8_t)3] = ReadCoils(starta + ((uint8_t)8*i), last_byte_bits);
						}
						else{
							dataToSend[i+(uint8_t)3] = ReadCoils(starta + ((uint8_t)8*i), 8);
						}
					}
					crc16bit = CRC16(dataToSend, (uint8_t)3 + bytecount);
					crc[0] = crc16bit;
					crc[1] = crc16bit >> (uint8_t)8;

					dataToSend[bytecount+(uint8_t)3] = crc[0];
					dataToSend[bytecount+(uint8_t)4] = crc[1];
				}
				send = 1;
				done = 1;
			}
			else if (data[1] == (uint8_t)0x02) //read input
			{
				starta = data[3];
				starta |= data[2]<<(uint8_t)8;
				bitcount = data[5];
				bitcount |= data[4]<<(uint8_t)8;
				bytecount = ceil((float)bitcount / 8);
				if ((bitcount < (uint8_t)1) || (bitcount > (uint16_t)65535))
				{
					Exeption(3);
					error = 1;
				}
				else if ((bitcount > (uint8_t)COILS) || ((starta+bitcount) > (uint8_t)COILS))
				{
					Exeption(2);
					error = 1;
				}
				else
				{
					dataToSend[0] = data[0];
					dataToSend[1] = data[1];
					dataToSend[2] = bytecount;
					uint16_t last_byte_bits = bitcount - (uint16_t)((float)bitcount / 8);
					for (uint16_t i = 0; i < bytecount; i++)
					{
						if (i == (bytecount - 1)){
							dataToSend[i+(uint8_t)3] = ReadInputs(starta + ((uint8_t)8*i), last_byte_bits);
						}
						else{
							dataToSend[i+(uint8_t)3] = ReadInputs(starta + ((uint8_t)8*i), 8);
						}
					}
					crc16bit = CRC16(dataToSend, (uint8_t)3+bytecount);
					crc[0] = crc16bit;
					crc[1] = crc16bit >> (uint8_t)8;

					dataToSend[bytecount+(uint8_t)3] = crc[0];
					dataToSend[bytecount+(uint8_t)4] = crc[1];
				}
				send = 1;
				done = 1;
			}
			/*else if (data[1] == (uint8_t)0x03) //read registers
			{
				starta = data[3];
				starta |= data[2]<<(uint8_t)8;
				bytecount = data[5];
				bytecount |= data[4]<<(uint8_t)8;
				if ((bitcount < (uint8_t)1) || (bitcount > (uint16_t)65535))
				{
					Exeption(3);
					error = 1;
				}
				else if ((bitcount > (uint8_t)COILS) || ((starta+bitcount) > (uint8_t)COILS))
				{
					Exeption(2);
					error = 1;
				}
				else
				{
					dataToSend[0] = data[0];
					dataToSend[1] = data[1];
					dataToSend[2] = bytecount;
					for (uint16_t i = 0; i < bytecount; i++)
					{
						dataToSend[i+(uint8_t)3] = ReadReg(starta, bitcount);
					}
					crc16bit = CRC16(dataToSend, (uint8_t)3+bytecount);
					crc[0] = crc16bit;
					crc[1] = crc16bit >> 8;

					dataToSend[bytecount+(uint8_t)3] = crc[0];
					dataToSend[bytecount+(uint8_t)4] = crc[1];
				}
				send = 1;
				done = 1;
			}*/
			else if (data[1] == (uint8_t)0x0F) //write multiple coils
			{
				starta = data[3];
				starta |= data[2]<<8;
				bitcount = data[5];
				bitcount |= data[4]<<8;
				bytecount = data[6];
				for (uint16_t i = 0; i < bytecount; i++)
				{
					write[i] = data[(uint8_t)7+i];
				}
				if ((bitcount < (uint8_t)1) || (bitcount > (uint16_t)65535))
				{
					Exeption(3);
					error = 1;
				}
				else if ((bitcount > (uint8_t)COILS) || ((starta+bitcount) > (uint8_t)COILS))
				{
					Exeption(2);
					error = 1;
				}
				else
				{
					dataToSend[0] = data[0];
					dataToSend[1] = data[1];
					dataToSend[2] = data[2];
					dataToSend[3] = data[3];
					dataToSend[4] = data[4];
					dataToSend[5] = data[5];
					for (uint16_t i = 0; i < bytecount; i++)
					{
						WriteCoils(starta, i, write, bitcount-(uint8_t)1);
					}
					if (efuse_1_config_error > 1 || efuse_2_config_error > 1 || efuse_3_config_error > 1 || efuse_4_config_error > 1){ //check for incorrect configuration 
						Exeption(3);
					}
					else{
						crc16bit = CRC16(dataToSend, 6);
						crc[0] = crc16bit;
						crc[1] = crc16bit >> (uint8_t)8;

						dataToSend[6] = crc[0];
						dataToSend[7] = crc[1];
					}
				}
				send = 1;
				done = 1;
				backup_state = 1;
			}
			else if (data[1] == (uint8_t)0x05) //write coil
			{
				starta = data[3];
				starta |= data[2]<<(uint8_t)8;
				write[0] = data[4] & (uint8_t)1 ;
				if ((starta < (uint8_t)0) || (starta > (uint16_t)65535))
				{
					Exeption(3);
					error = 1;
				}
				else if (starta > (uint8_t)COILS)
				{
					Exeption(2);
					error = 1;
				}
				else
				{
					dataToSend[0] = data[0];
					dataToSend[1] = data[1];
					dataToSend[2] = data[2];
					dataToSend[3] = data[3];
					dataToSend[4] = data[4];
					dataToSend[5] = data[5];
					WriteCoils(starta, 0, write, 0);
					crc16bit = CRC16(dataToSend, 6);
					crc[0] = crc16bit;
					crc[1] = crc16bit >> (uint8_t)8;

					dataToSend[6] = crc[0];
					dataToSend[7] = crc[1];
				}
				send = 1;
				done = 1;
				backup_state = 1;
			}
			else
			{
				Exeption(1);
				done = 1;
			}
		}
	}
	else
	{
		for (uint8_t i = 0; i < (uint8_t)6; ++i)
		{
			dataToSend[i] = 0;
		}
	}
	if (t35 == (uint8_t)1)
	{
		done = 0;
		stop_t35();
		t35 = 0;
		fsm_current_state = IDLE_STATE;
		call_handler[fsm_current_state](NULL);
	}
}

void emission_state(void *arg)
{
	static uint8_t done = 0;
	if (done == (uint8_t)0)
	{
		if (error == (uint8_t)1)
		{
			for (uint8_t i = 0; i < (uint8_t)5; ++i)
			{
				SerialUSB.write(dataToSend[i]);
				stop_t35();
				start_t35();
			}
		}
		else if ((data[1] == (uint8_t)0x01) || (data[1] == (uint8_t)0x02))
		{
			for (uint8_t i = 0; i < ((uint8_t)5+bytecount); ++i)
			{
				SerialUSB.write(dataToSend[i]); 
				stop_t35();
				start_t35();
			}
		}
		else if ((data[1] == (uint8_t)0x05) || (data[1] == (uint8_t)0x0F))
		{
			for (uint8_t i = 0; i < (uint8_t)8; ++i)
			{
				SerialUSB.write(dataToSend[i]);
				stop_t35();
				start_t35();
			}
		}
		else{}
		done = 1;
	}
	if (t35 == (uint8_t)1)
	{
		bytecount = 0;
		done = 0;
		error = 0;
		t35 = 0;
		stop_t35();
		fsm_current_state = IDLE_STATE;
	}
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
	return;
}

void UserInterface(void){
	//static uint8_t redraw = 1;
	static uint8_t position = 0;
	static uint8_t fuse = 0;
	uint8_t one = 1;
	uint8_t zero = 0;
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
		delay(20);
		if (!digitalRead(OK_BUTTON)){
			switch (position)
			{
			case 0:
				if (ReadCoils(fuse*4,1) > 0)
				{
					WriteCoils(fuse*4,0,&zero,0);
				}
				else{
					WriteCoils(fuse*4,0,&one,0);
				}
				break;
			default:
				WriteCoils(position+(fuse*4),0,&one,0);
				break;
			}
		}
	}
	else{}
	if (redraw == (uint8_t)1){
		uint8_t status = ReadCoils((fuse*4)+1,3);
		uint8_t onoff = ReadCoils(fuse*4,1);
		OledDisplay(position, fuse, status, onoff);
		redraw = 0;
		delay(1);
	}
	else{}
	while (!digitalRead(UP_BUTTON) || !digitalRead(OK_BUTTON) || !digitalRead(DOWN_BUTTON) || !digitalRead(LEFT_BUTTON) || !digitalRead(RIGHT_BUTTON))
	{
	}	
}

void Led_matrix(uint8_t row1, uint8_t row2, uint8_t row3, uint8_t row4){ //4 variable bitmap. example: all left leds on (0b001, 0b001, 0b001, 0b001)
	//write a 255 if row not changed
	static uint8_t x1bits = 0;
	static uint8_t x2bits = 0;
	static uint8_t x3bits = 0;
	static uint8_t x4bits = 0;

	if (row1 != 255){
		x1bits = row1;
	}
	if (row2 != 255){
		x2bits = row2;
	}
	if (row3 != 255){
		x3bits = row3;
	}
	if (row4 != 255){
		x4bits = row4;
	}

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

void T15_Handler(void)
{
	t15 = 1;
}

void T35_Handler(void)
{
	t35 = 1;
}

void Oled_Handler(void){
	redraw = 1;
}

void init_timer(void)
{
	Timer4.attachInterrupt(T15_Handler).setPeriod(1500);
	Timer5.attachInterrupt(T35_Handler).setPeriod(3500);
	Timer6.attachInterrupt(Oled_Handler).setPeriod(5000000).start();
}

void setup()
{

	// setup I/O
	pinMode(UP_BUTTON, INPUT_PULLUP);
	pinMode(OK_BUTTON, INPUT_PULLUP);
	pinMode(DOWN_BUTTON, INPUT_PULLUP);
	pinMode(LEFT_BUTTON, INPUT_PULLUP);
	pinMode(RIGHT_BUTTON, INPUT_PULLUP);

	pinMode(LED_X1, OUTPUT);
	pinMode(LED_X2, OUTPUT);
	pinMode(LED_X3, OUTPUT);
	pinMode(LED_X4, OUTPUT);
	pinMode(LED_Y1, OUTPUT);
	pinMode(LED_Y2, OUTPUT);
	pinMode(LED_Y3, OUTPUT);
	
	pinMode(RELAY1, OUTPUT);
	pinMode(LCL3_1, OUTPUT);
	pinMode(LCL2_1, OUTPUT);
	pinMode(LCL1_1, OUTPUT);
	pinMode(SHDN1, OUTPUT);
	pinMode(FLT1, INPUT);

	pinMode(RELAY2, OUTPUT);
	pinMode(LCL3_2, OUTPUT);
	pinMode(LCL2_2, OUTPUT);
	pinMode(LCL1_2, OUTPUT);
	pinMode(SHDN2, OUTPUT);
	pinMode(FLT2, INPUT);

	pinMode(RELAY3, OUTPUT);
	pinMode(LCL3_3, OUTPUT);
	pinMode(LCL2_3, OUTPUT);
	pinMode(LCL1_3, OUTPUT);
	pinMode(SHDN3, OUTPUT);
	pinMode(FLT3, INPUT);

	pinMode(RELAY4, OUTPUT);
	pinMode(LCL3_4, OUTPUT);
	pinMode(LCL2_4, OUTPUT);
	pinMode(LCL1_4, OUTPUT);
	pinMode(SHDN4, OUTPUT);
	pinMode(FLT4, INPUT);

	pinMode(LED_BUILTIN, OUTPUT);


	// i/o initial set
	digitalWrite(RELAY1, LOW);
	digitalWrite(SHDN1, LOW);
	digitalWrite(LCL3_1, HIGH);
	digitalWrite(LCL2_1, HIGH);
	digitalWrite(LCL1_1, LOW);

	digitalWrite(RELAY2, LOW);
	digitalWrite(SHDN2, LOW);
	digitalWrite(LCL3_2, HIGH);
	digitalWrite(LCL2_2, HIGH);
	digitalWrite(LCL1_2, LOW);

	digitalWrite(RELAY3, LOW);
	digitalWrite(SHDN3, LOW);
	digitalWrite(LCL3_3, HIGH);
	digitalWrite(LCL2_3, HIGH);
	digitalWrite(LCL1_3, LOW);

	digitalWrite(RELAY4, LOW);
	digitalWrite(SHDN4, LOW);
	digitalWrite(LCL3_4, HIGH);
	digitalWrite(LCL2_4, HIGH);
	digitalWrite(LCL1_4, LOW);

	digitalWrite(LED_X1, LOW);
	digitalWrite(LED_X2, LOW);
	digitalWrite(LED_X3, LOW);
	digitalWrite(LED_X4, LOW);
	digitalWrite(LED_Y1, HIGH);
	digitalWrite(LED_Y2, HIGH);
	digitalWrite(LED_Y3, HIGH);

	// setup Timers
	init_timer();

	// restore previous config
	/*uint8_t tmp = dueFlashStorage.read(0);
	WriteCoils(0,0,&tmp,8);
	tmp = dueFlashStorage.read(1);
	WriteCoils(8,0,&tmp,8);*/

	//setup serial
	SerialUSB.begin(9600,SERIAL_8N2);

	// setup oled display
	if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3D))
	{
		SerialUSB.println(F("SSD1306 allocation failed"));
		while (true){}
	}
	delay(2000);		 // wait for initializing
	oled.clearDisplay(); // clear display
	
	// initialize fsm
	fsm_current_state = IDLE_STATE;
	call_handler[fsm_current_state](NULL);
}

void loop()
{
	call_handler[fsm_current_state](NULL);
}
