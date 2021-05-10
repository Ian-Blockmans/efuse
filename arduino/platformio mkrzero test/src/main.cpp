#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FlashStorage.h>
#include <string.h>

#define SCREEN_WIDTH 128 // OLED display width,  in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define COILS 7
#define INPUTS 3

#define UP_BUTTON 5
#define OK_BUTTON 4
#define DOWN_BUTTON 3
#define SHDN 10
#define A42 A5
#define A28 A4
#define A14 7
#define OVP 6
#define UVP 2
#define PGOOD 1
#define FLT 0
#define RELAY A6



// declare an SSD1306 display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// storage object
FlashStorage(Restore_state, uint8_t);

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

uint8_t ReadCoils(uint16_t,uint16_t);
uint8_t ReadReg(uint16_t, uint16_t);
uint8_t ReadInputs(uint16_t, uint16_t);
void WriteCoils(uint16_t, uint16_t, uint8_t *, uint16_t);
void Exeption(uint8_t);

void UserInterface(void);
void OledDisplay(uint8_t);

// global variables
fsm_states_t fsm_current_state = INITIAL_STATE;
typedef void (*fsm_handler_t)(void *arg);
fsm_handler_t call_handler[6] = {initial_state, idle_state, emission_state, reception_state, control_and_wait_state};
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
uint8_t config_error = 0;

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
	REG_TC4_CTRLA |= TC_CTRLA_ENABLE; //enable timer4
	while ((REG_TC4_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	} //wait for sync
}

void stop_t15()
{
	REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE; //disable timer4
	while ((REG_TC4_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}						   //wait for sync
	REG_TC4_COUNT16_COUNT = 0; //clear counter
}

void start_t35()
{
	REG_TC5_CTRLA |= TC_CTRLA_ENABLE; //enable timer5
	while ((REG_TC5_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	} //wait for sync
}

void stop_t35()
{
	REG_TC5_CTRLA &= ~TC_CTRLA_ENABLE; //disable timer5
	while ((REG_TC5_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}						   //wait for sync
	REG_TC5_COUNT16_COUNT = 0; //clear counter
}

void start_timers()
{
	REG_TC4_CTRLA |= TC_CTRLA_ENABLE; //enable timer4
	while ((REG_TC4_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}								  //wait for sync
	REG_TC5_CTRLA |= TC_CTRLA_ENABLE; //enable timer5
	while ((REG_TC5_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	} //wait for sync
}

void stop_timers()
{
	REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE; //disable timer4
	while ((REG_TC4_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}								   //wait for sync
	REG_TC4_COUNT16_COUNT = 0;		   //clear counter
	REG_TC5_CTRLA &= ~TC_CTRLA_ENABLE; //disable timer5
	while ((REG_TC5_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}						   //wait for sync
	REG_TC5_COUNT16_COUNT = 0; //clear counter
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
	else if (Serial.available() > 0)
	{
		data[bytesRecieved] = Serial.read();
		bytesRecieved++;
		stop_t35();
		start_t35();
	}
	else{}	
}

void idle_state(void *arg)
{
	if (Serial.available() > 0)
	{
		data[bytesRecieved] = Serial.read();
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
			if (ReadCoils(0,8) != Restore_state.read()){
				//Restore_state.write(ReadCoils(0,8));
			}
		}
		if(config_error == 1){
			digitalWrite(RELAY, LOW);
			//turn on error led
		}
		else
		{
			//turn of error led
		}

		if (digitalRead(FLT) == LOW)
		{
			digitalWrite(RELAY, LOW);
			digitalWrite(SHDN, LOW);
		}
		else
		{
			//digitalWrite(RELAY, HIGH);
		}
		UserInterface();
	}
}

void reception_state(void *arg)
{
	if (Serial.available() > 0)
	{
		data[bytesRecieved] = Serial.read();
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
		if (i == (uint16_t)0)
		{
			retdata |= ((digitalRead(SHDN) & digitalRead(RELAY)) <<(i-starta));
		}
		if (i == (uint16_t)1)
		{
			retdata |= ((!digitalRead(A14))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)2)
		{
			retdata |= ((!digitalRead(A28))&HIGH)<<(i-starta);
		}
		if (i == (uint16_t)3)
		{
			if (digitalRead(UVP) && ((!digitalRead(OVP))&HIGH) && (!digitalRead(A42))&HIGH){
				retdata |= (uint16_t)1<<(i-starta);
			}
		}
		if (i == (uint16_t)4)
		{
			if (digitalRead(OVP) && digitalRead(UVP)){
				retdata |= (uint16_t)1<<(i-starta);
			}
		}
		if (i == (uint16_t)5)
		{
			if (((!digitalRead(UVP))&HIGH) && digitalRead(OVP)){
				retdata |= (uint16_t)1<<(i-starta);
			}
		}
		if (i == (uint16_t)6)
		{
			retdata |= digitalRead(LED_BUILTIN)<<(i-starta);
		}
		//... to add coils for read
	}
	return retdata;
}

uint8_t ReadReg(uint16_t starta, uint16_t byte){
	uint8_t reg[50];//make global
	return reg[starta+byte];
}

uint8_t ReadInputs(uint16_t starta, uint16_t bits)
{
	uint8_t retdata = 0;
	for (uint16_t i = starta; i < (bits+starta); i++)
	{
		int req = i;
		if (i == (uint16_t)0)
		{
			retdata |= digitalRead(PGOOD)<<(i-starta); //0 == good 
		}
		if (i == (uint16_t)1)
		{
			retdata |= digitalRead(FLT)<<(i-starta); //0 == fault
		}
		if (i == (uint16_t)2)
		{
			retdata |= config_error<<(i-starta); //0 == fault
		}
		//... to add coils for read
	}
	return retdata;
}

void WriteCoils(uint16_t starta, uint16_t byte, uint8_t* write,uint16_t bits)
{
	config_error = 0;
	//adress:                                                              ↓                          ↓                                ↓
	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)0)) && (starta <= (uint8_t)0) && ((bits+starta) >= (uint8_t)0)) //ON/OFF
	{
		digitalWrite(SHDN, HIGH);
		digitalWrite(RELAY, HIGH);
	}
	//adress:                    ↓                                ↓
	else if ((starta <= (uint8_t)0) && ((bits+starta) >= (uint8_t)0))
	{
		digitalWrite(SHDN, LOW);
		digitalWrite(RELAY, LOW);
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)1)) && (starta <= (uint8_t)1) && ((bits+starta) >= (uint8_t)1)) //LCL1
	{
		digitalWrite(A14, LOW);
		digitalWrite(A28, HIGH);
		digitalWrite(A42, HIGH);
		digitalWrite(OVP, LOW);
		digitalWrite(UVP, HIGH);
	}
	else if((starta <= (uint8_t)1) && ((bits+starta) >= (uint8_t)1))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)2)) && (starta <= (uint8_t)2) && ((bits+starta) >= (uint8_t)2)) //LCL2
	{
		digitalWrite(A14, HIGH);
		digitalWrite(A28, LOW);
		digitalWrite(A42, HIGH);
		digitalWrite(OVP, LOW);
		digitalWrite(UVP, HIGH);
	}
	else if((starta <= (uint8_t)2) && ((bits+starta) >= (uint8_t)2))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)3)) && (starta <= (uint8_t)3) && ((bits+starta) >= (uint8_t)3)) //LCL3
	{
		digitalWrite(A14, HIGH);
		digitalWrite(A28, HIGH);
		digitalWrite(A42, LOW);
		digitalWrite(OVP, LOW);
		digitalWrite(UVP, HIGH);
	}
	else if((starta <= (uint8_t)3) && ((bits+starta) >= (uint8_t)3))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)4)) && (starta <= (uint8_t)4) && ((bits+starta) >= (uint8_t)4)) //12V
	{
		digitalWrite(A14, HIGH);
		digitalWrite(A28, HIGH);
		digitalWrite(A42, LOW);
		digitalWrite(OVP, HIGH);
		digitalWrite(UVP, HIGH);
	}
	else if((starta <= (uint8_t)4) && ((bits+starta) >= (uint8_t)4))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)5)) && (starta <= (uint8_t)5) && ((bits+starta) >= (uint8_t)5)) //5V
	{
		digitalWrite(A14, HIGH);
		digitalWrite(A28, HIGH);
		digitalWrite(A42, LOW);
		digitalWrite(OVP, HIGH);
		digitalWrite(UVP, LOW);
	}
	else if((starta <= (uint8_t)5) && ((bits+starta) >= (uint8_t)5))
	{
	}
	else{}

	if (((write[byte]<<(starta+(byte*(uint8_t)8))) & ((uint8_t)1<<(uint8_t)6)) && (starta <= (uint8_t)6) && ((bits+starta) >= (uint8_t)6)) //buildin led
	{
		digitalWrite(LED_BUILTIN, HIGH);
	}
	else if((starta <= (uint8_t)6) && ((bits+starta) >= (uint8_t)6))
	{
		digitalWrite(LED_BUILTIN, LOW);
	}
	else{}

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
				bytecount = ((float)bitcount / 8) + 1;
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
					for (uint16_t i = 0; i < bytecount; i++)
					{
						dataToSend[i+(uint8_t)3] = ReadCoils(starta, bitcount);
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
				bytecount = ((float)bitcount / 8) + 1;
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
						dataToSend[i+(uint8_t)3] = ReadInputs(starta, bitcount);
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
			else if (data[1] == (uint8_t)0x03) //read registers
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
			}
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
				else if (((starta > 0)) && (starta <= 4) && ((((write[0] & (1+(2^(6-starta)))) & ((write[0] & (1+(2^(6-starta))))-1))) == 0) ){
					config_error = 1;
					Exeption(3);
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
			else if (data[1] == (uint8_t)0x05) //write coil
			{
				uint8_t write[10];
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
				Serial.write(dataToSend[i]);
				stop_t35();
				start_t35();
			}
		}
		else if ((data[1] == (uint8_t)0x01) || (data[1] == (uint8_t)0x02))
		{
			for (uint8_t i = 0; i < ((uint8_t)5+bytecount); ++i)
			{
				Serial.write(dataToSend[i]); 
				stop_t35();
				start_t35();
			}
		}
		else if ((data[1] == (uint8_t)0x05) || (data[1] == (uint8_t)0x0F))
		{
			for (uint8_t i = 0; i < (uint8_t)8; ++i)
			{
				Serial.write(dataToSend[i]);
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

void OledDisplay(uint8_t pos){
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
	return;
}

void UserInterface(void){
	static uint8_t redraw = 1;
	static uint8_t position = 0;
	uint8_t one = 1;
	uint8_t zero = 0;
	//enum menu {lcl1,lcl2,lcl3,V5,V12};

	if (!digitalRead(UP_BUTTON)){
		delay(20);
		if (!digitalRead(UP_BUTTON)){
			if(position == (uint8_t)4){
				position = 0;
			}
			else
			{
				position++;
			}
			redraw = 1;
		}
	}
	else if (!digitalRead(DOWN_BUTTON)){
		delay(20);
		if (!digitalRead(DOWN_BUTTON)){
			if(position == (uint8_t)0){
				position = 4;
			}
			else
			{
				position--;
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
				WriteCoils(1,0,&one,0);
				break;
			case 1:
				WriteCoils(2,0,&one,0);
				break;
			case 2:
				WriteCoils(3,0,&one,0);
				break;
			case 3:
				WriteCoils(4,0,&one,0);
				break;
			case 4:
				WriteCoils(5,0,&one,0);
				break;
			case 5:
				WriteCoils(0,0,&one,0);
				break;
			default:
				break;
			}
		}
	}
	else{}

	if (redraw == (uint8_t)1){
		OledDisplay(position);
		redraw = 0;
		delay(50);
		while (!digitalRead(UP_BUTTON) || !digitalRead(OK_BUTTON) || !digitalRead(DOWN_BUTTON))
		{
		}
	}
}

void TC4_Handler(void)
{
	t15 = 1;
	REG_TC4_INTFLAG = TC_INTFLAG_MC0;
}

void TC5_Handler(void)
{
	t35 = 1;
	REG_TC5_INTFLAG = TC_INTFLAG_MC0;
}

void init_timer(void)
{
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5; //enable generic clock for timer 4 and 5

	REG_PM_APBCMASK |= PM_APBCMASK_TC4;		  //enable clock for timer4
	REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT16;	  // set timer 16-bit mode
	REG_TC4_CTRLA |= TC_CTRLA_WAVEGEN_MFRQ;	  // set cc0 as top
	REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV4; // prescaler clock/4
	REG_TC4_COUNT16_CC0 |= 18000;			  //timer top 1.5ms/t15
	while ((REG_TC4_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}
	// Configure interrupt request
	NVIC_DisableIRQ(TC4_IRQn);
	NVIC_ClearPendingIRQ(TC4_IRQn);
	NVIC_SetPriority(TC4_IRQn, 0);
	NVIC_EnableIRQ(TC4_IRQn);
	REG_TC4_INTENSET |= TC_INTENSET_MC0; //enable interrupt for overflow cc0

	REG_PM_APBCMASK |= PM_APBCMASK_TC5;		  //enable clock for timer5
	REG_TC5_CTRLA |= TC_CTRLA_MODE_COUNT16;	  // set timer 16-bit mode
	REG_TC5_CTRLA |= TC_CTRLA_WAVEGEN_MFRQ;	  // set cc0 as top
	REG_TC5_CTRLA |= TC_CTRLA_PRESCALER_DIV4; // prescaler clock/4
	REG_TC5_COUNT16_CC0 |= 42000;			  //timer top 3.5ms/t35
	while ((REG_TC5_STATUS & TC_STATUS_SYNCBUSY) != NULL)
	{
	}
	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	NVIC_EnableIRQ(TC5_IRQn);
	REG_TC5_INTENSET |= TC_INTENSET_MC0; //enable interrupt for overflow cc0
	Serial.write("w0");
}

void setup()
{
	// setup I/O
	pinMode(SHDN, OUTPUT);
	pinMode(A42, OUTPUT);
	pinMode(A28, OUTPUT);
	pinMode(A14, OUTPUT);
	pinMode(OVP, OUTPUT);
	pinMode(UVP, OUTPUT);
	pinMode(PGOOD, INPUT);
	pinMode(FLT, INPUT);
	pinMode(RELAY, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(UP_BUTTON, INPUT_PULLUP);
	pinMode(OK_BUTTON, INPUT_PULLUP);
	pinMode(DOWN_BUTTON, INPUT_PULLUP);

	// i/o initial set
	digitalWrite(RELAY, LOW);
	digitalWrite(SHDN, LOW);
	digitalWrite(A14, HIGH);
	digitalWrite(A28, HIGH);
	digitalWrite(A42, HIGH);
	digitalWrite(OVP, LOW);
	digitalWrite(UVP, HIGH);

	// setup Timers
	init_timer();

	// restore previous config
	uint8_t tmp = Restore_state.read();
	WriteCoils(0,0,&tmp,8);

	//setup serial
	Serial.begin(9600,SERIAL_8N2);

	// setup oled display
	if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		Serial.println(F("SSD1306 allocation failed"));
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
