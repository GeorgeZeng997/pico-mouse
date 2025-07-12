#ifndef	__PAW3205_H__
#define __PAW3205_H__

#include "config.h"


sbit SCLK = P0^0;
sbit SDIO = P0^1;
sbit MOTSWK = P3^2;


//Register Addresses from datasheet
#define PROD_ID1 0x00
#define PROD_ID2 0x01
#define MOTION_STATUS 0X02
#define DEL_X 0x03
#define DEL_Y 0x04
#define OP_MODE 0x05
#define CONF 0x06
#define IMG_QUALITY 0x07
#define OP_STATE 0x08
#define W_PROTECT 0x09
#define SLEEP1_SET 0x0A
#define ENTER_TIME 0x0B
#define SLEEP2_SET 0x0C
#define IMG_THRESHOLD 0x0D
#define IMG_RECOG 0x0E


#define noSleep 0xA0

#define HIGH    1
#define LOW     0

#define digitalWrite(pin, state)    pin = state
#define digitalRead(pin)            pin




void mouseInit(void);
uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address, uint8_t sdata);



#endif
