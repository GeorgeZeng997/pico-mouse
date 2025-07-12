#include "paw3205.h"

void mouseInit(void) // function to initialize optical sensor.
{
    //EA = 0;
    
  digitalWrite(SCLK, HIGH);
  digitalWrite(SCLK, LOW);
  delay_us(1); // tRESYNC = 1us (mentioned in datasheet)
  digitalWrite(SCLK, HIGH);
  delay_ms(320); // tSIWTT = 320ms (mentioned in datasheet)
  digitalWrite(SCLK, LOW);
  writeRegister(OP_MODE, noSleep); // disable sleep mode
    
    //EA = 1;
}

uint8_t readRegister(uint8_t address) 
{
    
  //pinMode (SDIO, OUTPUT);
  uint8_t i;
  uint8_t res = 0;
    
    //EA = 0;
  
  for (i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  //pinMode (SDIO, INPUT);

  delay_us(3); // tHOLD = 3us (mentioned in datasheet)
  
  for (i=128; i >0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SCLK, HIGH);
    if( digitalRead (SDIO) == HIGH )
      res |= i;
  }

  delay_us(3); // tHOLD = 3us min. (mentioned in datasheet)
  
  //EA = 1;
 
  return res;
}

void writeRegister(uint8_t address, uint8_t sdata) 
{
  uint8_t i;   
    
    //EA = 0;
    
  address |= 0x80; // MSB indicates write mode.
  //pinMode (SDIO, OUTPUT); 
    
  for (i = 128; i > 0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  for (i = 128; i > 0 ; i >>= 1) {
    digitalWrite (SCLK, LOW);
    digitalWrite (SDIO, (sdata & i) != 0 ? HIGH : LOW);
    digitalWrite (SCLK, HIGH);
  }

  delay_us(100);
  
  //EA = 1;
}
















