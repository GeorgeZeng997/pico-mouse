#ifndef	__I2C_H__
#define __I2C_H__

#include "config.h"

#define DELAY_TIME	5
#define PHOTO_CHANNEL	0x41
#define REG_CHANNEL		0x43


uint8_t PCF8591_Adc(uint8_t channel_control);
void PCF8591_Dac(uint8_t tans_data);

void EEPROM_Write(uint8_t *EEPROM_String, uint8_t addr, uint8_t num);
void EEPROM_Read(uint8_t *EEPROM_Buff, uint8_t addr, uint8_t num);






#endif
