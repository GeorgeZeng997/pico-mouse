#include "key.h"

uint8_t Key_Read_BTN(void)
{
	uint8_t Key_Value;
	
	if(P33 == 0)	Key_Value = 4;
	else if(P32 == 0)	Key_Value = 5;
	else if(P31 == 0)	Key_Value = 6;
	else if(P30 == 0)	Key_Value = 7;
	else Key_Value = 0;
	
	return Key_Value;
}	

uint8_t Key_Read(void)
{
	uint8_t Key_Value;
	uint16_t Key_New;
	
	P44 = 0; P42 = 1; P35 = 1; P34 = 1;
	Key_New = P3&0x0f;
	P44 = 1; P42 = 0; P35 = 1; P34 = 1;
	Key_New = (Key_New<<4)|(P3&0x0f);
	P44 = 1; P42 = 1; P35 = 0; P34 = 1;
	Key_New = (Key_New<<4)|(P3&0x0f);
	P44 = 1; P42 = 1; P35 = 1; P34 = 0;
	Key_New = (Key_New<<4)|(P3&0x0f);
	
	switch(~Key_New)
	{
		case 0x8000:	Key_Value = 4;	break;
		case 0x4000:	Key_Value = 5;	break;
		case 0x2000:	Key_Value = 6;	break;
		case 0x1000:	Key_Value = 7;	break;
		
		case 0x0800:	Key_Value = 8;	break;
		case 0x0400:	Key_Value = 9;	break;
		case 0x0200:	Key_Value = 10;	break;
		case 0x0100:	Key_Value = 11;	break;
		
		case 0x0080:	Key_Value = 12;	break;
		case 0x0040:	Key_Value = 13;	break;
		case 0x0020:	Key_Value = 14;	break;
		case 0x0010:	Key_Value = 15;	break;
		
		case 0x0008:	Key_Value = 16;	break;
		case 0x0004:	Key_Value = 17;	break;
		case 0x0002:	Key_Value = 18;	break;
		case 0x0001:	Key_Value = 19;	break;
		
		default:			Key_Value = 0;  break;
	}
	
	return Key_Value;
}

























