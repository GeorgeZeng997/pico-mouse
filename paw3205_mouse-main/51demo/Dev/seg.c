#include "seg.h"

void Seg_Tran(uint8_t *Seg_tring, uint8_t *Seg_buf)
{
	uint8_t i,j;
	uint8_t temp;
	
	for(i=0,j=0; i<=7; i++,j++)
	{
		switch(Seg_tring[j])
		{
			case '0':	temp = 0xc0;	break;
			case '1':	temp = 0xf9;	break;
			case '2':	temp = 0xa4;	break;
			case '3':	temp = 0xb0;	break;
			case '4':	temp = 0x99;	break;
			case '5':	temp = 0x92;	break;
			case '6':	temp = 0x82;	break;
			case '7':	temp = 0xf8;	break;
			case '8':	temp = 0x80;	break;
			case '9':	temp = 0x90;	break;
			case 'A':	temp = 0x88;	break;
			case 'b':	temp = 0x83;	break;
			case 'C':	temp = 0xc6;	break;
			case 'd':	temp = 0xa1;	break;
			case 'E':	temp = 0x86;	break;
			case 'F':	temp = 0x8e;	break;
			case '-':	temp = 0xbf;	break;
			
			default :	temp = 0xff;	break;
		}
		
		if(Seg_tring[j+1]=='.')
		{
			temp &= 0x7f;
			j++;
		}
		
		Seg_buf[i] = temp;
	}
}	

void Seg_Disp(uint8_t *Seg_buf, uint8_t pos)
{
	//位选消影
	P0 = 0xff;
	P2 = P2&0x1f|0xc0;
	P2 &= 0x1f;
	//位选
	P0 = 1<<pos;
	P2 = P2&0x1f|0xc0;
	P2 &= 0x1f;
	//段选消影
	P0 = 0xff;
	P2 = P2&0x1f|0xe0;
	P2 &= 0x1f;
	//段选
	P0 = Seg_buf[pos];
	P2 = P2&0x1f|0xe0;
	P2 &= 0x1f;
}





















