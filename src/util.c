#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "drv_usart.h"
#include "drv_timer.h"

int _write(int fd, char *str, int len)
{
#if 0
    int i=0;

    for(i=0 ; i<len ; i++)
        ITM_SendChar((*str++));
#else

	for(int i=0;i<len;i++)
	{
		usart1_transmit_byte(*str++);
	}

#endif
    return len;
}

char* sub_three(float input)
{
	static char buf[4];
	int num[3];

	if(input <0)
    {
        input = -input;
    }
	
	int sub = ((int)(input * 1000)) %1000;
	num[0] = sub /100;
	num[1] = (sub /10) %10;
	num[2] = sub % 10;
	buf[0] = num[0] + '0';
	buf[1] = num[1] + '0';
	buf[2] = num[2] + '0';
	buf[3] = '\0';

	return buf;
}

char* system_elapsedTime(void)
{
	static char bf[8];
	uint32_t time = get_time_ms_cnt();
	uint32_t sec = (time /1000) % 60;
	uint32_t min = time /1000 /60;
	
	bf[0] = '[';
	bf[1] = ((min / 10) %10) + '0';
	bf[2] = (min %10) + '0';
	bf[3] = ':';
	bf[4] = ((sec / 10) %10) + '0';
	bf[5] = (sec %10) + '0';
	bf[6] = ']';
	bf[7] = '\0';

	return bf;
}

uint32_t times10(uint32_t input)
{
	int times=1;

	for(int j=0;j<input;j++)
	{
		times = 10*times;
	}

	return times;
}

int lock_cnt = 0;

static const uint8_t NUMSTR[2][16] ={
{'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'},
{'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'}};
#define UPPER 1
#define LOWER 0

void btoah(const uint8_t *src, uint8_t *dest, uint16_t srclen)
{
	int i,j;
	uint8_t lo,hi;
	
	for(i=0;i<srclen;i++)
	{
		lo = (uint8_t)(src[i] & 0x0f);
		hi = (uint8_t)(src[i] >>4);
		j = i<<1;
		
		dest[j] = NUMSTR[UPPER][hi];
		dest[j+1] = NUMSTR[UPPER][lo];
	}
}

int ahtob(const uint8_t *str)
{
	int16_t	ch;
	int16_t	i;
	int16_t	result=0;

	for(i=0;i<2;i++) {
		ch=str[i];

		result<<=4;
		if(ch>='0'&&ch<='9')
			result+=ch-'0';
		else
		if(ch>='a'&&ch<='f')
			result+=ch-'a'+10;
		else
		if(ch>='A'&&ch<='F')
			result+=ch-'A'+10;
		else
			return -1;
	}

	return result;
}

uint32_t convertTo_uint32(uint8_t *input)
{
    uint32_t rslt = 0;

    for(int i=0;i<4;i++)
    {
        rslt += (uint32_t)(input[i] <<((4-i -1)*8));
    }

    return rslt;
}

uint8_t atoh (uint8_t data)
 {
 	if (data > '9') 
    	{ 
    		data += 9;
    	}
	
    	return (data &= 0x0F);
 }
