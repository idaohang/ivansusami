#include "util.h"
#include "Arduino.h"

// converts float to *char
char *ftoa(char *a, double f, int precision)
{
	long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0') a++;
	*a++ = '.';
	long decimal = abs((long)((f - heiltal) * p[precision]));
	itoa(decimal, a, 10);
	return ret;
}

int free_ram()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

byte crc8(const uint8_t *data, uint8_t len)
{
	byte crc = 0x00;
	while (len--)
	{
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--)
		{
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum)
			{
				crc ^= 0x8C;
			}
			extract >>= 1;
		}
	}
	return crc;
}
