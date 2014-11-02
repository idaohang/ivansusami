#include <EEPROM.h>
#include <Arduino.h>

char *ftoa(char *a, double f, int precision);
long labs(long l);
int free_ram();
byte crc8(const byte *data, byte len);

template <class T> int EEPROM_write(int ee, const T &value)
{
	const byte *p = (const byte *)(const void *)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		EEPROM.write(ee++, *p++);
	return i;
}

template <class T> int EEPROM_read(int ee, T &value)
{
	byte *p = (byte *)(void *)&value;
	unsigned int i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = EEPROM.read(ee++);
	return i;
}