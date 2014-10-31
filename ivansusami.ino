/*
 * Ivan-s-usami DIY GPS tracker by Andrew Evdokimov
 *
 * Licensed under GPLv3
 */

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "gps.h"
#include "util.h"
#include <SoftwareSerial.h>
#include <sms.h>
#include <GSM.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/eeprom.h>

#define COMMAND_MAX_TOKENS 		6
#define PHONE_NUMBER_LENGTH		12
#define PIN_LENGTH				4
#define SMS_MAX_LENGTH			160		// should never exceed 160
#define SMS_QUEUE_LENGTH		10
#define SMS_MSG_LENGTH			20

#define SMS_NONE 0
#define SMS_LOCATION 1
#define SMS_EXEC_COMPLETE 2
#define SMS_EXEC_UNSUPPORTED 3
#define SMS_COMMAND_UNKNOWN 4
#define SMS_INVALID_PIN 5
#define SMS_HELLO_WORLD 6
#define SMS_ARMED 7
#define SMS_ERROR_QUEUE_FULL 8

char owner_phone_number[PHONE_NUMBER_LENGTH];
char caller_phone_number[PHONE_NUMBER_LENGTH];
char master_pin[PIN_LENGTH];
char *COMMAND_DELIMITER = "-";			// 1-char string

#ifdef SERIAL_DEBUG
SoftwareSerial debug_port(SERIAL_DEBUG_RX_PIN, SERIAL_DEBUG_TX_PIN);
#endif

SMSGSM sms;

typedef struct
{
	uint8_t sms_type;
	uint8_t command;
	char recepient[PHONE_NUMBER_LENGTH];
	char msg[SMS_MSG_LENGTH];
} sms_queue_entry_t;

sms_queue_entry_t sms_queue[SMS_QUEUE_LENGTH];
sms_queue_entry_t sms_error;

uint8_t sms_queue_counter = 0;

char *command_list[] = 			{"?", "alarm", "awake", "checkowner", "checkpin", "cutoff", "noalarm", "nocheckowner", "nocheckpin", "rate", "sleep", "w", "burst", "noburst"};
uint8_t command_param_count[] = {0,   0,       0,       0,            0,          0,        0,         0,              0,            1,      0,       0,   0,       0};

uint16_t location_rate_interval = 0;	// interval for sending location SMS, in minutes. 0 == do not send location SMS periodically

uint8_t reg;							// configuration,status and state register

#define CONFIG_CHECK_PIN	B00000001
#define CONFIG_CHECK_OWNER	B00000010
#define STATUS_BOOTED		B00000100
#define STATUS_ARMED		B00001000
#define STATE_ALARM_ON		B00010000
#define STATE_CUTOFF_ON		B00100000
#define STATE_SLEEPING		B01000000
#define STATE_BURST			B10000000
#define REG_DEFAULT			B00000000

#define CHECK_PIN 			((reg & CONFIG_CHECK_PIN) == CONFIG_CHECK_PIN)
#define CHECK_OWNER 		((reg & CONFIG_CHECK_OWNER) == CONFIG_CHECK_OWNER)

void get_gps_data()
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("get_gps_data() enter"));
#endif

	uint32_t t0 = millis();

	while (((millis() - t0) < SERIAL_GPS_LISTEN_TIME) || (Serial.available()))
	{
#if defined(SERIAL_GPS_NMEA)
		char c = Serial.read();
#elif defined(SERIAL_GPS_UBLOX)
		uint8_t c = Serial.read();
#endif
		if (c > 0)
		{
			if (gps_new_frame(c))
			{
				current_fix.dt = millis();
			}
		}
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("get_gps_data() exit"));
#endif
}

void process_gps_data()
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_gps_data() enter"));
#endif
	if (current_fix.fix == 3)	// we have 3D fix
	{
		if (reg & STATUS_ARMED != STATUS_ARMED)
		{
			reg |= STATUS_ARMED;
		}
		memcpy(&last_3d_fix, &current_fix, sizeof(current_fix));
	}
	else if (current_fix.fix = 2)
	{
		if (reg & STATUS_ARMED != STATUS_ARMED)
		{
			reg |= STATUS_ARMED;
		}
		memcpy(&last_2d_fix, &current_fix, sizeof(current_fix));
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_gps_data() exit"));
#endif
}

boolean enqueue_sms(uint8_t sms_type, uint8_t command, char *recepient, char *msg)
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("enqueue_sms() enter"));
	debug_port.print(F("sms_type="));
	debug_port.println(sms_type);
	debug_port.print(F("command="));
	debug_port.println(command);
	debug_port.print(F("recepient="));
	debug_port.println(recepient);
	debug_port.print(F("msg="));
	debug_port.println(msg);

#endif

	if (sms_queue_counter > (SMS_QUEUE_LENGTH - 1))
	{
		// unable to queue SMS, drop and set error condition
		sms_error.sms_type = SMS_ERROR_QUEUE_FULL;
		sms_error.command = 0;
		strncpy(sms_error.recepient, recepient, PHONE_NUMBER_LENGTH);
		strncpy(sms_error.msg, "SMS queue overflow!", 20);
		return false;
	}
	else
	{
		sms_queue[sms_queue_counter].sms_type = sms_type;
		sms_queue[sms_queue_counter].command = command;
		strncpy(sms_queue[sms_queue_counter].recepient, recepient, PHONE_NUMBER_LENGTH);
		strncpy(sms_queue[sms_queue_counter].msg, msg, SMS_MSG_LENGTH);
		sms_queue_counter++;
		return true;
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("enqueue_sms() exit"));
#endif
}

void enqueue_delayed_command(uint8_t command)
{

}

boolean valid_sms_sender(char *phone_number)
{
	if (CHECK_OWNER)
	{
		for (uint8_t i = 0; i < PHONE_NUMBER_LENGTH; i++)
		{
			if (owner_phone_number[i] != phone_number[i])
			{
				return false;
			}
		}
	}
	return true;
}

boolean valid_pin(char *pin)
{
	if (CHECK_PIN)
	{
		for (uint8_t i = 0; i < PIN_LENGTH; i++)
		{
			if (master_pin[i] != pin[i])
			{
				return false;
			}
		}
	}
	return true;
}

void execute_pcommand(uint8_t command, uint8_t *params, uint8_t param_count)
{
}

// execute command without parameters
void execute_command(uint8_t command)
{
	switch (command)
	{
	case 0:
	case 11:
		// get current or best GPS location
		enqueue_sms(SMS_LOCATION, NULL, caller_phone_number, NULL);
		break;
	case 1:
		// turn on alarm
#ifdef CAN_ALARM
		reg |= STATE_ALARM_ON;
#ifdef SEND_CONFIRMATION
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#endif
#else
		enqueue_sms(SMS_EXEC_UNSUPPORTED, command, caller_phone_number, NULL);
#endif
		break;
	case 6:
		// turn off alarm
#ifdef CAN_ALARM
		reg &= ~STATE_ALARM_ON;
#ifdef SEND_CONFIRMATION
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#endif
#else
		enqueue_sms(SMS_EXEC_UNSUPPORTED, command, caller_phone_number, NULL);
#endif
		break;
	case 10:
		// sleep
#ifdef CAN_SLEEP
		// go_sleep(); TBD
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#else
		enqueue_sms(SMS_EXEC_UNSUPPORTED, command, caller_phone_number, NULL);
#endif
		break;
	case 2:
		// awake after sleep
#ifdef CAN_SLEEP
		// go_awake(); TBD
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#else
		enqueue_sms(SMS_EXEC_UNSUPPORTED, command, caller_phone_number, NULL);
#endif
		break;
	case 3:
		// turn on owner check
		reg |= CONFIG_CHECK_OWNER;
		memcpy(&owner_phone_number, &caller_phone_number, sizeof(caller_phone_number));
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
		break;
	case 7:
		// turn off owner check
		reg &= ~CONFIG_CHECK_OWNER;
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
		break;
	case 4:
		// turn on PIN check
		reg |= CONFIG_CHECK_PIN;
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
		break;
	case 8:
		// turn off PIN check
		reg &= ~CONFIG_CHECK_PIN;
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
		break;
	case 5:
		// cut off power
#ifdef CAN_CUTOFF
		// will always send confirmation for this command
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#else
		enqueue_sms(SMS_EXEC_UNSUPPORTED, command, caller_phone_number, NULL);
#endif
		break;
	case 12:
		// send location every cycle
		reg |= STATE_BURST;
#ifdef SEND_CONFIRMATION
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#endif
		break;
	case 13:
		// cancel send location every cycle
		reg &= ~STATE_BURST;
#ifdef SEND_CONFIRMATION
		enqueue_sms(SMS_EXEC_COMPLETE, command, caller_phone_number, NULL);
#endif
		break;
	}
}

void feed_command_parser(char *token, boolean reset)
{
	static uint8_t prev_command = 255;
	static uint8_t params_remaining = 0;
	static uint8_t params[COMMAND_MAX_TOKENS - 1];
	static uint8_t params_counter = 0;
	static uint8_t command_list_count = sizeof(command_param_count) / sizeof(command_param_count[0]);
	uint8_t i = 0;
	uint8_t j = 0;

	if (reset)
	{
		prev_command = 255;
		params_remaining = 0;
		params_counter = 0;
		return;
	}

	if (params_remaining > 0)		// parsing parameters
	{
		params[params_counter] = atoi(token);
		params_counter++;
		params_remaining--;

		if (params_remaining == 0)	// all parameters OK
		{
			execute_pcommand(prev_command, params, params_counter);
			prev_command = 255;
			params_remaining = 0;
			params_counter = 0;
		}
	}
	else							// parsing new command
	{
		for (i = 0; i < command_list_count; i++)		// linear search
		{
			if (strlen(token) == strlen(command_list[i]))		// only if size matches
			{
				for (j = 0; j < strlen(token); j++)
				{
					if (token[j] != command_list[i][j])
					{
						break;
					}
				}
				if (j == strlen(token))
				{
					break;
				}
			}
		}

		if (i < command_list_count)		// command found
		{
			if (command_param_count[i] > 0)		// command takes parameters, need to parse next tokens
			{
				params_remaining = command_param_count[i];
				prev_command = i;
				return;
			}
			else
			{
				execute_command(i);
			}
		}
		else
		{
			enqueue_sms(SMS_COMMAND_UNKNOWN, 0, caller_phone_number, token);
		}
	}
}

void parse_sms_text(char *sms_text)
{
	char *token;
	char *p;
	uint8_t i = 0;

	if (sms_text[0] != COMMAND_DELIMITER[0])	// wrong message formatting, should always start with COMMAND_DELIMITER
	{
		return;
		// send diag SMS and return
	}
	for (token = strtok_r(sms_text, COMMAND_DELIMITER, &p); (token) && (i < COMMAND_MAX_TOKENS); token = strtok_r(NULL, COMMAND_DELIMITER, &p))
	{
		switch (i)
		{
		case 0:
		{
			if (valid_pin(token))
			{

			}
			else
			{
				enqueue_sms(SMS_INVALID_PIN, 0, caller_phone_number, token);
				i = i + COMMAND_MAX_TOKENS;		// break out of parsing loop
			}
			if (CHECK_PIN)	// if checking for pin, break out of switch for next token, otherwise continue (valid_pin() will return true if !CHECK_PIN)
			{
				break;
			}
		}
		default:
		{
			feed_command_parser(token, false);
			break;
		}
		}
		i++;
	}
	feed_command_parser(NULL, true);	// reset parser
}

void process_sms_orders()
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_sms_orders() enter"));
#endif

	uint8_t	unread_sms_position;
	char	sms_text[SMS_MAX_LENGTH];

	unread_sms_position = sms.IsSMSPresent(SMS_UNREAD);
	while (unread_sms_position > 0)
	{

		sms.GetSMS(unread_sms_position, caller_phone_number, sms_text, SMS_MAX_LENGTH);
		sms.DeleteSMS(unread_sms_position);
		if (valid_sms_sender(caller_phone_number))
		{
			parse_sms_text(sms_text);
		}
		else
		{
			// just skip messages from unknown numbers
		}
		unread_sms_position = sms.IsSMSPresent(SMS_UNREAD);
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_sms_orders() exit"));
#endif
}

boolean send_sms(uint8_t i, char *sms_buf)
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("send_sms() enter"));
	debug_port.print(F("sms = "));
	debug_port.println(sms_buf);
	debug_port.print(F("recepient = "));
	debug_port.println(sms_queue[i].recepient);

#endif


	if (sms.SendSMS(sms_queue[i].recepient, sms_buf) == 1)	// SMS sent
	{
		sms_queue[i].sms_type = SMS_NONE;
#ifdef SERIAL_DEBUG
		debug_port.println(F("send_sms() exit OK"));
#endif
		return true;
	}
	else
	{
#ifdef SERIAL_DEBUG
		debug_port.println(F("send_sms() exit ERROR"));
#endif
		return false;
	}
}

void process_sms_outbound_queue()
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_sms_outbound_queue() enter"));
#endif

	typedef struct
	{
		char buf[5];
	} fbuf;

	fbuf ftoa_buf[2];	// buffer for float-to-string conversion
	char eebuf[115];	// buffer for reading PROGMEM EEPROM variables, size appropriately

	static const prog_char fix_3d_location_template[] PROGMEM = "3D fix lat:%ld.%ld lon:%ld.%ld alt:%ld speed:%d hdop:%s vdop:%s nsat:%d\nhttp://maps.google.com/?q=%ld.%ld,%ld.%ld";
	static const prog_char fix_2d_location_template[] PROGMEM = "2D fix! lat:%ld.%ld lon:%ld.%ld hdop:%s nsat:%d\nLast 3D fix %d sec ago: lat:%ld.%ld lon:%ld.%ld alt:%ld speed:%d";
	static const prog_char fix_no_location_template[] PROGMEM = "No fix! Last fix %d sec ago: lat:%ld.%ld lon:%ld.%ld alt:%ld speed:%d hdop:%s vdop:%s nsat:%d fix:%dD";
	static const prog_char exec_complete_template[] PROGMEM = "Command '%s' executed";
	static const prog_char exec_unsupported_template[] PROGMEM = "Unsupported command '%s'";
	static const prog_char command_unknown_template[] PROGMEM = "Unknown command '%s'";
	static const prog_char invalid_pin_template[] PROGMEM = "Invalid PIN %s";
	static const prog_char hello_world_template[] PROGMEM = "Ivan-s-usami v%s booted";
	static const prog_char system_armed_template[] PROGMEM = "Ivan-s-usami v%s armed";

	char sms_buf[SMS_MAX_LENGTH];

	gps_data_t last_fix;

	for (uint8_t i = 0; i < SMS_QUEUE_LENGTH; i++)
	{
#ifdef SERIAL_DEBUG
		debug_port.print(F("psoq i = "));
		debug_port.println(i);
		debug_port.print(F("sms_queue[i].sms_type = "));
		debug_port.println(sms_queue[i].sms_type);
#endif
		switch (sms_queue[i].sms_type)
		{
		case SMS_LOCATION:
#ifdef SERIAL_DEBUG
			debug_port.print(F("current_fix.fix = "));
			debug_port.println(current_fix.fix);
#endif
			switch (current_fix.fix)
			{
			case 0:		// no fix, send last known 2D or 3D fix
			case 1:

				if (last_3d_fix.dt > last_2d_fix.dt)
				{
					last_fix = last_3d_fix;
				}
				else if (last_3d_fix.dt < last_2d_fix.dt)
				{
					last_fix = last_2d_fix;
				}
				else	// never had fix
				{
					last_fix = current_fix;
				}

				strcpy_P(eebuf, fix_no_location_template);
				sprintf(sms_buf, eebuf,
						(millis() - last_fix.dt) / 1000,
						last_fix.lat / 10000000L, abs(last_fix.lat % 10000000L),
						last_fix.lon / 10000000L, abs(last_fix.lon % 10000000L),
						last_fix.alt, last_fix.speed,
						ftoa(ftoa_buf[0].buf, last_fix.hdop, 2),
						ftoa(ftoa_buf[1].buf, last_fix.vdop, 2),
						last_fix.numsat, last_fix.fix);

				break;
			case 2:		// 2D fix, send current and last 3D fix
				strcpy_P(eebuf, fix_2d_location_template);
				sprintf(sms_buf, eebuf,
						current_fix.lat / 10000000L, abs(current_fix.lat % 10000000L),
						current_fix.lon / 10000000L, abs(current_fix.lon % 10000000L),
						ftoa(ftoa_buf[0].buf, current_fix.hdop, 2),
						current_fix.numsat,
						(millis() - last_3d_fix.dt) / 1000,
						last_3d_fix.lat / 10000000L, abs(last_3d_fix.lat % 10000000L),
						last_3d_fix.lon / 10000000L, abs(last_3d_fix.lon % 10000000L),
						last_3d_fix.alt, last_3d_fix.speed);

				break;
			case 3:		// 3D fix, send current
				strcpy_P(eebuf, fix_3d_location_template);
				sprintf(sms_buf, eebuf,
						current_fix.lat / 10000000L, abs(current_fix.lat % 10000000L),
						current_fix.lon / 10000000L, abs(current_fix.lon % 10000000L),
						current_fix.alt, current_fix.speed,
						ftoa(ftoa_buf[0].buf, current_fix.hdop, 2),
						ftoa(ftoa_buf[1].buf, current_fix.vdop, 2),
						current_fix.numsat,
						current_fix.lat / 10000000L, abs(current_fix.lat % 10000000L),
						current_fix.lon / 10000000L, abs(current_fix.lon % 10000000L));

				break;
			}
			break;

		case SMS_EXEC_COMPLETE:
			strcpy_P(eebuf, exec_complete_template);
			sprintf(sms_buf, eebuf, command_list[sms_queue[i].command]);
			break;
		case SMS_EXEC_UNSUPPORTED:
			strcpy_P(eebuf, exec_unsupported_template);
			sprintf(sms_buf, eebuf, command_list[sms_queue[i].command]);
			break;
		case SMS_COMMAND_UNKNOWN:
			strcpy_P(eebuf, command_unknown_template);
			sprintf(sms_buf, eebuf, sms_queue[i].msg);
			break;
		case SMS_INVALID_PIN:
			strcpy_P(eebuf, invalid_pin_template);
			sprintf(sms_buf, eebuf, sms_queue[i].msg);
			break;
		case SMS_HELLO_WORLD:
			strcpy_P(eebuf, hello_world_template);
			sprintf(sms_buf, eebuf, SOFTWARE_VERSION);
			break;
		case SMS_ARMED:
			strcpy_P(eebuf, system_armed_template);
			sprintf(sms_buf, eebuf, SOFTWARE_VERSION);
			break;
		case SMS_NONE:
			break;
		}

		if (sms_queue[i].sms_type != SMS_NONE)
		{
#ifdef SERIAL_DEBUG
			debug_port.println(F("Sending REGULAR SMS"));
			debug_port.print(F("sms = "));
			debug_port.println(sms_buf);
#endif
			send_sms(i, sms_buf);
		}
	}


	sms_queue_counter = 0;
	// scan queue for remaining SMSs and push them to queue head
	for (uint8_t i = 0; i < SMS_QUEUE_LENGTH; i++)
	{
		if (sms_queue[i].sms_type != SMS_NONE)
		{
			enqueue_sms(sms_queue[i].sms_type, sms_queue[i].command, owner_phone_number, sms_queue[i].msg);
		}
	}

	// send error SMS
	if (sms_error.sms_type != SMS_NONE)
	{
		sprintf(sms_buf, sms_error.msg);
		if (CHECK_OWNER)	// send error messages to owner, if configured, otherwise ignore
		{
			sms.SendSMS(owner_phone_number, sms_buf);
		}
		sms_error.sms_type = SMS_NONE;
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("process_sms_outbound_queue() exit"));
#endif
}

void read_config()
{
	if (EEPROM.read(4) == 255)	// notinitialized, PIN string should not contain 0xFF
	{
	}
	else
	{
		eeprom_read_block((void *)&reg, (void *)0, sizeof(reg));
		eeprom_read_block((void *)&location_rate_interval, (void *)1, sizeof(location_rate_interval));
		eeprom_read_block((void *)&master_pin, (void *)3, sizeof(master_pin));
		eeprom_read_block((void *)&owner_phone_number, (void *)(4 + PIN_LENGTH - 1), sizeof(owner_phone_number));
	}
	/*
	EEPROM_read(0, reg);
	EEPROM_read(1, location_rate_interval);
	EEPROM_read(3, master_pin);
	EEPROM_read(4 + PIN_LENGTH - 1, owner_phone_number);
	*/
#ifdef SERIAL_DEBUG
	debug_port.print(F("reg="));
	debug_port.println(reg);
	debug_port.print(F("location_rate_interval="));
	debug_port.println(location_rate_interval);
	debug_port.print(F("master_pin="));
	debug_port.println(master_pin);
	debug_port.print(F("owner_phone_number="));
	debug_port.println(owner_phone_number);
	debug_port.println(F("read_config() exit"));
#endif
	// need to add CRC check, util.h has crc8()
}

void write_config(boolean force)
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("write_config() enter"));
#endif

	static uint8_t prev_reg = 0;	// previous register setting

	if (force || (prev_reg != reg))
	{
#ifdef SERIAL_DEBUG
		debug_port.print(F("reg="));
		debug_port.println(reg);
		debug_port.print(F("location_rate_interval="));
		debug_port.println(location_rate_interval);
		debug_port.print(F("master_pin="));
		debug_port.println(master_pin);
		debug_port.print(F("owner_phone_number="));
		debug_port.println(owner_phone_number);
#endif
		eeprom_write_block((const void *)&reg, (void *)0, sizeof(reg));
		eeprom_write_block((const void *)&location_rate_interval, (void *)1, sizeof(location_rate_interval));
		eeprom_write_block((const void *)&master_pin, (void *)3, sizeof(master_pin));
		eeprom_write_block((const void *)&owner_phone_number, (void *)(4 + PIN_LENGTH - 1), sizeof(owner_phone_number));
		/*
		EEPROM_write(0, reg);
		EEPROM_write(1, location_rate_interval);
		EEPROM_write(3, master_pin);
		EEPROM_write(4 + PIN_LENGTH - 1, owner_phone_number);
		*/
		prev_reg = reg;
		// need to add CRC, util.h has crc8()
	}
#ifdef SERIAL_DEBUG
	debug_port.println(F("write_config() exit"));
#endif
}

void setup()
{
#ifdef SERIAL_DEBUG
	debug_port.begin(SERIAL_DEBUG_SPEED);
	debug_port.print(F("free_ram="));
	debug_port.println(free_ram());
#endif
	gsm.begin(SERIAL_SMS_SPEED);
	Serial.begin(SERIAL_GPS_SPEED);
#ifdef SERIAL_DEBUG
	debug_port.println(F("Ivan-s-usami DEBUG MODE"));
#endif
	current_fix.dt = 0;
	for (uint8_t i = 0; i < SMS_QUEUE_LENGTH; i++)
	{
		sms_queue[i].sms_type = SMS_NONE;
	}
	read_config();
	reg |= STATUS_BOOTED;
	reg &= ~STATUS_ARMED;
#ifdef SEND_HELLO
	if (CHECK_OWNER)	// send hello only when owner is set
	{
#ifdef SERIAL_DEBUG
		debug_port.println(F("Send HELLO SMS"));
#endif
		enqueue_sms(SMS_HELLO_WORLD, NULL, owner_phone_number, NULL);
	}
#endif
}

void loop()
{
#ifdef SERIAL_DEBUG
	debug_port.println(F("loop() enter"));
#endif
	get_gps_data();
	process_gps_data();

#ifdef SEND_ARMED		// send armed confirmation to owner
	static boolean armed_sent = false;
	if (!armed_sent)
	{
		if ((reg & STATUS_ARMED == STATUS_ARMED) && CHECK_OWNER)
		{
#ifdef SERIAL_DEBUG
			debug_port.print(F("Sending ARMED SMS to "));
			debug_port.println(owner_phone_number);
#endif
			enqueue_sms(SMS_ARMED, NULL, owner_phone_number, NULL);
			armed_sent = true;
		}
	}
#endif

	process_sms_orders();
	write_config(false);		// write to EEPROM if state of config has changed, will always write at first cycle
	if (reg & STATE_BURST == STATE_BURST)
	{
		enqueue_sms(SMS_LOCATION, NULL, owner_phone_number, NULL);
	}
	for (uint8_t i = 0; i < SMS_QUEUE_LENGTH; i++)
	{
		debug_port.print("sq[");
		debug_port.print(i);
		debug_port.print("]=");
		debug_port.println(sms_queue[i].sms_type);
	}
	process_sms_outbound_queue();
#ifdef SERIAL_DEBUG
	debug_port.println(F("loop() exit"));
#endif
}