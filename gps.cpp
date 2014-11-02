#include "config.h"
#include "gps.h"
#include "Arduino.h"

gps_data_t current_fix;
gps_data_t last_3d_fix;
gps_data_t last_2d_fix;

int32_t wrap_18000(int32_t error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

int32_t wrap_36000(int32_t angle)
{
	if (angle > 36000)	angle -= 36000;
	if (angle < 0)		angle += 36000;
	return angle;
}

#if defined(SERIAL_GPS_NMEA)

/* The latitude or longitude is coded this way in NMEA frames
  dddmm.mmmm   coded as degrees + minutes + minute decimal
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000
  I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS_coord_to_degrees(char *s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
		;
	q = s;

	// convert degrees
	while ((p - q) > 2)
	{
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q)
	{
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.')
	{
		q = p + 1;
		for (int i = 0; i < 4; i++)
		{
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

/* This is am expandable implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA, GSA and RMC frames to decode on the serial bus
   Using the following data :
   GGA
     - time
     - latitude
     - longitude
     - GPS fix
     - GPS num sat (5 is enough to be +/- reliable)
     - GPS alt
   GSA
     - 3D fix (it could be left out since we have a 3D fix if we have more than 4 sats
   RMC
     - GPS speed over ground, it will be handy for wind compensation (future)

*/

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3

bool gps_new_frame(uint8_t data)
{

	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, gps_frame = NO_FRAME;

	switch (data)
	{
	case '$': param = 0; offset = 0; parity = 0;
		break;
	case ',':
	case '*':  string[offset] = 0;
		if (param == 0)   //frame identification
		{
			gps_frame = NO_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
		}

		switch (gps_frame)
		{
		//************* GPGGA FRAME parsing
		case GPGGA_FRAME:
			switch (param)
			{
			//case 1: i2c_dataset.time = (atof(string) * 1000);    //up to .000 s precision not needed really but the data is there anyway
			//	break;
			case 2: current_fix.lat = GPS_coord_to_degrees(string);
				break;
			case 3: if (string[0] == 'S') current_fix.lat = -current_fix.lat;
				break;
			case 4: current_fix.lon = GPS_coord_to_degrees(string);
				break;
			case 5: if (string[0] == 'W') current_fix.lon = -current_fix.lon;
				break;
			case 7: current_fix.numsat = atoi(string);
				break;
			case 9: current_fix.alt = atol(string);
				break;
			}
			break;
		//************* GPGSA FRAME parsing
		case GPGSA_FRAME:
			switch (param)
			{
			case 2: if (string[0] == '3')
				{
					current_fix.fix = 3;
				}
				else if (string[0] == '2')
				{
					current_fix.fix = 2;
				}
				else
				{
					current_fix.fix = 0;
				}
				break;
			case 16: current_fix.hdop = atof(string);
				break;
			case 17: current_fix.vdop = atof(string);
				break;
			}
			break;
		//************* GPGSA FRAME parsing
		case GPRMC_FRAME:
			switch (param)
			{
			case 7: current_fix.speed = (uint8_t)((atof(string) * 0.5144444) * 3.6);  //convert to km/h
				break;
			}
			break;
		}

		param++; offset = 0;
		if (data == '*') checksum_param = 1;
		else parity ^= data;
		break;
	case '\r':
	case '\n':
		if (checksum_param)   //parity checksum
		{
			uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
			if (checksum == parity) frameOK = 1;
		}
		checksum_param = 0;
		break;
	default:
		if (offset < 15) string[offset++] = data;
		if (!checksum_param) parity ^= data;

	}
	return frameOK && (gps_frame == GPGGA_FRAME);
}

#elif defined(SERIAL_GPS_UBLOX)

struct ubx_header
{
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
};

struct ubx_nav_posllh
{
	uint32_t	time;				// GPS msToW
	int32_t		longitude;
	int32_t		latitude;
	int32_t		altitude_ellipsoid;
	int32_t		altitude_msl;
	uint32_t	horizontal_accuracy;
	uint32_t	vertical_accuracy;
};

struct ubx_nav_dop
{
	uint32_t	time;				// GPS msToW
	uint16_t	gdop;
	uint16_t	pdop;
	uint16_t	tdop;
	uint16_t	vdop;
	uint16_t	hdop;
	uint16_t	ndop;
	uint16_t	edop;
};

struct ubx_nav_solution
{
	uint32_t	time;
	int32_t		time_nsec;
	int16_t		week;
	uint8_t		fix_type;
	uint8_t		fix_status;
	int32_t		ecef_x;
	int32_t		ecef_y;
	int32_t		ecef_z;
	uint32_t	position_accuracy_3d;
	int32_t		ecef_x_velocity;
	int32_t		ecef_y_velocity;
	int32_t		ecef_z_velocity;
	uint32_t	speed_accuracy;
	uint16_t	position_DOP;
	uint8_t		res;
	uint8_t		satellites;
	uint32_t	res2;
};
struct ubx_nav_velned
{
	uint32_t	time;				// GPS msToW
	int32_t		ned_north;
	int32_t		ned_east;
	int32_t		ned_down;
	uint32_t	speed_3d;
	uint32_t	speed_2d;
	int32_t		heading_2d;
	uint32_t	speed_accuracy;
	uint32_t	heading_accuracy;
};

enum ubs_protocol_bytes
{
	PREAMBLE1 = 0xb5,
	PREAMBLE2 = 0x62,
	CLASS_NAV = 0x01,
	CLASS_ACK = 0x05,
	CLASS_CFG = 0x06,
	MSG_ACK_NACK = 0x00,
	MSG_ACK_ACK = 0x01,
	MSG_POSLLH = 0x2,
	MSG_STATUS = 0x3,
	MSG_DOP = 0x4,
	MSG_SOL = 0x6,
	MSG_VELNED = 0x12,
	MSG_CFG_PRT = 0x00,
	MSG_CFG_RATE = 0x08,
	MSG_CFG_SET_RATE = 0x01,
	MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type
{
	FIX_NONE = 0,
	FIX_DEAD_RECKONING = 1,
	FIX_2D = 2,
	FIX_3D = 3,
	FIX_GPS_DEAD_RECKONING = 4,
	FIX_TIME = 5
};
enum ubx_nav_status_bits
{
	NAV_STATUS_FIX_VALID = 1
};

// Packet checksum accumulators
static uint8_t		_ck_a;
static uint8_t		_ck_b;

// State machine state
static uint8_t		_step;
static uint8_t		_msg_id;
static uint16_t	_payload_length;
static uint16_t	_payload_counter;

static bool        next_fix;

static uint8_t     _class;

// do we have new position information?
static bool		_new_position;

// do we have new speed information?
static bool		_new_speed;

static uint8_t	    _disable_counter;

// Receive buffer
static union
{
	ubx_nav_posllh		posllh;
	ubx_nav_dop			dop;
	ubx_nav_solution	solution;
	ubx_nav_velned		velned;
	uint8_t	bytes[];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--)
	{
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

bool UBLOX_parse_gps(void)
{
	switch (_msg_id)
	{
	case MSG_POSLLH:
		current_fix.lon	                = _buffer.posllh.longitude;
		current_fix.lat	                = _buffer.posllh.latitude;
		current_fix.alt 	 	        = _buffer.posllh.altitude_msl / 10 / 100;     //alt in m
		_new_position = true;
		break;
	case MSG_DOP:
		current_fix.hdop                = ((float)_buffer.dop.hdop) / 100.0;
		current_fix.vdop	            = ((float)_buffer.dop.vdop) / 100.0;
		break;
	case MSG_SOL:
		current_fix.fix 				= _buffer.solution.fix_type;
		if (current_fix.fix > 3)
		{
			current_fix.fix = 3;
		}
		current_fix.numsat 				= _buffer.solution.satellites;
		break;
	case MSG_VELNED:
		current_fix.speed 				= (uint8_t)(_buffer.velned.speed_2d * 36 / 10000);
		_new_speed = true;
		break;
	default:
		return false;
	}

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	/*
	if (_new_position && _new_speed)
	{
		_new_speed = _new_position = false;
		return true;
	}
	return false;
	*/
	return true;
}


bool gps_new_frame(uint8_t data)
{
	bool parsed = false;

	switch (_step)
	{

	case 1:
		if (PREAMBLE2 == data)
		{
			_step++;
			break;
		}
		_step = 0;
	case 0:
		if (PREAMBLE1 == data) _step++;
		break;

	case 2:
		_step++;
		_class = data;
		_ck_b = _ck_a = data;			// reset the checksum accumulators
		break;
	case 3:
		_step++;
		_ck_b += (_ck_a += data);			// checksum byte
		_msg_id = data;
		break;
	case 4:
		_step++;
		_ck_b += (_ck_a += data);			// checksum byte
		_payload_length = data;				// payload length low byte
		break;
	case 5:
		_step++;
		_ck_b += (_ck_a += data);			// checksum byte
		_payload_length += (uint16_t)(data << 8);
		if (_payload_length > 512)
		{
			_payload_length = 0;
			_step = 0;
		}
		_payload_counter = 0;				// prepare to receive payload
		break;
	case 6:
		digitalWrite(13, HIGH);
		_ck_b += (_ck_a += data);			// checksum byte
		if (_payload_counter < sizeof(_buffer))
		{
			_buffer.bytes[_payload_counter] = data;
		}
		if (++_payload_counter == _payload_length)
			_step++;
		break;
	case 7:
		_step++;
		if (_ck_a != data) _step = 0;						// bad checksum
		break;
	case 8:
		_step = 0;
		if (_ck_b != data)  break; 							// bad checksum
		if (UBLOX_parse_gps())
		{
			parsed = true;
		}
	} //end switch
	return parsed;
}



#endif // SERIAL_GPS_UBLOX

