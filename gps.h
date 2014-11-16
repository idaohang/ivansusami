#include <stdint.h>

typedef struct
{
	long lat;
	long lon;
	float hdop;
	float vdop;
	uint8_t speed;
	long alt;
	uint8_t numsat;
	uint8_t fix;		// 1 - no fix, 2 - 2D, 3 - 3D
	uint32_t dt;
} gps_data_t;

extern gps_data_t current_fix;
extern gps_data_t last_3d_fix;
extern gps_data_t last_2d_fix;

extern uint8_t		x_msg_id;

bool gps_new_frame(uint8_t data);
void gps_reset_parser(void);