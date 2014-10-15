#define SERIAL_DEBUG
#define SERIAL_DEBUG_RX_PIN 2
#define SERIAL_DEBUG_TX_PIN 3
#define SERIAL_DEBUG_SPEED 9600

#define SERIAL_GPS_NMEA
//#define SERIAL_GPS_UBLOX
#define SERIAL_GPS_SPEED 9600
#define SERIAL_GPS_LISTEN_TIME	3000	// milliseconds
//#define SERIAL_GPS_SHARED				// GPS module shared with flight controller => unable to sleep, change precision etc.

#define SERIAL_SMS_RX_PIN 7
#define SERIAL_SMS_TX_PIN 8
#define SERIAL_SMS_SPEED 9600

#define CAN_ALARM						// tracker has alarm beeper
#define CAN_CUTOFF						// tracker has servo-based cutoff switch
#define CAN_SLEEP						// GPS module can be sent to sleep mode, disable this setting if SERIAL_GPS_SHARED is defined 

#define SEND_CONFIRMATION				// send SMS confirmation for each successfully executed command
#define SEND_HELLO						// send SMS upon tracker successful boot-up
#define SEND_ARMED						// send SMS right after first fix aquired. Highly recommended!