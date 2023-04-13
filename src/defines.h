#ifndef DEFINES
#define DEFINES

/* STATES Dimming */
#define IDLE 0
#define S_UP 1
#define S_DOWN 2
#define UP 3
#define DOWN 4

/* STATES Clap */
#define OFF 0
#define CLAP_ON 1
#define CLAP_ON2 2
#define ON 3
#define CLAP_OFF 4
#define CLAP_OFF2 5

/* STATES Emergency */
#define EM_ON 0
#define EM_OFF 1

/* SENSOR CONTROL PINS */
#define XSHUT1 9
#define XSHUT2 18

/* SENSORS ADDRESS */
#define Address_Right 0x31
#define Address_Left 0x33

/* OUTPUTS LED CONTROL*/
static volatile int Brightness; 
static volatile int LED_STATE;

#endif