#ifndef DEFINES
#define DEFINES

/* STATES Dimming */
#define IDLE 0
#define S_UP 1
#define S_DOWN 2
#define UP 3
#define DOWN 4

/* SENSOR CONTROL PINS */
#define XSHUT1 9
#define XSHUT2 18

/* SENSORS ADDRESS */
#define Address_Right 0x31
#define Address_Left 0x33

/* OUTPUTS LED CONTROL*/
static volatile int Brightness; 

#endif