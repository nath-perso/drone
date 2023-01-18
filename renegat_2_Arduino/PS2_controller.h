#ifndef __PS2_CONTROLLER_H__
#define __PS2_CONTROLLER_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <PS2X_lib.h>


/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
 /* Controller connection error codes */
#define PS2_CONNECT_NO_ERROR    0
#define PS2_CONNECT_NOT_FOUND   1
#define PS2_CONNECT_NO_COMMAND  2
#define PS2_CONNECT_NO_PRESSURE 3

/* Controller type codes */
#define PS2_TYPE_UNKNOWN    0
#define PS2_TYPE_DUALSHOCK  1
#define PS2_TYPE_GUITARHERO 2

/* Configuration */
#define PS2_PRESSURE  true
#define PS2_RUMBLE    false

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
uint8_t ps2ControllerSetup();
void handlePS2Controller();

#endif /* __PS2_CONTROLLER_H__ */