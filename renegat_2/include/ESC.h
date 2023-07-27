#ifndef __ESC_H__
#define __ESC_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "Servo.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define ESC1            1
#define ESC2            2
#define ESC3            3
#define ESC4            4

#define MOTOR_NUMBER    4

#define ESC_MIN_CMD     1000
#define ESC_MAX_CMD     2000

#define ESC_CALIBRATION  false

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
bool escSetup();
void escRun(int esc_num, int speed);
void escRunAll(int speed);
void escRunAllCommands(int *commands);

#endif /* __ESC_H__ */