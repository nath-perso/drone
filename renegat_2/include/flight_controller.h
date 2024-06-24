#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <Arduino.h>

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define NUMBER_OF_COMMANDS 4

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
void controllerGetCommands(float *commands);
void controllerMMA(float *setpoints, int *commands);

void updatePIDConstants();
void displayPIDConstants();

#endif /* __FLIGHT_CONTROLLER_H__ */