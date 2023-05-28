/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "flight_controller.h"
#include "pins.h"
#include "Arduino.h"
#include "nRF24.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define THRUST_COMMAND_INDEX    10
#define ROLL_COMMAND_INDEX      0
#define PITCH_COMMAND_INDEX     1
#define YAW_COMMAND_INDEX       2

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
extern byte keyValues[RADIO_FRAME_SIZE];


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */


/**
 * @brief       Convert key values into commands
 * @param       commands : [thrust, roll, pitch, yaw] commands
 */
void key2command(byte *commands)
{
    commands[0] = keyValues[THRUST_COMMAND_INDEX];
    commands[1] = keyValues[ROLL_COMMAND_INDEX];
    commands[2] = keyValues[PITCH_COMMAND_INDEX];
    commands[3] = keyValues[YAW_COMMAND_INDEX];
}