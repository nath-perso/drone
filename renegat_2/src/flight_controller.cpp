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
#define THRUST_UP_COMMAND_INDEX    8
#define THRUST_DOWN_COMMAND_INDEX  8
#define ROLL_COMMAND_INDEX      0
#define PITCH_COMMAND_INDEX     2
#define YAW_COMMAND_INDEX       4

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
void controllerGetCommands(float *commands)
{
    if(keyValues[THRUST_UP_COMMAND_INDEX] & 0x0001) commands[0] = 1;
    else    commands[0] = 0;
    if(keyValues[THRUST_DOWN_COMMAND_INDEX] & 0x0002) commands[0]--;
    commands[1] = (keyValues[ROLL_COMMAND_INDEX]    - 128)/128.0;
    commands[2] = (keyValues[PITCH_COMMAND_INDEX]   - 128)/128.0;
    commands[3] = (keyValues[YAW_COMMAND_INDEX]     - 128)/128.0;
}