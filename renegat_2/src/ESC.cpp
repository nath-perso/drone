/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "ESC.h"
#include "pins.h"
#include "Arduino.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
// #define ESC_CALIBRATION

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* ESC objects */
Servo esc1, esc2, esc3, esc4;



/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */


/**
 * @brief       Run specified ESC at defined "speed"
 * @param       esc_num : 1 to 4
 * @param       speed : between 0 and 1000
 */
void escRun(int esc_num, int speed) {
    /* Saturate speed */
    if (ESC_MIN_CMD + speed > ESC_MAX_CMD)
    {
        speed = ESC_MAX_CMD - ESC_MIN_CMD;
    }
    
   switch (esc_num)
   {
   case ESC1:
    esc1.writeMicroseconds(ESC_MIN_CMD + speed);
    break;
   case ESC2:
    esc2.writeMicroseconds(ESC_MIN_CMD + speed);
    break;
   case ESC3:
    esc3.writeMicroseconds(ESC_MIN_CMD + speed);
    break;
   case ESC4:
    esc4.writeMicroseconds(ESC_MIN_CMD + speed);
    break;
   
   default:
    break;
   }
}

/**
 * @brief       Run all ESCs at desired speed
 * @param       speed : between 0 and 1000
 */
void escRunAll(int speed) {
    /* Saturate speed */
    if (ESC_MIN_CMD + speed > ESC_MAX_CMD)
    {
        speed = ESC_MAX_CMD - ESC_MIN_CMD;
    }

    esc1.writeMicroseconds(ESC_MIN_CMD + speed);
    esc2.writeMicroseconds(ESC_MIN_CMD + speed);
    esc3.writeMicroseconds(ESC_MIN_CMD + speed);
    esc4.writeMicroseconds(ESC_MIN_CMD + speed);
}

/**
 * @brief       Run all ESCs at corresponding setpoint
 * @param       commands : motor speed commands, between 0 and max 
 */
void escRunAllCommands(int *commands) {

    esc1.writeMicroseconds(ESC_MIN_CMD + commands[0]);
    esc2.writeMicroseconds(ESC_MIN_CMD + commands[1]);
    esc3.writeMicroseconds(ESC_MIN_CMD + commands[2]);
    esc4.writeMicroseconds(ESC_MIN_CMD + commands[3]);
}

/**
 * @brief       Setup ESCs
 * @returns     True if successful
 */
bool escSetup() {
    esc1.attach(ESC_1_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc2.attach(ESC_2_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc3.attach(ESC_3_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc4.attach(ESC_4_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    
    escRunAll(0);
    return true;
}