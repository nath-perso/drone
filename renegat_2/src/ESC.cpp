/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "ESC.h"
#include "pins.h"
#include "Arduino.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */

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
    if (speed > ESC_MAX_SPEED)
    {
        speed = ESC_MAX_SPEED;
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
    if (speed > ESC_MAX_SPEED)
    {
        speed = ESC_MAX_SPEED;
    }

    esc1.writeMicroseconds(ESC_MIN_CMD + speed);
    esc2.writeMicroseconds(ESC_MIN_CMD + speed);
    esc3.writeMicroseconds(ESC_MIN_CMD + speed);
    esc4.writeMicroseconds(ESC_MIN_CMD + speed);
}

/**
 * @brief       Run all ESCs at max throttle (for calibration)
 */
void escRunAllMaxThrottle() {

    esc1.writeMicroseconds(ESC_MAX_CMD);
    esc2.writeMicroseconds(ESC_MAX_CMD);
    esc3.writeMicroseconds(ESC_MAX_CMD);
    esc4.writeMicroseconds(ESC_MAX_CMD);
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
 * @note        ESCs will beep power up on power up, but also at the first time after attaching the port (but only if you wait 5s) 
 *              Then starts the arming sequence. 
 *              If the command stays at 0, the arming sequence ends with one beep.
 *              If calibration is needed, send max throttle command at the start of arming sequence for at least 3 seconds, 
 *              then send min throttle command for at least 3 seconds.
 */
bool escSetup() {
    Serial.println("Setting up ESCs ...");
    escRunAll(ESC_MIN_SPEED);
    esc1.attach(ESC_1_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc2.attach(ESC_2_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc3.attach(ESC_3_PIN, ESC_MIN_CMD, ESC_MAX_CMD);
    esc4.attach(ESC_4_PIN, ESC_MIN_CMD, ESC_MAX_CMD);

    /* Wait for no reason, but we have to wait 5s for the ESCs to be ready for arming sequence */
    escRunAll(ESC_MIN_SPEED);
    delay(5000);

    /* Arming sequence start */
    if (ESC_CALIBRATION) {
        Serial.println("ESC calibration...");
        Serial.println("Sending max throttle for 3s");
        escRunAllMaxThrottle();
        delay(6000);
        
        Serial.println("Sending min throttle for 3s");
        escRunAll(ESC_MIN_SPEED);
        delay(6000);

        Serial.println("Calibration complete.");
    }
    else {
        Serial.println("Arming ESCs...");
        escRunAll(ESC_MIN_SPEED);
        delay(2000);
    }

    /* End of arming sequence. You should have heared one low long beep followed by one high long beep. */
    Serial.println("ESCs armed !");
    return true;
}