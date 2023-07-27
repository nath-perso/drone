/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "flight_controller.h"
#include "pins.h"
#include "Arduino.h"
#include "nRF24.h"
#include "ESC.h"
#include "mpu.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define THRUST_UP_COMMAND_INDEX    8
#define THRUST_DOWN_COMMAND_INDEX  8
#define PITCH_COMMAND_INDEX     0
#define ROLL_COMMAND_INDEX      2
#define YAW_COMMAND_INDEX       4

#define MAX_ANGLE   PI/2
#define MIN_ANGLE   -PI/2

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
extern byte keyValues[RADIO_FRAME_SIZE];


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

/**
 * @brief       Compute roll command
 * @param       raw_setpoint : roll raw setpoint from controller
 * @return      Roll command (between -1 and 1)
*/
float controllerGetRollCommand(float raw_setpoint){
    static float roll_last_error = 0;
    static float roll_last_integral = 0;
    static long roll_last_timestamp = 0;

    const float Kp = 0.4;
    const float Ki = 0;
    const float Kd = 0;

    float roll_value = mpuGetRoll();
    float roll_error = roll_value - raw_setpoint;

    float dt = (micros() - roll_last_timestamp)/1e6;
    roll_last_timestamp = micros();

    float roll_command = Kp*(roll_error) + Ki*(roll_last_integral + roll_error)*dt + Kd*(roll_error - roll_last_error);

    roll_command = constrain(roll_command, -1, 1);

    return (roll_command);
}

/**
 * @brief       Compute pitch command
 * @param       raw_setpoint : pitch raw setpoint from controller
 * @return      Pitch command (between -1 and 1)
*/
float controllerGetPitchCommand(float raw_setpoint){
    static float pitch_last_error = 0;
    static float pitch_last_integral = 0;
    static long pitch_last_timestamp = 0;

    const float Kp = 0.4;
    const float Ki = 0;
    const float Kd = 0;

    float pitch_value = mpuGetPitch();
    float pitch_error = pitch_value - raw_setpoint;

    float dt = (micros() - pitch_last_timestamp)/1e6;
    pitch_last_timestamp = micros();

    float pitch_command = Kp*(pitch_error) + Ki*(pitch_last_integral + pitch_error)*dt + Kd*(pitch_error - pitch_last_error);

    pitch_command = constrain(pitch_command, -1, 1);

    return (pitch_command);
}

/**
 * @brief       Compute yaw command
 * @param       raw_setpoint : yaw raw setpoint from controller
 * @return      Yaw command (between -1 and 1)
*/
float controllerGetYawCommand(float raw_setpoint){
    static float yaw_last_error = 0;
    static float yaw_last_integral = 0;
    static long yaw_last_timestamp = 0;

    const float Kp = 0.4;
    const float Ki = 0;
    const float Kd = 0;

    float yaw_value = mpuGetYaw();
    float yaw_error = yaw_value - raw_setpoint;

    float dt = (micros() - yaw_last_timestamp)/1e6;
    yaw_last_timestamp = micros();

    float yaw_command = Kp*(yaw_error) + Ki*(yaw_last_integral + yaw_error)*dt + Kd*(yaw_error - yaw_last_error);

    yaw_command = constrain(yaw_command, -1, 1);

    return (yaw_command);
}

/**
 * @brief       Convert key values into commands
 * @param       commands : [thrust, roll, pitch, yaw] commands
 */
void controllerGetCommands(float *commands)
{
    static float thrust_raw_setpoint = 0;
    float roll_raw_setpoint   = 0;
    float pitch_raw_setpoint  = 0;
    float yaw_raw_setpoint    = 0;

    if(keyValues[THRUST_UP_COMMAND_INDEX] & 0x0001) thrust_raw_setpoint += 0.001;
    if(keyValues[THRUST_DOWN_COMMAND_INDEX] & 0x0002) thrust_raw_setpoint -= 0.001;
    thrust_raw_setpoint = constrain(thrust_raw_setpoint, 0, 1);

    roll_raw_setpoint = (keyValues[ROLL_COMMAND_INDEX]    - 128)/128.0*MAX_ANGLE;
    pitch_raw_setpoint = (keyValues[PITCH_COMMAND_INDEX]   - 128)/128.0*MAX_ANGLE;
    yaw_raw_setpoint = (keyValues[YAW_COMMAND_INDEX]     - 128)/128.0*MAX_ANGLE;

    commands[0] = thrust_raw_setpoint;
    commands[1] = controllerGetRollCommand(roll_raw_setpoint);
    commands[2] = controllerGetPitchCommand(pitch_raw_setpoint);
    commands[3] = 0;    // controllerGetYawCommand(yaw_raw_setpoint);
}

/**
 * @brief       Motor Mixing Algorithm : convert flight commands (considered as setpoints) into motor commands
 * @param       setpoints : [thrust, roll, pitch, yaw] flight commands
 * @param       commands : motors commands table [Front right, front left, rear right, rear left]
*/
void controllerMMA(float *setpoints, int *commands) {
    commands[0] = (setpoints[0] + setpoints[1] + setpoints[2] + setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Front right */
    commands[1] = (setpoints[0] - setpoints[1] + setpoints[2] - setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Front left */
    commands[2] = (setpoints[0] + setpoints[1] - setpoints[2] - setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Rear right */
    commands[3] = (setpoints[0] - setpoints[1] - setpoints[2] + setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Rear left */

    /* Saturate commands */
    commands[0] = constrain(commands[0], 0, ESC_MAX_CMD - ESC_MIN_CMD);
    commands[1] = constrain(commands[1], 0, ESC_MAX_CMD - ESC_MIN_CMD);
    commands[2] = constrain(commands[2], 0, ESC_MAX_CMD - ESC_MIN_CMD);
    commands[3] = constrain(commands[3], 0, ESC_MAX_CMD - ESC_MIN_CMD);
}