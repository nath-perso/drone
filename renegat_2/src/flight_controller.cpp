/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "flight_controller.h"
#include "pins.h"
#include "Arduino.h"
#include "ESC.h"
#include "mpu.h"
#include "nRF24.h"
#include "PS_controls.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */

#define THRUST_UP_COMMAND_INDEX    8
#define THRUST_DOWN_COMMAND_INDEX  8
#define PITCH_COMMAND_INDEX     0
#define ROLL_COMMAND_INDEX      2
#define YAW_COMMAND_INDEX       4
#define EMEGENCY_STOP_COMMAND_INDEX  8

#define MAX_ANGLE   PI/200
#define MIN_ANGLE   -PI/200
#define MAX_ANGULAR_VELOCITY 400

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */

class PIDController {
private:
    float Kp;
    float Ki;
    float Kd;

    float last_error;
    long  last_timestamp;
    float integral;
    float last_derivative;

    float last_command;
    float a0;

    float min_integral;
    float max_integral;

public:
    PIDController(float _Kp, float _Ki, float _Kd, float _min_integral, float _max_integral, float _a0)
        : Kp(_Kp), Ki(_Ki), Kd(_Kd), 
        last_error(0), last_timestamp(0), 
        integral(0), last_derivative(0), 
        last_command(0), a0(_a0),
        min_integral(_min_integral), 
        max_integral(_max_integral)
        {}
    
    float computeCommand(float error) {
        float dt = max((micros() - last_timestamp)/1e6, 1e-6);

        integral += error*dt;
        float derivative = (error - last_error)/dt;
        derivative = a0*derivative + (1 - a0)*last_derivative;

        /* Anti-windup */
        integral = constrain(integral, min_integral/Ki, max_integral/Ki);

        float command = Kp*error + Ki*integral + Kd*derivative;
        command = constrain(command, -1, 1);

        last_command = command;
        last_error = error;
        last_derivative = derivative;
        last_timestamp = micros();
        return command;
    }

    void setConstants(float Kp_, float Ki_, float Kd_) 
    {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;

        Serial.print(Kp); Serial.print("\t");
        Serial.print(Ki); Serial.print("\t");
        Serial.print(Kd); Serial.print("\n");
        Serial.flush();
        Serial.readStringUntil('\n');
    }

    void setKp(float Kp_) { Kp = Kp_; }
    void setKi(float Ki_) { Ki = Ki_; }
    void setKd(float Kd_) { Kd = Kd_; }

    void incrementKp(float delta) { Kp += delta; Kp = max(Kp, 0);}
    void incrementKi(float delta) { Ki += delta; Ki = max(Ki, 0);}
    void incrementKd(float delta) { Kd += delta; Kd = max(Kd, 0);}

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }
};

float Kp_value = 0.23; // 0.2 - not able to get back up, 0.3 - unstable, 0.23 looks good
float Ki_value = 0.03; //0.1;
float Kd_value = 0.02; // Manipulable at 0.04 -> decrease by 20%
//P0.4, I0, D0.04
//P0.4, I0, D0.05 a bit better but still unstable
//P0.4, I0, D0.06 stable ? not at 2nd try
//P0.4, I0, D0.08 unstable
//P0.4, I0, D0.04 more unstable
//P0.3, I0.02, D0.04 correct
//P0.35, I0.01, D0.06 very correct, but still some static error
//P0.35, I0.02, D0.06 very very correct, but still some static error
//P0.35, I0.03, D0.06 let's test it -> P0.23, D0.03 after testing -> unstable, need to reduce gains ? see video

float integral_limit = 0.05;
float a0_value = 1;

PIDController rollPID(Kp_value, Ki_value, Kd_value, -integral_limit, integral_limit, a0_value);
PIDController pitchPID(1.57*Kp_value, 1.57*Ki_value, 1.57*Kd_value, -integral_limit, integral_limit, a0_value);
// PIDController pitchPID(0, 0, 0, -integral_limit, integral_limit, 0);
PIDController yawPID(0.00015, 0, 0, -integral_limit, integral_limit, 0);

/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

/**
 * @brief       Update PID Kp according to input
 * @param       PID : PID controller to update
 * @param       Kp_up : Kp up command
 * @param       Kp_down : Kp down command
 */
void updateKp(PIDController &PID, bool Kp_up, bool Kp_down)
{
    if(Kp_up) PID.incrementKp(0.01);
    if(Kp_down) PID.incrementKp(-0.01);
}

/**
 * @brief       Update PID Ki according to input
 * @param       PID : PID controller to update
 * @param       Ki_up : Ki up command
 * @param       Ki_down : Ki down command
 */
void updateKi(PIDController &PID, bool Ki_up, bool Ki_down)
{
    if(Ki_up) PID.incrementKi(0.001);
    if(Ki_down) PID.incrementKi(-0.001);
}

/**
 * @brief       Update PID Kd according to input
 * @param       PID : PID controller to update
 * @param       Kd_up : Kd up command
 * @param       Kd_down : Kd down command
 */
void updateKd(PIDController &PID, bool Kd_up, bool Kd_down)
{
    if(Kd_up) PID.incrementKd(0.001);
    if(Kd_down) PID.incrementKd(-0.001);
}

/**
 * @brief       Compute roll command
 * @param       raw_setpoint : roll raw setpoint from controller
 * @return      Roll command (between -1 and 1)
*/
float controllerGetRollCommand(float raw_setpoint){
    float roll_value = mpuGetRoll();
    float roll_error = roll_value - raw_setpoint;

    float roll_command = rollPID.computeCommand(roll_error);

    return (roll_command);
}

/**
 * @brief       Compute pitch command
 * @param       raw_setpoint : pitch raw setpoint from controller
 * @return      Pitch command (between -1 and 1)
*/
float controllerGetPitchCommand(float raw_setpoint){
    float pitch_value = mpuGetPitch();
    float pitch_error = pitch_value - raw_setpoint;

    float pitch_command = pitchPID.computeCommand(pitch_error);

    return (pitch_command);
}

/**
 * @brief       Compute yaw command
 * @param       raw_setpoint : yaw raw setpoint from controller
 * @return      Yaw command (between -1 and 1)
*/
float controllerGetYawCommand(float raw_setpoint){
    float yaw_value = mpuGetYawSpeed();
    float yaw_error = yaw_value - raw_setpoint;

    float yaw_command = yawPID.computeCommand(yaw_error);

    return (yaw_command);
}

/**
 * @brief       Convert key values into setpoints
 * @param       setpoints : [thrust, roll, pitch, yaw] setpoints
 */
void controllerGetCommands(float *setpoints)
{
    static float thrust_raw_setpoint = 0;
    float roll_raw_setpoint   = 0;
    float pitch_raw_setpoint  = 0;
    float yaw_raw_setpoint    = 0;

    if(L1()) thrust_raw_setpoint += 0.004;
    if(L2()) thrust_raw_setpoint -= 0.008;
    // Constrain thrust between 0 and 1
    thrust_raw_setpoint = constrain(thrust_raw_setpoint, 0, 1);
    // If emergency stop command active, set thrust to 0
    if(R3() || L3()) thrust_raw_setpoint = 0;

    // Convert roll, pitch and yaw commands into setpoints
    roll_raw_setpoint = pow(leftStickX(), 3)*MAX_ANGLE;
    pitch_raw_setpoint = pow(leftStickY(), 3)*MAX_ANGLE;
    yaw_raw_setpoint = pow(-rightStickX(), 3)*MAX_ANGULAR_VELOCITY;

    setpoints[0] = thrust_raw_setpoint;     // No control on thrust
    setpoints[1] = controllerGetRollCommand(roll_raw_setpoint);
    setpoints[2] = controllerGetPitchCommand(pitch_raw_setpoint);
    setpoints[3] = controllerGetYawCommand(yaw_raw_setpoint);
}

/**
 * @brief       Motor Mixing Algorithm : convert flight setpoints (considered as setpoints) into motor commands
 * @param       setpoints : [thrust, roll, pitch, yaw] flight setpoints
 * @param       commands : motors commands table [Front right, front left, rear right, rear left]
*/
void controllerMMA(float *setpoints, int *commands) {
    commands[0] = (setpoints[0] + setpoints[1] + setpoints[2] + setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Front right */
    commands[1] = (setpoints[0] - setpoints[1] + setpoints[2] - setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Front left */
    commands[2] = (setpoints[0] + setpoints[1] - setpoints[2] - setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Rear right */
    commands[3] = (setpoints[0] - setpoints[1] - setpoints[2] + setpoints[3])*(ESC_MAX_CMD - ESC_MIN_CMD); /* Rear left */

    /* Saturate commands */
    commands[0] = constrain(commands[0], 0, (ESC_MAX_CMD - ESC_MIN_CMD)*ESC_CMD_SAT_FACTOR);
    commands[1] = constrain(commands[1], 0, (ESC_MAX_CMD - ESC_MIN_CMD)*ESC_CMD_SAT_FACTOR);
    commands[2] = constrain(commands[2], 0, (ESC_MAX_CMD - ESC_MIN_CMD)*ESC_CMD_SAT_FACTOR);
    commands[3] = constrain(commands[3], 0, (ESC_MAX_CMD - ESC_MIN_CMD)*ESC_CMD_SAT_FACTOR);
}

/**
 * @brief       Update PID constants according to controller input
 */
void updatePIDConstants()
{
    updateKp(rollPID, triangle(), cross());
    updateKi(rollPID, circle(), square());

    updateKp(pitchPID, padUp(), padDown());
    updateKi(pitchPID, padRight(), padLeft());
}

/**
 * @brief       Display PID constants
 */
void displayPIDConstants()
{
    Serial.print(rollPID.getKp()); Serial.print("\t");
    Serial.print(rollPID.getKi()); Serial.print("\t");
    Serial.print(rollPID.getKd()); Serial.print("\t");

    Serial.print(pitchPID.getKp()); Serial.print("\t");
    Serial.print(pitchPID.getKi()); Serial.print("\t");
    Serial.print(pitchPID.getKd()); Serial.print("\t");

    Serial.print(yawPID.getKp()); Serial.print("\t");
    Serial.print(yawPID.getKi()); Serial.print("\t");
    Serial.print(yawPID.getKd()); Serial.print("\t");
}