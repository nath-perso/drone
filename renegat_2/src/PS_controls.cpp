/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <PS_controls.h>
#include <nRF24.h>

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define RIGHT_BUTTONS_INDEX     9
#define RL_BUTTONS_INDEX        8
#define UP_DOWN_PAD_BUTTONS_INDEX       8
#define LEFT_RIGHT_PAD_BUTTONS_INDEX    9
#define LEFT_STICK_X_INDEX      0
#define LEFT_STICK_Y_INDEX      2
#define RIGHT_STICK_X_INDEX     4
#define RIGHT_STICK_Y_INDEX     6

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
extern byte keyValues[RADIO_FRAME_SIZE];

/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

/**
 * @brief       Is triangle pressed
 * @return      true if triangle pressed
 */
bool triangle()
{
    return keyValues[RIGHT_BUTTONS_INDEX] & 0x0004;
}

/**
 * @brief       Is cross pressed
 * @return      true if cross pressed
 */
bool cross()
{
    return keyValues[RIGHT_BUTTONS_INDEX] & 0x0008;
}

/**
 * @brief       Is circle pressed
 * @return      true if circle pressed
 */
bool circle()
{
    return keyValues[RIGHT_BUTTONS_INDEX] & 0x0020;
}

/**
 * @brief       Is square pressed
 * @return      true if square pressed
 */
bool square()
{
    return keyValues[RIGHT_BUTTONS_INDEX] & 0x0010;
}

/**
 * @brief       Is L1 pressed
 * @return      true if L1 pressed
 */
bool L1()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0001;
}

/**
 * @brief       Is L2 pressed
 * @return      true if L2 pressed
 */
bool L2()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0002;
}

/**
 * @brief       Is L3 pressed
 * @return      true if L3 pressed
 */
bool L3()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0004;
}

/**
 * @brief       Is R1 pressed
 * @return      true if R1 pressed
 */
bool R1()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0008;
}

/**
 * @brief       Is R2 pressed
 * @return      true if R2 pressed
 */
bool R2()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0010;
}

/**
 * @brief       Is R3 pressed
 * @return      true if R3 pressed
 */
bool R3()
{
    return keyValues[RL_BUTTONS_INDEX] & 0x0020;
}

/**
 * @brief       Up pad button pressed
 */
bool padUp()
{
    return keyValues[UP_DOWN_PAD_BUTTONS_INDEX] & 0x0040;
}

/**
 * @brief       Down pad button pressed
 */
bool padDown()
{
    return keyValues[UP_DOWN_PAD_BUTTONS_INDEX] & 0x0080;
}

/**
 * @brief       Left pad button pressed
 */
bool padLeft()
{
    return keyValues[LEFT_RIGHT_PAD_BUTTONS_INDEX] & 0x0001;
}

/**
 * @brief       Right pad button pressed
 */
bool padRight()
{
    return keyValues[LEFT_RIGHT_PAD_BUTTONS_INDEX] & 0x0002;
}

/**
 * @brief       Left stick left/right value
 * @return      value between -1 and 1
 */
float leftStickX()
{
    return (keyValues[LEFT_STICK_X_INDEX] - 128) / 128.0;
}

/**
 * @brief       Left stick up/down value
 * @return      value between -1 and 1
 */
float leftStickY()
{
    return (keyValues[LEFT_STICK_Y_INDEX] - 128) / 128.0;
}

/**
 * @brief       Right stick left/right value
 * @return      value between -1 and 1
 */
float rightStickX()
{
    return (keyValues[RIGHT_STICK_X_INDEX] - 128) / 128.0;
}

/**
 * @brief       Right stick up/down value
 * @return      value between -1 and 1
 */
float rightStickY()
{
    return (keyValues[RIGHT_STICK_Y_INDEX] - 128) / 128.0;
}

/**
 * @brief        Comprehensive display of radio data
 */
void radioDisplayComprehensive()
{
    Serial.print(leftStickX()); Serial.print("\t");
    Serial.print(leftStickY()); Serial.print("\t");
    Serial.print(rightStickX()); Serial.print("\t");
    Serial.print(rightStickY()); Serial.print("\t");

    if(triangle()) Serial.print("T ");
    if(cross()) Serial.print("X ");
    if(circle()) Serial.print("O ");
    if(square()) Serial.print("S ");
    if(L1()) Serial.print("L1 ");
    if(L2()) Serial.print("L2 ");
    if(L3()) Serial.print("L3 ");
    if(R1()) Serial.print("R1 ");
    if(R2()) Serial.print("R2 ");
    if(R3()) Serial.print("R3 ");
    if(padUp()) Serial.print("UP ");
    if(padDown()) Serial.print("DOWN ");
    if(padLeft()) Serial.print("LEFT ");
    if(padRight()) Serial.print("RIGHT ");

    Serial.print("\t\t");
}