/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
/* System */
#include <Arduino.h>
#include "pins.h"

/* Drivers */
#include "mpu.h"
#include "nRF24.h"
#include "ESC.h"
#include "flight_controller.h"

#define BLINK_DELAY 200 /* (ms) */

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* Program state */
uint8_t setup_error = 0;

/* Blink variables */
bool blinkState = false;
uint32_t blinkLastTime = 0;

/* Frequency and time measuring */
uint32_t lastMainLoopTime = 0;
float mainLoopFrequency = 0;

/* Radio */
extern byte keyValues[RADIO_FRAME_SIZE];

/* Flight controller */
float flight_commands[NUMBER_OF_COMMANDS];  /* Flight commands table */
int   motors_setpoints[MOTOR_NUMBER];       /* Motors setpoints table */
/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

void blink();

void setup(void)
{
  /* Pins setup */
  pinMode(LED_PIN, OUTPUT);

  /* Serial port setup */
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  /* MPU setup */
  if (!mpuSetup())
  {
    Serial.println("Failed to setup MPU. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* Radio setup */
  if (!radioSetup())
  {
    Serial.println("Failed to setup radio. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* ESC setup */
  if (!escSetup())
  {
    Serial.println("Failed to setup ESCs. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* Reset all commands */
  for(int i = 0; i < NUMBER_OF_COMMANDS; i++)    flight_commands[i] = 0;
  for(int i = 0; i < MOTOR_NUMBER; i++)    motors_setpoints[i] = 0;

  Serial.println("Setup successful. Beginning superloop...");
}

void loop()
{
  if (setup_error)
  {
    /* Failed to setup the system. Do nothing. */
    return;
  }

  /* Receive radio data */
  radioReceiveData(keyValues);

  /* Display data received through radio */
  // radioDisplayData(keyValues);

  /* Get MPU data */
  mpuGetData();

  /* Display commands */
  controllerGetCommands(flight_commands);
  // Serial.print("Commands :\t");
  // for(int i = 0; i < NUMBER_OF_COMMANDS; i++){
  //   Serial.print(flight_commands[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();

  /* Display commands */
  controllerMMA(flight_commands, motors_setpoints);
  Serial.print("Setpoints :\t");
  for(int i = 0; i < MOTOR_NUMBER; i++){
    Serial.print(motors_setpoints[i]);
    Serial.print("\t");
  }
  Serial.println();

  /* Run ESCs */
  escRunAllCommands(motors_setpoints);

  /* Print the measures */
  // mpuDisplayData();
  delay(10);

  /* Blink */
  if (millis() - blinkLastTime > BLINK_DELAY)
  {
    blink();
    blinkLastTime = millis();
  }

  /* Frequency measure */
  // mainLoopFrequency = 1/((micros() - lastMainLoopTime)*1e-6);
  // Serial.println(mainLoopFrequency);
  // lastMainLoopTime = micros();
}

void blink()
{
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
