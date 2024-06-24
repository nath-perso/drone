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
#define BATTERY_CONVERTER 234/11.6  /* (cnts/V) */
#define BATTERY_VOLTAGE_WARNING   9*BATTERY_CONVERTER   /* (cnts) */

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
int   motors_commands[MOTOR_NUMBER];       /* Motors setpoints table */
/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

void checkBatteryVoltage();
void blink();
void radioDisplayComprehensive();

void setup(void)
{
  /* ESC setup */
  if (!escSetup())
  {
    Serial.println("Failed to setup ESCs. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* Pins setup */
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  /* Serial port setup */
  Serial.begin(115200);
  // while (!Serial)
  //   delay(10);

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

  /* Reset all commands */
  for(int i = 0; i < NUMBER_OF_COMMANDS; i++)    flight_commands[i] = 0;
  for(int i = 0; i < MOTOR_NUMBER; i++)    motors_commands[i] = 0;

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
  radioDisplayComprehensive();

  /* Get MPU data */
  mpuGetData();

  /* Display flight commands */
  controllerGetCommands(flight_commands);
  // Serial.print("Flight commands :\t");
  for(int i = 0; i < NUMBER_OF_COMMANDS; i++){
    Serial.print(flight_commands[i], 4);
    Serial.print("\t");
  }
  // Serial.println();

  /* Display commands */
  controllerMMA(flight_commands, motors_commands);
  // Serial.print("Commands :\t");
  for(int i = 0; i < MOTOR_NUMBER; i++){
    Serial.print(motors_commands[i]);
    Serial.print("\t");
  }
  // Serial.println();

  /* Run ESCs */
  escRunAllCommands(motors_commands);

  /* Set PID constants */
  updatePIDConstants();
  displayPIDConstants();
  // if (Serial.available()) { 
  //   float Kp_des = Serial.parseFloat();
  //   Serial.println("Ki ? "); Serial.flush();
  //   Serial.readStringUntil('\n');

  //   while (!Serial.available()){Serial.flush();}
  //   float Ki_des = Serial.parseFloat();
  //   Serial.println("Kd ? ");Serial.flush();
  //   Serial.readStringUntil('\n');

  //   while (!Serial.available()){Serial.flush();}
  //   float Kd_des = Serial.parseFloat();

  //   controllerSetRollPID(Kp_des, Ki_des, Kd_des);
  // }


  /* Print the measures */
  mpuDisplayData();
  delay(10);

  /* Check battery voltage */
  checkBatteryVoltage();

  /* Frequency measure */
  // mainLoopFrequency = 1/((micros() - lastMainLoopTime)*1e-6);
  // Serial.println(mainLoopFrequency);
  // lastMainLoopTime = micros();

  Serial.println();
}

void blink()
{
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}


void checkBatteryVoltage()
{
  int voltage = analogRead(BATTERY_VOLTAGE_PIN);
  if (voltage < BATTERY_VOLTAGE_WARNING)
  {
    /* Blink */
    if (millis() - blinkLastTime > BLINK_DELAY)
    {
      blink();
      blinkLastTime = millis();
    }
  }
  else digitalWrite(LED_PIN, LOW);
}

