/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
/* System */
#include <Arduino.h>
#include "pins.h"

/* Drivers */
#include "mpu.h"
#include "nRF24.h"

#include "Servo.h"

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

/* ESC control */
Servo esc;
int consigne = 0;

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

  //  /* ESC setup */
  //  if (esc.attach(5)) {
  //    Serial.println("Failed to attach PIN 5 to ESC. Exiting setup ...");
  // setup_error = 1;
  //    return;
  //  };

  Serial.println("End of setup function. Beginning superloop...");
}

void loop()
{
  if (setup_error)
  {
    /* Failed to setup the system. Do nothing. */
    return;
  }

  /* Receive radio data */
  handleRadioReception();

  /* Get MPU data */
  mpuGetData();

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

  //  esc.writeMicroseconds(consigne);
  //
  //  // consigne = ps2x.Analog(PSS_LY) - 128;
  //  if (Serial.available()) {
  //    int value = Serial.parseInt();
  //    if (value)
  //      consigne = value;
  //  }
  //  Serial.println(consigne);
}

void blink()
{
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
