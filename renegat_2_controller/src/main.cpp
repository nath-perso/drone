/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <Arduino.h>
#include "PS2_controller.h"
#include "nRF24.h"
#include "pins.h"
#include "PS_controls.h"

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

/* PS2 */
keys keyValues;


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

void blink() {
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}


void setup(void) {
  /* Pins setup */
  pinMode(LED_PIN, OUTPUT);

  /* Serial port setup */
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  /* PS2 controller setup */
  if (ps2ControllerSetup()) {
    Serial.println("Failed to setup PS2 controller. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* Radio setup */
  if (!radioSetup()) {
    Serial.println("Failed to setup radio. Exiting setup ...");
    setup_error = 1;
    return;
  };

  Serial.println("End of setup. Beginning superloop...");
}

void loop() {
  if (setup_error)
  {
    /* Failed to setup the system. Do nothing. */
    return;
  }
  
  /* Get controller data */
  handlePS2Controller(&keyValues);

  /* Blink */
  if (millis() - blinkLastTime > BLINK_DELAY) {
    blink();
    blinkLastTime = millis();
  }

  /* Transmit data through radio */
  radioTransmit(keyValues);

  /* Frequency measure */
  // mainLoopFrequency = 1/((micros() - lastMainLoopTime)*1e-6);
  // Serial.println(mainLoopFrequency);
  // lastMainLoopTime = micros();
}


