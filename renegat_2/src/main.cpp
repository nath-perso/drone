/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "mpu.h"
#include "PS2_controller.h"
#include "pins.h"


#define BLINK_DELAY 200 /* (ms) */

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* Blink variables */
bool blinkState = false;
uint32_t blinkLastTime = 0;

/* Frequency and time measuring */
uint32_t lastMainLoopTime = 0;
float mainLoopFrequency = 0;


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

void blink() {
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}


void setup(void) {
  /* Setup pins */
  pinMode(LED_PIN, OUTPUT);

  /* Setup Serial port */
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  /* Setup MPU */
  if (!mpuSetup()) {
    Serial.println("Failed to setup MPU. Exiting program ...");
    return;
  };

  /* Setup PS2 controller */
  if (ps2ControllerSetup()) {
    Serial.println("Failed to setup PS2 controller. Exiting program ...");
    return;
  };
}

void loop() {
  /* Get controller data */
  handlePS2Controller();

  /* Get MPU data */
  mpuGetData();

  /* Print the measures */
  // mpuDisplayData();
  delay(10);

  /* Blink */
  if (millis() - blinkLastTime > BLINK_DELAY) {
    blink();
    blinkLastTime = millis();
  }

  /* Frequency measure */
  // mainLoopFrequency = 1/((micros() - lastMainLoopTime)*1e-6);
  // Serial.println(mainLoopFrequency);
  // lastMainLoopTime = micros();
}


