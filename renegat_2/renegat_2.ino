/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "mpu.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define INTERRUPT_PIN 2
#define LED_PIN       13
#define BLINK_DELAY   200   /* (ms) */    

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
bool blinkState = false;
uint32_t blinkLastTime = 0;


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
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
}

void loop() {
  mpuGetData();

  /* Print the measures */
  mpuDisplayData();
  delay(10);

  /* Blink */
  if (millis() - blinkLastTime > BLINK_DELAY) {
    blink();
    blinkLastTime = millis();
  }
}



void blink() {
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}