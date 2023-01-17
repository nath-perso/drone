/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "mpu.h"
#include "PS2_controller.h"

#include <Servo.h>

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
 /* Pins */
#define INTERRUPT_PIN 2
#define LED_PIN       13
#define PS2_DATA_PIN      9 
#define PS2_ATTENTION_PIN 10
#define PS2_COMMAND_PIN   11
#define PS2_CLK_PIN       12

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

/* ESC control */
Servo esc;
int consigne = 0;


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

  /* Setup PS2 controller */
  if (ps2ControllerSetup()) {
    Serial.println("Failed to setup PS2 controller. Exiting program ...");
    return;
  };

  /* ESC setup */
  if (esc.attach(5)) {
    Serial.println("Failed to attach PIN 5 to ESC. Exiting program ...");
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

  esc.writeMicroseconds(consigne);

  // consigne = ps2x.Analog(PSS_LY) - 128;
  if (Serial.available()) {
    int value = Serial.parseInt();
    if (value)
      consigne = value;
  }
  Serial.println(consigne);
}



void blink() {
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}