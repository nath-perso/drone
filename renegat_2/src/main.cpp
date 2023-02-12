/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <Arduino.h>
#include "mpu.h"
#include "PS2_controller.h"
#include "pins.h"

#include "Servo.h"


#define BLINK_DELAY 200 /* (ms) */

/* RADIO */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define RF_CE_PIN_RECEIVER 7
#define RF_CSN_PIN_RECEIVER 8

// Define the address for each board
const uint64_t address[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// Initialize the radio module
RF24 radio(RF_CE_PIN_RECEIVER, RF_CSN_PIN_RECEIVER);

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
    Serial.println("Failed to setup MPU. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* Setup PS2 controller */
  // if (ps2ControllerSetup()) {
  //   Serial.println("Failed to setup PS2 controller. Exiting setup ...");
  //   setup_error = 1;
  //   return;
  // };

//  /* ESC setup */
//  if (esc.attach(5)) {
//    Serial.println("Failed to attach PIN 5 to ESC. Exiting setup ...");
    // setup_error = 1;
//    return;
//  };

  /* RADIO */
  radio.begin();
  Serial.println("Setting up the radio communication ...");
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(100);
  radio.openWritingPipe(address[1]);
  radio.openReadingPipe(1, address[0]);
  radio.startListening();
}

void loop() {
  if (setup_error)
  {
    /* Failed to setup the system. Do nothing. */
    return;
  }
  
  /* Get controller data */
  // handlePS2Controller();

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


//  esc.writeMicroseconds(consigne);
//
//  // consigne = ps2x.Analog(PSS_LY) - 128;
//  if (Serial.available()) {
//    int value = Serial.parseInt();
//    if (value)
//      consigne = value;
//  }
//  Serial.println(consigne);


// Receiver board code
  if (radio.available()) {
    char received[32];
    radio.read(received, 32);
    Serial.println("Received: " + String(received));
    Serial.println(received);
    if (String(received) == "1")
      digitalWrite(5, HIGH);
    else if (String(received) == "0")
      digitalWrite(5, LOW);
  }
}


