/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <Arduino.h>
#include "PS2_controller.h"
#include "pins.h"

#define BLINK_DELAY 200 /* (ms) */

/* RADIO */
#include <SPI.h>
#include "RF24.h"
#include "printf.h"
#define RF_CE_PIN_SENDER  7
#define RF_CSN_PIN_SENDER 8
#define RF_PAYLOAD_SIZE	  10

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

/* RADIO */
// Define the address for each board
const uint64_t address[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// Initialize the radio module
RF24 radio(RF_CE_PIN_SENDER, RF_CSN_PIN_SENDER);
unsigned int rx_status[2] = { 0, 0};

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
  /* Setup pins */
  pinMode(LED_PIN, OUTPUT);

  /* Setup Serial port */
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  /* Setup PS2 controller */
  if (ps2ControllerSetup()) {
    Serial.println("Failed to setup PS2 controller. Exiting setup ...");
    setup_error = 1;
    return;
  };

  /* RADIO */
  radio.begin();
  Serial.println("Setting up the radio communication ...");
	radio.setAutoAck(false);		
	// radio.enableAckPayload();
	// radio.enableDynamicPayloads();
	// radio.setRetries(5, 5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(100);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.startListening();

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

  /* Radio code */
  // if (Serial.available()) {
  //   radio.stopListening();
  //   String input = Serial.readStringUntil('\n');
  //   radio.write(input.c_str(), input.length());
  //   Serial.print("Sending '"); Serial.print(input.c_str()); Serial.println("'");
  //   radio.startListening();
  // }

  // if (ps2IsButtonPressed(PSB_BLUE)) {
  //   String test = "1";
  //   radio.stopListening();
  //   radio.write(test.c_str(), test.length());
  //   Serial.print("Sending '"); Serial.print(test); Serial.println("'");
  //   radio.startListening();
  // }

  // if (ps2IsButtonPressed(PSB_RED)) {
  //   String testoff = "0";
  //   radio.stopListening();
  //   radio.write(testoff.c_str(), testoff.length());
  //   Serial.print("Sending '"); Serial.print(testoff); Serial.println("'");
  //   radio.startListening();
  // }

  radio.stopListening();
	if (radio.write(&keyValues, sizeof(keyValues)) )
	{
		Serial.println("...tx success");
    Serial.println(keyValues.digitalKeys.bits.tr);
		// if (radio.isAckPayloadAvailable())
		// {
		// 	radio.read(rx_status, sizeof(rx_status));
		// 	Serial.print("received ack payload is : ");
		// 	Serial.println(rx_status[0]);
		// }
		// else
		// {
		// 	Serial.println("status has become false so stop here....");
		// }
	}
  radio.startListening();

  /* Frequency measure */
  // mainLoopFrequency = 1/((micros() - lastMainLoopTime)*1e-6);
  // Serial.println(mainLoopFrequency);
  // lastMainLoopTime = micros();
}


