/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "nRF24.h"
#include "pins.h"
#include "ESC.h"

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
#define RF_CHANNEL  100
#define PIPE_NUMBER 1
#define IS_CONTROLLER   false

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* Define the address for each board */
const uint64_t address[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};

/* Initialize the radio module */
RF24 radio(RF_CE_PIN_RECEIVER, RF_CSN_PIN_RECEIVER);

/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */

/**
 * @brief       Set up the nRF24L01 radio module
 * @returns     True if succes
 */
bool radioSetup()
{
    Serial.println("Setting up radio communication ...");
    bool result = radio.begin();
    radio.setAutoAck(false);
    result = result & radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.setChannel(RF_CHANNEL);
    radio.openWritingPipe(address[(IS_CONTROLLER ? 0 : 1)]);
    radio.openReadingPipe(PIPE_NUMBER, address[(IS_CONTROLLER ? 1 : 0)]);
    radio.startListening();

    return result;
}

/**
 * @brief       Handle radio data reception
 */
void handleRadioReception()
{
    if (radio.available())
    {
        byte received[32];
        radio.read(received, 32);
        Serial.print("Received : \t");
        for (int i = 0; i < 16; i++)
        {
            Serial.print(received[i]);
            Serial.print("\t");
        }
        Serial.println();

        /* ACTION : Turn on/off a LED and an ESC */
        if (received[9] & 0x08) {
            digitalWrite(RADIO_LED_PIN, HIGH);
            escRun(ESC1, 30);
        }
        else {
            digitalWrite(RADIO_LED_PIN, LOW);
            escRun(ESC1, 0);
        }
        if (received[9] & 0x04)
            escRun(ESC2, 10);
        else
            escRun(ESC2, 0);
        if (received[9] & 0x16)
            escRun(ESC3, 50);
        else
            escRun(ESC3, 0);
        if (received[9] & 0x32)
            escRun(ESC4, 30);
        else
            escRun(ESC4, 0);

        int consigne0 = (received[0] - 128)/255.0*1000;
        if (consigne0 > 0)
            escRun(ESC1, consigne0);
        else if (consigne0 < 0)
                escRun(ESC2, -consigne0);
            else {
                escRun(ESC1, 0);
                escRun(ESC2, 0);
            }

        int consigne2 = (received[2] - 128)/255.0*1000;
        if (consigne2 > 0)
            escRun(ESC3, consigne2);
        else if (consigne2 < 0)
                escRun(ESC4, -consigne2);
            else {
                escRun(ESC3, 0);
                escRun(ESC4, 0);
            }
        
    }
}