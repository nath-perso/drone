/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "nRF24.h"
#include "pins.h"

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

        if (received[9] & 0x08)
            digitalWrite(RADIO_LED_PIN, HIGH);
        else
            digitalWrite(RADIO_LED_PIN, LOW);
    }
}