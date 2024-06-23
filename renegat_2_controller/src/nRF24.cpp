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
RF24 radio(RF_CE_PIN_SENDER, RF_CSN_PIN_SENDER);

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
 * @brief       Transmit data through radio
 * @return      True if success
 */
bool radioTransmit(keys keys)
{
    radio.stopListening();

    if (!radio.write(&keys, sizeof(keys)))
    {
        Serial.println("TX failed !");
        radio.startListening();
        return false;
    }
    radio.startListening();
    
    return true;
}

/**
 * @brief       Receive data through radio
 * @return      True if new data received
 */
bool radioReceive(byte *data)
{
    if (radio.available())
    {
        radio.read(data, sizeof(data));
        return true;
    }
    return false;
}