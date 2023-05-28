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

byte keyValues[RADIO_FRAME_SIZE];

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
 * @param       storageTable : global table to store data received
 */
void radioReceiveData(byte *storageTable)
{
    if (radio.available())
    {
        radio.read(storageTable, RADIO_FRAME_SIZE);
    }
}

/**
 * @brief       Display received data from radio
 * @param       storageTable : global table in which data received is stored
 */
void radioDisplayData(byte *storageTable)
{
    Serial.print("Received : \t");
    for (int i = 0; i < RADIO_FRAME_SIZE/2; i++)
    {
        Serial.print(storageTable[i]);
        Serial.print("\t");
    }
    Serial.println();
}