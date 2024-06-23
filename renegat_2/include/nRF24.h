#ifndef __NRF24_H__
#define __NRF24_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
#define RADIO_FRAME_SIZE    32

/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
bool radioSetup();
void radioReceiveData(byte *storageTable);
void radioDisplayData(byte *storageTable);
void radioSendData(byte *data);

#endif /* __NRF24_H__ */