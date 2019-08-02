/* 
 * File:   transmit.h
 * Author: Scott
 *
 * Created on January 24, 2018, 3:05 PM
 */

#ifndef TRANSMIT_H
#define	TRANSMIT_H

#include <stdint.h>

enum TransmitType {alarm, test, tamper, supervisory, lowBatt, err, restore, temp};

extern void createPacket(uint8_t txBuffer[]);
extern void burstTX(uint8_t txBuffer[], uint8_t len, bool isAlarm);
void writePreamble();
void transmit (enum TransmitType txType);
void writePacket(uint8_t txBuffer[], uint8_t len);
void delaySlots(uint8_t numSlotsToWait);

uint16_t CRC_16( uint16_t crcVal, uint8_t data);
uint16_t CS_rf_protocol_CRC16_byteBuffer(uint8_t* byteBuffer, uint8_t bufferLength);
uint16_t random(uint8_t seed, uint8_t option);

extern const uint8_t serialNumberL @ 0x0020;
extern const uint8_t serialNumberM @ 0x0021;
extern const uint8_t serialNumberH @ 0x0022;

#endif	/* TRANSMIT_H */