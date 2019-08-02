/* 
 * File:   batteryRead.h
 * Author: Scott
 *
 * Created on January 24, 2018, 2:51 PM
 */

#ifndef BATTERYREAD_H
#define	BATTERYREAD_H

#include <xc.h>
#include <stdint.h>

extern void batteryTest();
extern bool isLowBattery();
static void startReadingFVR();
static bool isFVReadDone();
static uint16_t getVoltage();

uint8_t consecutiveLowReadings = 0;
uint16_t lowVoltageThr = 0x018E;//CF;

#endif	/* BATTERYREAD_H */