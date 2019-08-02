

#include <xc.h>
#include <stdint.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "batteryRead.h"
#include "transmit.h"
#include "spi.h"
#include "IO.h"

uint16_t crc16;
uint16_t timeTo6s = 0;
bool isAlarm, isMySupervisor, isTest, isTemp;

void transmit (enum TransmitType txType)
{
    /* Save WDT duty and reassign to 16s. Preamble is 6s long. Re-enable. */
    uint8_t wdtSaveBits = WDTCONbits.WDTPS;
    WDTCONbits.SWDTEN = 0;
    CLRWDT();
    WDTCONbits.WDTPS = 0b01110;
    WDTCONbits.SWDTEN = 1;
    
    uint8_t txBuffer[6];
    
    initSPI();
    
    
    /* Write Radio Regs */
    registerConfig();
    
    /* Calibrate */
    manualCalibration();
    calibrateRCOsc();
    
    if (txType == alarm)
    {
        /* Blink LED during alarm transmission */
        blinkLED();
        
        /* Set critical status bits */
        txPacket.TRIGGER = 1;
        txPacket.SUPERVISORY = 0;
        txPacket.TAMPER_p = stillTampered;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = isLowBattery();
    }
    else if (txType == tamper)
    {
        /* Blink LED during tamper transmission */
        blinkLED();       // blink LED in different part of program
        
        /* Set critical status bits */
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 0;
        txPacket.ERR = 0;
        txPacket.TAMPER_p = 1;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = isLowBattery();
    }
    else if (txType == test)
    {
        
        /* Blink LED during test transmission */
        blinkLED();
        
        /* Set critical status bits */
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 0;
        txPacket.ERR = 0;
        txPacket.TAMPER_p = stillTampered;
        txPacket.TEST = 1;
        txPacket.LOW_BATT = isLowBattery();
    }
    else if (txType == restore)
    {
        /* Blink LED during restore transmission */
        blinkLED();
        
        /* Set critical status bits */
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 0;
        txPacket.ERR = 0;
        txPacket.TAMPER_p = 0;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = isLowBattery();
    }
    else if (txType == supervisory)
    {
        /* Don't blink LED for supervisory */
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 1;
        txPacket.ERR = 0;
        txPacket.TAMPER_p = stillTampered;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = isLowBattery();
    }
    else if (txType == lowBatt)
    {
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 0;
        txPacket.ERR = 0;
        txPacket.TAMPER_p = stillTampered;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = 1;//isLowBattery();          // note that isLowBattery might be more suitable here?
    }
    else if (txType == err)
    {
        txPacket.TRIGGER = stillTriggered;
        txPacket.SUPERVISORY = 0;
        txPacket.ERR = 1;
        txPacket.TAMPER_p = stillTampered;
        txPacket.TEST = 0;
        txPacket.LOW_BATT = isLowBattery();          // note that isLowBattery might be more suitable here?
    }
    if (txType != temp)
    {
    assignSerial();
    
    /* Form packet */
    createPacket(txBuffer);
    appendCRC(txBuffer);
    
    /* Send packet */
    isAlarm = (txType == alarm);
    isMySupervisor = (txType == supervisory);
    isTest = (txType == test);
    burstTX(txBuffer, sizeof(txBuffer), isAlarm);
//    ledTimerOff();
    }
    CLRWDT();
    WDTCONbits.SWDTEN = 0;//0; //REMOVE
    WDTCONbits.WDTPS = wdtSaveBits;
    WDTCONbits.SWDTEN = 1;//0; //REMOVE
    
    CLRWDT();
    while((cc1120GetTxStatus() & 0x70) != 0)
    {
        NOP();
        cmdStrobe(CC1120_SIDLE);
        __delay_ms(5);
    }
    cmdStrobe(CC1120_SWORRST);
    __delay_ms(5);
    cmdStrobe(CC1120_SWOR);     // Put CC115L to sleep
    __delay_us(50);
    
//    disableSPI();
}

extern void burstTX(uint8_t txBuffer[], uint8_t len, bool isAlarm)
{
    unsigned uint8_t error_cnt, error_flag;
    uint8_t count = 4, numSlots = 6, slotDelay = 1, slotRepeat = 2;     // Repeat twice for alarm; once for any other signal
    
    if (!isAlarm && !isTest)        // Supervisory, tamper, low batt, or err/malfunction.
    {
        slotRepeat = 1;
        if (isMySupervisor)
        {
            count = 1;
        }
        for (uint8_t t = 0; t < count; t++)
        {
            if (t == 0)             // normal transmission, slot 1 always first.
            {
                for (uint8_t j = 0; j < slotRepeat; j++)
                    writePacket(txBuffer, len);
                if (!isMySupervisor)
                    delaySlots(5);      // fixed number slots to wait, if not a supervisory message
            }
            else
            {
                slotDelay = random(serialNumberL, 0);
                delaySlots((uint8_t)(slotDelay - slotRepeat));
                for (uint8_t j = 0; j < slotRepeat; j++)
                    writePacket(txBuffer, len);
                delaySlots((uint8_t)(numSlots - (slotDelay + (slotRepeat - 1))));
            }
        }
    }
    else if (isAlarm)
    {
        for (uint8_t t = 0; t < count; t++)
        {
            if (t == 0)             // normal transmission, slot 1 always first.
            {
                for (uint8_t j = 0; j < slotRepeat; j++)
                    writePacket(txBuffer, len);
                delaySlots(4);      // fixed number slots to wait
            }
            else
            {
                slotDelay = random(serialNumberL, 1);
                delaySlots((uint8_t)(slotDelay - (slotRepeat - 1)));
                for (uint8_t j = 0; j < slotRepeat; j++)
                    writePacket(txBuffer, len);
                delaySlots((uint8_t)(numSlots - (slotDelay + (slotRepeat - 1))));
            }
        }
    }
    else        // isTest
    {
        slotRepeat = 0x03;         // three consecutive transmissions
        for (uint8_t j = 0; j < slotRepeat; j++)
            writePacket(txBuffer, len);
    }
}

void writePreamble()
{
    initSPI();
//    manualCalibration();
    CLRWDT();
    cmdStrobe(CC1120_SIDLE);
    __delay_us(10);
    cmdStrobe(CC1120_STX);
    for (uint16_t i = 0; i < 6000; i++)
    {
        __delay_ms(1);
        CLRWDT();
    }
    batteryTest();         // Check battery 3s into preamble
//    for (uint16_t i = 0; i < 2999; i++)     // Max 1ms lost during battery test
//    {
//        __delay_ms(1);
//        CLRWDT();
//    }
    timeTo6s = 0;
}

uint16_t random(uint8_t seed, uint8_t option)
{
    crc16 = CRC_16(crc16, seed);
    if (option == 0)        // to generate random non-alarm slot
        return (uint16_t)(crc16 % 0x06 + 0x01);     // Range is 1-6 for any of the 6 slots
    else                    // to generate random alarm slot
        return (uint16_t)(crc16 % 0x05 + 0x01);     // Range is 1-5 for alarm, since two alarm signals must be 
                                                    // transmitted consecutively
}

void writePacket(uint8_t txBuffer[], uint8_t len)
{
    writePreamble();
    CLRWDT();
    cmdStrobe(CC1120_SIDLE);
    while((cc1120GetTxStatus() & 0x70) != 0);         /* Wait until TX status<6:4> returns 0, to indicate "ready" */
    /* Write packet to TX FIFO */
    cc1120SpiWriteTxFifo(txBuffer, len);
        
    /* Strobe TX to send packet */
    cmdStrobe(CC1120_STX);
    
    /* Wait and flush buffer */
    __delay_ms(200);
    cmdStrobe(CC1120_SIDLE);
    while((cc1120GetTxStatus() & 0x70) != 0);
    cmdStrobe(CC1120_SFTX);
    
    /* Wait and calibrate */
    cmdStrobe(CC1120_SIDLE);
    while((cc1120GetTxStatus() & 0x70) != 0);
    cmdStrobe(CC1120_SCAL);
    __delay_ms(2);
    
    
    
            
    /* Wait for the rest of 6s to elapse */
//    for (uint8_t j = 0; j < timeTo6s; j++)
//    {
//        CLRWDT();
//        __delay_ms(1);
//    }
    CLRWDT();
    while((cc1120GetTxStatus() & 0x70) != 0)
    {
        cmdStrobe(CC1120_SIDLE);
        __delay_ms(5);
    }
    cmdStrobe(CC1120_SWORRST);
    __delay_ms(5);
    cmdStrobe(CC1120_SWOR);     // Put CC115L to sleep
    __delay_us(50);
}


void delaySlots(uint8_t numSlotsToWait)
{
    for (uint8_t slotCount = 0; slotCount < numSlotsToWait; slotCount++)
    {
        for (uint16_t i = 0; i < 600; i++)//prev to 600
        {
            __delay_ms(10);
            CLRWDT();
        }
    }
}