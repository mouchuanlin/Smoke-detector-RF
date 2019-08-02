/*
 * File:   main.c
 * Author: JV
 *
 * Code Version: 0.1
 * 
 * Created on January 04, 2019. JV.
 * 
 * This program is intended to run on the Instant Care
 * smoke detector slave, as part of the complete 906MHz 
 * property management system.
 * 
 * 002->003: 16x packets per burst, 3 bursts per transmission. Random preamble lengths after first packet,
 *           random gaps between burst. Defined serial ID in program memory 0x0020 - 0x0022. 
 * 003->004: Corrected transmission: 2x consecutive packets per frame, set by random slot choice (out of 7
 *           slots), 3 frames for non-alarm. Briefly listens for ACK right after. 
 * 004->006: 27-08-2018 (JV). Also detects low battery from motherboard, ERR.
 * 006->007: 12-10-2018 (JV). Adjusted supervisory interval to 1 day. Added logic to prevent unnecessary slot delay 
 *           in supervisory transmission. Added 3x test signal with no delay.
 * 007->008: 12-10-2018 (JV). Added malfunction reporting per IC WoR protocol rev. 014.
 *           Removed led blink on low battery transmission.
 * 008->009: 04-12-2018 (JV). 20min wait after initial tamper to buffer the transmission of a restore status.
 * 009->010: 03-01-2019 (JV). Tuned tx power to 10dBm.
 * 010->011: 07-01-2019 (JV). Changed hardware to C01 with CC1120 radio, which has cleaner modulation.
 * 011->012: 09-01-2019 (JV). TEMP/TESTING ONLY. Continuous packet transmit, for RF/power tuning purposes.
 * 012->013: 23-01-2019 (JV). Tuned power, 3s WOR.
 * 013->014: 17-04-2019 (JV). Continuous transmit for FCC tuning.
 * 014->015: 03-05-2019 (JV). Tuned power, continuous transmit.
 * 015->016: 15-05-2019 (JV). Normal operation, tuned power. 6s WOR. Updated port configs for power optimization.
 * 016->017: 30-05-2019 (JV). DEMO ONLY. Lower current via WOR.
 * 017->019: 11-06-2019 (JV). TEST ONLY. Cts packets.
 * 019->020: 12-06-2019 (JV). DEMO ONLY. 3pkts per signal only.
 *                            Tamper restore after every battery change, except on power-up.
 * 020->021: 31-07-2019 (JV). Normal code. Changed # sync bits to 32.
 */

#include <xc.h>
//#include <htc.h>
#include "config.h"
#include "cc1120_reg_config.h"
#include "batteryRead.h"
#include "transmit.h"
#include "spi.h"
#include "IO.h"

#define VERSION "017"
#define ALREADY_BEEN_POWERED    0x02        // eeprom address

/**************************************
 * FUNCTION PROTOTYPES
 *************************************/
static void startupBlink();
static void wakeup();

/**************************************
 * VARIABLES
 *************************************/
uint32_t SMOKE_ID = 0x00000000;     // only lower 20 bits are used



void __interrupt isr()
{
    if (IOCAFbits.IOCAF4)       // for detecting low batt, err
    {
        IOCAFbits.IOCAF4 = 0;
        tampIntOccurred = true;
    }
}



void main(void)
{
    /**********************************
     * INIT
     *********************************/
    WDTCONbits.SWDTEN = 0;
    WDTCONbits.WDTPS = 0b10010;     // 256s WDT
    WDTCONbits.SWDTEN = 1;
    
    initSmoke();
    initSPI();
    registerConfig();
    
    if (eeprom_read(ALREADY_BEEN_POWERED) != 0x57)
        eeprom_write(ALREADY_BEEN_POWERED, 0x57);
    else
        stillTampered = true;       // starts 20min restore timer
    startupBlink();
    /**********************************
     * END INIT
     *********************************/
 
    CLRWDT();
    WDTCONbits.SWDTEN = 0;
    WDTCONbits.WDTPS = 0b00111;     // 128ms WDT
    WDTCONbits.SWDTEN = 1;
    
//            IOCAPbits.IOCAP4 = 1;
//            RESET_NINTCONbits.IOCIE = 1;
//            INTCONbits.IOCIF = 0;
    /* Write Radio Regs */
    registerConfig();
    WPUBbits.WPUB7 = 1;
    WPUBbits.WPUB5 = 1;
    WPUCbits.WPUC1 = 1;
    /* Calibrate */
    manualCalibration();
    calibrateRCOsc();
    
    while((cc1120GetTxStatus() & 0x70) != 0)
    {
        NOP();
        cmdStrobe(CC1120_SIDLE);
        __delay_ms(5);
        CLRWDT();
    }
    __delay_ms(10);
    cmdStrobe(CC1120_SWORRST);
    __delay_ms(1);
    cmdStrobe(CC1120_SWOR);
//    cmdStrobe(CC1120_SPWD);
//    disableSPI();
    while(1)
    {
        
        CLRWDT();
        /* Check critical pins */
            checkTamper();
            checkAlarm();
            checkTest();
            
            /* Check if time for supervisory */
            checkForSupervisory();
            
            /* Verify we're not transmitting & shut down radio */
//            if (readyForSleep())
//            {
                WPUB4 = 1;
                SLEEP();
                NOP();
                WPUB4 = 0;
//                wakeup();
//            }
        if (systemTicks >= 0xFFFFFF00) systemTicks = 0xFFFFFF00;        // Juuuuuuust in case, prevent overflow
    }
}


void initSmoke(void)
{
    CLRWDT();
    SWDTEN = 0;                     /* Turn off WDT for now */
    
    OSCCONbits.SPLLEN = 0b0;        /* Disable PLL */
    OSCCONbits.SCS = 0b00;          /* CLK Source is set by config word */
    OSCCONbits.IRCF = 0b1101;       /* Internal OSC = 4MHz */
    
    OSCTUNE = 0;
    
    INTCON = 0;                     /* Disable all interrupts for now */

    OPTION_REGbits.nWPUEN = 0b0;    /* WPU are enabled by individ. WPU latch vals */
    OPTION_REGbits.T0SE = 0b0;      /* Timer0 increments on low-to-high edge of RA4 (alarm pin) */
    OPTION_REGbits.T0CS = 0b0;      /* Timer0 clk source = F_osc/4 */
    OPTION_REGbits.PS = 0b111;      /* Prescaler rate = 1:256 */
    OPTION_REGbits.PSA = 0b0;       /* Prescaler assigned to timer0 */
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
    
    OPTION_REG = 0b00000111;  // WPU are enabled by individ. WPU latch vals.
    APFCON0 = 0b00000000;   // RC4->USART TX , RC5->USART RX
    TRISA = 0b00011111;
    TRISB = 0b10110000;
    TRISC = 0b00000011;
    // **** Test, Alarm, Error pins need external pull-downs
    WPUA = 0x01;
    WPUB = 0xA0;
    WPUC = 0x03;
//    LATCbits.LATC0 = 1;
    
    /* Double-check IO polarity */
    GLED_INPUT = 0;
    RLED_INPUT = 0;
    JUMPER_INPUT = 1;
    GPIO0_INPUT = 1;
    GPIO2_INPUT = 1;
    GPIO3_INPUT = 1;
    ALARM_INPUT = 1;
    TAMPER_INPUT = 1;
    TESTPIN_INPUT = 1;
    BATDECT_INPUT = 0;    // detector circuit is NP
    
    TRXEM_SPI_END();
    
    
    JUMPER_PU = 1;
    nRESET_PU = 1;
    
    G_LED = 0;
    R_LED = 0;
    
    txPacket.TYPE = DEVICE_TYPE;         /* Smoke has device type 0x8 */
    txPacket.SERIAL_IDH = serialNumberH & 0x0F;
    txPacket.SERIAL_IDM = serialNumberM;
    txPacket.SERIAL_IDL = serialNumberL;
    txPacket.SUPERVISORY = 0;
    
    INTCONbits.GIE = 1;
    
    
    // enable the fixed voltage reference spec
    // this is a known voltage that we use to measure against
    // the battery voltage. This code just sets up the
    // reference voltage
    FVRCONbits.ADFVR = 0b01; // Fixed voltage ref is 1x or (1.024 V)
    FVRCONbits.CDAFVR = 0b00; // not using DAC and CPS
    FVRCONbits.TSRNG = 0b0; // not using temp indicator
    FVRCONbits.TSEN = 0b0;  // temp sensor disabled
}


void checkForSupervisory()
{
    
    if (systemTicks++ >= SupervisoryTimer)
    {
        systemTicks = 0;
//        storeLowBatt();
//        batteryTest();
        // don't transmit message if not in supervisory mode
        if (isSupervisoryMode)
        {
            transmit(supervisory);
        }
    }
}


void assignSerial()         // Assigns Serial ID and device type for every transmission
{
    txPacket.TYPE = DEVICE_TYPE;         /* Smoke has device type ID 0x8 */
    txPacket.SERIAL_IDH = serialNumberH & 0x0F;
    txPacket.SERIAL_IDM = serialNumberM;
    txPacket.SERIAL_IDL = serialNumberL;
}



extern void sendAck(void)
{
    CLRWDT();
    // Initialize packet buffer of size PKTLEN + 1
    unsigned char txBuffer[1] = {0};
    
    // Calibrate radio according to errata
    manualCalibration();
    CLRWDT();
    // Create a packet for ACK transmission (consists of short preamble, 16bit sync, single byte ACK)
    createAckPacket(txBuffer);
    cc1120SpiWriteTxFifo(txBuffer, sizeof(txBuffer));
    // Strobe TX to send packet
    cmdStrobe(CC1120_STX);
    __delay_ms(20);
    cmdStrobe(CC1120_SFTX);
    __delay_ms(10);
    /* Write Radio Regs */
    registerConfig();
    
    /* Calibrate */
    manualCalibration();
    calibrateRCOsc();
    cmdStrobe(CC1120_SIDLE);
    __delay_ms(10);
    /* Start WOR */
    cmdStrobe(CC1120_SPWD);
    CLRWDT();
}



bool readyForSleep()
{
    CLRWDT();
    WPUB4 = 0;
    while((cc1120GetTxStatus() & 0x70) != 0)
    {
        NOP();
        cmdStrobe(CC1120_SIDLE);
        __delay_ms(5);
    }
//    if((getTXStatus() & 0x70) != 0)
//    {
//        return false;
//    }
    cmdStrobe(CC1120_SWORRST);
    __delay_ms(5);
    cmdStrobe(CC1120_SWOR);     // Put CC115L to sleep
    __delay_us(50);
    WPUB4 = 1;
    return true;
}


static void startupBlink()
{
    GREEN_ON();
    RED_ON();
    __delay_ms(500);
    GREEN_OFF();
    RED_OFF();
//    __delay_ms(8000);
}