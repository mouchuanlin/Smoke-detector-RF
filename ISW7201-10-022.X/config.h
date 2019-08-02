/* 
 * File:   config.h
 * Author: Scott
 *
 * Created on November 14, 2017, 2:49 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

// CONFIG1
#pragma config FOSC = INTOSC       // Oscillator Selection (ECH, External Clock, High Power Mode (4-32 MHz): device clock supplied to CLKIN pin)
#pragma config WDTE = SWDTEN        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config CPD = ON         // Data Memory Code Protection (Data memory code protection is enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ  4000000
#define BAUD        9600
#define SDI_PIN     TRISB4
#define SDO_PIN     TRISC7
//#define JUMPER    RC4

#define WATCHDOG_SLEEP_256ms    0b01000
#define WATCHDOG_SLEEP_512ms    0b01001
#define WATCHDOG_SLEEP_1S       0b01010
#define WATCHDOG_SLEEP_2S       0b01011
#define WATCHDOG_SLEEP_4S       0b01100
#define WATCHDOG_SLEEP_8S       0b01101
#define WATCHDOG_SLEEP_16S      0b01110
#define WATCHDOG_SLEEP_32S      0b01111
#define WATCHDOG_SLEEP_64S      0b10000
#define WATCHDOG_SLEEP_128S     0b10001
#define WATCHDOG_SLEEP_256S     0b10010


#define G_LED           LATCbits.LATC2
#define R_LED           LATCbits.LATC3
#define SW_1            RA0                 // Active-low
#define GPIO0           RB5
#define GPIO2           RC1
#define GPIO3           RB7

#define nTEST_JUMPER    RA0
#define ALARM           RA2
#define TAMPER          RA4
#define TEST_PIN        RA1
#define LBATT_DETECT    RC4

#define GLED_INPUT      TRISC2
#define RLED_INPUT      TRISC3
#define JUMPER_INPUT    TRISA0
#define GPIO0_INPUT     TRISB5
#define GPIO2_INPUT     TRISC1
#define GPIO3_INPUT     TRISB7
#define ALARM_INPUT     TRISA2
#define TAMPER_INPUT    TRISA4
#define TESTPIN_INPUT   TRISA1
#define BATDECT_INPUT   TRISC4
#define nRESET_INPUT    TRISC0

#define JUMPER_PU       WPUA0
#define nRESET_PU       WPUC0

#define TEST_PIN_FLAG   IOCAF1
#define ALARM_PIN_FLAG  IOCAF2
#define TAMP_PIN_FLAG   IOCAF4

#define IDH             0x00
#define IDM             0x02
#define IDL             0x04

#define _20MIN          9372
#define _5MIN           2343


#define SupervisoryTimer        675000//1 day supervisory, based on 128ms WDT
#define CRC16                   0x8005
#define DEVICE_TYPE             0b1000

/*******************************************
 * CUSTOM FUNCTION DEFINES
 ******************************************/
#define EN_TEST_INTH()          IOCAP1=1        // Enable interrupt on TEST pin change low-to-high
#define DISABLE_TEST_INTH()     IOCAP1=0        // Disable above interrupt
#define EN_ALARM_INTH()         IOCAP2=1        // Enable interrupt on ALARM pin change lo-hi
#define DISABLE_ALARM_INTH()    IOCAP2=0        // Disable above interrupt
#define EN_TAMP_INTH()          IOCAP4=1        // Enable interrupt on ERROR pin change lo-hi
#define DISABLE_TAMP_INTH()     IOCAP4=0        // Disable above interrupt
#define EN_TAMP_INTL()          IOCAN4=1
#define DISABLE_TAMP_INTL()     IOCAN4=0
#define GREEN_ON()              G_LED=1
#define GREEN_OFF()             G_LED=0
#define RED_ON()                R_LED=1
#define RED_OFF()               R_LED=0


/*******************************************
 * STRUCTS
 ******************************************/
struct PacketData {                     // in LSB -> MSB order
    unsigned TRIGGER            : 1;
    unsigned TAMPER_p           : 1;
    unsigned TEST               : 1;
    unsigned unused1            : 1;
    unsigned LOW_BATT           : 1;
    unsigned unused2            : 1;
    unsigned ERR                : 1;
//    unsigned TEST               : 1;    // OPTIONAL
    unsigned SUPERVISORY        : 1;
    unsigned SERIAL_IDL         : 8;
    unsigned SERIAL_IDM         : 8;
    unsigned SERIAL_IDH         : 4;
    unsigned TYPE               : 4;    // Alarm type (flood/smoke/glass/etc.)
};


/*******************************************
 * INCLUDES
 ******************************************/
#include <stdint.h>
#include <stdbool.h>

/*******************************************
 * PROTOTYPES
 ******************************************/
void initSmoke();
void registerConfig (void);
void cfgInterrupts();
extern uint8_t isLowBattery();
void checkForSupervisory();
void storeLowBatt();
void assignSerial();
void appendCRC(uint8_t txBuf[]);
bool readyForSleep();


/*******************************************
 * VARIABLES
 ******************************************/
    struct PacketData txPacket;
    unsigned short packetCounter;
    uint8_t lowBattCount = 0;
    uint16_t preambleTimespan = 6000;
    uint32_t systemTicks = 0;
    bool waitingFor20s = false;
    bool low_bat_flag = false;


const uint8_t serialNumberL @ 0x0020 = 0x31;
const uint8_t serialNumberM @ 0x0021 = 0x2C;
const uint8_t serialNumberH @ 0x0022 = 0x89;

#endif	/* CONFIG_H */