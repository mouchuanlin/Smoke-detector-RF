/* 
 * Engages the peripherals of the smoke detector: checks for statuses of alarm, test mode
 * (via test button), and tamper.
 */
#include "config.h"
#include "IO.h"
#include "transmit.h"

bool sentErr = false;
uint16_t tamperOpenTimerCnt = 0;
bool powerOnTamper = true;
uint16_t cnt = 100;

void checkAlarm() 
{
    /* Once it goes off, checks alarm and re-sends alarm signal with every supervisory */
    if (ALARM && !stillTriggered && !TEST_PIN)
    {
        CLRWDT();
        __delay_ms(50);
        if (ALARM && !TEST_PIN)
        {
            transmit(alarm);
            stillTriggered = true;
            
            // assume 6 seconds
            systemTicks += 6 * 50;
        }
    }
    else if (!ALARM && stillTriggered)
        checkToResetAlarm();
}


void checkTamper()
{
    if (powerOnTamper && !TAMPER)       // ignore tamper on boot
    {
        CLRWDT();
//        if (!TAMPER && cnt-- > 0)       // buffer the tamper switch on initial boot by 10s
//        {                               // based on 128ms WDT
//            __delay_ms(1);
//            CLRWDT();
//        }
        cnt--;                  // buffer tamper sw on initial boot by 10s
                                // based on 128ms WDT
        if (!TAMPER && cnt <= 0)
        {
            powerOnTamper = false;
            IOCAPbits.IOCAP4 = 1;
            INTCONbits.IOCIE = 1;
            INTCONbits.IOCIF = 0;
        }
    }
    else if (powerOnTamper && TAMPER)
    {
        cnt = 100;
        IOCAPbits.IOCAP4 = 0;
        INTCONbits.IOCIE = 0;
        INTCONbits.IOCIF = 0;
        tampIntOccurred = false;
    }
    
    
    if (tampIntOccurred && TAMPER && !powerOnTamper)// && !stillTampered)         // low batt and ERR detection.
    {
        tampIntOccurred = false;
        CLRWDT();
        __delay_ms(2);
        if (TAMPER)
        {
            if (tampIntCnt == 0)
            {
                _43sTmrCnt = 0;
                _43sTmrOn = true;
            }
            tampIntCnt++;
        }
    }
    if (_43sTmrOn)// && !stillTampered)
    {
        if (_43sTmrCnt++ >= _43s)
        {
            _43sTmrDone = true;
        }
        if (_43sTmrDone)
        {
            _43sTmrDone = false;
            if (tampIntCnt == 1)    // Low batt from smoke
            {
//                LED = (consecutiveLowBattCnt % 2 == 0);
                _43sTmrOn = false;
                tampIntCnt = 0;
                if (!isLowBattery())
                    consecutiveLowBattCnt++;
            }
            else if (tampIntCnt == 3)   // Err from smoke
            {
                _43sTmrOn = false;
                tampIntCnt = 0;
                if (!sentErr)
                    errRepeatCnt++;
            }
            else
            {
                _43sTmrOn = false;
                tampIntCnt = 0;
                consecutiveLowBattCnt = 0;
                errRepeatCnt = 0;
            }
            if (consecutiveLowBattCnt >= REPEAT_CNT)
            {
                if (!low_bat_flag)
                {
                    low_bat_flag = true;    // low batt flag only reset upon device power-cycle
                    transmit(lowBatt);      // add lowBatt to transmission
                }
            }
            if (errRepeatCnt >= REPEAT_CNT && !sentErr)
            {
                sentErr = true;         // latch
                transmit(err);          // add err to transmission
            }
            
        }
    }
//    __delay_ms(30);     // don't count the pulse above as a tamper signal
    if(TAMPER && !stillTampered && !powerOnTamper)
    {
        /* Once it goes off, checks tamper and re-sends tamper signal with every supervisory */
        CLRWDT();
        __delay_ms(50);
        CLRWDT();
        
        if (TAMPER)
        {
//            blinkLED();
            transmit(tamper);
            stillTampered = true;
            tampIntCnt = 0;
            errRepeatCnt = 0;
            consecutiveLowBattCnt = 0;
            _43sTmrOn = false;              // turn off 43s timer if it's a long pulse; reset counters
            _20minTimerCnt = 0;
        }
    }
    else if (TAMPER && stillTampered)
    {
        __delay_ms(30);         // wait past the longest (20ms) pulse expected from mother
        if (TAMPER)             // then check the port
        {
//            blinkLED();   // only on tamper edge
            tampIntCnt = 0;
            errRepeatCnt = 0;
            consecutiveLowBattCnt = 0;
            _43sTmrOn = false;
            _20minTimerCnt = 0;
            tamperOpenTimerCnt++;
            if (tamperOpenTimerCnt == 1)
                blinkLED();
        }
        else
            checkToResetTamper();   // this allows the unit to restore tamper even with low batt
    }

    else if (!TAMPER && stillTampered)
        checkToResetTamper();
}



void checkTest()
{
    
    if(TEST_PIN && !stillTesting)
    {
        /* Once it goes off, checks tamper and re-sends tamper signal with every supervisory */
        CLRWDT();
        __delay_ms(50);
        CLRWDT();
        
        if (TEST_PIN)
        {
            transmit(test);
            stillTesting = true;
            
            // assume 6 seconds
            systemTicks += 6 * 50;
        }
    }

    else if (!TEST_PIN)
        checkToResetTest();
}

void checkToResetAlarm()
{
    if (stillTriggered && !ALARM)
    {
        CLRWDT();
        __delay_ms(50);
        CLRWDT();
    
        if (!ALARM)
            stillTriggered = false;
    }
}


void checkToResetTamper()
{
//    if (_20minTimerCnt % 30 == 0)
//        LED_OFF();
//    else if (_20minTimerCnt % 30 == 23)     // slow status blink
//        LED_ON();
    if (stillTampered && !TAMPER)
    {
        CLRWDT();
        _20minTimerCnt++;
        tamperOpenTimerCnt = 0;
        if (_20minTimerCnt == 1)        // blink LED on tamper close
            blinkLED();
        if (_20minTimerCnt >= _20MIN)
        {
            GREEN_OFF();
            RED_OFF();
            _20minTimerCnt = 0;     // reset tamper after 20min of being "restored"
            stillTampered = false;
            transmit(restore);
        }
    
    }
}

void checkToResetTest()
{
    if (stillTesting && !TEST_PIN)
    {
        CLRWDT();
        __delay_ms(50);
    
        if (!TEST_PIN)
            stillTesting = false;
    }
}


void blinkLED()
{
    CLRWDT();
    for(uint8_t i = 0; i < 1; i ++)
    {
        if (low_bat_flag)
            RED_ON();
        else
            GREEN_ON();
        __delay_ms(100);
        CLRWDT();
        __delay_ms(100);
        RED_OFF();
        GREEN_OFF();
        CLRWDT();
        __delay_ms(100);
        CLRWDT();
        __delay_ms(100);
        CLRWDT();
//        __delay_ms(100);
//        CLRWDT();
//        __delay_ms(100);
//        CLRWDT();
    }
}