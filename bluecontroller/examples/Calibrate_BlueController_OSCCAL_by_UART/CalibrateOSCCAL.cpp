 /*
 Copyright (c) 2013 by Michael Dreher <michael@5dot1.de>
 This example code is in the public domain.
 */

#include <Arduino.h>
#include "CalibrateOSCCAL.h"

////////////////////////////////////////////////////////////////////////////////
// global variables
////////////////////////////////////////////////////////////////////////////////
// the tolerance used for comparing bit times (+-1/10=+-10%)
const uint8_t tolerance = 1;
const uint8_t tolerancebase = 10;
    
uint8_t g_origOSCCAL;
const uint8_t medianDiv = 4;
uint8_t g_medianOSCCAL;

uint8_t g_OSCCAL_lbound;
uint8_t g_OSCCAL_ubound;
bool g_waitForTimeout = true;
int16_t g_tDiff; // time difference from previous ISR call
uint16_t g_prevEvent = 0;

int16_t g_BitTime1;
int16_t g_BitTime1Min;
int16_t g_BitTime1Max;
int16_t g_BitTime2;
int16_t g_BitTime2Min;
int16_t g_BitTime2Max;
int16_t g_BitTime3;
int16_t g_BitTime3Min;
int16_t g_BitTime3Max;


static inline void InitTimerPcintIrq(void)
{
  // enable pin change IRQ
  unsigned char sreg = SREG;
  cli();
  sbi(PCMSK2, PCINT16);
  sbi(PCICR, PCIE2);

  TCNT1 = 0;
  OCR1A = TCNT1 - 1;
  TIFR1 = _BV(OCF1A); 

  TCCR1A = 0x00;
  TCCR1B = _BV(CS10);
  SREG = sreg;
}

void InitOSCCALCalibration(void)
{
  g_origOSCCAL = OSCCAL;
  g_medianOSCCAL = OSCCAL;
  
  g_BitTime1 = SER_BIT_TIME(1);
  g_BitTime1Min = (g_BitTime1 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime1Max = (g_BitTime1 * (tolerancebase + tolerance)) / tolerancebase;     

  g_BitTime2 = SER_BIT_TIME(2);
  g_BitTime2Min = (g_BitTime2 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime2Max = (g_BitTime2 * (tolerancebase + tolerance)) / tolerancebase;     

  g_BitTime3 = SER_BIT_TIME(3);
  g_BitTime3Min = (g_BitTime3 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime3Max = (g_BitTime3 * (tolerancebase + tolerance)) / tolerancebase;     

  g_OSCCAL_lbound = ((uint16_t)g_origOSCCAL * 9) / 10;
  g_OSCCAL_ubound = ((uint16_t)g_origOSCCAL * 11) / 10;

  if(g_OSCCAL_ubound <= g_origOSCCAL) // overflow
    g_OSCCAL_ubound = 255;
  
  // stay in same segment as original OSCCAL, TODO: use both segments
  if(g_origOSCCAL >= 128)
  {
    if(g_OSCCAL_lbound < 128)
      g_OSCCAL_lbound = 128;
  }
  else
  {
    if(g_OSCCAL_ubound > 127)
      g_OSCCAL_ubound = 127;
  }

  InitTimerPcintIrq();
}

static inline void SetOSCCAL(int8_t offset)
{
  int16_t newOSCCAL = OSCCAL + offset;
  if((newOSCCAL >= g_OSCCAL_lbound) && (newOSCCAL <= g_OSCCAL_ubound))
  {
    OSCCAL = newOSCCAL;
    g_medianOSCCAL = ((uint16_t)g_medianOSCCAL * (medianDiv - 1) + newOSCCAL + (medianDiv / 2)) / medianDiv;
  }
}

static inline void AdaptOSCCAL(int16_t clkDeviation, uint8_t numberOfBits)
{
  // actually 'clkDeviation' has to be divided by 'numberOfBits', to get the number of cycles per bits
  // but for the algorithm currently used this would only waste CPU time
#if 0
  if(clkDeviation > 20)
    SetOSCCAL(-4);
  else if(clkDeviation < -20)
    SetOSCCAL(4);
  else
#endif
  if(clkDeviation > 0)
    SetOSCCAL(-1);
  else if(clkDeviation < 0)
    SetOSCCAL(1);
}

static inline void StorePrevTime(uint16_t t)
{
  g_prevEvent = t;
  OCR1A = t;
  TIFR1 = _BV(OCF1A); // overflow might be set, so clear it here
}

// PD0, USART RxD, PCINT16
ISR(PCINT2_vect)
{
  uint16_t t = TCNT1;
  
  if(g_waitForTimeout)
  {
    // wait for timer compare value reached (some delay until OSCCAL has stabilized)
    if(TIFR1 & _BV(OCF1A))
      g_waitForTimeout = false;
  }
  else
  {
    if(TIFR1 & _BV(OCF1A))
    {
      // ignore the time difference
    }
    else
    {
      //uint16_t tDiff = t - g_prevEvent;
      g_tDiff = t - g_prevEvent;
  
      // The time of a single bit is too short because this ISR takes too much time
      // For times longer than 3 bits, the time error would be too high: With a +-20% tolerance
      // it could not be distinguished between 4 short bits (4*0.8=3.2) and 3 long bits (3*1.2=3.6)
      // Therefore we use only 2 and 3 bit times.      
      if((g_tDiff >= g_BitTime3Min) && (g_tDiff <= g_BitTime3Max))
      {
        AdaptOSCCAL(g_tDiff - g_BitTime3, 3); // the difference to the optimum frequency
      }
      else if((g_tDiff >= g_BitTime2Min) && (g_tDiff <= g_BitTime2Max))
      {
        AdaptOSCCAL(g_tDiff - g_BitTime2, 2); // the difference to the optimum frequency
      }
      
      g_waitForTimeout = true;
    }
  }

  StorePrevTime(t);

  // it is ok to miss one IRQ, but it's not good when it is served too late
  // and the time diff is not correct, so clear an already pendig IRQ here
  PCIFR = _BV(PCIF2);
}


