/*
 Calibrate the OSCCAL value on the UART of the Bluetooth controller without using additional wiring (ICP1 would have been helpful)
 This compensates the following timing errors:
 1: the baud rate error (which is -3.5% at 115200 baud)
 2: the oscillator speed
 
 The oscillator speed adapts to the serial speed of the communication partner, so that the ATmega matches the baud rate
 of its partner. Only tolerances of around +-10% of the original RC oscillator can be compensated but for tuning
 this is normally enough.
 
 Instructions to use:
 1. Adapt the following values in the sketch:
    ARDUINO_SERIAL_BAUDRATE (only needed when you are not using my Bluecontroller Arduino Environment adaption)
 2. Start the sketch and send either the character 'D' (0x44 => 3*0 bit time at the beginning) or 'B' (0x42 => 2*0 bit time at the beginning).
    Keep on sending as long as the OSCCAL values are changing. Using 'D' will be a bit more accurate.
    When the OSCCAL values stabilizes after some time, the calibration is finished.
 3. Using the characters '+' and '-' you can manually change OSCCAL
 4. Use the printed value of 'g_medianOSCCAL' to set -DOSCCAL_VALUE=xxx' in the BlueController optiboot Makefile. This will permanently set the
    OSCCAL value of the boot loader and the value will also be active when the boot loader is exited and the main program runs
 
 Copyright (c) 2013 by Michael Dreher <michael@5dot1.de>
 This example code is in the public domain.
 */

#include <avr/wdt.h>

// Pin 13 has an LED connected on most Arduino boards.
const uint8_t led = LED_BUILTIN;

#if 0 // use fixed values or calculate the at runtime
// these constants are for 115200 baud with a oscillator freq. of 8MHz
#define SER_DBLSPEED (1) // 0 or 1
#define SER_UBRR (8)
#else
// dynamically calculate the values, the code will be ~1300 byte larger because it uses 32 bit calculations.
#define SER_DBLSPEED ((UCSR0A >> U2X0) & 0x01) // 0 or 1
#define SER_UBRR (UBRR0)
#endif

// this calculates the real baudrate (incl. error), not the baud rate which is used as parameter for Serial.begin()
#define SER_MUL ((2 - (SER_DBLSPEED ? 1 : 0)) * 8)
#define SERIALBAUDRATE (F_CPU / (SER_MUL * (SER_UBRR + 1)))
#define SER_BIT_TIME(bits) ((((2 * bits * F_CPU) / SERIALBAUDRATE) + 1) / 2) // round up

// the tolarance used for comparing bit times (+-1/10=+-10%)
const uint8_t tolerance = 1;
const uint8_t tolerancebase = 10;
    
const uint8_t g_origOSCCAL = OSCCAL;
const uint8_t medianDiv = 4;
uint8_t g_medianOSCCAL = OSCCAL;

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

////////////////////////////////////////////////////////////////////////////////
// Helper macros
////////////////////////////////////////////////////////////////////////////////
#define STRINGIFY(sym) #sym
#define TRACEVAR(x) \
  Serial.print(STRINGIFY(x) "="); \
  Serial.println(x);

#define TRACENAMEVAR(name, value) \
  Serial.print(name "="); \
  Serial.println(value);

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit 
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit
#define __CAT2(base, portname) base##portname // internally needed by CAT2
#define CAT2(prefix, num) __CAT2(prefix, num) // build a define name from 2 params
#define __CAT3(prefix, num, postfix) prefix##num##postfix // internally needed by CAT3
#define CAT3(prefix, num, postfix) __CAT3(prefix, num, postfix) // build a define name from 3 params

// the setup routine runs once when you press reset:
void setup() {
  // ARDUINO_SERIAL_BAUDRATE is either defined in .../hardware/<boardname>/variants/<variant>/pins_arduino.h
  // or you have to set it manually here when your board has no fixed baud rate
  Serial.begin(ARDUINO_SERIAL_BAUDRATE);
  g_BitTime1 = SER_BIT_TIME(1);
  g_BitTime1Min = (g_BitTime1 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime1Max = (g_BitTime1 * (tolerancebase + tolerance)) / tolerancebase;     

  g_BitTime2 = SER_BIT_TIME(2);
  g_BitTime2Min = (g_BitTime2 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime2Max = (g_BitTime2 * (tolerancebase + tolerance)) / tolerancebase;     

  g_BitTime3 = SER_BIT_TIME(3);
  g_BitTime3Min = (g_BitTime3 * (tolerancebase - tolerance)) / tolerancebase;     
  g_BitTime3Max = (g_BitTime3 * (tolerancebase + tolerance)) / tolerancebase;     

  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  g_OSCCAL_lbound = ((uint16_t)g_origOSCCAL * 9) / 10;
  g_OSCCAL_ubound = ((uint16_t)g_origOSCCAL * 11) / 10;

  if(g_OSCCAL_ubound <= g_origOSCCAL) // overflow
    g_OSCCAL_ubound = 255;
  
  if(g_OSCCAL_lbound < 1L) // overflow
    g_OSCCAL_lbound = 1;
  
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

  TRACENAMEVAR("F_CPU", F_CPU);
  TRACENAMEVAR("SERIALBAUDRATE", SERIALBAUDRATE);
  TRACEVAR(g_BitTime1);
  TRACEVAR(g_BitTime1Min);
  TRACEVAR(g_BitTime1Max);
  TRACEVAR(g_BitTime2);
  TRACEVAR(g_BitTime2Min);
  TRACEVAR(g_BitTime2Max);
  TRACEVAR(g_BitTime3);
  TRACEVAR(g_BitTime3Min);
  TRACEVAR(g_BitTime3Max);
  TRACEVAR(g_OSCCAL_lbound);
  TRACEVAR(g_OSCCAL_ubound);

  InitTimerPcintIrq();
}

// the loop routine runs over and over again forever:
void loop() {
#if 1
  static int16_t prevTdiff;
  if(g_tDiff != prevTdiff)
  {
    Serial.print("g_tDiff=");
    Serial.println(g_tDiff);
    prevTdiff = g_tDiff;
  }
#endif

  static uint8_t prevOSCCAL;
  if(OSCCAL != prevOSCCAL)
  {
    printOSCCAL();
    prevOSCCAL = OSCCAL;
  }

  //bool newLine = false;
  if(Serial.available())
  //while(Serial.available())
  {
    unsigned char c = Serial.read();
    if(c=='r')
    {
      Serial.println("Software reset");
      soft_reset();
    }
    else if(c=='b')
    {
      Serial.println("Entering boot loader");
      enter_bootloader();
    }
    else if(c=='+' && OSCCAL < 255)
    {
      OSCCAL++;
      printOSCCAL();
    }
    else if(c=='-' && OSCCAL > 0)
    {
      OSCCAL--;
      printOSCCAL();
    }
    else if(c=='=')
    {
      printOSCCAL();
    }
    else
    {
      Serial.write(c);
      Serial.write(' ');
      Serial.print(c, HEX);
      Serial.write(' ');
      Serial.print(c, BIN);
      Serial.println(" 01"); // start bit and state before start bit (previous stop bit or silence)
    }
  }
}


void printOSCCAL(void)
{
  // these two values are used for checking the baud rate with an oscilloscope or logic analyzer
  Serial.write(0x55);
  Serial.write(0x00);

  Serial.print("  OSCCAL=");
  Serial.print(OSCCAL);
  Serial.print("  g_medianOSCCAL=");
  Serial.println(g_medianOSCCAL);
}


void InitTimerPcintIrq(void)
{
  // enable pin change IRQ
  unsigned char sreg = SREG;
  cli();
  sbi(PCMSK2, PCINT16);
  sbi(PCICR, PCIE2);

  TCNT1 = 0;
  OCR1A = TCNT1 - 1;
  TIFR1 = _BV(OCF1A); 
#if USE_OVL_ISR
  TIMSK1 = _BV(OCIE1A); // enable output compare match IRQ
#endif

  TCCR1A = 0x00;
  TCCR1B = _BV(CS10);
  SREG = sreg;
}

inline void SetOSCCAL(int8_t offset)
{
  int16_t newOSCCAL = OSCCAL + offset;
  if((newOSCCAL >= g_OSCCAL_lbound) && (newOSCCAL <= g_OSCCAL_ubound))
  {
    OSCCAL = newOSCCAL;
    g_medianOSCCAL = ((uint16_t)g_medianOSCCAL * (medianDiv - 1) + newOSCCAL + (medianDiv / 2)) / medianDiv;
  }
}

inline void AdaptOSCCAL(int16_t clkDeviation, uint8_t numberOfBits)
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

inline void StorePrevTime(uint16_t t)
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


#if USE_OVL_ISR
ISR(TIMER1_COMPA_vect)
{
  g_timerOvl = true;
}
#endif

#define ENTER_BL_MAGIC_VAL (0xc49e);
#define ENTER_BL_MAGIC_ADR (*((volatile uint16_t*)0x100))
void enter_bootloader(void)
{
  noInterrupts();
  //resetBtm222();
  ENTER_BL_MAGIC_ADR = ENTER_BL_MAGIC_VAL;
  wdt_enable(WDTO_15MS);
  for(;;) {
  }
}

void soft_reset(void)
{
  noInterrupts();
  ENTER_BL_MAGIC_ADR = 0;
  wdt_enable(WDTO_15MS);
  for(;;) {
  }
}


