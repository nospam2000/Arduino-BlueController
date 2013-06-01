 /*
 Copyright (c) 2013 by Michael Dreher <michael@5dot1.de>
 This example code is in the public domain.
 */
#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Public API
void InitOSCCALCalibration(void);

////////////////////////////////////////////////////////////////////////////////
// Public global variables (can be read for debugging purposes)
extern uint8_t g_medianOSCCAL; // this is the result of the calibration
extern int16_t g_BitTime1;
extern int16_t g_BitTime1Min;
extern int16_t g_BitTime1Max;
extern int16_t g_BitTime2;
extern int16_t g_BitTime2Min;
extern int16_t g_BitTime2Max;
extern int16_t g_BitTime3;
extern int16_t g_BitTime3Min;
extern int16_t g_BitTime3Max;
extern uint8_t g_OSCCAL_lbound;
extern uint8_t g_OSCCAL_ubound;
extern int16_t g_tDiff;


#if 1 // use fixed values to calculate the bit clocks at compile time or calculate the bit clocks at runtime
// dynamically calculate the values from UBRR and U2X0, the code will be betwenn 500 and 1300 byte larger because it uses 32 bit calculations.
#define SER_DBLSPEED ((UCSR0A >> U2X0) & 0x01) // 0 or 1
#define SER_UBRR (UBRR0)
#else
// these constants are for 115200 baud with a oscillator freq. of 8MHz
#define SER_DBLSPEED (1) // 0 or 1
#define SER_UBRR (8)
#endif

// this calculates the real baudrate (incl. error), not the baud rate which is used as parameter for Serial.begin()
#define SER_MUL ((2 - (SER_DBLSPEED ? 1 : 0)) * 8)
#define SERIALBAUDRATE_FROM_UBRR (F_CPU / (SER_MUL * (SER_UBRR + 1)))
#define SER_BIT_TIME(bits) (bits * (SER_MUL * (SER_UBRR + 1)))
#define F_CPU_CALIBRATED(baudrate) (baudrate * (SER_MUL * (SER_UBRR + 1)))

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


