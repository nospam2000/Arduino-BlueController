/*
 Calibrate the OSCCAL value to match the baud rate of the communication partner as best as possible

 What does this project do?
   The internal RC oscillator speed is adapted to the serial speed of the communication partner (e.g. a USB or blueetooth
   to serial converter), so that the ATmega matches the baud rate of its partner. Only deviations of +-10% of the
   original RC oscillator can be compensated but normally this is enough.

 Why you might need this project?
   When your ATmega is clocked using the internal RC oscillation and you want to use baud rates >38400 bps
   or when the factory calibration of OSCCAL is bad and you want to do your own calibration without additional
   hardware (e.g. STK500 programmer).
   
 Can you give me an example?
   Assume you want to use an internal RC clock rate of 8Mhz and a baud rate of 115200 bps. 
   The UART baud rate cannot be exactly set to 115200, the best approximation can be achieved
   by using the divider value UBRRn=8 and U2Xn=1 (double speed mode) which gives (8MHz / (8*(8+1)) = 111111 bps.
   This alone gives an error of 111111 / 115200 = -3.5% which might already be too much.
   According to http://www.maxim-ic.com/appnotes.cfm/an_pk/2141 the maximum tolerable error is 3.3%.
  
   Additionally the mcu clock rate might be up to 10% off the nominal clock value (e.g. 7.2 Mhz instead of 8MHz)
   so instead of 111111 bps you might get 111111 bps * 0.9 = 100000 bps.
   The baud rate error is then 100000 / 115200 = -13.5% which makes UART communication impossible.

 Are there other solutions?
   Yes:
    - Use slower baud rates. Typical baud rates up to 38400 bps (with F_CPU=8MHz) give an error smaller than 0.2%. 
    - Use an external osciallator which is a multiple of the baud rate (baud rate oscillator, e.g. 7,3728 or 14,7456 MHz http://www.mikrocontroller.net/articles/Baudratenquarz )
    - Use other ways of OSCCAL calibration (use STK500; read Atmel document AVR054; use Logic Analyzer or Oscilloscope)
   
 Why should I use this project instead of other solutions?
   - This project neither requires additional hardware (like a logic analyzer, oscilloscope or special programmer like STK500)
      and doesn't need hardware modifications of your board like additional wiring.
   - It is free.

 What are the the disadvantages when using this project?
  - Your mcu runs with a changed clock (not at F_CPU). A perfect calibration for the 8MHz/115200 bps example above would give 
    a real clock rate of 115200 * (8*(8+1)) = 8.2944 MHz (depending on the UART clock error of your communication partner).
    For time calculations, you should use this value instead of F_CPU.
  - I only tested it with 115200 bps and 8 MHz.

 How does it work?
  The bit-time of the UART transmission is measured and compared to the setpoint value. The OSCCAL is changed

 Can the solution only be used for a one time calibration or can I use it to run always in the background?
   It could be used to run always in the background but I suggest some code changes if you plan to do this.
   With the current code, the drawback is that every received byte triggers up to 10 IRQs which
   will consume a lot of CPU time. Better would be to enable it only for some time and disable it, after the
   calibration has finished.

 Instructions to use:
  1. Adapt the baud rate of Serial.begin() in the sketch source code
  2. Upload the sketch and let the sketch run for some 10 minutes under the temperature and voltage conditions it will be used in future.
     For normal room temperature (20 to 25 °C) this is not so critical, but when you want to use it in a closed cabinet
     at temperatures >40 or <5 °C this ensures better calibration.
  3. Connect to your board using the Arduino serial monitor or any other terminal program
  4. In the serial monitor send the following characters (one by one with a pause of at least 5m in between, not as one long string):
      'D' (hex 0x44, bin 01000100 => 2*0 bit time + start bit at the beginning)
      'B' (hex 0x42, bin 01000010 => 1*0 bit time + start bit at the beginning).
     Keep on sending the characters as long as the OSCCAL values are changing in the same direction, normally 15 charaters are enough.
     When the OSCCAL values stabilizes after sending some characters and jumps only back and forth by 1, the calibration is finished.
  5. Use the printed value of 'g_medianOSCCAL' to set '-DOSCCAL_VALUE=xxx' in the BlueController optiboot Makefile
     (replace xxx with the value of g_medianOSCCAL). This will permanently set the OSCCAL value of the boot loader
     and the value will also be active when the boot loader is exited and the main program runs
     You have to use the special version of the optiboot from here:
     https://code.google.com/r/michaeldreher42-bluecontroller/source/browse/#hg%2Fbluecontroller%2Fbootloaders%2Foptiboot    

   Sending the characters '+' and '-' you can manually change OSCCAL and watch the effect on g_tDiff.
   The whole calibration runs in the ISR, the main loop only prints the current values of the calibration.
 
 Copyright (c) 2013 by Michael Dreher <michael@5dot1.de>
 This example code is in the public domain.
 */

#include "CalibrateOSCCAL.h"

// Pin 13 has an LED connected on most Arduino boards.
const uint8_t led = LED_BUILTIN;

// the setup routine runs once when you press reset:
void setup() {
  // ARDUINO_SERIAL_BAUDRATE is either defined in .../Arduino/hardware/<boardname>/variants/<variant>/pins_arduino.h
  // or you have to set it manually here when your board has no fixed baud rate defined there
  // currently only the BlueController boards define this constant
  Serial.begin(ARDUINO_SERIAL_BAUDRATE);
  pinMode(led, OUTPUT);

  InitOSCCALCalibration(); // must be called _after_ Serial.begin()

  TRACENAMEVAR("F_CPU", F_CPU);
  TRACENAMEVAR("Nominal baud rate", SERIALBAUDRATE);
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
}

// the loop routine runs over and over again forever:
void loop() {
  static int16_t prevTdiff;
  if(g_tDiff != prevTdiff)
  {
    Serial.print("g_tDiff=");
    Serial.println(g_tDiff);
    prevTdiff = g_tDiff;
  }

  static uint8_t prevOSCCAL;
  if(OSCCAL != prevOSCCAL)
  {
    printOSCCAL();
    prevOSCCAL = OSCCAL;
  }

  if(Serial.available())
  {
    unsigned char c = Serial.read();
    if(c=='+' && OSCCAL < 255)
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
  // these values are useful for checking the baud rate with an oscilloscope or logic analyzer
  Serial.write(0x55); // logic levels on the TXD line: 10 10101010 1
  Serial.write(0x00); // logic levels on the TXD line:  0 00000000 1

  Serial.print("  OSCCAL=");
  Serial.print(OSCCAL);
  Serial.print("  g_medianOSCCAL=");
  Serial.println(g_medianOSCCAL);
}



