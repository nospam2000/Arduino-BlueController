/* Bluecontroller settings: configure the BTM-222 module
 * This sketch only works, when __no__ bluetooth connection is established or when the escape sequence is __not__ disabled
 * Copyright (c) by Michael Dreher <michael(at)5dot1.de>

 * Additionally this is a demo of new bootloader functions:
 *   soft_reset() (works with optiboot and other bootloaders which clear the wdt and MCUSR)
 *   enter_bootloader() (works only with BlueController optiboot bootloader and magic-word mechanism)

 * refer to http://lucmorton.com/lucstuf/docs/Technical%20Q&A.doc for some interesting commands
 * which cannot be found in the official documentation
 * also interesting, but in German:
 *   http://robotrack.org/BTM222/BTM-indexD09.htm
 *   http://www.blog.1000und1led.de/?p=103
 *   http://www.mikrokopter.de/ucwiki/BTM-222
 */

#include <inttypes.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>

const int ledPin = 20;          // LED connected to digital pin 14
const int resetPin = 21;        // BT module uses pin 15 for reset

void setup()
{
  clear_wdt(); // only necessary without bootloader (which clears wdt and MCUSR) to avoid endless boot loop
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output

  resetBtm222(); // make sure there is no active connection; escape sequence won't work
  blinkLed(200);
  delay(6000); //wait until the bt module is up and running
  set_btbaudrate(115200); // must match the bootloader setting

  // these setting make the connection as transparent as possible
  sendBtCmd("\rAT"); // make sure the module is not in sleep mode
  sendBtCmd("ATE0"); // disable echo
  sendBtCmd("ATQ1"); // disable result code
  sendBtCmd("ATC1"); // enable flow control: without this, avrdude under Windows will hang
  sendBtCmd("ATR1"); // device is slave  
  sendBtCmd("ATN=BlueController"); // set new name
  sendBtCmd("ATP=1234"); // set PIN
  sendBtCmd("ATD0"); // accept connections from any bt device
  sendBtCmd("ATX0"); // disable escape character (default)
  //sendBtCmd("ATS1"); // enable powerdown of rs-232 driver (default)
  sendBtCmd("ATO"); // reconnect to peer

  delay(1000); // allow module to save setting in flash
  resetBtm222(); // activate new settings
} 


void loop()
{
  byte ch;

  if (Serial.available()) {
    ch = Serial.read();
    switch(ch)
    {
      case '1':
        Serial.println("Test 123");
        break;
  
      case '2':
        bt_escape_sequence();
        break;
  
      case 'x':
        Serial.println("soft_reset()");
        soft_reset();
        break;
  
      case 'y':
        Serial.println("enter_bootloader()");
        enter_bootloader();
        break;

      case 'z':
        Serial.println("resetBtm222()");
        resetBtm222();
        break;
    }
  }
}

// the order follows the most likely current configuration of the module
uint32_t baudrates[] = {
  19200, 38400, 57600, 115200, 9600, 4800, 2400, 1200
};

// must match the order of baudrates[]
uint8_t baudrate_char[] = {
  '2', '3', '4', '5', '1', '0', '*', '#'
};

bool set_btbaudrate(uint32_t newbaudrate)
{
  uint8_t i;
  int8_t new_baudrate_index = -1;
  bool result = false;

  for(i = 0; i < sizeof(baudrates) / sizeof(baudrates[0]); i++)
  {
    if(newbaudrate == baudrates[i])
    {
      new_baudrate_index = i;
      break;
    }
  }

  if(new_baudrate_index >= 0)
  {
    // we dont't know the old baudrate, therefore we try all baudrates
    for(i = 0; i < sizeof(baudrates) / sizeof(baudrates[0]); i++)
    {
      //if(i != new_baudrate_index)
      {
        blinkLed(50);
        Serial.begin(baudrates[i]);
        sendBtCmdStr("Baud rate: ");
        Serial.print(baudrates[i]);
        sendBtCmd("\rAT");

        //bt_escape_sequence();

        sendBtCmdStr("\rATL");
        sendBtCmdCh(baudrate_char[new_baudrate_index]);
        sendBtCmdCh('\r');
        // TODO: check response
      }
    }

    Serial.begin(baudrates[new_baudrate_index]);
    result = true;
  }
}

void bt_escape_sequence(void)
{
  delay(1200);
  sendBtCmdStr("+++");
  delay(1200);  
}

void sendBtCmd(char* pc)
{
  sendBtCmdStr(pc);
  sendBtCmdCh('\r');
}

void sendBtCmdStr(char* pc)
{
  while(*pc)
  {
    sendBtCmdCh(*(pc++));
  }
}

void sendBtCmdCh(char c)
{
  Serial.write(c);
  delay(50); // the BTM-222 is quite slow when interpreting commands, so we have to slow down communication
}

#define LED static_cast<byte>(6)  // select the pin for the LED
#define LEDPORT PORTB // select the port for the LED
#define LEDPIN PINB
#define LEDDDR DDRB

#define BTM222RESET static_cast<byte>(7)  // select the pin for the LED
#define BTM222RESETPORT PORTB // select the port for the LED
#define BTM222RESETPIN PINB
#define BTM222RESETDDR DDRB

#define ENTER_BL_MAGIC_VAL (0xc49e);
#define ENTER_BL_MAGIC_ADR (*((volatile uint16_t*)0x100))

// reset the bluetooth module
// this terminates the current bluetooth connection so it is not necessary to
// terminate it manually from the Android phone
void resetBtm222(void)
{
  BTM222RESETDDR |= _BV(BTM222RESET);
  BTM222RESETPORT &= ~_BV(BTM222RESET);
  delay(50);
  BTM222RESETPORT |= _BV(BTM222RESET);
  BTM222RESETDDR &= ~_BV(BTM222RESET);
}

void enter_bootloader(void)
{
  noInterrupts();
  //resetBtm222();
  ENTER_BL_MAGIC_ADR = ENTER_BL_MAGIC_VAL;
  wdt_enable(WDTO_15MS);
  for(;;) {}
}

void soft_reset(void)
{
  noInterrupts();
  ENTER_BL_MAGIC_ADR = 0;
  wdt_enable(WDTO_15MS);
  for(;;) {}
}

void clear_wdt()
{
  uint8_t mucsr = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

void blinkLed(uint16_t t)
{
  delay(t);
  digitalWrite(ledPin, 1);
  delay(t);
  digitalWrite(ledPin, 0);  
}


