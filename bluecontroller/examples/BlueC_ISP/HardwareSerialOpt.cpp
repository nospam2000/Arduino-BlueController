/*
  HardwareSerialOpt.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 2011 by Michael Dreher <michael@5dot1.de> to suppport buffer sizes >256 bytes
*/

// Some methods are not IRQ-safe, that means some of the variables are also modified in the ISR (especially rx_buffer->head)
// Rules for disabling IRQ:
//  _rx_buffer->tail and _rx_buffer->head are 16-bit variables and cannot be accessed atomically on 8-Bit ATmegas. They have
//     to be volatile and access must be protected. This doesn't matter as long as RX_BUFFER_SIZE is less or equal than 0x100.
//  _rx_buffer->tail is only modified by app and never by ISR => IRQs only need to be locked when modifying
//  _rx_buffer->head is only modified by ISR and not by app (there are two exceptions: ctor() and flush()) => always lock IRQ when reading or modifying it
//  the filled part of the ringbuffer belongs to the app and doesn't have to be locked, just make sure to read the values before changing _rx_buffer->tail
//  the emtpy part of the ringbuffer belongs to the ISR and doesn't have to be locked

//#include <stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <inttypes.h>
#include "wiring_private.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  //#include "wiring.h"
  //#include "pins_arduino.h"
  //#include "WProgram.h"
  #include "WConstants.h"
#endif

// this next line disables the entire HardwareSerialOpt.cpp, 
// this is so I can support Attiny series and any other chip without a uart
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#include "HardwareSerialOpt.h"

#if defined(UBRRH) || defined(UBRR0H)
  ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR1H)
  ring_buffer rx_buffer1  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR2H)
  ring_buffer rx_buffer2  =  { { 0 }, 0, 0 };
#endif
#if defined(UBRR3H)
  ring_buffer rx_buffer3  =  { { 0 }, 0, 0 };
#endif

// avoid 16-bit division for modulo and replace it by AND operation (only possible for power of two buffer sizes)
#if(RX_BUFFER_SIZE == 0x10 || RX_BUFFER_SIZE == 0x20 || RX_BUFFER_SIZE == 0x40 || RX_BUFFER_SIZE == 0x80 || RX_BUFFER_SIZE == 0x100 || RX_BUFFER_SIZE == 0x200)
#define MODULO_BUFSIZE(n) (n & (RX_BUFFER_SIZE - 1))
#else
// TODO: doesn't work, don't know why...
#error "use a power of two as buffer size..."
#define MODULO_BUFSIZE(n) ((uint16_t)n % (uint16_t)RX_BUFFER_SIZE)
#endif

inline void store_char(unsigned char c, ring_buffer *rx_buffer)
{
  register int head = rx_buffer->head;
  register int i = MODULO_BUFSIZE(head + 1);

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer->tail) {
    rx_buffer->buffer[head] = c;
    rx_buffer->head = i;
  }
#if 0
  else
  {
    // TODO: for debugging
    PORTB |= 0x40;
    //delay(5000);
    //DDRB &= ~0x40;
    //delay(3000);
  }
#endif
}

#if defined(USART_RX_vect)
  SIGNAL(USART_RX_vect)
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;
  #elif defined(UDR)
    unsigned char c  =  UDR;  //  atmega8535
  #else
    #error UDR not defined
  #endif
    store_char(c, &rx_buffer);
  }
#elif defined(SIG_USART0_RECV) && defined(UDR0)
  SIGNAL(SIG_USART0_RECV)
  {
    unsigned char c  =  UDR0;
    store_char(c, &rx_buffer);
  }
#elif defined(SIG_UART0_RECV) && defined(UDR0)
  SIGNAL(SIG_UART0_RECV)
  {
    unsigned char c  =  UDR0;
    store_char(c, &rx_buffer);
  }
//#elif defined(SIG_USART_RECV)
#elif defined(USART0_RX_vect)
  // fixed by Mark Sproul this is on the 644/644p
  //SIGNAL(SIG_USART_RECV)
  SIGNAL(USART0_RX_vect)
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;
  #elif defined(UDR)
    unsigned char c  =  UDR;  //  atmega8, atmega32
  #else
    #error UDR not defined
  #endif
    store_char(c, &rx_buffer);
  }
#elif defined(SIG_UART_RECV)
  // this is for atmega8
  SIGNAL(SIG_UART_RECV)
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;  //  atmega645
  #elif defined(UDR)
    unsigned char c  =  UDR;  //  atmega8
  #endif
    store_char(c, &rx_buffer);
  }
#elif defined(USBCON)
  #warning No interrupt handler for usart 0
  #warning SerialOpt(0) is on USB interface
#else
  #error No interrupt handler for usart 0
#endif

//#if defined(SIG_USART1_RECV)
#if defined(USART1_RX_vect)
  //SIGNAL(SIG_USART1_RECV)
  SIGNAL(USART1_RX_vect)
  {
    unsigned char c = UDR1;
    store_char(c, &rx_buffer1);
  }
#elif defined(SIG_USART1_RECV)
  #error SIG_USART1_RECV
#endif

#if defined(USART2_RX_vect) && defined(UDR2)
  SIGNAL(USART2_RX_vect)
  {
    unsigned char c = UDR2;
    store_char(c, &rx_buffer2);
  }
#elif defined(SIG_USART2_RECV)
  #error SIG_USART2_RECV
#endif

#if defined(USART3_RX_vect) && defined(UDR3)
  SIGNAL(USART3_RX_vect)
  {
    unsigned char c = UDR3;
    store_char(c, &rx_buffer3);
  }
#elif defined(SIG_USART3_RECV)
  #error SIG_USART3_RECV
#endif


// Constructors ////////////////////////////////////////////////////////////////

HardwareSerialOpt::HardwareSerialOpt(ring_buffer *rx_buffer,
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *udr,
  uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x)
{
  _rx_buffer = rx_buffer;
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _udr = udr;
  _rxen = rxen;
  _txen = txen;
  _rxcie = rxcie;
  _udre = udre;
  _u2x = u2x;
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerialOpt::begin(long baud)
{
  uint16_t baud_setting;
  bool use_u2x = true;

#if F_CPU == 16000000UL
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    use_u2x = false;
  }
#endif
  
  if (use_u2x) {
    *_ucsra = 1 << _u2x;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  sbi(*_ucsrb, _rxen);
  sbi(*_ucsrb, _txen);
  sbi(*_ucsrb, _rxcie);
}

void HardwareSerialOpt::end()
{
  cbi(*_ucsrb, _rxen);
  cbi(*_ucsrb, _txen);
  cbi(*_ucsrb, _rxcie);  
}

int HardwareSerialOpt::available(void)
{
  DISABLE_IRQ;
  int avail = MODULO_BUFSIZE(RX_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail);
  RESTORE_IRQ;
  return avail;
}


int HardwareSerialOpt::peek(void)
{
  return peek(0);
}

int HardwareSerialOpt::peek(int index)
{
  register int i = MODULO_BUFSIZE(_rx_buffer->tail + index);
  DISABLE_IRQ;
  register int head = _rx_buffer->head;
  RESTORE_IRQ;
  bool isOutOfRange = (i == head);

  if(isOutOfRange)
  {
    return -1;
  }
  else
  {
    return _rx_buffer->buffer[i];
  }
}

int HardwareSerialOpt::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  register int c = -1;
  register int tail = _rx_buffer->tail;
  DISABLE_IRQ;
  register int head = _rx_buffer->head;
  
  if (head != tail) {
    c = _rx_buffer->buffer[tail];
    _rx_buffer->tail = MODULO_BUFSIZE(tail + 1);
  }
  RESTORE_IRQ;
  
  return c;
}

// needed when using peek(n) to move the read pointer ahead
// the return value tells the number of bytes that really have been skipped
int HardwareSerialOpt::skip(int n)
{
  DISABLE_IRQ;
  int avail = available();
  if(n > avail)
  {
    n = avail;
  }
  _rx_buffer->tail = MODULO_BUFSIZE(_rx_buffer->tail + n);
  RESTORE_IRQ;
  return n;
}

void HardwareSerialOpt::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.

  // a 16-bit read or write is not atomic on a 8-Bit MCU, so every access to _rx_buffer->head
  // outside of the ISR has to be protected by cli/sei
  DISABLE_IRQ;
  _rx_buffer->head = _rx_buffer->tail;
  RESTORE_IRQ;
}

WRITE_RET_TYPE HardwareSerialOpt::write(uint8_t c)
{
  while (!((*_ucsra) & (1 << _udre)))
    ;

  *_udr = c;
  return (WRITE_RET_TYPE)1;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

#if defined(UBRRH) && defined(UBRRL)
  HardwareSerialOpt SerialOpt(&rx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRE, U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
  HardwareSerialOpt SerialOpt(&rx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRE0, U2X0);
#elif defined(USBCON)
  #warning no serial port defined  (port 0)
#else
  #error no serial port defined  (port 0)
#endif

#if defined(UBRR1H)
  HardwareSerialOpt SerialOpt1(&rx_buffer1, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1, RXEN1, TXEN1, RXCIE1, UDRE1, U2X1);
#endif
#if defined(UBRR2H)
  HardwareSerialOpt SerialOpt2(&rx_buffer2, &UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UDR2, RXEN2, TXEN2, RXCIE2, UDRE2, U2X2);
#endif
#if defined(UBRR3H)
  HardwareSerialOpt SerialOpt3(&rx_buffer3, &UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UDR3, RXEN3, TXEN3, RXCIE3, UDRE3, U2X3);
#endif

#endif // whole file

