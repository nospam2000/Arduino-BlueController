/*
  HardwareSerialOpt.h - Hardware serial library for Wiring
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

  Modified 28 September 2010 by Mark Sproul
  Modified 2011 by Michael Dreher to suppport buffer sizes >256 bytes
*/

#ifndef HardwareSerialOpt_h
#define HardwareSerialOpt_h

//#include <inttypes.h>

#include "Stream.h"

#define DISABLE_IRQ   uint8_t sreg = SREG; cli();
#define RESTORE_IRQ   SREG = sreg;

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
// The number of bytes in the buffer is actually one less than the size of the buffer!
// Use a power of two as buffer size, this will speed up modulo operations by avoiding 16 bit divisions.
#if (RAMEND < 1000)
  #define RX_BUFFER_SIZE (31+1)
#else
  #define RX_BUFFER_SIZE (4*128)
  //#define RX_BUFFER_SIZE (127+1)
#endif

#if defined(ARDUINO) && ARDUINO >= 100
  #define WRITE_RET_TYPE size_t
#else
  #define WRITE_RET_TYPE void
#endif

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
};

class HardwareSerialOpt : public Stream
{
  private:
    ring_buffer *_rx_buffer;
    volatile uint8_t *_ubrrh;
    volatile uint8_t *_ubrrl;
    volatile uint8_t *_ucsra;
    volatile uint8_t *_ucsrb;
    volatile uint8_t *_udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udre;
    uint8_t _u2x;
  public:
    HardwareSerialOpt(ring_buffer *rx_buffer,
      volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
      volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
      volatile uint8_t *udr,
      uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x);
    void begin(long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    int peek(int index);
    int skip(int n);
    virtual int read(void);
    virtual void flush(void);
    virtual WRITE_RET_TYPE write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
};

#if defined(UBRRH) || defined(UBRR0H)
  extern HardwareSerialOpt SerialOpt;
#elif defined(USBCON)
  #include "usb_api.h"
#endif
#if defined(UBRR1H)
  extern HardwareSerialOpt SerialOpt1;
#endif
#if defined(UBRR2H)
  extern HardwareSerialOpt SerialOpt2;
#endif
#if defined(UBRR3H)
  extern HardwareSerialOpt SerialOpt3;
#endif

#endif
