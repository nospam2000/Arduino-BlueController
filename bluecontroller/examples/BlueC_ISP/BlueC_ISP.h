
// documentation can be found in:
//   AVR doc8271.pdf, chapter 27.8.3 "Serial Programming Instruction set"
//   AVR doc2549.pdf, chapter 30.8.3 "Serial Programming Instruction set"
//   AVR doc0943.pdf, "AVR910: In System Programming"
//   AVR doc2525.pdc, AVR061: STK500 Communication Protocol

// using the following pins:
// 10: slave reset
// 11: MOSI
// 12: MISO
// 13: SCK

// Put an LED (with resistor) on the following pins:
// 9: Heartbeat - shows the programmer is running
// 8: Error - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave

// August 2011 by Michael Dreher <michael@5dot1.de>
//  - Performance enhancements for Bluetooth transmission:
//    - New protocol for asynchronous transmission using a sliding window mechanism.
//      This is needed for high performance programming with high-latency communication devices like Bluetooth SPP
//      (escpecially for iMac's using a Bluetooth mouse and keyboard which need around 8s using
//      the new protocol and 180s with the original protocol),
//      Could also be useful for WiFi and LAN Arduinos.
//    - Automatic verify after writing each page (during communication is still running) without transmitting the data again
//    - Compression of the transmitted data (RLE=run lenght encoding) 

// June 2011 by Michael Dreher
//  - fixed issue 22 (http://code.google.com/p/mega-isp/issues/detail?id=22)
//    (shortened the time from 50ms to 1ms)
//  - made LED support optional, just comment out the unneeded LED defines

// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp

//#include "pins_arduino.h"
//#include "stk500.h"
#include "HardwareSerialOpt.h"
#include "stk500_private.h"
#define RESET SS

//#define LED_HB 9 // TODO: consumes 40ms for each cycle and will kill the performance when enabled!!!
//#define LED_ERR 8
//#define LED_PMODE 7
#define PROG_FLICKER false
#define LED_BLOCKOP_TOGGLE 20
//#define USE_BUFFERED_WRITE

#define HWVER 2
#define SWMAJ 1
#define SWMIN 19

#define CMD_MODE_NORMAL     0x00
#define CMD_MODE_BULK_WRITE 0x01

#define WRITEBUF_SIZE 24 // max is 255 because of uint8_t writebufpos and uint8_tloopvar in flush_writebuf()

// some instructions have more than one byte, therefore the _1 and _2 defines
#define STK_OPCODE_PROG_ENABLE_1         0xAC
#define STK_OPCODE_PROG_ENABLE_2         0x53
//#define STK_OPCODE_CHIP_ERASE_1          0xAC
//#define STK_OPCODE_CHIP_ERASE_2          0x80
#define STK_OPCODE_POLL_RDY_BSY          0xF0
#define STK_OPCODE_LOAD_EXT_ADDR_BYTE    0x4D
#define STK_OPCODE_LOAD_PROG_PAGE_HI     0x48
#define STK_OPCODE_LOAD_PROG_PAGE_LO     0x40
#define STK_OPCODE_LOAD_EEPROM_PAGE      0xC1
#define STK_OPCODE_READ_PROG_MEM_HI      0x28
#define STK_OPCODE_READ_PROG_MEM_LO      0x20
#define STK_OPCODE_READ_EEPROM_MEM       0xA0
//#define STK_OPCODE_READ_LOCKBITS         0x58
#define STK_OPCODE_READ_SIGBYTE          0x30
//#define STK_OPCODE_READ_FUSEBITS_1       0x50
//#define STK_OPCODE_READ_FUSEBITS_2       0x00
//#define STK_OPCODE_READ_FUSEBITS_HI_1    0x58
//#define STK_OPCODE_READ_FUSEBITS_HI_2    0x08
//#define STK_OPCODE_READ_FUSEBITS_EXT_1   0x50
//#define STK_OPCODE_READ_FUSEBITS_EXT_2   0x08
//#define STK_OPCODE_READ_CALIB_BYTE       0x38
#define STK_OPCODE_WRITE_PROG_MEM_PAGE   0x4C
#define STK_OPCODE_WRITE_EEPROM_MEM      0xC0
#define STK_OPCODE_WRITE_EEPROM_MEM_PAGE 0xC2

// avrdude uses the 'universal' command for these STK commands
//#define STK_OPCODE_WRITE_LOCKBITS_1      0xAC
//#define STK_OPCODE_WRITE_LOCKBITS_2      0xE0
//#define STK_OPCODE_WRITE_FUSEBITS_1      0xAC
//#define STK_OPCODE_WRITE_FUSEBITS_2      0xA0
//#define STK_OPCODE_WRITE_FUSEHIBITS_1    0xAC
//#define STK_OPCODE_WRITE_FUSEHIBITS_2    0xA8
//#define STK_OPCODE_WRITE_FUSEEXTBITS_1   0xAC
//#define STK_OPCODE_WRITE_FUSEEXTBITS_2   0xA4

#define peekLe16(index) (SerialOpt.peek(index) + SerialOpt.peek(index+1) * 0x100)
#define peekBe16(index) (SerialOpt.peek(index) * 0x100 + SerialOpt.peek(index+1))
#define peekLe32(index) (SerialOpt.peek(index) + SerialOpt.peek(index+1) * 0x100 + SerialOpt.peek(index+2) * 0x10000 + SerialOpt.peek(index+3) * 0x1000000)
#define peekBe32(index) (SerialOpt.peek(index) * 0x1000000 + SerialOpt.peek(index+1) * 0x10000 + SerialOpt.peek(index+2) * 0x100 + SerialOpt.peek(index+3))

typedef struct DeviceParameters_struct {
  //uint8_t devicecode;
  //uint8_t revision;
  //uint8_t progtype;
  //uint8_t parmode;
  uint8_t polling;
  //uint8_t selftimed;
  //uint8_t lockbytes;
  //uint8_t fusebytes;
  //uint8_t flashpollval1;
  //uint8_t flashpollval2;
  //uint8_t eeprom_readback_p1;
  //uint8_t eeprom_readback_p2;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;

  uint8_t sckDuration; // the actual time is: sckDuration * 8.0 / STK500_XTAL
  uint8_t oscPScale; // OSC_PSCALE
  uint8_t oscCMatch; // OSC_CMATCH
} DeviceParameters;

