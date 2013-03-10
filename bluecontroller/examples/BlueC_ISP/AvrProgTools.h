//  Copyright (c) 2011 Michael Dreher <michael@5dot1.de>.  All rights reserved.

// here you find the functions which are directly related to the ATmega hardware
// all of the functions are 'inline', so this must be a .h file

// TODO: these constants have to be defined dependent on the MCU clk
#define SCK_DUR_OSC_128 (15)
#define SCK_DUR_OSC_64 (7)
#define SCK_DUR_OSC_32 (4)
#define SCK_DUR_OSC_16 (2)
#define SCK_DUR_OSC_8 (1)

inline void spi_wait() {
  while (!(SPSR & (1 << SPIF))) {}
}

inline uint8_t spi_send(uint8_t val) {
  SPDR = val;
  spi_wait();
  return (uint8_t)SPDR; // automatically clears SPIF
}

inline uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  spi_send(a); 
  spi_send(b);
  spi_send(c);
  return spi_send(d);
}

inline uint8_t flash_read(uint8_t hilo, flashAddr16 addr) {
  // TODO: when using addresses >64k, set_ext_addr() must be used
  return spi_transaction(hilo ? STK_OPCODE_READ_PROG_MEM_HI : STK_OPCODE_READ_PROG_MEM_LO,
    (addr >> 8) & 0xFF,
    addr & 0xFF,
    0);
}

// Wait until BSY is no longer set, or the timeout value is reached
// TODO: The max_write_delay value from the avrdude configfile should be used, but they are not supplied by avrdude as parameters.
// According to AVR doc8271.pdf table 27-18 t_WD_FLASH has to be at least 4.5ms for ATmega328P, but
// some other devices like ATtiny12 need times up to 20ms. An ATmega103 would even need 56ms according to the avrdude config file.
#define max_write_delay (20)
inline void WaitForProgramMemPageFinished()
{
  uint32_t start = millis();
  for(;;)
  {
    if((millis() - start) > max_write_delay) // wait t_WD_FLASH delay time after programming
      break; // timeout

    // TODO: some parts need a timed base wait instead of using the 0xFO instruction, but avrdude always sets g_deviceParam.polling to 1,
    // so there is actually no chance to do it right
    if(g_deviceParam.polling)
    {
      // see documentation in doc8271, section below table Table 27-19 "Serial Programming Instruction Set (Hexadecimal values)"
      uint8_t bsy = spi_transaction(STK_OPCODE_POLL_RDY_BSY, 0x00, 0x00, 0x00);
      if(!(bsy & 0x01))
        break;
    }
  }
}

// write (length) bytes, (start) is a byte address
inline uint8_t write_eeprom_chunk(flashAddr16 start, uint16_t length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)

  // TODO: which delay time is correct?
  // According to doc8271.pdf and doc2549.pdf t_WD_EEPROM is 3.6ms for ATmega328
  //              doc2490.pdf                 t_WD_EEPROM is   9ms for ATmega64(L)
  // TODO: better use polling
  //Data Polling EEPROM
  //When a new byte has been written and is being programmed into EEPROM, reading the address location being programmed will give the value 0xFF.
  //At the time the device is ready for a new byte, the programmed value will read correctly. This is used to determine when the next byte can be
  //written. This will not work for the value 0xFF, but the user should have the following in mind: As a chip erased device contains 0xFF in all
  //locations, programming of addresses that are meant to contain 0xFF, can be skipped. This does not apply if the EEPROM is re-pro- grammed
  //without chip erasing the device. In this case, data polling cannot be used for the value 0xFF, and the user will have to wait at least
  //tWD_EEPROM before programming the next byte. See Table 128 for tWD_EEPROM value.
  
  /* maximum max_write_delay times extracted from avrdude 5.11 config files
  max_write_delay = 20000;
    ATtiny12: size=64
    AT90s2333, AT90s2343 (also AT90s2323 and ATtiny22): size=128
    AT90s4414, AT90s4434, AT90s4433: size = 256
    AT90s8535 size=512
  
  max_write_delay = 50000
    ATmega128RFA1: size=4096
  */

  uint16_t tWD_EEPROM = 20+1; // works for almost all devices
  if(g_deviceParam.eepromsize > 512)
    // will work for any devices with more than 512 byte flash, except ATmega128RFA1 (when avrdude.conf 5.11 data is correct)
    tWD_EEPROM = 9+1;

  prog_lamp(LOW);
  for (uint16_t x = 0; x < length; x++)
  {
    flashAddr16 addr = start + x;
    if(!waitAvailable(1))
    {
      errorNoSync();
      return Resp_STK_NOSYNC;
    }
    uint8_t eeVal = getchT();
    uint8_t eeReadback = spi_transaction(STK_OPCODE_READ_EEPROM_MEM, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    if(eeReadback != eeVal) // skip programming when value is already correct
    {
      spi_transaction(STK_OPCODE_WRITE_EEPROM_MEM, (addr>>8) & 0xFF, addr & 0xFF, eeVal);

      //if((!g_deviceParam.polling) || (eeVal == g_deviceParam.eeprom_readback_p1) || (eeVal == g_deviceParam.eeprom_readback_p2))
      if((!g_deviceParam.polling) || (eeVal == 0xff)) // the values from avrdude.conf don't seem to be reliable, so always use 0xff instead
      {
        delay(tWD_EEPROM);
      }
      else
      {
        uint32_t startTime = millis();
        uint32_t diffTime;
        do
        {
          eeReadback = spi_transaction(STK_OPCODE_READ_EEPROM_MEM, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
          diffTime = millis() - startTime;
        } while ((eeReadback != eeVal) && (diffTime <= tWD_EEPROM));
      }
    }
  }
  prog_lamp(HIGH); 
  return Resp_STK_OK;
}


inline char flash_read_page(uint16_t length) {
  for (uint16_t x = 0; x < length; x+=2) {
    uint8_t low = flash_read(LOW, g_loadAddr);
    bufferedWrite(low);
    uint8_t high = flash_read(HIGH, g_loadAddr);
    bufferedWrite(high);
    g_loadAddr++;
  }
  //flush_writebuf(); // is done in read_page()
  return Resp_STK_OK;
}

inline char eeprom_read_page(uint16_t length) {
  // here again we have a word address
  flashAddr16 start = g_loadAddr * a_div;
  //uint16_t start = g_loadAddr; // TODO: clarify if eeprom address is always a byte address

  for (uint16_t x = 0; x < length; x++) {
    flashAddr16 addr = start + x;
    uint8_t ee = spi_transaction(STK_OPCODE_READ_EEPROM_MEM, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    bufferedWrite(ee);
  }
  //flush_writebuf(); // is done in read_page()
  return Resp_STK_OK;
}

inline void read_page() {
  uint8_t result = Resp_STK_FAILED;

  if(waitAvailable(4) && (SerialOpt.peek(3) == Sync_CRC_EOP))
  {
    uint16_t length = peekBe16(0);
    char memtype = SerialOpt.peek(2);
    consumeInputBuffer(4);
    bufferedWrite(Resp_STK_INSYNC);
    if (memtype == 'F') { result = flash_read_page(length); }
    else if (memtype == 'E') { result = eeprom_read_page(length); }
    bufferedWrite(result);
    flush_writebuf();
  }
  else
  {
    errorNoSync();
  }
}

// this is only used by the programmer type "arduino", the "stk500v1" uses the universal command
inline void read_signature() {
  if (!sync()) {
    return;
  }
  uint8_t high   = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x00, 0x00);
  uint8_t middle = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x01, 0x00);
  uint8_t low    = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x02, 0x00);
  
  // remap ATmega88P to ATmega88 to satisfy the Arduino environment, which only knows the ATmega88
  // the bootloader of the ATmega88P also lies about its identity so this ensures compatibility
  if(high == 0x1e && middle == 0x93 && low == 0x0f)
  {
    high = 0x1e;
    middle = 0x93;
    low = 0x0a;
  }
  SerialOpt.write(high);
  SerialOpt.write(middle);
  SerialOpt.write(low);
  SerialOpt.write(Resp_STK_OK);
}

inline void universal()
{
  if(waitAvailable(4))
  {
    uint8_t rc = spi_transaction(SerialOpt.peek(0), SerialOpt.peek(1), SerialOpt.peek(2), SerialOpt.peek(3));
    consumeInputBuffer(4);
    sync_breply(rc);
  }
  else
  { 
    errorNoSync();
  }
}

// needed for flash size > 64k words, e.g. ATmega2561, refer to doc2549.pdf
// TODO: needs to be tested
inline void set_ext_addr(flashAddrExt8 addr)
{
  spi_transaction(STK_OPCODE_LOAD_EXT_ADDR_BYTE, 0x00, addr, 0x00);
}

inline void flashByte(uint8_t hilo, flashAddr16 addr, uint8_t data) {
  // TODO: when using addresses >64k, set_ext_addr() must be used
  spi_transaction(hilo ? STK_OPCODE_LOAD_PROG_PAGE_HI : STK_OPCODE_LOAD_PROG_PAGE_LO, 
    (addr>>8) & 0xFF,
    addr & 0xFF,
    data);
}



