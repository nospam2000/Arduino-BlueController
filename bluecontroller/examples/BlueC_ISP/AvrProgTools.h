
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

inline uint8_t flash_read(uint8_t hilo, int addr) {
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
    if((millis() - start) >= max_write_delay) // wait t_WD_FLASH delay time after programming
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
inline uint8_t write_eeprom_chunk(int start, int length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)

  if(waitAvailable(length))
  {
    prog_lamp(LOW);
    for (int x = 0; x < length; x++) {
      int addr = start + x;
      spi_transaction(STK_OPCODE_WRITE_EEPROM_MEM, (addr>>8) & 0xFF, addr & 0xFF, SerialOpt.peek(x));
      delay(45); // TODO: is this constant correct? According to doc8271.pdf and doc2549.pdf t_WD_EEPROM is 3.6ms for ATmega328
    }
    consumeInputBuffer(length);
    prog_lamp(HIGH); 
    return Resp_STK_OK;
  }
  else
  {
    errorNoSync();
    return Resp_STK_NOSYNC;
  }
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
  uint16_t start = g_loadAddr * 2;
  for (uint16_t x = 0; x < length; x++) {
    uint16_t addr = start + x;
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
    int length = peekBe16(0);
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
inline void set_ext_addr(uint32_t addr)
{
  spi_transaction(STK_OPCODE_LOAD_EXT_ADDR_BYTE, 0x00, (addr >> 16) & 0x00, 0x00);
}

inline void flashByte(uint8_t hilo, int addr, uint8_t data) {
  spi_transaction(hilo ? STK_OPCODE_LOAD_PROG_PAGE_HI : STK_OPCODE_LOAD_PROG_PAGE_LO, 
    (addr>>8) & 0xFF, // TODO: according to AVR doc8271.pdf, chapter 27.8.3 "Serial Programming Instruction set", table 27-19, this should be 0x00
    addr & 0xFF,
    data);
  g_outStandingCommit = true;
}



