// this sketch turns the Arduino into a AVRISP
// please refer to "ISP_SlidingWing.h" for more information
#include "ISP_SlidingWin.h"

// TODO: does the hardware work with 5V targets? How does the power have to be supplied => docu

// TODO: these constants have to be defined dependent on the MCU clk and moved to the header file
#define SCK_DUR_OSC_128 (15)
#define SCK_DUR_OSC_64 (7)
#define SCK_DUR_OSC_32 (4)
#define SCK_DUR_OSC_16 (2)
#define SCK_DUR_OSC_8 (1)

int error = 0;
int pmode = 0;

uint16_t curPage = 0; // base address of the current page (as word address)
uint16_t loadAddr = 0; // address for reading and writing, set by 'U' command (as word address)
uint16_t verifyAddr = 0;

uint32_t byteAddr = 0;
uint32_t readPos = 0; // counting of the read protocol bytes, starts after reading BULK_WRITE_START_ACK
uint32_t ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK
uint32_t prevCmdTime = 0; // the time when the previous command has been processed
uint32_t prevAckTime = 0;


DeviceParameters g_deviceParam;

const uint32_t timeOut = 5*1000; // milliseconds
const uint16_t mtu = 126; // this is what I get for a RFComm commnection on Mac OSX 10.7 TODO: should be read from serial driver after connecting
const uint16_t windowSize = RX_BUFFER_SIZE - 1; // the capacity of the ring buffer is one byte less than its size
const uint8_t supportedOptions = Optn_STK_BULK_WRITE_VERIFY | Optn_STK_BULK_WRITE_RLE;

uint8_t commandMode = CMD_MODE_NORMAL;
uint8_t bulkOptions = 0;
uint8_t a_div = 2; // 1 for byte addressing, 2 for word addressing
bool outStandingCommit = false;
uint8_t writebufpos = 0;
uint8_t buff[256]; // global block storage, only used for verify

#if defined(USE_BUFFERED_WRITE)
uint8_t writebuf[WRITEBUF_SIZE];
inline void bufferedWrite(uint8_t c)
{
  writebuf[writebufpos++] = c;
  if(writebufpos >= WRITEBUF_SIZE)
    flush_writebuf();
}

inline void flush_writebuf()
{
  for(uint8_t i = 0; i < writebufpos; i++)
  {
    SerialOpt.write(writebuf[i]);
  }
  writebufpos = 0;
  //digitalWrite(LED_BLOCKOP_TOGGLE, !digitalRead(LED_BLOCKOP_TOGGLE));
}
#else
inline void bufferedWrite(uint8_t c) { SerialOpt.write(c); }
inline void flush_writebuf() {}
#endif

void setup() {
  SerialOpt.begin(115200);
  // according to errata 1280 the first character afer initializing the serial port should be ignored
  delay(10);
  SerialOpt.flush(); // make sure the receive buffer is empty
  g_deviceParam.sckDuration = 2;

#if defined(LED_PMODE)
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
#endif
#if defined(LED_ERR)
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
#endif
#if defined(LED_HB)
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
#endif
#if defined(LED_BLOCKOP_TOGGLE)
  pinMode(LED_BLOCKOP_TOGGLE, OUTPUT);
  pulse(LED_BLOCKOP_TOGGLE, 2);
#endif
}

// this provides a heartbeat on pin 9, so you can tell the software is running.
inline void heartbeat() {
#if defined(LED_HB)
  static uint8_t hbval=128;
  static int8_t hbdelta=8;

  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
  delay(40);
#endif
}
  

inline void spi_wait() {
  do {
  } 
  while (!(SPSR & (1 << SPIF)));
}

inline uint8_t spi_send(uint8_t b) {
  SPDR = b;
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

// TODO: max_write_delay value from configfile should be used, but they are not supplied by avrdude as parameters
// t_WD_FLASH delay time after programming. According to AVR doc8271.pdf table 27-18 this has to be at least 4.5ms for ATmega328P
// some devices like ATtiny12 need times up to 20ms, an ATmega103 would even need 56ms according to the avrdude config file
#define max_write_delay (20)
inline void WaitForProgramMemPageFinished()
{
  uint32_t start = millis();
  for(;;)
  {
    if((millis() - start) >= max_write_delay)
      break; // timeout

    //if(1)// TODO: remove, only used for debugging
    //if(0)// TODO: remove, only used for debugging
    // TODO: some parts need a timed base wait instead of using the 0xFO instruction, avrdude always sets g_deviceParam.polling to 1
    if(g_deviceParam.polling)
    {
      // TODO: verify that this is true:
      // see documentation in doc8271, section below table Table 27-19 "Serial Programming Instruction Set (Hexadecimal values)"
      uint8_t bsy = spi_transaction(STK_OPCODE_POLL_RDY_BSY, 0x00, 0x00, 0x00);
      if(!(bsy & 0x01))
        break;
    }
  }
}

// getch with timeOut, return (int)-1 on timeOut
inline int16_t getchT() {
  waitAvailable(1);
  int16_t c = SerialOpt.read();
  
  if(c >= 0)
    readPos++;

  return c;
}

// wait until n characters are available or timeout occurs
// be careful: does not increment readPos
bool waitAvailable(int n) {
  uint32_t startTime = millis();
  uint32_t diffTime;
  int avail;

  do
  {
    diffTime = millis() - startTime;
  } while(((avail = SerialOpt.available()) < n) && (diffTime < timeOut));

  return (avail >= n);
}


inline void consumeInputBuffer(int n)
{
  readPos += SerialOpt.skip(n);
}


#define PTIME 200
void pulse(int pin, int times) {
  while (--times) {
    digitalWrite(pin, HIGH);
    delay(PTIME * 0.3);
    digitalWrite(pin, LOW);
    delay(PTIME * 1.7);
  } 
}

inline void prog_lamp(int state) {
#if defined(LED_PMODE)
  if (PROG_FLICKER)
    digitalWrite(LED_PMODE, state);
#endif
}


inline void spi_init() {
  /* // avoid floating point calculations and additional lookup table by directly using precalculated values
  uint16_t divisor = (g_deviceParam.sckDuration * 8.0 / STK500_XTAL) / (1.0/F_CPU);
  if(divisor < 8)
    divisor = 8;
  else if(divisor > 128)
    divisor = 128;
  */

  // calculate SCK divisor bits SPR0, SPR1 and SPI2X
  uint8_t spr;
  if(g_deviceParam.sckDuration >= (SCK_DUR_OSC_64 + 1)) // divisor >= 69
  {
    spr = (1<<SPR1) | (1<<SPR0); // f_OSC / 128
    g_deviceParam.sckDuration = SCK_DUR_OSC_128;
  }
  else if(g_deviceParam.sckDuration >= (SCK_DUR_OSC_32 + 1)) // divisor >= 43
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR1) | (1<<SPR0); // f_OSC / 64
    g_deviceParam.sckDuration = SCK_DUR_OSC_64;
  }
  else if(g_deviceParam.sckDuration >= (SCK_DUR_OSC_16 + 1)) // divisor >= 26
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR1); // f_OSC / 32
    g_deviceParam.sckDuration = SCK_DUR_OSC_32;
  }
  else if(g_deviceParam.sckDuration >= (SCK_DUR_OSC_8 + 1)) // divisor > 8
  {
    spr = (1<<SPR0); // f_OSC / 16
    g_deviceParam.sckDuration = SCK_DUR_OSC_16;
  }
  else
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR0); // smallest value: f_OSC / 8
    g_deviceParam.sckDuration = SCK_DUR_OSC_8;
  }

  SPCR = (SPCR & ~((1<<SPR1) | (1<<SPR0))) | (spr & ((1<<SPR1) | (1<<SPR0))) | (1<<SPE) | (1<<MSTR);
  SPSR = (SPSR & ~(1<<SPI2X)) | ((spr >> 2) & (1<<SPI2X));
  volatile uint8_t x;
  x = SPSR; // clear SPIF by reading SPSR and SPDR
  x = SPDR; // clear SPIF by reading SPSR and SPDR
}


void errorNoSync(void)
{
    error++;
    delay(150); // there might be some packages in flight, give them time to arrive and then throw them away
    SerialOpt.flush();
    SerialOpt.write(Resp_STK_NOSYNC);
}

inline bool sync()
{
  if(getchT() != Sync_CRC_EOP) {
    errorNoSync();
    return false;
  }
  
  SerialOpt.write(Resp_STK_INSYNC);
  return true;
}

// <0 means: unknown command, with -cmd as parameter
inline void sync_breply(int16_t b) {
  if(sync()) {
    if(b >= 0)
    {
      SerialOpt.write(b);
      SerialOpt.write(Resp_STK_OK);
    }
    else
    {
      SerialOpt.write(-b);
      SerialOpt.write(Resp_STK_FAILED);
    }
  }
}

inline void sync_reply(uint8_t rc) {
  if(sync()) {
    SerialOpt.write(rc);
  }
}

inline void sync_reply_ok() {
  sync_reply(Resp_STK_OK);
}

void get_parameter(uint8_t c) {
  switch(c) {
  case Parm_STK_HW_VER:
    sync_breply(HWVER);
    break;
  case Parm_STK_SW_MAJOR:
    sync_breply(SWMAJ);
    break;
  case Parm_STK_SW_MINOR:
    sync_breply(SWMIN);
    break;
  case Parm_STK_PROGMODE:
    sync_breply('S'); // serial programmer
    break;
  case Parm_STK_SCK_DURATION:
    // TODO: return the value
    sync_breply(g_deviceParam.sckDuration);
    break;
  case Parm_STK_OSC_PSCALE:
    sync_breply(g_deviceParam.oscPScale);
    break;
  case Parm_STK_OSC_CMATCH:
    sync_breply(g_deviceParam.oscCMatch);
    break;
  case Parm_STK_VTARGET:
  case Parm_STK_VADJUST:
    sync_breply(0);
    break;
  case Param_STK500_TOPCARD_DETECT:
    sync_breply(3);
    break;
  default:
    sync_breply(-c);
    break;
  }
}

void set_parameter(uint8_t c, uint8_t value) {
  if(c == Parm_STK_SCK_DURATION)
  {
    g_deviceParam.sckDuration = value;
    spi_init();
    sync_reply_ok();
  }
  else if(c == Parm_STK_OSC_PSCALE)
  {
      g_deviceParam.oscPScale = value;
      // TODO: use the value
      sync_reply_ok();
  }
  else if(c == Parm_STK_OSC_CMATCH)
  {
      g_deviceParam.oscCMatch = value;
      // TODO: use the value
      sync_reply_ok();
  }
  else
  {
      sync_breply(-c);
  }
}

inline void set_device() {
  if(waitAvailable(20))
  {
    //g_deviceParam.devicecode = SerialOpt.peek(0); // avrdude: p->stk500_devcode
    //g_deviceParam.revision = SerialOpt.peek(1); // avrdude V5.10 always sets this to 0
    //g_deviceParam.progtype = SerialOpt.peek(2); // must be 0 when device supports serial programming
    //g_deviceParam.parmode = SerialOpt.peek(3); // pseudo (0) or full (1) parallel interface
    g_deviceParam.polling = SerialOpt.peek(4); // avrdude V5.10 always sets this to 1
    //g_deviceParam.selftimed = SerialOpt.peek(5); // TODO: honor this parameter; avrdude V5.10 always sets this to 1
    //g_deviceParam.lockbytes = SerialOpt.peek(6); // number of lock-bytes
    //g_deviceParam.fusebytes = SerialOpt.peek(7); // number of fuse bytes
    //g_deviceParam.flashpollval1 = SerialOpt.peek(8);  // TODO: honor this parameter
    //g_deviceParam.flashpollval2 = SerialOpt.peek(9);  // same as flashpollval1, so no need to read it
    //g_deviceParam.eeprom_readback_p1 = SerialOpt.peek(10);  // TODO: honor this parameter
    //g_deviceParam.eeprom_readback_p2 = SerialOpt.peek(11);  // TODO: honor this parameter
    // following are 16 bits (big endian)
    g_deviceParam.pagesize = peekBe16(12);
    g_deviceParam.eepromsize = peekBe16(14);
  
    // 32 bits flashsize (big endian)
    g_deviceParam.flashsize = peekBe32(16);
    consumeInputBuffer(20);
    sync_reply_ok();
  }
  else
  { 
    errorNoSync();
  }
}

inline void set_device_ext() {
// the length is variable, in avrdude the length is limited to 16 including Cmnd_STK_SET_DEVICE_EXT and Sync_CRC_EOP
// refer to stk500.c:stk500_initialize() how to use the parsameters
// according to avrdude 5.10 sources: prommer firmware > 1.10 has 4 parameters, versions <=1.10 have 3 parameters

// meaning of the data in the format <offset>: <description>
// 0: number of following parameters + 1
// 1: EEPROM page size or 0
// 2: pagel from config file
// 3: bs2 from config file
// 4: 0=(reset_disposition=RESET_DEDICATED); 1=other

  if(waitAvailable(1 + 4 + 1)) // len + data + Sync_CRC_EOP
  {
    // TODO: ignore for now


    consumeInputBuffer(4 + 1); // don't consume the Sync_CRC_EOP, this will be done in sync_reply_ok
    sync_reply_ok();
  }
  else
  {
    errorNoSync();
  }
}


void start_pmode() {
  // the timeout in avrdude is 1s, so we don't have much time!
  uint8_t sckDurOrig = g_deviceParam.sckDuration;
  for(;;)
  {
    for(uint8_t initCount = 3; initCount > 0; --initCount)
    {
      // switch everything to input to avoid short curcuit
      pinMode(MISO, INPUT);
      digitalWrite(MISO, LOW); // disable pull-up
      pinMode(MOSI, INPUT);
      digitalWrite(MOSI, LOW); // disable pull-up
      digitalWrite(SCK, LOW);
      pinMode(SCK, OUTPUT);
      digitalWrite(RESET, HIGH);
      pinMode(RESET, OUTPUT);
      delay(2); // RESET must be given a positive pulse of at least two CPU clock cycles duration after SCK has been set to “0”
      digitalWrite(RESET, LOW);
      pinMode(MOSI, OUTPUT);
      digitalWrite(MISO, HIGH); // enable pull-up
      delay(30); // After pulling Reset low, wait at least 20 ms before issuing the first command.
    
      spi_init();
      spi_send(STK_OPCODE_PROG_ENABLE_1); 
      spi_send(STK_OPCODE_PROG_ENABLE_2);
      uint8_t loopBack = spi_send(0x00);
      spi_send(0x00);
    
      if (loopBack == 0x53)
      {
        pmode = 1;
        sync_reply_ok();
        return;
      }
    }
    
    if(g_deviceParam.sckDuration < SCK_DUR_OSC_128)
    {
      // automatically switch to slower SCK speed when communication fails
      g_deviceParam.sckDuration <<= 1;
      if(g_deviceParam.sckDuration == 8)
      {
        // otherwise we would skip one clock step
        g_deviceParam.sckDuration = 5;
      }
    }
    else
    {
      // no device found
      g_deviceParam.sckDuration = sckDurOrig; // when connect fails, restore original SCK duration
      sync_reply(Resp_STK_NODEVICE);
      return;
    }
  }
}


void end_pmode()
{
  SPCR &= ~((1<<SPE) | (1<<MSTR)); // disable ISP
  pinMode(MISO, INPUT);
  digitalWrite(MISO, LOW); // disable pullup
  pinMode(MOSI, INPUT);
  digitalWrite(MOSI, LOW); // disable pullup
  pinMode(SCK, INPUT);
  digitalWrite(SCK, LOW); // disable pullup
  pinMode(RESET, INPUT);
  digitalWrite(RESET, HIGH); // enable pullup
  pmode = 0;
}

inline void universal() {
  if(waitAvailable(4))
  {
    uint8_t ch = spi_transaction(SerialOpt.peek(0), SerialOpt.peek(1), SerialOpt.peek(2), SerialOpt.peek(3));
    consumeInputBuffer(4);
    sync_breply(ch);
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
  outStandingCommit = true;
}

inline bool commit(int addr) {
  bool rc = true;
  if(outStandingCommit)
  {
    if (PROG_FLICKER)
      prog_lamp(LOW);

    // there is at least one byte different from 0xff and the current page is worth programming
    spi_transaction(STK_OPCODE_WRITE_PROG_MEM_PAGE, (addr >> 8) & 0xFF, addr & 0xFF, 0);
    WaitForProgramMemPageFinished();
        
    if (PROG_FLICKER) {
      delay(PTIME);
      prog_lamp(HIGH);
    }
    outStandingCommit = false;
  }

  return rc;
}

inline bool commitWithVerify(int addr) {
  bool rc = true;
  if(outStandingCommit)
  {
    for(int i = 0; i < verifyAddr; i++)
    {
      if(buff[i] != 0xff)
      {
        // there is at least one byte different from 0xff and the current page is worth programming
        rc = commit(addr);
        break;
      }
    }
    
    outStandingCommit = false;

    if((bulkOptions & Optn_STK_BULK_WRITE_VERIFY) && (verifyAddr > 0))
      rc = verify_current_page(); // only do this in bulk mode; outside bulk mode verifyAddr will be always 0
    //digitalWrite(LED_BLOCKOP_TOGGLE, !digitalRead(LED_BLOCKOP_TOGGLE));
  }
  verifyAddr = 0;

  return rc;
}

inline bool verify_current_page(void)
{
  bool rc = true;
  for(int i = 0; i < verifyAddr; i++)
  {
    uint8_t flashVal;
    if(a_div == 2)
      flashVal = flash_read(i & 0x01, curPage + i/2); // word address
    else
      flashVal = flash_read(HIGH, curPage + i); // byte address // TODO: do we need HIGH or LOW as parameter?
    if(flashVal != buff[i])
    {
      uint32_t localByteAddr = curPage * a_div;
      bufferedWrite(Resp_STK_BULK_WRITE_VRFY_ERR);
      bufferedWrite(localByteAddr & 0xff);
      bufferedWrite((localByteAddr >> 8) & 0xff);
      bufferedWrite((localByteAddr >> 16) & 0xff);
      bufferedWrite((localByteAddr >> 24) & 0xff);

      bufferedWrite(verifyAddr & 0xff);
      bufferedWrite((verifyAddr >> 8) & 0xff);

      int j;
      for(j = 0; j < i; j++) // the part before the current index is identical to the buffer, so we don't have to read it again from flash
      {
        bufferedWrite(buff[j]);
      }
      bufferedWrite(flashVal); // the one which was different
      j++;
      for( ; j < verifyAddr; j++) // the second part has to be read from flash
      {
        if(a_div == 2)
          flashVal = flash_read(j & 0x01, curPage + j/2); // word address
        else
          flashVal = flash_read(HIGH, curPage + j); // byte address // TODO: do we need HIGH or LOW as parameter?
        bufferedWrite(flashVal);
      }

      bufferedWrite(Sync_CRC_EOP);
      flush_writebuf();
      
      rc = false;
      break;
    }
  }
  
  verifyAddr = 0;
  return rc;
}


// returns the base address of the loadAddr; works only with page sizes with a power of two
inline uint16_t current_page(uint16_t addr) {
  return (addr & ~((g_deviceParam.pagesize / a_div) - 1));
}


inline uint8_t write_flash_pages(int length) {
  uint8_t rc = Resp_STK_OK;
  int page = current_page(loadAddr);
  for(int x = 0; x < length; ) {
    if (page != current_page(loadAddr)) {
      commit(page);
      page = current_page(loadAddr);
    }
    flashByte(LOW, loadAddr, SerialOpt.peek(x++));
    flashByte(HIGH, loadAddr, SerialOpt.peek(x++));
    loadAddr++;
  }

  commit(page);

  return rc;
}

inline void write_flash(int length) {
  if(waitAvailable(length + 1) && (SerialOpt.peek(length) == Sync_CRC_EOP))
  {
    uint8_t rc = write_flash_pages(length);
    consumeInputBuffer(length + 1 - 1); // don't consume the Sync_CRC_EOP at the end, this is the task of sync_reply()
    sync_reply(rc);
  }
  else
  {
    errorNoSync();
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

// TODO: is this constant correct for all devices?
#define EECHUNK (32)
inline uint8_t write_eeprom(int length) {
  // loadAddr is a word address, get the byte address
  int start = loadAddr * 2;
  int remaining = length;
  if (length > g_deviceParam.eepromsize) {
    error++;
    return Resp_STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return Resp_STK_OK;
}

inline void program_page() {
  char result = (char) Resp_STK_FAILED;

  if(waitAvailable(3))
  {
    int length = peekBe16(0);
    char memtype = SerialOpt.peek(2);
    consumeInputBuffer(3);
    
    // flash memory @loadAddr, (length) bytes
    if (memtype == 'F') {
      write_flash(length);
    }
    else if (memtype == 'E') {
      result = (char)write_eeprom(length);
      sync_reply(result);
    }
    else {
      sync_reply(Resp_STK_FAILED);
    }
  }
  else
  {
    errorNoSync();
  }
}


inline char flash_read_page(uint16_t length) {
  for (uint16_t x = 0; x < length; x+=2) {
    uint8_t low = flash_read(LOW, loadAddr);
    bufferedWrite(low);
    uint8_t high = flash_read(HIGH, loadAddr);
    bufferedWrite(high);
    loadAddr++;
  }
  //flush_writebuf(); // is done in read_page()
  return Resp_STK_OK;
}

inline char eeprom_read_page(uint16_t length) {
  // here again we have a word address
  uint16_t start = loadAddr * 2;
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

inline void read_signature() {
  if (!sync()) {
    return;
  }
  uint8_t high   = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x00, 0x00);
  uint8_t middle = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x01, 0x00);
  uint8_t low    = spi_transaction(STK_OPCODE_READ_SIGBYTE, 0x00, 0x02, 0x00);
  SerialOpt.write(high);
  SerialOpt.write(middle);
  SerialOpt.write(low);
  SerialOpt.write(Resp_STK_OK);
}

//=== <<< normal mode <<< ==============================================================================================

inline void processNormalModeCommand(uint8_t ch)
{
  // the cases are ordered by the number of calls avrdude makes to reduce the number of comparisons
  if(ch == Cmnd_STK_LOAD_ADDRESS) // 'U' 0x55 set address (word)
  {
    if(waitAvailable(2))
    {
      loadAddr = peekLe16(0);
      consumeInputBuffer(2);
      sync_reply_ok();
    }
    else
    {
      errorNoSync();
    }
  }
  else if(ch == Cmnd_STK_READ_PAGE) // 't' 0x74
  {
    read_page();    
  }
  else if(ch == Cmnd_STK_PROG_PAGE) // 0x64
  {
    program_page();
  }
  else if(ch == Cmnd_STK_UNIVERSAL) // 'V' 0x56
  {
    universal();
  }
  else if(ch == Cmnd_STK_GET_SYNC) // '0' 0x30 signon
  {
    error = 0;
    sync_reply_ok();
  }
  else if(ch == Cmnd_STK_GET_PARAMETER) // 'A' 0x41
  {
    int param = getchT();
    if(param >= 0)
    {
      get_parameter(param);
    }
    else
    {
      errorNoSync();
    }
  }
  else if(ch == Cmnd_STK_SET_PARAMETER) // '@' 0x40
  {
    if(waitAvailable(2))
    {
      set_parameter(SerialOpt.peek(0), SerialOpt.peek(1));
      consumeInputBuffer(2);
    }
    else
    {
      errorNoSync();
    }
  }
  else if(ch == Cmnd_STK_SET_DEVICE) // 'B' 0x42
  {
    set_device();
  }
  else if(ch == Cmnd_STK_BULK_WRITE_START) // start bulk programming mode
  {
    processBulkModeCommand(ch);
  }
  else if(ch == Cmnd_STK_SET_DEVICE_EXT) // 'E' 0x45 extended parameters - ignore for now
  {
    set_device_ext();
  }
  else if(ch == Cmnd_STK_ENTER_PROGMODE) // 'P' 0x50
  {
    start_pmode();
  }
  else if(ch == Cmnd_STK_LEAVE_PROGMODE) // 'Q' 0x51
  {
    error = 0;
    end_pmode();
    sync_reply_ok();
  }
  else if(ch == Cmnd_STK_READ_SIGN) // 'u' 0x75
  {
    read_signature();
  }
  else if(ch == Cmnd_STK_GET_SIGN_ON) // '1' 0x31
  {
    if(sync())
    {
        //SerialOpt.print("BluePrg");
        SerialOpt.print("AVR ISP"); // TODO: consumes RAM, should be replaced by PROGMEM
        SerialOpt.write(Resp_STK_OK);
    }
  }
  else if(ch == Cmnd_STK_PROG_FLASH) // 0x60
  {
    if(waitAvailable(2))
    {
      // TODO: implement command
      
      consumeInputBuffer(2);
      sync_reply_ok();
    }
    else
    {
      errorNoSync();
    }
  }
  else if(ch == Cmnd_STK_PROG_DATA) // 0x61
  {
    int16_t data = getchT();
    if(data >= 0)
    {
      // TODO: implement command
      sync_reply_ok();
    }
    else
    {
      errorNoSync();
    }
  }
  // expecting a command, not Sync_CRC_EOP
  // this is how we can get back in sync
  else if(ch == Sync_CRC_EOP)
  {
    errorNoSync();
  }    
  // anything else we will return Resp_STK_UNKNOWN
  else
  {
    if (Sync_CRC_EOP == getchT())
    {
      error++;
      SerialOpt.write(Resp_STK_UNKNOWN);
    }
    else
    {
      errorNoSync();
    }
  }
}

//=== <<< bulk mode <<< ==============================================================================================

inline bool ProgByteLocation(uint32_t progByteAddr, uint8_t val)
{
  bool rc = true;
  uint32_t localLoadAddr = (a_div == 2) ? (progByteAddr / 2) : progByteAddr;
  int newPage = current_page(localLoadAddr);
  if (newPage != curPage)
  {
    // this has to be handled every time the byteAddr is changed and on the very end of programming   
    rc = commitWithVerify(curPage);
    if((curPage >> 16) != (newPage >> 16))
    {
      set_ext_addr(newPage);
    }
    curPage = newPage;
  }

  // account for byte/word addressing and if HIGH or LOW opcode for even/odd addresses have to be used
  // TODO: handle byte addressing, that means do we have to use HIGH or LOW in case of byte addressing?
  uint8_t hiLo = (a_div == 2) ? ((progByteAddr & 0x01) ? HIGH : LOW) : HIGH;

  flashByte(hiLo, localLoadAddr, val);
  buff[verifyAddr++] = val;
}

inline void exitBulkMode(void)
{
  commandMode = CMD_MODE_NORMAL;
  commitWithVerify(curPage); // must be always called when setting commandMode back to CMD_NORMAL_MODE
  verifyAddr = 0;
  bulkOptions = 0;
}

inline void abortBulkMode(void)
{
    exitBulkMode();
    flush_writebuf(); // there might be some chars left, flush them before, because errorNoSync() doesn't use the writebuffer
    errorNoSync();
}

// block data is at the beginning of the serial receive buffer
inline void ProcessReceivedBlock(uint16_t blockLen)
{
  for(int i = 0; i < blockLen; i++)
  {
    uint8_t val = SerialOpt.peek(i);
    uint16_t repCnt = 1;
    if((bulkOptions & Optn_STK_BULK_WRITE_RLE) && (val == 0xff))
    {
      if(++i >= blockLen)
        goto rleError;
      repCnt = SerialOpt.peek(i);
      if(repCnt == 0x00)
      {
        repCnt = 1; // 0xFF 0x00 => 0xFF 
      }
      else if(repCnt == 0x01)
      {
        repCnt = 2; // 0xFF 0x01 => 0xFF 0xFF
      }
      else if(repCnt == 0xff) // 16-bit repCnt
      {
        i += 3;
        if(i >= blockLen)
          goto rleError;
        repCnt = SerialOpt.peek(i - 2) + SerialOpt.peek(i - 1) << 8;
        val = SerialOpt.peek(i);
      }
      else // 8-bit repCnt
      {
        if(++i >= blockLen)
          goto rleError;
        val = SerialOpt.peek(i);
      }
    }
      
    while(repCnt--)
    {
      ProgByteLocation(byteAddr++, val);
    }
  }
  return;

rleError:
  abortBulkMode();
  return;
}

inline void processBulkModeCommand(uint8_t ch)
{
  if(ch == Cmnd_STK_BULK_WRITE_START) // start bulk programming mode
  {
    if(commandMode == CMD_MODE_BULK_WRITE)
    {
      // Error: we are already in bulk mode and this command is not allowed to be sent a second time
      abortBulkMode();
    }
    else
    {
      commandMode = CMD_MODE_BULK_WRITE;
      // read rest of Cmnd_STK_BULK_WRITE_START telegram
      if(!waitAvailable(3 - 1) || (SerialOpt.peek(2 - 1) != Sync_CRC_EOP)) // offset -1 because cmdByte has already be consumed
      {
        abortBulkMode();
      }
      else
      {
        bulkOptions = SerialOpt.peek(2 - 1) & supportedOptions;
        bufferedWrite(Resp_STK_BULK_WRITE_START_ACK);
        bufferedWrite(bulkOptions);
        bufferedWrite(windowSize & 0xff);
        bufferedWrite((windowSize >> 8) & 0xff);
        bufferedWrite(Sync_CRC_EOP);
        flush_writebuf();
        consumeInputBuffer(3 - 1);
      }

      readPos = 0; // counting of the read protocol bytes, starts after reading BULK_WRITE_START_ACK
      ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK
      byteAddr = 0;
      verifyAddr = 0;
      curPage = 0;
      outStandingCommit = false;
      a_div = (bulkOptions & Optn_STK_BULK_WRITE_BYTE_ADR) ? 1 : 2; // 1: byte addressing, 2: word addressing
      if((g_deviceParam.flashsize / a_div) > 0x10000)
      {
        set_ext_addr(curPage);
      }
    }  
  }
  else if(ch == Cmnd_STK_BULK_WRITE_END)
  {
    // read rest of CMD_MODE_BULK_WRITE_END telegram
    if(!waitAvailable(2 - 1) || (SerialOpt.peek(1 - 1) != Sync_CRC_EOP)) // offset -1 because cmdByte has already be consumed
    {
      abortBulkMode();
    }
    else
    {
      consumeInputBuffer(2 - 1);
      uint8_t errorCode = 0; // TODO: set the correct error bits
      bufferedWrite(Resp_STK_BULK_WRITE_END_ACK);
      bufferedWrite(errorCode);
      
      bufferedWrite(byteAddr & 0xff);
      bufferedWrite((byteAddr >> 8) & 0xff);
      bufferedWrite((byteAddr >> 16) & 0xff);
      bufferedWrite((byteAddr >> 24) & 0xff);
    
      bufferedWrite(Sync_CRC_EOP);
      flush_writebuf();
      exitBulkMode();
    }
  }
  else if(ch == Cmnd_STK_BULK_WRITE_DATA)
  {
    // read rest of Cmnd_STK_BULK_WRITE_DATA telegram
    if(!waitAvailable(5 - 1)) // offset -1 because cmdByte has already be consumed
    {
      abortBulkMode();
    }
    else
    {
      uint16_t crc = peekLe16(1 - 1); // offset -1 because cmdByte has already be consumed
      uint16_t blockLen = peekLe16(3 - 1);
      consumeInputBuffer(5 - 1);
      if(!waitAvailable(blockLen + 1) || (SerialOpt.peek(blockLen) != Sync_CRC_EOP)) // blockLen+1 for Sync_CRC_EOP
      {
        abortBulkMode();
      }
      else
      {
        // start programming using the current byteAddr
        // the buffer may already contain a partial page
        ProcessReceivedBlock(blockLen);
        
        // at the end consume the bytes from the serial receive buffer, and adjust the readPos
        consumeInputBuffer(blockLen + 1);        
      }
    }
  }
  else if(ch == Cmnd_STK_BULK_SET_BYTEADDR)
  {
    // read rest of Cmnd_STK_BULK_LOAD_ADDRESS telegram
    if(!waitAvailable(6 - 1) || (SerialOpt.peek(5 - 1) != Sync_CRC_EOP)) // offset -1 because cmdByte has already be consumed
    {
      abortBulkMode();
    }
    else
    {
      // when changing the byteAddr, there might be some bytes in the page buffer which have to be processed before
      // this is done automatically when in ProgByteLocation() after calculating the loadAddr
      byteAddr = peekLe32(1 - 1);
      consumeInputBuffer(6 - 1);
    }
  }
  else
  {
    // Error: no valid command found, exiting bulk mode
    exitBulkMode();
    bufferedWrite(Resp_STK_UNKNOWN); // the error code is different to abortBulkMode()
    flush_writebuf();
  }

  sendAckWhenAppropriate();
}

inline bool isAckAppropriate()
{
  bool sendAck = false;
  uint32_t lag = readPos - ackPos;
  if(lag > 0)
  {
    // if lag gets too big compared to the previous sent ack, send an ack
    // refer to AckTimeCalculator.ods for the values
    uint32_t diffTime = millis() - prevAckTime;
    const uint32_t lagOffset = 1*mtu; // send ack when lag >=lagOffset, without any delay
    const uint32_t timeOffset = 14*2; // even when lag is only 1 byte, send ack after this time (ca. 14ms per 126 byte packet)
    if(((lag + lagOffset) * (diffTime + timeOffset)) >= (2*lagOffset*timeOffset))
    {
      sendAck = true;
    } 
  }

  return sendAck;
}

inline void sendAckWhenAppropriate()
{
  if(isAckAppropriate())
  {
    ackPos = readPos;
    uint8_t ackState = Stat_STK_BULK_ACKSTATE_ACK; // TODO: handle NAK
    bufferedWrite(Resp_STK_BULK_WRITE_ACK);
    bufferedWrite(ackState);

    bufferedWrite(byteAddr & 0xff);
    bufferedWrite((byteAddr >> 8) & 0xff);
    bufferedWrite((byteAddr >> 16) & 0xff);
    bufferedWrite((byteAddr >> 24) & 0xff);

    bufferedWrite(ackPos & 0xff);
    bufferedWrite((ackPos >> 8) & 0xff);
    bufferedWrite((ackPos >> 16) & 0xff);
    bufferedWrite((ackPos >> 24) & 0xff);
    bufferedWrite(Sync_CRC_EOP);
    flush_writebuf();
    prevAckTime = millis();
  }  
}

//=== >>> bulk mode >>> ==============================================================================================

inline void processCmds(void)
{ 
  int ch = SerialOpt.read();

  if(ch >= 0)
  {
    if(commandMode == CMD_MODE_BULK_WRITE)
    {
      processBulkModeCommand(ch);
    }
    else
    {
      processNormalModeCommand(ch);
    }
    prevCmdTime = millis();
  }
  else
  {
    uint32_t diffTime = millis() - prevCmdTime;
    if(diffTime > timeOut)
    {
      // client is not reacting or no client connected; reset state of programmer
      if(commandMode != CMD_MODE_NORMAL)
      {
        abortBulkMode();
      }
    }
  }
}

void loop(void) {
#if defined(LED_PMODE)
  // is pmode active?
  digitalWrite(LED_PMODE, pmode ? HIGH : LOW); 
#endif
#if defined(LED_ERR)
  // is there an error?
  digitalWrite(LED_ERR, error ? HIGH : LOW); 
#endif
  
  heartbeat(); // light the heartbeat LED
  processCmds(); // drive command loop
}



