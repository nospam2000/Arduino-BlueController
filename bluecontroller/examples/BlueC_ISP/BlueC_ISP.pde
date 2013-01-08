/*
  BlueC_ISP - Turns your BlueController board into an Bluetooth ISP programmer
  Copyright (c) 2011 Michael Dreher <michael@5dot1.de>.  All rights reserved.
  please refer to "BlueC_ISP.h" for more information
*/

#include "BlueC_ISP.h"

// TODO: does the hardware work with 5V targets? How does the power have to be supplied => docu

int error = 0;
int pmode = 0;

uint16_t g_curPage = 0; // base address of the current page (as word address)
uint16_t g_loadAddr = 0; // address for reading and writing, set by 'U' command (as word address)
uint16_t g_verifyAddr = 0;

uint32_t g_byteAddr = 0;
uint32_t g_readPos = 0; // counting of the read protocol bytes, starts after reading BULK_WRITE_START_ACK
uint32_t g_ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK
uint32_t g_prevCmdTime = 0; // the time when the previous command has been processed
uint32_t g_prevAckTime = 0;


DeviceParameters g_deviceParam;

const uint32_t timeOut = 5*1000; // milliseconds
const uint16_t mtu = 126; // this is what I get for a RFComm commnection on Mac OSX 10.7 TODO: should be read from serial driver after connecting
const uint16_t windowSize = RX_BUFFER_SIZE - 1; // the capacity of the ring buffer is one byte less than its size
const uint8_t supportedOptions = Optn_STK_BULK_WRITE_VERIFY | Optn_STK_BULK_WRITE_RLE;

uint8_t g_commandMode = CMD_MODE_NORMAL;
uint8_t g_bulkOptions = 0;
uint8_t a_div = 2; // 1 for byte addressing, 2 for word addressing
uint8_t g_verifyBuf[256]; // only used for verify


// this uses some definitions from above, don't move it before this definitions!
#include "CommTools.h"
#include "AvrProgTools.h"
#include "BulkMode.h"

void setup() {
  // ARDUINO_SERIAL_BAUDRATE is either defined in .../hardware/<boardname>/variants/<variant>/pins_arduino.h
  // or you have to set it manually here when your board has no fixed baud rate
  SerialOpt.begin(ARDUINO_SERIAL_BAUDRATE);
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


inline void spi_init(uint8_t sckDuration) {
  /* // avoid floating point calculations and additional lookup table by directly using precalculated values
  uint16_t divisor = (g_deviceParam.sckDuration * 8.0 / STK500_XTAL) / (1.0/F_CPU);
  if(divisor < 8)
    divisor = 8;
  else if(divisor > 128)
    divisor = 128;
  */

  // calculate SCK divisor bits SPR0, SPR1 and SPI2X
  uint8_t spr;
  if(sckDuration >= (SCK_DUR_OSC_64 + 1)) // divisor >= 69
  {
    spr = (1<<SPR1) | (1<<SPR0); // f_OSC / 128
    sckDuration = SCK_DUR_OSC_128;
  }
  else if(sckDuration >= (SCK_DUR_OSC_32 + 1)) // divisor >= 43
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR1) | (1<<SPR0); // f_OSC / 64
    sckDuration = SCK_DUR_OSC_64;
  }
  else if(sckDuration >= (SCK_DUR_OSC_16 + 1)) // divisor >= 26
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR1); // f_OSC / 32
    sckDuration = SCK_DUR_OSC_32;
  }
  else if(sckDuration >= (SCK_DUR_OSC_8 + 1)) // divisor > 8
  {
    spr = (1<<SPR0); // f_OSC / 16
    sckDuration = SCK_DUR_OSC_16;
  }
  else
  {
    spr = (1<<SPI2X)<<2 | (1<<SPR0); // smallest value: f_OSC / 8
    sckDuration = SCK_DUR_OSC_8;
  }

  SPCR = (SPCR & ~((1<<SPR1) | (1<<SPR0))) | (spr & ((1<<SPR1) | (1<<SPR0))) | (1<<SPE) | (1<<MSTR);
  SPSR = (SPSR & ~(1<<SPI2X)) | ((spr >> 2) & (1<<SPI2X));
  volatile uint8_t x;
  x = SPSR; // clear SPIF by reading SPSR and SPDR
  x = SPDR; // clear SPIF by reading SPSR and SPDR
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
      sync_breply_failed(c);
      break;
  }
}

void set_parameter(uint8_t c, uint8_t value) {
  if(c == Parm_STK_SCK_DURATION)
  {
    g_deviceParam.sckDuration = value;
    spi_init(g_deviceParam.sckDuration);
    sync_reply_ok();
  }
  else if(c == Parm_STK_OSC_PSCALE)
  {
      // TODO: use the value
      g_deviceParam.oscPScale = value;
      sync_reply_ok();
  }
  else if(c == Parm_STK_OSC_CMATCH)
  {
      // TODO: use the value
      g_deviceParam.oscCMatch = value;
      sync_reply_ok();
  }
  else
  {
      sync_reply_failed();
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
// 2: pagel from config file (only needed for parallel programming)
// 3: bs2 from config file (only needed for parallel programming)
// 4: 0=(reset_disposition=RESET_DEDICATED); 1=other

  if(waitAvailable(1 + 4 + 1)) // len + data + Sync_CRC_EOP
  {
    // TODO: use the values

    consumeInputBuffer(4 + 1); // don't consume the Sync_CRC_EOP, this will be done in sync_reply_ok
    sync_reply_ok();
  }
  else
  {
    errorNoSync();
  }
}


void start_pmode() {
  // the timeout in avrdude is 1s, so we don't have much time, especially when falling back to lower SCK clocks!
  uint8_t sckDur = g_deviceParam.sckDuration;
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
    
      spi_init(sckDur);
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
    
    if(sckDur < SCK_DUR_OSC_128)
    {
      // automatically switch to slower SCK speed when communication fails
      sckDur <<= 1;
      if(sckDur == 8)
      {
        // otherwise we would skip one clock step
        sckDur = 5;
      }
    }
    else
    {
      // no device found
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

inline bool commit(int addr)
{
  bool rc = true;
  if (PROG_FLICKER)
    prog_lamp(LOW);

  // there is at least one byte different from 0xff and the current page is worth programming
  spi_transaction(STK_OPCODE_WRITE_PROG_MEM_PAGE, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  WaitForProgramMemPageFinished();
      
  if (PROG_FLICKER)
    prog_lamp(HIGH);

  return rc;
}



// returns the base address of the loadAddr; works only with page sizes with a power of two
inline uint16_t pageFromAddr(uint16_t addr) {
  return (addr & ~((g_deviceParam.pagesize / a_div) - 1));
}


inline uint8_t write_flash_pages(int length) {
  uint8_t rc = Resp_STK_OK;
  int page = pageFromAddr(g_loadAddr);
  for(int x = 0; x < length; ) {
    if (page != pageFromAddr(g_loadAddr)) {
      commit(page);
      page = pageFromAddr(g_loadAddr);
    }
    flashByte(LOW, g_loadAddr, SerialOpt.peek(x++));
    flashByte(HIGH, g_loadAddr, SerialOpt.peek(x++));
    g_loadAddr++;
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

// TODO: is this constant correct for all devices?
#define EECHUNK (32)
inline uint8_t write_eeprom(int length) {
  // g_loadAddr is a word address, get the byte address
  int start = g_loadAddr * 2;
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
    
    // flash memory @g_loadAddr, (length) bytes
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


inline void processNormalModeCommand(uint8_t cmd)
{
  // the cases are ordered by the number of calls avrdude makes to reduce the number of comparisons
  if(cmd == Cmnd_STK_LOAD_ADDRESS) // 'U' 0x55 set address (word)
  {
    if(waitAvailable(2))
    {
      g_loadAddr = peekLe16(0);
      consumeInputBuffer(2);
      sync_reply_ok();
    }
    else
    {
      errorNoSync();
    }
  }
  else if(cmd == Cmnd_STK_READ_PAGE) // 't' 0x74
  {
    read_page();    
  }
  else if(cmd == Cmnd_STK_PROG_PAGE) // 0x64
  {
    program_page();
  }
  else if(cmd == Cmnd_STK_UNIVERSAL) // 'V' 0x56
  {
    universal();
  }
  else if(cmd == Cmnd_STK_GET_SYNC) // '0' 0x30 signon
  {
    error = 0;
    sync_reply_ok();
  }
  else if(cmd == Cmnd_STK_GET_PARAMETER) // 'A' 0x41
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
  else if(cmd == Cmnd_STK_SET_PARAMETER) // '@' 0x40
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
  else if(cmd == Cmnd_STK_SET_DEVICE) // 'B' 0x42
  {
    set_device();
  }
  else if(cmd == Cmnd_STK_BULK_WRITE_START) // start bulk programming mode
  {
    processBulkModeCommand(cmd);
  }
  else if(cmd == Cmnd_STK_SET_DEVICE_EXT) // 'E' 0x45 extended parameters - ignore for now
  {
    set_device_ext();
  }
  else if(cmd == Cmnd_STK_ENTER_PROGMODE) // 'P' 0x50
  {
    start_pmode();
  }
  else if(cmd == Cmnd_STK_LEAVE_PROGMODE) // 'Q' 0x51
  {
    error = 0;
    end_pmode();
    sync_reply_ok();
  }
  else if(cmd == Cmnd_STK_READ_SIGN) // 'u' 0x75
  {
    read_signature();
  }
  else if(cmd == Cmnd_STK_GET_SIGN_ON) // '1' 0x31
  {
    if(sync())
    {
        SerialOpt.print("BlueC P");
        //SerialOpt.print("AVR ISP"); // TODO: consumes RAM, should be replaced by PROGMEM
        SerialOpt.write(Resp_STK_OK);
    }
  }
  else if(cmd == Cmnd_STK_PROG_FLASH) // 0x60
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
  else if(cmd == Cmnd_STK_PROG_DATA) // 0x61
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
  else if(cmd == Sync_CRC_EOP)
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


inline void processCmds(void)
{ 
  int cmd = SerialOpt.read();

  if(cmd >= 0)
  {
    if(g_commandMode == CMD_MODE_BULK_WRITE)
    {
      processBulkModeCommand(cmd);
    }
    else
    {
      processNormalModeCommand(cmd);
    }
    g_prevCmdTime = millis();
  }
  else
  {
    uint32_t diffTime = millis() - g_prevCmdTime;
    if(diffTime > timeOut)
    {
      // client is not reacting or no client connected; reset state of programmer
      if(g_commandMode != CMD_MODE_NORMAL)
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



