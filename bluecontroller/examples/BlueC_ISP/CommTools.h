// Copyright (c) 2011 Michael Dreher <michael@5dot1.de>.  All rights reserved.
// here you find the lowlevel communication functions
// all of the functions are 'inline', so this must be a .h file
// getch with timeOut, return (int)-1 on timeOut

// wait until n characters are available or timeout occurs
// be careful: does not increment g_readPos
inline bool waitAvailable(int n) {
  uint32_t startTime = millis();

  for(;;)
  {
    uint32_t diffTime = millis() - startTime;
    if(diffTime >= timeOut)
      return false;
    
    if(SerialOpt.available() >= n)
      return true;
  }
}

inline int16_t getchT() {
  waitAvailable(1);
  int16_t c = SerialOpt.read();
  
  if(c >= 0)
    g_readPos++;

  return c;
}

#if defined(USE_BUFFERED_WRITE)
uint8_t g_writebufpos = 0;
uint8_t g_writebuf[WRITEBUF_SIZE];
inline void bufferedWrite(uint8_t c)
{
  g_writebuf[g_writebufpos++] = c;
  if(g_writebufpos >= WRITEBUF_SIZE)
    flush_writebuf();
}

inline void flush_writebuf()
{
  for(uint8_t i = 0; i < g_writebufpos; i++)
  {
    SerialOpt.write(g_writebuf[i]);
  }
  g_writebufpos = 0;
  //digitalWrite(LED_BLOCKOP_TOGGLE, !digitalRead(LED_BLOCKOP_TOGGLE));
}
#else
inline void bufferedWrite(uint8_t c) { SerialOpt.write(c); }
inline void flush_writebuf() {}
#endif

inline void consumeInputBuffer(int n)
{
  g_readPos += SerialOpt.skip(n);
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

inline void sync_breply(uint8_t b) {
  if(sync()) {
    SerialOpt.write(b);
    SerialOpt.write(Resp_STK_OK);
  }
}

inline void sync_breply_failed(uint8_t b) {
  if(sync()) {
    SerialOpt.write(b);
    SerialOpt.write(Resp_STK_FAILED);
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

inline void sync_reply_failed() {
  sync_reply(Resp_STK_FAILED);
}

