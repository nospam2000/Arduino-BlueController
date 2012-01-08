/*
  BulkMode.h - Functions for asynchronous communication using a sliding window
  Copyright (c) 2011 Michael Dreher <michael@5dot1.de>.  All rights reserved.
*/

inline bool verify_current_page(void)
{
  bool rc = true;
  for(int i = 0; i < g_verifyAddr; i++)
  {
    uint8_t flashVal;
    if(a_div == 2)
      flashVal = flash_read(i & 0x01, g_curPage + i/2); // word address
    else
      flashVal = flash_read(HIGH, g_curPage + i); // byte address // TODO: do we need HIGH or LOW as parameter?
    if(flashVal != g_verifyBuf[i])
    {
      uint32_t localByteAddr = g_curPage * a_div;
      bufferedWrite(Resp_STK_BULK_WRITE_VRFY_ERR);
      bufferedWrite(localByteAddr & 0xff);
      bufferedWrite((localByteAddr >> 8) & 0xff);
      bufferedWrite((localByteAddr >> 16) & 0xff);
      bufferedWrite((localByteAddr >> 24) & 0xff);

      bufferedWrite(g_verifyAddr & 0xff);
      bufferedWrite((g_verifyAddr >> 8) & 0xff);

      int j;
      for(j = 0; j < i; j++) // the part before the current index is identical to the buffer, so we don't have to read it again from flash
      {
        bufferedWrite(g_verifyBuf[j]);
      }
      bufferedWrite(flashVal); // the one which was different
      j++;
      for( ; j < g_verifyAddr; j++) // the second part has to be read from flash
      {
        if(a_div == 2)
          flashVal = flash_read(j & 0x01, g_curPage + j/2); // word address
        else
          flashVal = flash_read(HIGH, g_curPage + j); // byte address // TODO: do we need HIGH or LOW as parameter?
        bufferedWrite(flashVal);
      }

      bufferedWrite(Sync_CRC_EOP);
      flush_writebuf();
      
      rc = false;
      break;
    }
  }
  
  g_verifyAddr = 0;
  return rc;
}

inline bool commitWithVerify(int addr) {
  bool rc = true;
  if(g_outStandingCommit)
  {
    rc = commit(addr);

    if((g_bulkOptions & Optn_STK_BULK_WRITE_VERIFY) && (g_verifyAddr > 0))
      rc = verify_current_page(); // only do this in bulk mode; outside bulk mode g_verifyAddr will be always 0
    //digitalWrite(LED_BLOCKOP_TOGGLE, !digitalRead(LED_BLOCKOP_TOGGLE));
  }
  g_verifyAddr = 0;

  return rc;
}

inline bool ProgByteLocation(uint32_t progByteAddr, uint8_t val)
{
  bool rc = true;
  uint32_t localLoadAddr = (a_div == 2) ? (progByteAddr / 2) : progByteAddr;
  int newPage = pageFromAddr(localLoadAddr);
  if (newPage != g_curPage)
  {
    // this has to be handled every time the page is changed and on the very end of programming   
    rc = commitWithVerify(g_curPage);
    if((g_curPage >> 16) != (newPage >> 16))
    {
      set_ext_addr(newPage);
    }
    g_curPage = newPage;
  }

  // account for byte/word addressing and if HIGH or LOW opcode for even/odd addresses have to be used
  // TODO: handle byte addressing, that means do we have to use HIGH or LOW in case of byte addressing?
  uint8_t hiLo = (a_div == 2) ? ((progByteAddr & 0x01) ? HIGH : LOW) : HIGH;

  flashByte(hiLo, localLoadAddr, val);
  g_verifyBuf[g_verifyAddr++] = val;
}

inline void exitBulkMode(void)
{
  g_commandMode = CMD_MODE_NORMAL;
  commitWithVerify(g_curPage); // must be always called when setting g_commandMode back to CMD_NORMAL_MODE
  g_verifyAddr = 0;
  g_bulkOptions = 0;
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
    if((g_bulkOptions & Optn_STK_BULK_WRITE_RLE) && (val == 0xff))
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
      ProgByteLocation(g_byteAddr++, val);
    }
  }
  return;

rleError:
  abortBulkMode();
  return;
}

inline bool isAckAppropriate()
{
  bool sendAck = false;
  uint32_t lag = g_readPos - g_ackPos;
  if(lag > 0)
  {
    // if lag gets too big compared to the previous sent ack, send an ack
    // refer to AckTimeCalculator.ods for the values
    uint32_t diffTime = millis() - g_prevAckTime;
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
    g_ackPos = g_readPos;
    uint8_t ackState = Stat_STK_BULK_ACKSTATE_ACK; // TODO: handle NAK
    bufferedWrite(Resp_STK_BULK_WRITE_ACK);
    bufferedWrite(ackState);

    bufferedWrite(g_byteAddr & 0xff);
    bufferedWrite((g_byteAddr >> 8) & 0xff);
    bufferedWrite((g_byteAddr >> 16) & 0xff);
    bufferedWrite((g_byteAddr >> 24) & 0xff);

    bufferedWrite(g_ackPos & 0xff);
    bufferedWrite((g_ackPos >> 8) & 0xff);
    bufferedWrite((g_ackPos >> 16) & 0xff);
    bufferedWrite((g_ackPos >> 24) & 0xff);
    bufferedWrite(Sync_CRC_EOP);
    flush_writebuf();
    g_prevAckTime = millis();
  }  
}

inline void processBulkModeCommand(uint8_t cmd)
{
  if(cmd == Cmnd_STK_BULK_WRITE_START) // start bulk programming mode
  {
    if(g_commandMode == CMD_MODE_BULK_WRITE)
    {
      // Error: we are already in bulk mode and this command is not allowed to be sent a second time
      abortBulkMode();
    }
    else
    {
      g_commandMode = CMD_MODE_BULK_WRITE;
      // read rest of Cmnd_STK_BULK_WRITE_START telegram
      if(!waitAvailable(3 - 1) || (SerialOpt.peek(2 - 1) != Sync_CRC_EOP)) // offset -1 because cmdByte has already be consumed
      {
        abortBulkMode();
      }
      else
      {
        g_bulkOptions = SerialOpt.peek(2 - 1) & supportedOptions;
        bufferedWrite(Resp_STK_BULK_WRITE_START_ACK);
        bufferedWrite(g_bulkOptions);
        bufferedWrite(windowSize & 0xff);
        bufferedWrite((windowSize >> 8) & 0xff);
        bufferedWrite(Sync_CRC_EOP);
        flush_writebuf();
        consumeInputBuffer(3 - 1);
      }

      g_readPos = 0; // counting of the read protocol bytes, starts after reading BULK_WRITE_START_ACK
      g_ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK
      g_byteAddr = 0;
      g_verifyAddr = 0;
      g_curPage = 0;
      g_outStandingCommit = false;
      a_div = (g_bulkOptions & Optn_STK_BULK_WRITE_BYTE_ADR) ? 1 : 2; // 1: byte addressing, 2: word addressing
      if((g_deviceParam.flashsize / a_div) > 0x10000)
      {
        set_ext_addr(g_curPage);
      }
    }  
  }
  else if(cmd == Cmnd_STK_BULK_WRITE_END)
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
      
      bufferedWrite(g_byteAddr & 0xff);
      bufferedWrite((g_byteAddr >> 8) & 0xff);
      bufferedWrite((g_byteAddr >> 16) & 0xff);
      bufferedWrite((g_byteAddr >> 24) & 0xff);
    
      bufferedWrite(Sync_CRC_EOP);
      flush_writebuf();
      exitBulkMode();
    }
  }
  else if(cmd == Cmnd_STK_BULK_WRITE_DATA)
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
        // start programming using the current g_byteAddr
        // the buffer may already contain a partial page
        ProcessReceivedBlock(blockLen);
        
        // at the end consume the bytes from the serial receive buffer, and adjust the g_readPos
        consumeInputBuffer(blockLen + 1);        
      }
    }
  }
  else if(cmd == Cmnd_STK_BULK_SET_BYTEADDR)
  {
    // read rest of Cmnd_STK_BULK_LOAD_ADDRESS telegram
    if(!waitAvailable(6 - 1) || (SerialOpt.peek(5 - 1) != Sync_CRC_EOP)) // offset -1 because cmdByte has already be consumed
    {
      abortBulkMode();
    }
    else
    {
      // when changing the g_byteAddr, there might be some bytes in the page buffer which have to be processed before
      // this is done automatically when in ProgByteLocation() after calculating the loadAddr
      g_byteAddr = peekLe32(1 - 1);
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


