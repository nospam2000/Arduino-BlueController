/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2009 Lars Immisch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* $Id: bluec.c 874 2009-11-02 23:52:52Z mludvig $ */

/*
 * avrdude interface for BlueController programmer
 *
 * The BlueController programmer is mostly a STK500v1, just with additional
 * features for better communication performance.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "pgm.h"
#include "stk500_private.h"
#include "stk500.h"
#include "serial.h"
#include "avr.h"

static int bluec_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  // XXX TODO: move this to a programmer specific file
  page_size = n_bytes;

  unsigned char buf[16];
  int memtype;
  unsigned int addr;
  int a_div;
  int tries;
  unsigned int n;
  int block_size;

  if (strcmp(m->desc, "flash") == 0) {
    memtype = 'F';
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    memtype = 'E';
  }
  else {
    return -2;
  }

  if ((m->op[AVR_OP_LOADPAGE_LO]) || (m->op[AVR_OP_READ_LO]))
    a_div = 2;
  else
    a_div = 1;

  if (n_bytes > m->size) {
    n_bytes = m->size;
    n = m->size;
  }
  else {
    if ((n_bytes % page_size) != 0) {
      n = n_bytes + page_size - (n_bytes % page_size);
    }
    else {
      n = n_bytes;
    }
  }

  for (addr = 0; addr < n; addr += page_size) {
    report_progress (addr, n_bytes, NULL);

    // MIB510 uses fixed blocks size of 256 bytes
    if ((strcmp(ldata(lfirst(pgm->id)), "mib510") != 0) &&
	(addr + page_size > n_bytes)) {
	   block_size = n_bytes % page_size;
	}
	else {
	   block_size = page_size;
	}
  
    tries = 0;
  retry:
    tries++;
    stk500_loadaddr(pgm, addr/a_div);
    buf[0] = Cmnd_STK_READ_PAGE;
    buf[1] = (block_size >> 8) & 0xff;
    buf[2] = block_size & 0xff;
    buf[3] = memtype;
    buf[4] = Sync_CRC_EOP;
    stk500_send(pgm, buf, 5);

    if (stk500_recv(pgm, buf, 1) < 0)
      exit(1);
    if (buf[0] == Resp_STK_NOSYNC) {
      if (tries > 33) {
        fprintf(stderr, "\n%s: stk500_paged_load(): can't get into sync\n",
                progname);
        return -3;
      }
      if (stk500_getsync(pgm) < 0)
	return -1;
      goto retry;
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      fprintf(stderr,
              "\n%s: stk500_paged_load(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -4;
    }

    if (stk500_recv(pgm, &m->buf[addr], block_size) < 0)
      exit(1);

    if (stk500_recv(pgm, buf, 1) < 0)
      exit(1);

    if(strcmp(ldata(lfirst(pgm->id)), "mib510") == 0) {
      if (buf[0] != Resp_STK_INSYNC) {
      fprintf(stderr,
              "\n%s: bluec_paged_load(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -5;
    }
  }
    else {
      if (buf[0] != Resp_STK_OK) {
        fprintf(stderr,
                "\n%s: bluec_paged_load(): (a) protocol error, "
                "expect=0x%02x, resp=0x%02x\n",
                progname, Resp_STK_OK, buf[0]);
        return -5;
      }
    }
  }

  return n_bytes;
}


static int bluec_bulk_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int rc = n_bytes; // return code, negative on failure

  const uint16_t bufSize = 1024;
  uint8_t buf[bufSize]; // for receiving a failed verify notification we need some reserve
  //int block_size;
  //int tries;
  int32_t i;
  //int flash;

  int a_div; // 1 means byte adressing, 2 means word addressing
  if ((m->op[AVR_OP_LOADPAGE_LO]) || (m->op[AVR_OP_READ_LO]))
  {
    a_div = 2;
  }
  else
  {
    // TODO: to make this work, the programmer code has to be adapted as well
    // TODO: add support of byte addresses and the OP-codes AVR_OP_WRITE_LO and AVR_OP_LOADPAGE_LO. Upload
    // the opcode table to the programmer
    a_div = 1;
    return -1000;
  }

  if (n_bytes > m->size) {
    n_bytes = m->size;
  }

  // TODO: implement RLE and CRC check on client and server side
  unsigned char bulkOptions = Optn_STK_BULK_WRITE_VERIFY;
  i = 0;
  buf[i++] = Cmnd_STK_BULK_WRITE_START;
  buf[i++] = bulkOptions;
  buf[i++] = Sync_CRC_EOP;
  stk500_send( pgm, buf, i);
  if (stk500_recv(pgm, buf, 5) < 0 || buf[0] != Resp_STK_BULK_WRITE_START_ACK || buf[4] != Sync_CRC_EOP)
  {
    fprintf(stderr,
          "%s: bluec_bulk_write(): BULK_WRITE_START failed, "
          "expect=0x%02x/0x%02x, resp=0x%02x/0x%02x\n", 
          progname, Resp_STK_BULK_WRITE_START_ACK, Sync_CRC_EOP, buf[0], buf[4]);
    rc = -1;
    return rc;
  }
  bulkOptions &= buf[1]; // maybe not all requested options could be enabled, so mask out the ones which aren't active
  const uint16_t windowSize = buf[2] | buf[3] << 8;
  uint32_t writePos = 0; // counting of the written protocol bytes, starts after reading BULK_WRITE_START_ACK
  uint32_t ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK

  const uint16_t mtu = 126; // this is what I get for a RFComm commnection on Mac OSX 10.7 TODO: should be read from serial driver after connecting
  //const uint16_t minimumBlockSize = 64; // don't send smaller packets
  const uint16_t writeDataOverhead = 6; // the protocol overhead of STK_BULK_WRITE_DATA
  const uint16_t writeVrfyOverhead = 8; // the protocol overhead of BULK_WRITE_VRFY_ERR
  uint16_t block_size = mtu - writeDataOverhead; // netto data of one block
  if((block_size + writeDataOverhead) > bufSize)
    block_size = bufSize - writeDataOverhead;
  if(block_size > (windowSize - 2) / 2) // make sure at least two blocks can be traveling at the same time
    block_size = (windowSize - 2) / 2;

  // for(ever)
  //   while(currentWindowsize allows sending BULK_WRITE_DATA)
  //     if EOF
  //       BULK_WRITE_END, exit both loops
  //     BULK_WRITE_DATA, depending on time and size conditions even packets smaller than MTU may be send
  //   read_event (and sleep until data has arrived)
  //     process events
  unsigned int byteAddr = 0;
  for(;;)
  {
    //report_progress (byteAddr, n_bytes, NULL); // TODO: disabled during debugging

    int32_t remain;
    int32_t availWindowSize;
    //int32_t loadAddr; // not really needed here
    //loadAddr = byteAddr / a_div; // TODO: calculate from byteAddr when needed
    // TODO: handle the case when no whole block is available but no ACKs are arriving. Fallback to smaller blocks
    while((availWindowSize = (windowSize - (writePos - ackPos))) >= block_size)
    {
      // prog command
      remain = n_bytes - byteAddr;

      if(remain > 0)
      {
        uint16_t cursize = block_size;
        if(cursize > remain)
          cursize = remain;
        uint16_t crc = 0;
        i = 0;
        buf[i++] = Cmnd_STK_BULK_WRITE_DATA;
        buf[i++] = crc & 0xff;
        buf[i++] = (crc >> 8) & 0xff;
        buf[i++] = cursize & 0xff;
        buf[i++] = (cursize >> 8) & 0xff;
        memcpy(&buf[i], &m->buf[byteAddr], cursize);
        i += cursize;
        buf[i++] = Sync_CRC_EOP;
        stk500_send(pgm, buf, i); // TODO: handle error
        writePos += i;
        byteAddr += cursize;
fprintf(stderr, "send(%6d, %6d, %6d, %6d)\n", writePos, ackPos, byteAddr, cursize);
      }
      else
      {
        break;
      }
    }

    const int BULK_WRITE_END_TELEGRAM_LEN = 2;
    remain = n_bytes - byteAddr;
    availWindowSize = (windowSize - (writePos - ackPos));
    if((remain == 0) && (availWindowSize >= BULK_WRITE_END_TELEGRAM_LEN))
    {
      // end of data reached, just handle outstanding notifications as long as ackPos < writePos
fprintf(stderr, "END\n");
      i = 0;
      buf[i++] = Cmnd_STK_BULK_WRITE_END;
      buf[i++] = Sync_CRC_EOP;
      stk500_send( pgm, buf, i);
      writePos += i;
      byteAddr += 1; // prohibits another sending of STK_BULK_WRITE_END
    }

    if (stk500_recv(pgm, buf, 1) < 0)
    {
      fprintf(stderr, "%s: bluec_bulk_write(): Timeout while waiting for event\n", progname);
      rc = -2;
      break;
    }
    else
    {
//fprintf(stderr, "recv(%6d, %6d, %6d, cmd=0x%02x)\n", writePos, ackPos, byteAddr, buf[0]);
      switch(buf[0])
      {
        case Resp_STK_BULK_WRITE_ACK:
          if (stk500_recv(pgm, &buf[1], 11 - 1) < 0)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Timeout while reading BULK_WRITE_ACK\n", progname);
            rc = -6;
            goto bulkReceiveEnd;
          }
          if(buf[10] != Sync_CRC_EOP)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Out of sync while reading BULK_WRITE_ACK\n", progname);
            rc = -9;
            goto bulkReceiveEnd;
          }
          ackPos = buf[6] | (buf[7] << 8) | (buf[8] << 16) | (buf[8] << 24);
          uint32_t ackAddr = buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24);
fprintf(stderr, "ackn(%6d, %6d, %6d, %6d, cmd=0x%02x)\n", writePos, ackPos, byteAddr, ackAddr, buf[0]);
          if(buf[1] == Stat_STK_BULK_ACKSTATE_NAK)
          {
            // retransmit from given address; be careful not to mix byteAddr and loadAddr!
            byteAddr = ackAddr; // set send position back to addr before error
            i = 0;
            buf[i++] = Cmnd_STK_BULK_SET_BYTEADDR;
            buf[i++] = byteAddr & 0xff;
            buf[i++] = (byteAddr >> 8) & 0xff;
            buf[i++] = (byteAddr >> 16) & 0xff;
            buf[i++] = (byteAddr >> 24) & 0xff;
            buf[i++] = Sync_CRC_EOP;
            stk500_send(pgm, buf, i); // TODO: handle error
            writePos += i;
            // TODO: limit number of retries
          }
          else
          {
            // TODO: handle other cases

            // end of data reached, just handle outstanding notifications as long as ackPos < writePos
          }
          break;

        case Resp_STK_BULK_WRITE_VRFY_ERR:
          if (stk500_recv(pgm, &buf[1], 7-1) < 0)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Timeout while reading VRFY_ERR\n", progname);
            rc = -4;
            goto bulkReceiveEnd;
          }
          uint32_t vrfyAddr = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
          uint16_t curSize = buf[5] | (buf[6] << 8);
          if ((curSize + writeVrfyOverhead) > bufSize)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Overflow while reading data of VRFY_ERR\n", progname);
            rc = -7;
            goto bulkReceiveEnd;
          }
          if (stk500_recv(pgm, &buf[7], curSize + 1) < 0) // +1 for the ending Sync_CRC_EOP
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Timeout while reading page data of VRFY_ERR\n", progname);
            rc = -5;
            goto bulkReceiveEnd;
          }
          if(buf[curSize + writeVrfyOverhead - 1] != Sync_CRC_EOP)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Out of sync while reading data of VRFY_ERR\n", progname);
            rc = -8;
            goto bulkReceiveEnd;
          }
          // TODO: add detailed error message. We continue flashing but show the error.
          fprintf(stderr, "%s: bluec_bulk_write(): Verify error for addr 0x%04x, page size %u\n", progname, vrfyAddr, curSize);
          fprintf(stderr, "  addr:  read != orig\n");
          int vi;
          for(vi = 0; vi < curSize; vi++)
          {
            if(buf[vi + writeVrfyOverhead - 1] != m->buf[vrfyAddr + vi])
            {
              fprintf(stderr, "0x%04x:  0x%02x != 0x%02x\n", vrfyAddr + vi, buf[vi + writeVrfyOverhead - 1], m->buf[vrfyAddr + vi]);
            }
          }
          //rc = -3;
          break; // continue the loop

        case Resp_STK_BULK_WRITE_END_ACK:
fprintf(stderr, "END Ack\n");
          if (stk500_recv(pgm, &buf[1], 7-1) < 0)
          {
            fprintf(stderr, "%s: bluec_bulk_write(): Timeout while reading BULK_WRITE_END_ACK\n", progname);
            rc = 100;
          }
          else if(buf[6] != Sync_CRC_EOP)
          {
            fprintf(stderr,
                  "%s: bluec_bulk_write(): BULK_WRITE_END failed, "
                  "expect=0x%02x, resp=0x%02x\n", 
                  progname, Sync_CRC_EOP, buf[6]);
            rc = -101;
          }
          else if(buf[1] != 0)
          {
            // TODO: detailed analysis of the error
            fprintf(stderr,
                  "%s: bluec_bulk_write(): BULK_WRITE_END failed, "
                  "ACK.error=0x%02x\n", 
                  progname, buf[1]);
            rc = -102;
          }
          // after writing the last data byte, the byteAddr is 1 greater than the buffer size, therefore
          // we must use byteAddr - 1 here
          else if((buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24)) != (byteAddr - 1))
          {
            fprintf(stderr,
                  "%s: bluec_bulk_write(): BULK_WRITE_END failed, "
                  "ACK.addr(0x%04x) != endAddr(0x%04x)\n", 
                  progname, buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24), byteAddr - 1);
            rc = -103;
          }
          goto bulkReceiveEnd;
          break;

        default:
          fprintf(stderr, "%s: bluec_bulk_write(): Unknown event received, id=0x%02x\n", progname, buf[0]);
          rc = -3;
          goto bulkReceiveEnd;
          break;
      }
    }
  }
bulkReceiveEnd:

  
  if(rc > 0)
  {
  }

  return rc;
}


static int bluec_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int memtype;
  unsigned int addr;
  int a_div;
  int block_size;
  int tries;
  unsigned int n;
  unsigned int i;
  int flash;

  if (page_size == 0) {
    // MIB510 uses page size of 256 bytes
    if (strcmp(ldata(lfirst(pgm->id)), "mib510") == 0) {
      page_size = 256;
    }
    else {
      page_size = 128;
    }
  }

  unsigned char buf[4 + page_size + 16];
  //fprintf(stderr, "\n%s: bluec_paged_write(): page_size=%d\n", progname, page_size);

  if (strcmp(m->desc, "flash") == 0) {
    memtype = 'F';
    flash = 1;
  }
  else if (strcmp(m->desc, "eeprom") == 0) {
    memtype = 'E';
    flash = 0;
  }
  else {
    return -2;
  }

  if ((m->op[AVR_OP_LOADPAGE_LO]) || (m->op[AVR_OP_READ_LO]))
    a_div = 2;
  else
    a_div = 1;

  if (n_bytes > m->size) {
    n_bytes = m->size;
    n = m->size;
  }
  else {
    if ((n_bytes % page_size) != 0) {
      n = n_bytes + page_size - (n_bytes % page_size);
    }
    else {
      n = n_bytes;
    }
  }

#if 0
  fprintf(stderr, 
          "n_bytes   = %d\n"
          "n         = %u\n"
          "a_div     = %d\n"
          "page_size = %d\n",
          n_bytes, n, a_div, page_size);
#endif     

  if(flash == 1)
  {
    int bulk_rc = bluec_bulk_write(pgm, p, m, page_size, n_bytes);
    if(bulk_rc >= 0)
      return bulk_rc;

    // when bulk write failed, fall back to paged_write
  }

  int windowSize = 0;
  long curSend = 0;
  long curAck = 0;
  for (addr = 0; addr < n; addr += page_size) {
    report_progress (addr, n_bytes, NULL);
    
    // MIB510 uses fixed blocks size of 256 bytes
    if ((strcmp(ldata(lfirst(pgm->id)), "mib510") != 0) &&
      (addr + page_size > n_bytes)) {
	      block_size = n_bytes % page_size;
    }
	  else {
	   block_size = page_size;
	  }

    /* Only skip on empty page if programming flash. */
    if (flash) {
      if (stk500_is_page_empty(addr, block_size, m->buf)) {
          continue;
      }
    }
    tries = 0;
//  retry:
    tries++;
    //stk500_loadaddr(pgm, addr/a_div);
    //curSend++;

    /* build command block and avoid multiple send commands as it leads to a crash
        of the silabs usb serial driver on mac os x */
    i = 0;
    // loadaddress command
    // TODO: combining two commands together won't work on slow programmers
    unsigned int loadAddr = addr/a_div;
    buf[i++] = Cmnd_STK_LOAD_ADDRESS;
    buf[i++] = loadAddr & 0xff;
    buf[i++] = (loadAddr >> 8) & 0xff;
    buf[i++] = Sync_CRC_EOP;

    // prog command
    buf[i++] = Cmnd_STK_PROG_PAGE;
    buf[i++] = (block_size >> 8) & 0xff;
    buf[i++] = block_size & 0xff;
    buf[i++] = memtype;
    memcpy(&buf[i], &m->buf[addr], block_size);
    i += block_size;
    buf[i++] = Sync_CRC_EOP;
    stk500_send( pgm, buf, i);
    curSend += 2; // 2 commands

    while(curSend > (curAck + windowSize*2))
    {
      curAck++;
      //fprintf(stderr, "\n%s: stk500_paged_write(): curSend=%ld, curAck=%ld\n", progname, curSend, curAck);
      if (stk500_recv(pgm, buf, 1) < 0)
        exit(1);
      if (buf[0] == Resp_STK_NOSYNC) {
        //if (tries > 33) {
          fprintf(stderr, "\n%s: stk500_paged_write(): can't get into sync\n",
                  progname);
          return -3;
        //}
        //if (stk500_getsync(pgm) < 0)
    //return -1;
        //goto retry; // TODO: will not work with windowed ack
      }
      else if (buf[0] != Resp_STK_INSYNC) {
        fprintf(stderr,
                "\n%s: stk500_paged_write(): (a) protocol error, "
                "expect=0x%02x, resp=0x%02x\n", 
                progname, Resp_STK_INSYNC, buf[0]);
        return -4;
      }
      
      if (stk500_recv(pgm, buf, 1) < 0)
        exit(1);
      if (buf[0] != Resp_STK_OK) {
        fprintf(stderr,
                "\n%s: stk500_paged_write(): (a) protocol error, "
                "expect=0x%02x, resp=0x%02x\n", 
                progname, Resp_STK_INSYNC, buf[0]);
        return -5;
      }
    }
  }


  while(curSend > curAck)
  {
    curAck++;
    //fprintf(stderr, "\n%s: stk500_paged_write(): curAck=%ld\n", progname, curAck);
    if (stk500_recv(pgm, buf, 1) < 0)
      exit(1);
    if (buf[0] == Resp_STK_NOSYNC) {
      //if (tries > 33) {
        fprintf(stderr, "\n%s: stk500_paged_write(): can't get into sync\n",
                progname);
        return -3;
      //}
      //if (stk500_getsync(pgm) < 0)
  //return -1;
      //goto retry; // TODO: will not work with windowed ack
    }
    else if (buf[0] != Resp_STK_INSYNC) {
      fprintf(stderr,
              "\n%s: stk500_paged_write(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -4;
    }
    
    if (stk500_recv(pgm, buf, 1) < 0)
      exit(1);
    if (buf[0] != Resp_STK_OK) {
      fprintf(stderr,
              "\n%s: stk500_paged_write(): (a) protocol error, "
              "expect=0x%02x, resp=0x%02x\n", 
              progname, Resp_STK_INSYNC, buf[0]);
      return -5;
    }
  }

  return n_bytes;
}



void bluec_initpgm(PROGRAMMER * pgm)
{
	/* This is mostly a STK500; just using bulk communication optimized for
	 * bluetooth.
         * The DTR signal doesn't matter because it doesn't support auto-reset via DTR */
  stk500_initpgm(pgm);

  strcpy(pgm->type, "BlueController");
  //pgm->read_sig_bytes = arduino_read_sig_bytes;
  pgm->paged_write    = bluec_paged_write;
  pgm->paged_load     = bluec_paged_load;
  pgm->page_size      = 256;
}

// TODO: remove before committing patch
// vim:ts=2:sw=2:expandtab:smartindent:
