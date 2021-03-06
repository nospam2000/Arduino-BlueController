/*
 * avrdude support for BlueController ISP programmer
 * Copyright (C) 2011 Michael Dreher <michael@5dot1.de>
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

/* $Id: bluec.c $ */

/*
 * avrdude interface for BlueController programmer
 *
 * The BlueController programmer is mostly a STK500v1, just with additional
 * features for better communication performance over high latency communication
 * paths like Bluetooth or WLAN (around factor 5 faster, especially on iMacs).
 * Uses asynchronous acknowledge, RLE compression and implicit verify.
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "stk500_private.h"
#include "stk500.h"
#include "serial.h"

typedef long flashSize_t;
typedef long flashOffset_t;
typedef unsigned short blockSize_t;

const blockSize_t mtu = 126; // this is what I get for a RFComm connnection on Mac OSX 10.7
//const blockSize_t minimumBlockSize = 64; // don't send smaller packets
const unsigned long writeDataOverhead = 6; // the protocol overhead of STK_BULK_WRITE_DATA
const unsigned long writeVrfyOverhead = 8; // the protocol overhead of BULK_WRITE_VRFY_ERR

static int bluec_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  // using a large page_size (which is in fact a block_size and not a page_size!) 
  return stk500_paged_load(pgm, p, m, 16384, n_bytes);
}

// 'dstLen' is the maximum length of the _payload_, for the complete allowed frame length you have to add the protocol overhead
int bluec_stuff_sendbuf(unsigned char* dstBuf, flashSize_t dstLen, unsigned char* srcBuf,
  flashSize_t srcLen, flashSize_t* dstUsed, flashSize_t *srcUsed)
{
  flashSize_t cursize = dstLen;
  if(cursize > srcLen)
    cursize = srcLen;
  unsigned short crc = 0; // TODO: has to be calculated
  int i = 0;
  dstBuf[i++] = Cmnd_STK_BULK_WRITE_DATA;
  dstBuf[i++] = crc & 0xff;
  dstBuf[i++] = (crc >> 8) & 0xff;
  dstBuf[i++] = cursize & 0xff;
  dstBuf[i++] = (cursize >> 8) & 0xff;

  memcpy(&dstBuf[i], srcBuf, cursize);
  i += cursize;
  dstBuf[i++] = Sync_CRC_EOP;

  *dstUsed = i;
  *srcUsed = cursize;

  return 0;
}

// compress the data before sending using a simple run-length-encoding
int bluec_stuff_sendbuf_rle(unsigned char* dstBuf, flashSize_t dstLen, unsigned char* srcBuf,
  flashSize_t srcLen, flashSize_t* dstUsed, flashSize_t *srcUsed)
{
  unsigned short crc = 0; // TODO: has to be calculated
  flashSize_t i = 0;
  dstBuf[i++] = Cmnd_STK_BULK_WRITE_DATA;
  i += 2*2; // skip crc and payload length fields here, these will be filled below

  int srcRemain;
  int dstRemain;
  flashSize_t j;
  flashSize_t repCnt;
  for(j = 0; ((srcRemain = srcLen - j) > 0) && ((dstRemain = dstLen - i) > 0); j += repCnt)
  {
    unsigned char val = srcBuf[j];
    for(repCnt = 1; (repCnt <= srcRemain) && (repCnt < 0xffff) && (val == srcBuf[j + repCnt]); repCnt++)
    { }

    if((val == 0xff) && (repCnt == 1) && (dstRemain >= 2))
    {
      // escape a single 0xFF => 0xFF 0x00
      dstBuf[i++] = 0xFF;
      dstBuf[i++] = 0x00;
    }
    else if((val == 0xff) && (repCnt == 2) && (dstRemain >= 2))
    {
      // optimized case for 0xFF 0xFF => 0xFF 0x01
      dstBuf[i++] = 0xFF;
      dstBuf[i++] = 0x01;
    }
    else if((val != 0xff) && (repCnt <= 3) && (dstRemain >= 1))
    {
      // normal case, directly submit value
      repCnt = 1;
      dstBuf[i++] = val;
    }
    else if((repCnt <= 254) && (dstRemain >= 3))
    {
      // 8-bit repeat count
      dstBuf[i++] = 0xFF;
      dstBuf[i++] = repCnt;
      dstBuf[i++] = val;
    }
    else if((repCnt > 254) && (dstRemain >= 5))
    {
      // 16-bit repeat count
      dstBuf[i++] = 0xFF;
      dstBuf[i++] = 0xFF;
      dstBuf[i++] = repCnt & 0xff;
      dstBuf[i++] = (repCnt >> 8) & 0xff;
      dstBuf[i++] = val;
    }
    else
    {
      // doesn't fit into current frame
      break;
    }
  }

  dstBuf[i++] = Sync_CRC_EOP;
  dstBuf[1] = crc & 0xff;
  dstBuf[2] = (crc >> 8) & 0xff;
  dstBuf[3] = (i - writeDataOverhead) & 0xff;
  dstBuf[4] = ((i - writeDataOverhead) >> 8) & 0xff;

  *dstUsed = i;
  *srcUsed = j;
  return 0;
}


static int bluec_bulk_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int rc = n_bytes; // return code, negative on failure
  long orig_serial_recv_timeout = serial_recv_timeout;

  const blockSize_t bufSize = 2048; // for receiving a failed verify notification we need some reserve
  unsigned char buf[bufSize];
  flashSize_t i;

  // the bulk mode currently only supports page mode programming
  if (!(m->op[AVR_OP_LOADPAGE_LO]))
  {
    return -1000;
  }

  if (n_bytes > m->size) {
    n_bytes = m->size;
  }

  // TODO: implement CRC check on client and server side
  unsigned char bulkOptions = Optn_STK_BULK_WRITE_VERIFY | Optn_STK_BULK_WRITE_RLE;
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
  // The RLE compression may transmit a lot of bytes which need to be processed, so the programmer
  // will be busy for a long time. This happened with the default timeout (5 seconds) and a 32kB
  // file which consists of very good compressable data.
  // TODO: could be solved by more acks for such large blocks
	serial_recv_timeout = 15000;
  bulkOptions &= buf[1]; // maybe not all requested options could be enabled, so mask out the ones which aren't active
  const blockSize_t windowSize = buf[2] | buf[3] << 8;
  flashOffset_t writePos = 0; // counting of the written protocol bytes, starts after reading BULK_WRITE_START_ACK
  flashOffset_t ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK

  blockSize_t block_size = mtu - writeDataOverhead; // netto data of one block
  if((block_size + writeDataOverhead) > bufSize)
    block_size = bufSize - writeDataOverhead;
  if(block_size > (windowSize - 2) / 2) // make sure at least two blocks can be traveling at the same time
    block_size = (windowSize - 2) / 2;

  flashOffset_t byteAddr = 0;
  for(;;)
  {
    report_progress (byteAddr, n_bytes, NULL);

    flashSize_t remain;
    flashSize_t availWindowSize;
    // TODO: handle the case when no whole block is available but no ACKs are arriving. Fallback to smaller blocks
    if (verbose > 3)
      fprintf(stderr, "\n%s: bluec_bulk_write() availWin=%ld, win=%u, writePos=%ld, ackPos=%ld, block_size=%u\r",
        progname, windowSize - (writePos - ackPos), windowSize, writePos, ackPos, block_size);
    while((availWindowSize = (windowSize - (writePos - ackPos))) >= block_size)
    {
      // prog command
      remain = n_bytes - byteAddr;
      if(remain > 0)
      {
        i = 0;
        flashSize_t dstUsed;
        flashSize_t srcUsed;
        if(bulkOptions & Optn_STK_BULK_WRITE_RLE)
          bluec_stuff_sendbuf_rle(buf, block_size, &m->buf[byteAddr], remain, &dstUsed, &srcUsed);
        else
          bluec_stuff_sendbuf(buf, block_size, &m->buf[byteAddr], remain, &dstUsed, &srcUsed);
        i += dstUsed;
        stk500_send(pgm, buf, i);
        writePos += i;
        byteAddr += srcUsed;
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
          ackPos = buf[6] | (buf[7] << 8) | (buf[8] << 16) | (buf[9] << 24);
          unsigned long ackAddr = buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24);
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
            stk500_send(pgm, buf, i);
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
          unsigned long vrfyAddr = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
          blockSize_t curSize = buf[5] | (buf[6] << 8);
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
          fprintf(stderr, "%s: bluec_bulk_write(): Verify error for page 0x%04lx, page size %u\n", progname, vrfyAddr, curSize);
          fprintf(stderr, "  addr:  read != orig\n");
          int vi;
          for(vi = 0; vi < curSize; vi++)
          {
            if(buf[vi + writeVrfyOverhead - 1] != m->buf[vrfyAddr + vi])
            {
              fprintf(stderr, "0x%04lx:  0x%02x != 0x%02x\n", vrfyAddr + vi, buf[vi + writeVrfyOverhead - 1], m->buf[vrfyAddr + vi]);
            }
          }
          // TODO: We continue flashing
          //rc = -3;
          break; // continue the loop

        case Resp_STK_BULK_WRITE_END_ACK:
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
                  "ACK.addr(0x%04lx) != endAddr(0x%04lx)\n", 
                  progname,
                  (flashOffset_t)buf[2] | ((flashOffset_t)buf[3] << 8) | ((flashOffset_t)buf[4] << 16) | ((flashOffset_t)buf[5] << 24),
                  byteAddr - 1);
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

	serial_recv_timeout = orig_serial_recv_timeout;
  return rc;
}


static int bluec_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  if (strcmp(m->desc, "flash") == 0)
  {
    int rc = bluec_bulk_write(pgm, p, m, page_size, n_bytes);
    if(rc >= 0)
      return rc;
    // when failed try it again with the standard stk500 function
  }
  else if (strcmp(m->desc, "eeprom") == 0)
  {
    // This is a workaround because stk500v1 assumes word addressing only when either the
    // AVR_OP_LOADPAGE_LO or the AVR_OP_READ_LO instruction is supported. Force it here to
    // always use a word address (a_div=2).
    // TODO actually I think it is a bug in stk500v1 that stk500_paged_write() uses a_div=1 with the page write algorithm
    // TODO stk500v1 should fall back to not use page write at all when the opcode is not supported
    OPCODE* pOrigOp = m->op[AVR_OP_READ_LO];
    OPCODE dummyOp;
    int i;
    for(i = 0; i < (sizeof(dummyOp.bit) / sizeof(dummyOp.bit[0])); i++)
    {
      dummyOp.bit[i].type = AVR_CMDBIT_IGNORE;
      dummyOp.bit[i].bitno = 0;
      dummyOp.bit[i].value = 0;
    }
    if (!(m->op[AVR_OP_READ_LO]))
      m->op[AVR_OP_READ_LO] = &dummyOp; // use dummy op during call
    int rc = stk500_paged_write(pgm, p, m, 3*128, n_bytes);
    m->op[AVR_OP_READ_LO] = pOrigOp; // restore original op
    if(rc >= 0)
      return rc;
    // when failed try it again with the standard stk500 function
  }

  // other mem types and when the bluec funtions fails use the standard stk500v1 functions and parameters
  return stk500_paged_write(pgm, p, m, page_size, n_bytes);
}


void bluec_initpgm(PROGRAMMER * pgm)
{
  /* This is mostly a STK500; just using bulk communication optimized for
   * bluetooth.
   * The DTR signal doesn't matter because it doesn't support auto-reset via DTR */
  stk500_initpgm(pgm);

  strcpy(pgm->type, "BlueController");
  pgm->paged_write    = bluec_paged_write;
  pgm->paged_load     = bluec_paged_load;
}

// TODO: remove before committing patch
// vim:ts=2:sw=2:expandtab:smartindent:autoindent
