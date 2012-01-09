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

static int bluec_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  // using a large page_size (which is in fact a block_size and not a page_size!) 
  return stk500_paged_load(pgm, p, m, 16384, n_bytes);
}


static int bluec_bulk_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                              int page_size, int n_bytes)
{
  int rc = n_bytes; // return code, negative on failure

  const unsigned long bufSize = 1024;
  unsigned char buf[bufSize]; // for receiving a failed verify notification we need some reserve
  int32_t i;

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
  const unsigned short windowSize = buf[2] | buf[3] << 8;
  unsigned long writePos = 0; // counting of the written protocol bytes, starts after reading BULK_WRITE_START_ACK
  unsigned long ackPos = 0; // counting of the acknowledged protocol bytes, starts after reading BULK_WRITE_START_ACK

  const unsigned short mtu = 126; // this is what I get for a RFComm commnection on Mac OSX 10.7 TODO: should be read from serial driver after connecting
  //const unsigned short minimumBlockSize = 64; // don't send smaller packets
  const unsigned long writeDataOverhead = 6; // the protocol overhead of STK_BULK_WRITE_DATA
  const unsigned long writeVrfyOverhead = 8; // the protocol overhead of BULK_WRITE_VRFY_ERR
  unsigned short block_size = mtu - writeDataOverhead; // netto data of one block
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
    report_progress (byteAddr, n_bytes, NULL);

    int32_t remain;
    int32_t availWindowSize;
    // TODO: handle the case when no whole block is available but no ACKs are arriving. Fallback to smaller blocks
    while((availWindowSize = (windowSize - (writePos - ackPos))) >= block_size)
    {
      // prog command
      remain = n_bytes - byteAddr;

      if(remain > 0)
      {
        unsigned short cursize = block_size;
        if(cursize > remain)
          cursize = remain;
        unsigned short crc = 0;
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
#ifdef DEBUG_TRACE_BULK
fprintf(stderr, "send(%6d, %6d, %6d, %6d)\n", writePos, ackPos, byteAddr, cursize);
#endif
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
#ifdef DEBUG_TRACE_BULK
fprintf(stderr, "END\n");
#endif
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
#ifdef DEBUG_TRACE_BULK
//fprintf(stderr, "recv(%6d, %6d, %6d, cmd=0x%02x)\n", writePos, ackPos, byteAddr, buf[0]);
#endif
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
          unsigned long ackAddr = buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24);
#ifdef DEBUG_TRACE_BULK
fprintf(stderr, "ackn(%6d, %6d, %6d, %6d, cmd=0x%02x)\n", writePos, ackPos, byteAddr, ackAddr, buf[0]);
#endif
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
          unsigned long vrfyAddr = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);
          unsigned short curSize = buf[5] | (buf[6] << 8);
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
          fprintf(stderr, "%s: bluec_bulk_write(): Verify error for addr 0x%04lx, page size %u\n", progname, vrfyAddr, curSize);
          fprintf(stderr, "  addr:  read != orig\n");
          int vi;
          for(vi = 0; vi < curSize; vi++)
          {
            if(buf[vi + writeVrfyOverhead - 1] != m->buf[vrfyAddr + vi])
            {
              fprintf(stderr, "0x%04lx:  0x%02x != 0x%02x\n", vrfyAddr + vi, buf[vi + writeVrfyOverhead - 1], m->buf[vrfyAddr + vi]);
            }
          }
          //rc = -3;
          break; // continue the loop

        case Resp_STK_BULK_WRITE_END_ACK:
#ifdef DEBUG_TRACE_BULK
fprintf(stderr, "END Ack\n");
#endif
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
  if (strcmp(m->desc, "flash") == 0)
  {
    int bulk_rc = bluec_bulk_write(pgm, p, m, page_size, n_bytes);
    if(bulk_rc >= 0)
      return bulk_rc;

  }

  // when bulk write failed or memtype!='flash' fall back to paged_write
  return stk500_paged_write(pgm, p, m, page_size, n_bytes);
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
}

// TODO: remove before committing patch
// vim:ts=2:sw=2:expandtab:smartindent:
