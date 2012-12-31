/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright (C) 2006 Joerg Wunsch <j@uriah.heep.sax.de>
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

/* $Id: bt_macos.m xxx 2011-08-27 10:03:23Z michael_dreher $ */

/*
 * bluetooth interface for avrdude.
 */

#if !defined(WIN32NATIVE)

/*
   * If the port name starts with "bt", divert the serial routines
   * to the Bluetooth ones.  The serial_open() function for bluetooth
   * uses the following format for the port name:
   *   bt:mac=11-22-33-44-55-66:pin=1234
   *   bt:name=BlueController:pin=1234
   *   the ":pin=" part is optional and only needed if the device is not already paired
*/

#import "RFComm.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "avrdude.h"
#include "serial.h"

double bt_recv_timeout = 5.000; /* seconds */


static int bt_setspeed(union filedescriptor *fd, long baud)
{
  return 0;
}

static int bt_set_dtr_rts(union filedescriptor *fdp, int is_on)
{
#if 0
  unsigned int	ctl;
  int           r;

  r = ioctl(fdp->ifd, TIOCMGET, &ctl);
  if (r < 0) {
    perror("ioctl(\"TIOCMGET\")");
    return -1;
  }

  if (is_on) {
    /* Clear DTR and RTS */
    ctl &= ~(TIOCM_DTR | TIOCM_RTS);
  }
  else {
    /* Set DTR and RTS */
    ctl |= (TIOCM_DTR | TIOCM_RTS);
  }

  r = ioctl(fdp->ifd, TIOCMSET, &ctl);
  if (r < 0) {
    perror("ioctl(\"TIOCMSET\")");
    return -1;
  }
#endif

  return 0;
}

static void bt_open(char * port, long baud, union filedescriptor *fdp)
{
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];

  /*
   * If the port is of the form "bt:mac=<port>:name=<name>:pin=<pin>", then
   *   bt:mac=11-22-33-44-55-66:pin=1234
   *   bt:name=BlueController:pin=1234
   *   the ":pin=" part is optional and only needed if the device is not already paired
   * handle it as a TCP connection to a terminal server.
   */
  if (strncmp(port, "bt:", strlen("bt:")) != 0) {
    fprintf(stderr, "Portname for bluetooth must start with \"bt:\"\n");
    exit(1);
  }

  /*
   * open the serial port
   */
  //NSString *macAdrStr = @"00-12-6F-20-DF-55"; // TODO: MAC of BlueController2; the address must be a variable parameter
  NSString *macAdrStr = @"00-12-6F-21-F5-FC"; // TODO: MAC of BlueController3: the address must be a variable parameter
  RFComm *rfc = [[RFComm alloc] init];
  IOReturn ioret = [rfc connectToPairedSppDevByMacAddr:macAdrStr]; 
  if (!rfc || ioret != kIOReturnSuccess) {
    fprintf(stderr, "%s: bt_open(): can't open device \"%s\": %x\n",
            progname, port, ioret);
    [rfc release];
    rfc = nil;
    exit(1);
  }
  fdp->pfd = rfc;

  [pool drain];
}


static void bt_close(union filedescriptor *fdp)
{
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
  RFComm *rfc = fdp->pfd;
  if(rfc)
  {
    IOReturn ioret = [rfc closeChannel];
    [rfc release];
    rfc = fdp->pfd = nil;
  }

  [pool drain];
}


static int bt_send(union filedescriptor *fdp, unsigned char * buf, size_t buflen)
{
  int rc = -1;
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
  RFComm *rfc = fdp->pfd;
  if(rfc && buflen > 0)
  {
    if (verbose > 3)
    {
        fprintf(stderr, "%s: Send: ", progname);

        while (buflen) {
          unsigned char c = *buf;
          if (isprint(c)) {
            fprintf(stderr, "%c ", c);
          }
          else {
            fprintf(stderr, ". ");
          }
          fprintf(stderr, "[%02x] ", c);

          buf++;
          buflen--;
        }

        fprintf(stderr, "\n");
    }
    int bytesWritten = [rfc writeSync:buf nbyte:buflen]; // TODO: add custom timeout
    if(bytesWritten != buflen)
    {
      fprintf(stderr, "%s: bt_send(): write error\n", progname);
      rc = -2; // TODO: use a real error code here or better exit(1)?
      //bt_close(fdp);
      //exit(1);
    }
    else
    {
      rc = 0;
    }
  }

  [pool drain];

  return rc;
}


static int bt_recv(union filedescriptor *fdp, unsigned char * buf, size_t buflen)
{
  int rc = -1;
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
  RFComm *rfc = fdp->pfd;
  if(rfc && buflen > 0)
  {
    size_t bytesRead = [rfc readSync:buf nbyte:buflen timeOut:bt_recv_timeout];
    if(bytesRead != buflen)
    {
      fprintf(stderr, "%s: bt_recv(): programmer is not responding\n", progname);
      rc = -2; // TODO: use a real error code here or better exit(1)?
      //bt_close(fdp);
      //exit(1);
    }
    else
    {
      rc = 0;
      if (verbose > 3)
      {
          fprintf(stderr, "%s: Recv: ", progname);

          size_t len;
          unsigned char *p = buf;
          for(len = bytesRead; len > 0; len--, p++) {
            unsigned char c = *p;
            if (isprint(c)) {
              fprintf(stderr, "%c ", c);
            }
            else {
              fprintf(stderr, ". ");
            }
            fprintf(stderr, "[%02x] ", c);
          }
          fprintf(stderr, "\n");
      }
    }
  }
  else if(buflen == 0)
  {
    rc = 0;
  }

  [pool drain];

  return rc;
}


static int bt_drain(union filedescriptor *fdp, int display)
{
  int rc = 0;
  NSAutoreleasePool * pool = [[NSAutoreleasePool alloc] init];
  RFComm *rfc = fdp->pfd;
  if (display) {
    fprintf(stderr, "drain>");
  }
  if(rfc)
  {
    int bytesRead;
    unsigned char buf;
    do {
      bytesRead = [rfc readSync:&buf nbyte:sizeof(buf) timeOut:0.250];
      if(display && bytesRead == sizeof(buf)) {
          fprintf(stderr, "%02x ", buf);
      }
    } while(bytesRead == sizeof(buf));
  }
  if (display) {
    fprintf(stderr, "<drain\n");
  }

  [pool drain];

  return rc;
}

struct serial_device bt_serdev =
{
  .open = bt_open,
  .setspeed = bt_setspeed,
  .close = bt_close,
  .send = bt_send,
  .recv = bt_recv,
  .drain = bt_drain,
  .set_dtr_rts = bt_set_dtr_rts,
  .flags = SERDEV_FL_CANSETSPEED,
};

#endif  /* WIN32NATIVE */
