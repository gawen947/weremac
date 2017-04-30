/* Copyright (c) 2017, David Hauweele <david@hauweele.net>
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <err.h>

#include "xatoi.h"
#include "uart.h"
#include "g3-plc/g3plc.h"

static int fd;

speed_t baud(const char *arg)
{
  int err;

  const struct {
    int     intval;
    speed_t baud;
  } *b, bauds[] = {
    { 230400, B230400 },
    { 115200, B115200 },
    { 57600, B57600 },
    { 38400, B38400 },
    { 19200, B19200 },
    { 9600, B9600 },
    { 4800, B4800 },
    { 2400, B2400 },
    { 1800, B1800 },
    { 1200, B1200 },
    { 300, B300 },
    { 200, B200 },
    { 150, B150 },
    { 134, B134 },
    { 110, B110 },
    { 75, B75 },
    { 50, B50 },
    { 0,  B0 }};

  int arg_val = xatou(arg, &err);
  if(err)
    goto ERR;

  for(b = bauds; b->intval ; b++)
    if(b->intval == arg_val)
      return b->baud;

ERR:
  errx(EXIT_FAILURE, "unrecognized speed");
}

void serial_init(const char *path, speed_t speed)
{
  struct termios tty = {
    .c_cflag = (CS8 | CLOCAL | CREAD) & ~CRTSCTS,
    .c_iflag = IGNPAR,
    .c_oflag = 0,
    .c_lflag = 0,
    .c_cc[VMIN]  = 1,
    .c_cc[VTIME] = 0,
  };

  fd = open(path, O_RDWR | O_NOCTTY);
  if(fd < 0)
    err(EXIT_FAILURE, "cannot open serial port");

  /* initial checks */
  if(!isatty(fd))
    err(EXIT_FAILURE, "invalid serial port");

  /* we only setup the speed if requested */
  if(speed != B0)
    cfsetspeed(&tty, speed);

  if(tcsetattr(fd, TCSANOW, &tty) < 0)
    err(EXIT_FAILURE, "cannot set tty attributes");

  /* Some operating systems (eg Linux) bufferise the UART input
     even when the file descriptor is not opened. This may be
     useful in a lot of ca ses. However we may lay with incomplete
     messages on the UART buffer. Therefore we have to flush the
     buffer ourself. However for a reason unknown to me, we have
     to wait a bit before actually flushing. Otherwise the flush
     command would have no effect. */
  usleep(500);
  tcflush(fd, TCIOFLUSH);
}

int uart_send(const void *buf, unsigned int size)
{
  int r = write(fd, buf, size);

  if(r < 0)
    return r;
  return 0;
}

void uart_read_loop(void)
{
  unsigned char buf[UART_BUFFER_SIZE];

  /* loop for messages */
  while(1) {
    int i;
    ssize_t size = read(fd, buf, UART_BUFFER_SIZE);
    if(size <= 0) {
      if(errno == EINTR)
        /* signal caught */
        continue;
      err(EXIT_FAILURE, "cannot read");
    }

    /* flush buffer */
    for(i = 0 ; i < size ; i++)
      loramac_uart_putc(buf[i]);
  }
}
