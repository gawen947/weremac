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

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <err.h>

#include "safe-call.h"
#include "string-utils.h"
#include "g3plc-str.h"
#include "g3-plc/g3plc.h"
#include "version.h"
#include "common.h"
#include "mode.h"
#include "main.h"
#include "help.h"

/*
  The Unix mode read and write frames from/to a DGRAM UNIX socket.
  Send are not blocked and there is no confirmation for
  successful transmission. The message format for incoming
  and outgoing messages is specified below (see cb_recv() and
  start()).
*/

#define BUF_SIZE G3PLC_MAX_FRAME

static int sd;
static const char *socket_driver_path = PACKAGE "-driver.sock";
static const char *socket_app_path = PACKAGE "-app.sock";
static struct sockaddr_un client;

struct option unix_opts[] = {
  { "driver-path", required_argument, NULL, 'L' },
  { "app-path", required_argument, NULL, 'R' },
  { NULL, 0, NULL, 0 }
};
struct opt_help unix_messages[] = {
  { 'L', "driver-path", "Driver Unix socket path" },
  { 'R', "app-path", "Application Unix socket path" },
  { 0, NULL, NULL }
};

static void exit_clean(void)
{
  unlink(socket_driver_path);
  unlink(socket_app_path);
}

static void cb_recv(uint16_t src, uint16_t dst,
                    const void *payload, unsigned int payload_size,
                    int status, void *data)
{
  UNUSED(data);

  /* recv message format:
     [status (u8)][src (u16)][dst (u16)][payload] */
  unsigned char buf[BUF_SIZE];
  unsigned char *b = buf;
  size_t  len = payload_size + sizeof(uint8_t) + sizeof(uint16_t) * 2;
  ssize_t n;

  *(uint8_t  *)b = status; b += sizeof(uint8_t);
  *(uint16_t *)b = src;    b += sizeof(uint16_t);
  *(uint16_t *)b = dst;    b += sizeof(uint16_t);

  memcpy(b, payload, payload_size);

  n = sendto(sd, buf, len, 0, (struct sockaddr *)&client, sizeof(struct sockaddr_un));
  if(n != (ssize_t)len)
    warn("network error"); /* we don't fail on client error */
}

static void init(const struct context *ctx, struct g3plc_config *g3plc)
{
  struct sockaddr_un s_addr = { .sun_family = AF_UNIX };

  /* configure the G3-PLC layer */
  g3plc->cb_recv = cb_recv;

  /* create socket */
  sd = xsocket(AF_UNIX, SOCK_DGRAM, 0);

  /* bind to the specified unix socket */
  unlink(socket_driver_path);
  xstrcpy(s_addr.sun_path, socket_driver_path, sizeof(s_addr.sun_path));
  xbind(sd, (struct sockaddr *)&s_addr, SUN_LEN(&s_addr));

  /* prepare client address */
  client.sun_family = AF_UNIX;
  xstrcpy(client.sun_path, socket_app_path, sizeof(s_addr.sun_path));

  /* now we may register the exit function */
  atexit(exit_clean);

  IF_VERBOSE(ctx, printf("Socket created at %s", socket_driver_path));
}

static void start(const struct context *ctx)
{
  /* send message format:
     [dst (u16)][payload] */
  unsigned char buf[BUF_SIZE];
  uint16_t dst;
  unsigned int tx;
  int n, ret;

  while(1) {
    n = recv(sd, buf, BUF_SIZE, 0);
    if(n < 0) {
      warn("network error");
      continue; /* we don't fail on client error */
    }

    if(n <= (int)sizeof(uint16_t)) {
      warnx("message too short");
      continue;
    }

    dst = *(uint16_t *)buf;

    IF_VERBOSE(ctx, printf("Sending %d bytes to %04X\n",
                           n - (int)sizeof(uint16_t), dst));
    ret = g3plc_send(dst,
                       buf + sizeof(uint16_t),
                       n   - sizeof(uint16_t),
                       &tx);
    IF_VERBOSE(ctx, printf("TX STATUS: %s (%d)\n", g3plc_send2str(ret), ret));
    IF_VERBOSE(ctx, printf("TX COUNT : %d\n", tx));
    IF_VERBOSE(ctx, printf("---------\n"));
  }
}

static void destroy(const struct context *ctx)
{
  UNUSED(ctx);

  exit_clean();
}

static int parse_option(const struct context *ctx, int c)
{
  UNUSED(ctx);

  switch(c) {
  case 'L':
    socket_driver_path = optarg;
    return 1;
  case 'R':
    socket_app_path = optarg;
    return 1;
  }

  return 0;
}

struct iface_mode iface_mode = {
  .name = "unix",
  .description = "Read and write frame on a Unix socket",

  .optstring      = "R:L:",
  .long_opts      = unix_opts,
  .extra_messages = unix_messages,
  .parse_option   = parse_option,

  .init    = init,
  .destroy = destroy,
  .start   = start
};
