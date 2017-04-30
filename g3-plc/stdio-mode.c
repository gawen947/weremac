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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <err.h>

#include "g3-plc/g3plc-str.h"
#include "g3-plc/g3plc.h"
#include "help.h"
#include "main.h"
#include "dump.h"
#include "common.h"
#include "mode.h"

#define PROMPT   "input> "
#define BUF_SIZE G3PLC_MAX_PAYLOAD


static void cb_recv(const struct g3plc_data_hdr *hdr,
                    const void *payload, unsigned payload_size,
                    int status, void *data)
{
  UNUSED(data);

  putchar('\n');
  printf("FROM %04X TO %04X:\n", (uint16_t)hdr->src_addr, (uint16_t)hdr->dst_addr);
  hex_dump(payload, payload_size);
  printf("RX STATUS: %s (%d)\n", g3plc_rcv2str(status), status);
}

static void init(const struct context  *ctx, struct g3plc_config *g3plc)
{
  UNUSED(ctx);
  g3plc->callbacks.cb_recv = cb_recv;
}

static void start(const struct context *ctx)
{
  int ret;
  char buf[BUF_SIZE];

  while(1) {
    printf(PROMPT);

    /* Interruption often occurs on stdin because
       we use the timers a lot. So we *have* to
       handle syscall interruptions. */
  RESTART:
    if(!fgets(buf, sizeof(buf), stdin)) {
      switch(errno) {
      case EINTR:
        goto RESTART;
      default:
        errx(EXIT_FAILURE, "cannot read from stdin");
      }
    }

    /* strip newline */
    strtok(buf, "\n");

    if(!strcmp(buf, "quit"))
      return;
    else if(!strcmp(buf, "exit"))
      return;

    ret = g3plc_send(ctx->dst_mac, buf, strlen(buf));
    printf("TX STATUS: %s (%d)\n", g3plc_send2str(ret), ret);
  }
}

static void destroy(const struct context *ctx)
{
  UNUSED(ctx);
}

static int parse_option(const struct context *ctx, int c)
{
  UNUSED(ctx);
  UNUSED(c);

  return 0; /* option unknown by this module,
               can be be parsed by next module */
}

struct option stdio_opts[]       = { { NULL, 0, NULL, 0 } };

struct iface_mode iface_mode = {
  .name        = "stdio",
  .description = "Read output frame and print received frames on stdio",

  .optstring      = "",
  .long_opts      = stdio_opts,
  .extra_messages = NULL,
  .parse_option   = parse_option,

  .init    = init,
  .destroy = destroy,
  .start   = start
};
