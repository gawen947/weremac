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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

#include "loramac.h"
#include "help.h"
#include "main.h"
#include "dump.h"
#include "common.h"
#include "mode.h"

#define BUF_SIZE 0x80

static void cb_recv(uint16_t src, uint16_t dst,
                    const void *payload, unsigned int payload_size,
                    int status, void *data)
{
  UNUSED(data);

  /* FIXME: Use generic show in another unit. */
  printf("%04X->%04X (status: %d):\n", src, dst, status);
  hex_dump(payload, payload_size);
}

static void init(const struct context  *ctx, struct loramac_config *loramac)
{
  UNUSED(ctx);
  loramac->cb_recv = cb_recv;
}

static void start(const struct context *ctx)
{
  char buf[BUF_SIZE];

  while(1) {
    printf("input> ");
    fgets(buf, sizeof(buf), stdin);

    if(!strcmp(buf, "quit"))
      return;
    else if(!strcmp(buf, "exit"))
      return;

    loramac_send(ctx->dst_mac, buf, strlen(buf));
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
