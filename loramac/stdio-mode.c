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
#include <string.h>

#include "loramac.h"
#include "main.h"
#include "dump.h"
#include "common.h"
#include "stdio-mode.h"

#define BUF_SIZE 0x80

void cb_recv(uint16_t src, uint16_t dst,
             const void *payload, unsigned int payload_size,
             int status)
{
  /* FIXME: Use generic show in another unit. */
  printf("%04X->%04X (status: %d):\n", src, dst, status);
  hex_dump(payload, payload_size);
  printf("-----------\n");
}

void before(const struct context  *ctx,
            struct loramac_config *loramac)
{
  UNUSED(ctx);

  loramac->cb_recv = cb_recv;
}

void input(const struct context *ctx)
{
  char buf[BUF_SIZE];

  /* FIXME: read lines correctly, use iobuf? */
  while(1) {
    printf("input> ");
    fgets(buf, sizeof(buf), stdin);
    loramac_send(ctx->dst_mac, buf, strlen(buf));
  }
}

void after(const struct context *ctx)
{
  UNUSED(ctx);
}

struct iface_mode stdio_mode = {
  .name = "stdio",
  .description = "Read output frame and print received frames on stdio.",

  .before = before,
  .input  = input,
  .after  = after
};
