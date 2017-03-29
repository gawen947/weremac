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

#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include <err.h>

#include "common.h"
#include "main.h"
#include "string-utils.h"
#include "ping-mode.h"

enum ping_msg_type {
  PING_ECHO_REQUEST,
  PING_ECHO_REPLY
};

static void cb_recv(uint16_t src, uint16_t dst,
                    const void *payload, unsigned int payload_size,
                    int status, void *data)
{

}

static void before(const struct context  *ctx,
            struct loramac_config *loramac)
{
  UNUSED(ctx);
  loramac->cb_recv = cb_recv;
}

static void input(const struct context *ctx)
{
  /* send ping messages */
  seqno_t seqno = 0;
  unsigned int  count = ctx->count;
  unsigned char sbuf[LORAMAC_MAX_PAYLOAD];
  unsigned char *buf = sbuf;

  assert(ctx->size + PING_HDR_SIZE < LORAMAC_MAX_PAYLOAD);

  /* We fill the message with random bytes */
  fill_with_random(sbuf + PING_HDR_SIZE, ctx->size);

  while(1) {
    struct timeval tv;

    usleep(ctx->interval);

    /* Create the ping template */
    gettimeofday(&tv, NULL);

    /* ping header */
    *((uint8_t *)buf) = PING_ECHO_REQUEST; buf += sizeof(uint8_t);
    *((seqno_t *)buf) = seqno; buf += sizeof(seqno_t);
    *((struct timeval *)buf) = tv;

    /* stdout if needed */
    if(ctx->flood)
      write_slit(STDOUT_FILENO, ".");

    seqno++;

    if(count != 0) {
      count--;
      if(ctx->count == 0)
        break;
    }
  }
}

static void after(const struct context *ctx)
{
  UNUSED(ctx);
}

struct iface_mode ping_mode = {
  .name = "ping",
  .description = "Send echo-requests and answer with echo-replies",

  .before = before,
  .input  = input,
  .after  = after
};
