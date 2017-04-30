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
#include <stdint.h>

#include "lora/loramac.h"
#include "g3plc/g3plc.h"
#include "hybrid.h"

/* Check the return value of the function.
   Exit with its error when it is different than success. */
#define x_(n, fun, ...) do { \
    n = fun(__VA_ARGS__);    \
    if(n)                    \
      return n;              \
  } while(0)

static struct hybrid_config  hybrid;
static struct loramac_config lora;
static struct g3plc_config   g3plc;

void hybrid_lora_recv(uint16_t src, uint16_t dst,
                     const void *payload, unsigned int payload_size,
                     int status, void *data)
{

}

void hybrid_g3plc_recv(const struct g3plc_data_hdr *hdr,
                       const void *payload, unsigned payload_size,
                       int status, void *data)
{

}

int hybrid_init(const struct hybrid_config *conf)
{
  hybrid = *conf;

  /* derive child MAC layers from hybrid configuration */
  lora = (struct loramac_config){
    .uart_send   = conf->uart_lora_send,
    .cb_recv     = hybrid_lora_recv,
    .start_timer = conf->start_timer,
    .stop_timer  = conf->stop_timer,
    .wait_timer  = conf->wait_timer,
    .lock        = conf->lock,
    .unlock      = conf->unlock,
    .htons       = conf->htons,
    .ntohs       = conf->ntohs,
    .recv_frame  = conf->lora_recv_frame,
    .seqno       = conf->lora.seqno,
    .mac_address = conf->mac_address,
    .retrans     = conf->lora.retrans,
    .timeout     = conf->lora.timeout,
    .sifs        = conf->lora.sifs,
    .flags       = 0
  };
  g3plc = (struct g3plc_config){
    .callbacks = (struct g3plc_callbacks){
      .raw = NULL,
      .cb_recv = hybrid_g3plc_recv
    },

    .start_timer    = conf->start_timer,
    .stop_timer     = conf->stop_timer,
    .wait_timer     = conf->wait_timer,
    .uart_send      = conf->uart_g3plc_send,
    .uart_read      = conf->uart_g3plc_read,
    .set_uart_speed = conf->set_uart_g3plc_speed,
    .recv_frame     = conf->g3plc_recv_frame,
    .reset_clear    = conf->reset_clear,
    .reset_set      = conf->reset_set,
    .htons          = conf->htons,
    .htonl          = conf->htonl,
    .ntohs          = conf->ntohs,
    .ntohl          = conf->ntohl,
    .usleep         = conf->usleep,
    .bandplan       = conf->g3plc.bandplan,
    .pan_id         = conf->g3plc.pan_id,
    .mac_address    = conf->mac_address,
    .ext_address    = conf->g3plc.ext_address,
    .retrans        = conf->g3plc.retrans,
    .timeout        = conf->g3plc.timeout,
    .flags          = 0
  };

  if(conf->flags & HYBRID_INVALID) {
    lora.flags  |= LORAMAC_INVALID;
    g3plc.flags |= G3PLC_INVALID;
  }
  if(conf->flags & HYBRID_NOACK) {
    lora.flags  |= LORAMAC_NOACK;
    g3plc.flags |= G3PLC_NOACK;
  }

  /* init child layers */
  x_(loramac_init, &lora);
  x_(g3plc_init, &g3plc);

  return 0;
}

