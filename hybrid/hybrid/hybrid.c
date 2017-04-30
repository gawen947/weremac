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

/* child layers error code */
int lora_errno;
int g3plc_errno;

/* Check the return value of the function.
   Exit with its error when it is different than success. */
#define xLORA_(n, fun, ...) do { \
    n = fun(__VA_ARGS__);        \
    if(n) {                      \
      lora_errno = n;            \
      return HYBRID_ERR_LORA;    \
    }                            \
  } while(0)
#define xG3PLC_(n, fun, ...) do { \
    n = fun(__VA_ARGS__);        \
    if(n) {                      \
      g3plc_errno = n;           \
      return HYBRID_ERR_G3PLC;   \
    }                            \
  } while(0)

static struct hybrid_config  hybrid;
static struct loramac_config lora;
static struct g3plc_config   g3plc;

/* Initialization status */
enum hybrid_init_status {
  HYBRID_INIT_SUCCESS,
};

/* Status of a sent frame/command */
enum hybrid_send_status {
  HYBRID_SND_SUCCESS,
  HYBRID_SND_TOOLONG,       /* payload too long */
  HYBRID_SND_NOACK,         /* maximum number of retransmissions reached */
};

void hybrid_lora_recv(uint16_t src, uint16_t dst,
                     const void *payload, unsigned int payload_size,
                     int status, void *data)
{
  hybrid.cb_recv(src, dst, payload, payload_size, status,
                 HYBRID_SOURCE_LORA, hybrid.data);
}

void hybrid_g3plc_recv(const struct g3plc_data_hdr *hdr,
                       const void *payload, unsigned payload_size,
                       int status, void *data)
{
  hybrid.cb_recv(hdr->src_addr, hdr->dst_addr,
                 payload, payload_size,
                 status, HYBRID_SOURCE_G3PLC, hybrid.data);
}

int hybrid_init(const struct hybrid_config *conf)
{
  int n;

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
    .flags       = 0,
    .data        = conf->data
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
    .flags          = 0,
    .data           = conf->data
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
  xLORA_(n, loramac_init, &lora);
  xG3PLC_(n, g3plc_init, &g3plc);

  return HYBRID_INIT_SUCCESS;
}

static int hybrid_lora_send(uint16_t dst, const void *payload, unsigned int payload_size)
{
  int r;

  r = loramac_send(dst, payload, payload_size, NULL);
  switch(r) {
  case LORAMAC_SND_SUCCESS:
    return HYBRID_SND_SUCCESS; /* finally! */
  case LORAMAC_SND_NOACK: /* nothing we can do... */
    return HYBRID_SND_NOACK;
  default:
    lora_errno = r;
    return HYBRID_ERR_LORA;
  }
}

int hybrid_send(uint16_t dst, const void *payload, unsigned int payload_size)
{
  int r;

  if(payload_size > HYBRID_MAX_PAYLOAD)
    return HYBRID_SND_TOOLONG;

  r = g3plc_send(dst, payload, payload_size);
  switch(r) {
  case G3PLC_SND_SUCCESS:
    return HYBRID_SND_SUCCESS; /* great! */
  case G3PLC_SND_NOACK:
  case G3PLC_SND_ACCESS: /* let's try LoRa instead */
    return hybrid_lora_send(dst, payload, payload_size);
  default:
    g3plc_errno = r;
    return HYBRID_ERR_G3PLC;
  }
}

int hybrid_lora_recv_frame(void)
{
  return loramac_recv_frame();
}

int hybrid_g3plc_recv_frame(void)
{
  return g3plc_recv_frame();
}

int hybrid_lora_uart_putc(unsigned char c)
{
  return loramac_uart_putc(c);
}

int hybrid_g3plc_uart_putc(unsigned char c)
{
  return g3plc_uart_putc(c);
}
