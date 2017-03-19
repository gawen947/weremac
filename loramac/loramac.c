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

/* This is the platform independent part of our LoRaMAC driver
   It does not have any code dependent on a specific platform
   (such as S7G2, RPi or Linux), it's just pure C. */

#include <string.h>

#include "loramac.h"
#include "crc-ccitt.h"

/* LoRaMAC configuration with platform dependent functions,
   source mac address and flags. */
static struct loramac_config mac_conf;

/* Internal state */
static uint8_t seqno;
static uint8_t last_ack_seqno;
static unsigned int wait_ack;

/* receive and send packetbuf [sz][frame...] */
static unsigned char snd_pktbuf[LORAMAC_MAX_FRAME + 1];
static unsigned char rcv_pktbuf[LORAMAC_MAX_FRAME + 1];

void loramac_init(const struct loramac_config *conf)
{
  mac_conf = *conf;
}

#define COPY_U16(crc, buf, v) do {                \
    *(uint16_t *)buf = mac_conf.htons(v);         \
    crc  = crc_ccitt(buf, sizeof(uint16_t), crc); \
    buf += sizeof(uint16_t);                      \
  } while(0)

#define COPY_U8(crc, buf, v) do {               \
    *(uint8_t *)buf = v;                        \
    crc = crc_ccitt(buf, sizeof(uint8_t), crc); \
    buf++;                                      \
  } while(0)

static int send_ack(uint16_t src, uint8_t seqno)
{
  unsigned char *buf = snd_pktbuf;

  *(uint8_t  *)buf = LORAMAC_ACK_SIZE; buf += sizeof(uint8_t);
  *(uint16_t *)buf = src;              buf += sizeof(uint16_t);
  *(uint8_t  *)buf = seqno;

  /* send packet */
  return mac_conf.uart_send(snd_pktbuf, snd_pktbuf[0] + 1);
}

static int loramac_send_helper(uint16_t dst, const void *payload, unsigned int payload_size)
{
  unsigned char *buf = snd_pktbuf + 1;
  uint16_t crc = CRC_CCITT_INIT;
  int ret;

  /* copy header */
  COPY_U16(crc, buf, mac_conf.mac_address);
  COPY_U16(crc, buf, dst);
  COPY_U8(crc, buf, ++seqno);

  /* copy payload */
  if(payload_size > LORAMAC_MAX_PAYLOAD)
    return LORAMAC_SND_TOOLONG;
  memcpy(buf, payload, payload_size);
  crc  = crc_ccitt(buf, payload_size, crc);
  buf += payload_size;

  /* copy CRC */
  *(uint16_t *)buf = mac_conf.htons(crc);

  /* copy frame size */
  snd_pktbuf[0] = LORAMAC_HDR_SIZE + payload_size;

  /* send packet */
  ret = mac_conf.uart_send(snd_pktbuf, snd_pktbuf[0] + 1);
  if(ret < 0)
    return ret;

  /* If we disabled ACK, we are done here.
     Otherwise we need to wait and check
     the last received ACK. */
  if(mac_conf.flags & LORAMAC_NOACK)
    return LORAMAC_SND_SUCCESS;

  wait_ack = 1;
  mac_conf.start_ack_timer(mac_conf.timeout);
  mac_conf.wait_ack_timer();

  if(last_ack_seqno != seqno)
    return LORAMAC_SND_NOACK;
  return LORAMAC_SND_SUCCESS;
}

int loramac_send(uint16_t dst, const void *payload, unsigned int payload_size)
{
  int ret;
  int retransmission;

  mac_conf.lock();
  for(retransmission = mac_conf.retrans ; retransmission ; retransmission--) {
    ret = loramac_send_helper(dst, payload, payload_size);

    if(ret == LORAMAC_SND_SUCCESS)
      break;
  }
  mac_conf.unlock();

  return ret;
}

#define READ_U16(status, buf, dst) do {         \
    buf -= sizeof(uint16_t);                    \
    if(buf <= rcv_pktbuf) {                     \
      status = LORAMAC_RCV_INVALID_HDR;         \
      goto PARSING_COMPLETED;                   \
    }                                           \
    else                                        \
      dst = mac_conf.ntohs(*(uint16_t *)buf);   \
  } while(0)

#define READ_U8(status, buf, dst) do {           \
    buf -= sizeof(uint8_t);                      \
    if(buf <= rcv_pktbuf) {                      \
      status = LORAMAC_RCV_INVALID_HDR;          \
      goto PARSING_COMPLETED;                    \
    }                                            \
    else                                         \
      dst = *(uint8_t *)buf;                     \
  } while(0)

static int recv_ack(void)
{
  unsigned char *buf = rcv_pktbuf + LORAMAC_ACK_SIZE + 1;
  uint8_t seqno;
  uint16_t src_mac;
  int status = LORAMAC_RCV_SUCCESS;

  /* parse ACK header */
  READ_U8(status, buf, seqno);
  READ_U16(status, buf, src_mac);

PARSING_COMPLETED:
  if(status != LORAMAC_RCV_SUCCESS)
    /* should not happen because the size
       is already fixed when we parse ACK. */
    return status;

  if(wait_ack && \
     src_mac == mac_conf.mac_address) {
    last_ack_seqno = seqno;
    wait_ack = 0;
    mac_conf.stop_ack_timer();
  }

  return status;
}

static int recv_data(unsigned int size)
{
  unsigned char *buf = rcv_pktbuf + size + 1;
  uint16_t expected_crc = CRC_CCITT_INIT;
  uint16_t frame_crc;
  uint16_t dst_mac;
  uint16_t src_mac;
  uint8_t  seqno;
  int status = LORAMAC_RCV_SUCCESS;

  /* for CRC we skip the size (first byte) and frame CRC (last two bytes) */
  expected_crc = crc_ccitt(rcv_pktbuf + 1, size - 2, expected_crc);

  /* parse CRC */
  READ_U16(status, buf, frame_crc);
  if(frame_crc != expected_crc) {
    status = LORAMAC_RCV_INVALID_CRC;
    goto PARSING_COMPLETED;
  }

  /* parse header [src_mac][dst_mac][seqno] */
  buf = rcv_pktbuf + sizeof(uint16_t) * 2 + sizeof(uint8_t) + 1;
  READ_U8(status, buf, seqno);
  READ_U16(status, buf, dst_mac);
  READ_U16(status, buf, src_mac);

  if(dst_mac != mac_conf.mac_address && \
     dst_mac != 0xffff) {
    /* wrong destination */
    status = LORAMAC_RCV_DESTINATION;
    goto PARSING_COMPLETED;
  }

PARSING_COMPLETED:
  /* based on parsing status and iface_flags
     we either return directly or pass the
     frame to the upper layer */
  switch(status) {
  case LORAMAC_RCV_INVALID_CRC:
  case LORAMAC_RCV_INVALID_HDR:
    if(!(mac_conf.flags & LORAMAC_INVALID))
      return status;
  case LORAMAC_RCV_DESTINATION:
    if(!(mac_conf.flags & LORAMAC_PROMISCUOUS))
      return status;
  case LORAMAC_RCV_BROADCAST:
    if(mac_conf.flags & LORAMAC_NOBROADCAST)
      return LORAMAC_RCV_BROADCAST;
  }

  /* send ACK when enabled */
  if(!(mac_conf.flags & LORAMAC_NOACK) && \
     status == LORAMAC_RCV_SUCCESS)
    send_ack(src_mac, seqno);

  /* send frame to upper layer */
  mac_conf.cb_recv(src_mac, dst_mac,
                   rcv_pktbuf + sizeof(uint16_t) * 2 + sizeof(uint8_t) + 1,
                   size - LORAMAC_HDR_SIZE,
                   status);
  return status;
}

int loramac_recv_frame(void)
{
  int size = rcv_pktbuf[0];

  if(size == LORAMAC_ACK_SIZE)
    return recv_ack();
  return recv_data(size);
}

int loramac_uart_putc(unsigned char c)
{
  static unsigned char *rcv_ptr = rcv_pktbuf;
  static uint8_t size;
  int status = LORAMAC_RCV_CONT; /* need more data */

  if(rcv_ptr == rcv_pktbuf)
    /* first byte (size) */
    size = c + 1;

  *rcv_ptr++ = c;
  size--;

  if(size == 0) {
    /* full frame received */
    status = mac_conf.recv_frame();

    /* reset pktbuf */
    rcv_ptr = rcv_pktbuf;
  }

  return status;
}
