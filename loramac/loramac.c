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

/* Sender internal state */
static uint8_t seqno;
static uint8_t last_ack_seqno;
static unsigned int wait_ack;

/* receive and send packetbuf [sz][frame...] */
static unsigned char snd_pktbuf[LORAMAC_MAX_FRAME + 1];
static unsigned char rcv_pktbuf[LORAMAC_MAX_FRAME + 1];

/* Receiver ACK FIFO.
   We keep the seqno of the last ACK for the last n senders.
   We can use this to avoid displaying frames duplicated
   because of retransmissions. It is allocated once and we
   can ensure that it is large enough for the configured
   value of timeout and SIFS. */
static int oldest; /* index of oldest element in FIFO */
static int newest; /* index of newest element in FIFO */
static int ack_fifo_size; /* size of the FIFO */
static struct last_ack {
  uint16_t sender;
  uint8_t  seqno;
} *ack_fifo;

/* Search the ACK FIFO for a sender.
   Returns -1 if it wasn't found. */
static int ack_fifo_search(uint16_t sender)
{
  int i;
  for(i = 0 ; i < ack_fifo_size ; i++)
    if(ack_fifo[i].sender != 0xffff && ack_fifo[i].sender == sender)
      return i;
  return -1;
}

/* Insert a new sender into the FIFO.
   This does not check if the sender is already present.
   If the queue is full, it always removes the oldest
   element and insert this one in place. */
static void ack_fifo_insert(uint16_t sender, uint8_t seqno)
{
  if((newest + 1) % ack_fifo_size == oldest) {
    /* queue is full */
    ack_fifo[oldest] = (struct last_ack){ .sender = sender,
                                          .seqno  = seqno };
    oldest = (oldest + 1) % ack_fifo_size;
  }
  else {
    /* room available */
    newest = (newest + 1) % ack_fifo_size;
    ack_fifo[newest] = (struct last_ack){ .sender = sender,
                                          .seqno  = seqno };
  }
}

int loramac_init(const struct loramac_config *conf)
{
  int i;

  mac_conf = *conf;

  /* SIFS is the time until the receiver can send its ACK.
     So if timeout is lower than this value, ACKs will never
     be received in time by the sender. */
  if(mac_conf.timeout < mac_conf.sifs)
    return LORAMAC_INIT_TIMEVAL;

  /* The optimal ACK FIFO size on node 0 is for all node i:
       size_0 = max_i(timeout_i) / SIFS_0 + 1
     For simplification here we suppose that:
       max_i(timeout_i) = timeout_0
     Generally this value is really low since timeout is
     only slightly larger than SIFS. */
  ack_fifo_size = mac_conf.timeout / mac_conf.sifs + 1;
  if(ack_fifo_size > LORAMAC_MAX_ACK_FIFO)
    return LORAMAC_INIT_ACK_FIFO;

  ack_fifo = mac_conf.malloc(sizeof(struct last_ack) * ack_fifo_size);
  if(!ack_fifo)
    return LORAMAC_INIT_OOM;

  /* Initialize all sender to 0xffff.
     We know that a sender will never
     have 0xffff as its source address
     since this is the broadcast address.

     Note that we also initialize the seqno to 0.
     Otherwise an attacker might use this to snoop
     around into uninitialized memory. */
  for(i = 0 ; i < ack_fifo_size ; i++)
    ack_fifo[i] = (struct last_ack){ .sender = 0xffff,
                                     .seqno  = 0 };

  return LORAMAC_INIT_SUCCESS;
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
  COPY_U8(crc, buf, seqno);

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
  mac_conf.start_timer(mac_conf.timeout);
  mac_conf.wait_timer();

  if(last_ack_seqno != seqno)
    return LORAMAC_SND_NOACK;
  return LORAMAC_SND_SUCCESS;
}

int loramac_send(uint16_t dst, const void *payload, unsigned int payload_size, unsigned int *tx)
{
  int ret = LORAMAC_SND_NOACK;
  unsigned int retransmission;

  /* We lock the packet buffer when sending a packet.
     This will lock for the complete transmission
     (including ACK and retransmissions). */
  mac_conf.lock();
  {
    seqno++; /* Use same sequence number for retransmitted frames. */

    for(retransmission = 1 ; retransmission < mac_conf.retrans ; retransmission++) {
      ret = loramac_send_helper(dst, payload, payload_size);

      if(ret == LORAMAC_SND_SUCCESS)
        break;
    }
  }
  mac_conf.unlock();

  if(tx)
    *tx = retransmission;

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
    mac_conf.stop_timer();
  }

  return status;
}

static int recv_data(unsigned int size)
{
  unsigned char *buf = rcv_pktbuf + size + 1;
  uint16_t expected_crc = CRC_CCITT_INIT;
  uint16_t frame_crc;
  uint16_t dst_mac = 0x0000; /* invalid address */
  uint16_t src_mac = 0x0000; /* invalid address */
  uint8_t  seqno;
  int status = LORAMAC_RCV_SUCCESS;
  int i;

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
     status == LORAMAC_RCV_SUCCESS) {
    /* We have to wait before sending the ACK,
       otherwise the message is not sent correctly.
       Note that we don't use usleep and that the
       SIFS time can be quite large (>500ms). */
    mac_conf.lock();
    {
      mac_conf.start_timer(mac_conf.sifs);
      mac_conf.wait_timer();
      send_ack(src_mac, seqno);
    }
    mac_conf.unlock();

    /* check for retransmissions */
    i = ack_fifo_search(src_mac);
    if(i < 0)
      ack_fifo_insert(src_mac, seqno);
    else if(seqno == ack_fifo[i].seqno)
      /* skip retransmission */
      goto EXIT;
    else
      /* update seqno */
      ack_fifo[i].seqno = seqno;
  }

  /* send frame to upper layer */
  mac_conf.cb_recv(src_mac, dst_mac,
                   rcv_pktbuf + sizeof(uint16_t) * 2 + sizeof(uint8_t) + 1,
                   size - LORAMAC_HDR_SIZE,
                   status, mac_conf.data);

EXIT:
  return status;
}

int loramac_recv_frame(void)
{
  int size = rcv_pktbuf[0];

  if(size == LORAMAC_ACK_SIZE)
    return recv_ack();
  else
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
