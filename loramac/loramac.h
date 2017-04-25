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

#ifndef _LORAMAC_H_
#define _LORAMAC_H_

#include <stdint.h>

#define LORAMAC_MAJOR       3
#define LORAMAC_MINOR       4

/* We limit the frame size to 63 bytes. After reading the code
   on the LoRaMAC module, a larger frame would result in a buffer
   overflow that could change the internal state of the module. */
#define LORAMAC_MAX_FRAME   0x3f
#define LORAMAC_HDR_SIZE    (sizeof(uint16_t) * 3 + sizeof(uint8_t)) /* src, dst, crc, seqno */
#define LORAMAC_ACK_SIZE    (sizeof(uint16_t) + sizeof(uint8_t))     /* src, seqno */
#define LORAMAC_MAX_PAYLOAD LORAMAC_MAX_FRAME - LORAMAC_HDR_SIZE

/*
   LoRaMAC data frame format:
     [src_mac (16)][dst_mac (16)][seqno (8)]<payload...>[crc-ccitt(16)]

   LoRaMAC ACK frame format:
     [src_mac (16)][seqno (8)]
*/

/* LoRaMAC driver initialization flags */
enum loramac_flags {
  LORAMAC_PROMISCUOUS = 0x1, /* do not filter packets to another destination */
  LORAMAC_INVALID     = 0x2, /* do not filter invalid packets (packet header, CRC) */
  LORAMAC_NOBROADCAST = 0x4, /* ignore broadcast messages (0xffff) */
  LORAMAC_NOACK       = 0x8, /* do not answer nor expect ACKs */
};

/* Status of a received frame */
enum loramac_receive_status {
  LORAMAC_RCV_SUCCESS,
  LORAMAC_RCV_CONT,        /* no frame were parsed (need more data) */
  LORAMAC_RCV_INVALID_CRC, /* CRC does not match received frame */
  LORAMAC_RCV_INVALID_HDR, /* invalid header */
  LORAMAC_RCV_DESTINATION, /* not destinated to this interface */
  LORAMAC_RCV_BROADCAST,   /* broadcast message ignored */
};

/* Status of a sent frame */
enum loramac_send_status {
  LORAMAC_SND_SUCCESS,
  LORAMAC_SND_TOOLONG, /* payload too long */
  LORAMAC_SND_NOACK    /* maximum number of retransmissions reached */
};

struct loramac_config {
  /* The driver will use the uart_send() function to write
     the resulting frame on the device's UART. This
     functions should return a negative value in case of
     error or 0 on success. */
  int  (*uart_send)(const void *buf, unsigned int size);

  /* The driver will call cb_recv() when a frame has been
     received (frames may be filtered according to the
     loramac_flags). The status argument of this function reflect
     the parsing status (see loramac_receive_status). It is also
     possible to pass a context pointer to this callback. This
     should be initialized in loramac_config. */
  void (*cb_recv)(uint16_t src, uint16_t dst,
                  const void *payload, unsigned int payload_size,
                  int status, void *data);

  /* The driver will use those two functions to start, stop and wait
     for the ACK timer. The stop function should also drop any wait in
     place on the timer. */
  void (*start_timer)(unsigned int us);
  void (*stop_timer)(void);
  void (*wait_timer)(void);

  /* We only send one packet at a time. We are forced to do
     this unless we can start multiple referenced timers
     within the same period. So until then we lock/unlock
     the send function to ensure this behavior. We also lock
     the receive function that also uses the timer to send
     its ACK in response to the received packet. Note that
     we use the same lock for sending and receiving since we
     don't generally send and receive at the same time. */
  void (*lock)(void);
  void (*unlock)(void);

  /* Not all platform provide byte ordering functions
     with the same names as POSIX. */
  uint16_t (*htons)(uint16_t v);
  uint16_t (*ntohs)(uint16_t v);

  /* The function loramac_uart_putc() is generally called
     from an interrupt handler. Since we cannot parse
     the frame in this handler, we defer processing when
     a complete frame has been received. Control is then
     transferred to this function which can either directly
     be loramac_recv_frame() or use semaphores to defer
     outside of the interrupt context. */
  int (*recv_frame)(void);

  uint16_t mac_address;  /* device short MAC address */
  unsigned int  retrans; /* maximum number of retransmissions */
  unsigned int  timeout; /* ACK timeout in us */
  unsigned int  sifs;    /* Short Inter Frame Spacing time in us */
  unsigned long flags;   /* (see loramac_flags) */

  void *data; /* context data passed to user callbacks */
};

/* Initialize the LoRaMAC driver (see loramac_config) */
void loramac_init(const struct loramac_config *conf);

/* Assemble and send a frame to the specified destination using LoRaMAC.
   The broadcast address is 0xffff. When ACK is enabled, this function
   will block until the packet has been successfully transmitted. For
   the error returned see loramac_send_status. */
int loramac_send(uint16_t dst, const void *payload, unsigned int payload_size);

/* Start the processing of a frame. Can be called either automatically
   when a complete frame has been received or manually from another thread. */
int loramac_recv_frame(void);

/* Called by the platform dependent part of the driver when a character
   has been received on UART from the device. This function can block
   when a full frame has been received. It may also block indefinitely
   if the receive callback itself is blocked. Note that this function
   is *NOT* reentrant. You have to wait for its completion until you
   can call it again. */
int loramac_uart_putc(unsigned char c);

#endif /* _LORAMAC_H_ */
