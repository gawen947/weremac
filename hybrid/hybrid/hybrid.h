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

/* This is the platform independent part of our hybrid driver
   It does not have any code dependent on a specific platform
   (such as S7G2, RPi or Linux), it's just pure C. */

#ifndef _HYBRID_H_
#define _HYBRID_H_

#include <stdint.h>

#define HYBRID_MAJOR 1
#define HYBRID_MINOR 4

/* When one of the child layers result in an error,
   this error is reported in one of the two variables.
   The error code returned by the hybrid layer specifies
   if the LoRa MAC layer or G3PLC is at fault. */
extern int lora_errno;
extern int g3plc_errno;

enum hybrid_flags {
  HYBRID_INVALID = 0x1, /* do not filter invalid packets (packet header, CRC) */
  HYBRID_NOACK   = 0x2, /* enable ACK communications */
};

enum hybrid_source {
  HYBRID_SOURCE_LORA,  /* packet received from LoRa */
  HYBRID_SOURCE_G3PLC, /* packet received from G3PLC */
};

struct hybrid_config {
  /* The driver will call cb_recv() when a frame has been
     received (frames may be filtered according to the
     loramac_flags). The status argument of this function reflect
     the parsing status (see *_receive_status). The physical interface
     on which the packet was received is specified in source (see hybrid_source)
     It is also possible to pass a context pointer to this callback. This
     should be initialized in hybrid_config. */
  void (*cb_recv)(uint16_t src, uint16_t dst,
                  const void *payload, unsigned int payload_size,
                  int status, int source, void *data);

    /* The driver will use those functions to start, stop and wait
     for timers. The stop function should also drop any wait in
     place on the timer. */
  void (*start_timer)(unsigned int us);
  void (*stop_timer)(void);
  void (*wait_timer)(void);

  /* The driver will use the uart_send() function to write
     the resulting frame on the device's UART. This
     function should return a negative value in case of
     error or 0 on success. */
  int (*uart_lora_send)(const void *buf, unsigned int size);
  int (*uart_g3plc_send)(const void *buf, unsigned int size);

  /* The g3plc_reset() function will use this to read
     from the device during the boot sequence. This
     function should return a negative value in case of
     error or 0 on success. */
  int (*uart_g3plc_read)(void *buf, unsigned int size);

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

  /* Change UART speed.
     This take an integer (not a speed_t type).
     It should handle the following speed:
       - 1M
       - 500k
       - 115.2k
     and return a negative number on error. */
  int (*set_uart_g3plc_speed)(unsigned int speed);

  /* The function *_uart_putc() is generally called
     from an interrupt handler. Since we cannot parse
     the frame in this handler, we defer processing when
     a complete frame has been received. Control is then
     transferred to this function which can either directly
     be hybrid_recv_frame() or use semaphores to defer
     outside of the interrupt context. */
  int (*lora_recv_frame)(void);
  int (*g3plc_recv_frame)(void);

  /* Clear/set the reset pin. */
  void (*reset_clear)(void);
  void (*reset_set)(void);

  /* Not all platform provide byte ordering functions
     with the same names as POSIX. */
  uint16_t (*htons)(uint16_t v);
  uint32_t (*htonl)(uint32_t v);
  uint16_t (*ntohs)(uint16_t v);
  uint32_t (*ntohl)(uint32_t v);

  /* Microseconds sleep. */
  void (*usleep)(unsigned long us);

  /* LoRaMAC options */
  struct lora_opt {
    /* Initial sequence number.
       This can be randomized so that multiple instances
       of  the same host (i.e. same source address) with
       ACK e  nabled do not result in frames being filtered
       as retran  smissions by the receiver (see ACK FIFO). */
    uint8_t seqno;
    unsigned int  retrans; /* maximum number of retransmissions */
    unsigned int  timeout; /* ACK timeout in us */
    unsigned int  sifs;    /* Short Inter Frame Spacing time in us */
  } lora;

  /* G3-PLC options */
  struct g3plc_opt {
    uint8_t bandplan;     /* bandplan (see g3plc_bandplan) */
    uint16_t pan_id;      /* PAN ID */
    uint64_t ext_address; /* extended 64-bit address */
    unsigned int retrans; /* maximum number of retransmissions */
    unsigned int timeout; /* request timeout in us */
  } g3plc;

  unsigned long flags;   /* (see hybrid_flags) */
  uint16_t mac_address;  /* device short MAC address */
  void *data;            /* context data passed to user callbacks */
};

/* Initialize the HYBRID driver (see hybrid_config).
   Return 0 on success, for other errror codes see hybrid_init_status. */
int hybrid_init(const struct hybrid_config *conf);

/* Assemble and send a frame to the specified destination using HYBRID.
   When ACK is enabled, this function will block until the packet has
   been successfully transmitted. For the error see hybrid_send_status.
   If the tx pointer is not null, it is replaced with the number of
   transmissions necessary to succesfully send the packet. */
int hybrid_send(uint16_t dst, const void *payload, unsigned int payload_size);

/* Start the processing of a frame. Can be called either automatically
   when a complete frame has been received or manually from another thread. */
int hybrid_lora_recv_frame(void);
int hybrid_g3plc_recv_frame(void);

/* Called by the platform dependent part of the driver when a character
   has been received on UART from the device. This function can block
   when a full frame has been received. It may also block indefinitely
   if the receive callback itself is blocked. Note that this function
   is *NOT* reentrant. You have to wait for its completion until you
   can call it again. */
int hybrid_lora_uart_putc(unsigned char c);
int hybrid_g3plc_uart_putc(unsigned char c);

#endif /* _HYBRID_H_ */
