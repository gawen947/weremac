/* Copyright (c) 2017, Aurélien Van Laere <aurelien.vanlaere@gmail.com>
                       David Hauweele <david@hauweele.net>
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

/* This is the platform independent part of our G3-PLC driver
   It does not have code dependent on a specific platform
   (such as S7G2, RPi or Linux), it's just standard ISO C. */

#ifndef _G3PLC_H_
#define _G3PLC_H_

#include <stdint.h>

#include "g3plc-cmd.h"
#include "cmdbuf.h"

#define G3PLC_MAJOR 2
#define G3PLC_MINOR 3

#define G3PLC_DATA_HDR_SIZE 28 /* see G3-PLC Serial Command Spec. p51 */
#define G3PLC_MAX_PAYLOAD   G3PLC_MAX_CMD - G3PLC_DATA_HDR_SIZE - sizeof(struct g3plc_cmd)

enum g3plc_flags {
  G3PLC_INVALID = 0x1, /* do not filter invalid packets (packet header, CRC) */
  G3PLC_NOACK   = 0x2, /* enable ACK communications */
};

/* Initialization status */
enum g3plc_init_status {
  G3PLC_INIT_SUCCESS,
  G3PLC_INIT_BOOT_ERROR, /* error during boot sequence */
};

/* Status of a received frame/command */
enum g3plc_receive_status {
  G3PLC_RCV_SUCCESS,
  G3PLC_RCV_IGNORED,     /* frame ignored or not supported */
  G3PLC_RCV_CONT,        /* no frame were parsed (need more data) */
  G3PLC_RCV_INVALID_CRC, /* CRC does not match received command */
  G3PLC_RCV_INVALID_HDR, /* invalid command header (too short) */
};

/* Status of a sent frame/command */
enum g3plc_send_status {
  G3PLC_SND_SUCCESS,
  G3PLC_SND_INVALID_PARAM, /* invalid parameter (out of range) */
  G3PLC_SND_INVALID_HDR,   /* invalid command header (too short) */
  G3PLC_SND_TOOLONG,       /* payload too long */
  G3PLC_SND_NOACK,         /* maximum number of retransmissions reached */
  G3PLC_SND_CONFIRM,       /* cannot confirm transmission */
  G3PLC_SND_ACCESS,        /* did not transmit because of activity on the channel */
  G3PLC_SND_OOM,           /* out of memory in internal buffer */
  G3PLC_SND_FAILURE,       /* (any other reason) */
};

/* Information about a received data frame */
struct g3plc_data_hdr {
  uint8_t  src_mode;    /* source address mode */
  uint16_t src_pan;     /* source PAN ID */
  uint64_t src_addr;    /* source short address */

  uint8_t  dst_mode;    /* destination address mode */
  uint16_t dst_pan;     /* destination PAN ID */
  uint64_t dst_addr;    /* destination short address */

  uint8_t  handle;      /* handle associated to MSDU */
  uint8_t  seqno;       /* sequence number */
  uint32_t time;        /* time, in symbols, at which the data were transmitted */

  uint8_t  sec_level;   /* security level */
  uint8_t  key_id_mode; /* key identification mode */
  uint64_t key_source;  /* (not used) */
  uint8_t  key_index;   /* key index */
  uint8_t  QoS;         /* Quality of Service */
  uint8_t  estimated;   /* ? */
  uint32_t tonemap;     /* estimated tonemap */
};

struct g3plc_config {
    struct g3plc_callbacks {
    /* The raw callback is called for each received command packet.
       By default invalid command packet (bad CRC or HDR) are filtered.
       This is useful for user that wants to implement their own dissector. */
    void (*raw)(const struct g3plc_cmd *cmd, unsigned int size, int status, void *data);

    void (*cb_recv)(const struct g3plc_data_hdr *hdr,
                    const void *payload, unsigned payload_size,
                    int status, void *data);
  } callbacks;

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
  int (*uart_send)(const void *buf, unsigned int size);

  /* The g3plc_reset() function will use this to read
     from the device during the boot sequence. This
     function should return a negative value in case of
     error or 0 on success. */
  int (*uart_read)(void *buf, unsigned int size);

  /* Change UART speed.
     This take an integer (not a speed_t type).
     It should handle the following speed:
       - 1M
       - 500k
       - 115.2k
     and return a negative number on error. */
  int (*set_uart_speed)(unsigned int speed);


  /* The function g3plc_uart_putc() is generally called
     from an interrupt handler. Since we cannot parse
     the frame in this handler, we defer processing when
     a complete frame has been received. Control is then
     transferred to this function which can either directly
     be g3plc_recv_frame() or use semaphores to defer
     outside of the interrupt context. */
  int (*recv_frame)(void);

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

  uint8_t bandplan;     /* bandplan (see g3plc_bandplan) */
  uint16_t pan_id;      /* PAN ID */
  uint16_t mac_address; /* device short MAC address */
  uint64_t ext_address; /* extended 64-bit address */
  unsigned int retrans; /* maximum number of retransmissions */
  unsigned int timeout; /* request timeout in us */
  unsigned long flags;  /* (see g3plc_flags) */

  void *data; /* context data passed to user callbacks */
};

/* Initialize the G3PLC driver (see g3plc_config).
   Return 0 on success, for other errror codes see g3plc_init_status. */
int g3plc_init(const struct g3plc_config *conf);

/* Send a command to the G3PLC device.
   The command before must be four bytes longer than actually announced
   since the function will append a CRC to the supplied buffer. */
int g3plc_command(struct g3plc_cmd *cmd, unsigned int size);

/* Assemble and send a frame to the specified destination using G3PLC.
   When ACK is enabled, this function will block until the packet has
   been successfully transmitted. For the error see g3plc_send_status.
   If the tx pointer is not null, it is replaced with the number of
   transmissions necessary to succesfully send the packet. */
int g3plc_send(uint16_t dst, const void *payload, unsigned int payload_size);

/* Start the processing of a frame. Can be called either automatically
   when a complete frame has been received or manually from another thread. */
int g3plc_recv_frame(void);

/* Called by the platform dependent part of the driver when a character
   has been received on UART from the device. This function can block
   when a full frame has been received. It may also block indefinitely
   if the receive callback itself is blocked. Note that this function
   is *NOT* reentrant. You have to wait for its completion until you
   can call it again. */
int g3plc_uart_putc(unsigned char c);

/* Reset the modem. */
int g3plc_reset(void);

#endif /* _G3PLC_H_ */
