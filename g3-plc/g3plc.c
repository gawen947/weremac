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

#include <string.h>

#include "firmware.h"
#include "cmdbuf.h"
#include "pack.h"
#include "g3plc-cmd.h"
#include "g3plc.h"

/* Maximum size of write during boot sequence segment upload. */
#define BOOT_SEGMENT_CHUNK 8092

/* Check that callbacks are configured before calling them. */
#define CB(cb, ...) if(g3plc_conf.callbacks.cb) g3plc_conf.callbacks.cb(__VA_ARGS__)

/* Check the return value of the function.
   Exit with its error when it is different than success. */
#define x_(n, fun, ...) do { \
    n = fun(__VA_ARGS__);    \
    if(n)                    \
      return n;              \
  } while(0)

/* G3PLC configuration with platform dependent functions,
   source mac address, callbacks and flags. */
static struct g3plc_config g3plc_conf;

/* Size of the received command packet.
   Glue between uart_putc() and recv_frame(). */
static unsigned int rcv_size;

static uint64_t htonll(uint64_t v)
{
  return ((uint64_t)g3plc_conf.htonl(v & 0xffffffff) << 32) | g3plc_conf.htonl(v >> 32);
}

static int g3_init_request(uint16_t neighbour,  /* number of neighbour table */
                           uint16_t device,     /* number of device table */
                           uint16_t pan         /* max. number of PAN obtained with a scan */ )
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_G3CTR,
    .cmd      = G3PLC_CMD_G3_INIT
  };

  /* check values */
  if(neighbour > 1536)
    return G3PLC_SND_INVALID_PARAM;
  if(device > 1536)
    return G3PLC_SND_INVALID_PARAM;
  if(pan < 1 || pan > 128)
    return G3PLC_SND_INVALID_PARAM;

  *(uint8_t  *)dat = 0x03; dat += sizeof(uint8_t); /* g3mode */
  *(uint16_t *)dat = g3plc_conf.htons(neighbour); dat += sizeof(uint16_t);
  *(uint16_t *)dat = g3plc_conf.htons(device);    dat += sizeof(uint16_t);
  *(uint16_t *)dat = g3plc_conf.htons(pan);       dat += sizeof(uint16_t);

  return g3plc_command(cmd, dat - snd_cmdbuf);
}

static int g3_setconfig_request(uint8_t  bandplan, /* band plan */
                                uint64_t extaddr   /* extended address */ )
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_G3CTR,
    .cmd      = G3PLC_CMD_G3_SETCONFIG
  };

  *(uint8_t  *)dat = 0x03;            dat += sizeof(uint8_t);  /* g3mode */
  *(uint8_t  *)dat = bandplan;        dat += sizeof(uint8_t);
  *(uint32_t *)dat = 0;               dat += sizeof(uint32_t); /* reserved */
  *(uint64_t *)dat = htonll(extaddr); dat += sizeof(uint64_t);

  return g3plc_command(cmd, dat - snd_cmdbuf);
}

static int mlme_reset_request(uint8_t default_pib /* reset PIB to default (1) or not (0) */ )
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_UMAC,
    .cmd      = G3PLC_CMD_MLME_RESET
  };

  *(uint8_t *)dat = default_pib; dat += sizeof(uint8_t);

  return g3plc_command(cmd, dat - snd_cmdbuf);
}

static int mlme_set_request(uint16_t attr_id,    /* PIB attribute ID */
                            uint16_t attr_idx,   /* index within the table for PIB attribute */
                            unsigned char *attr, /* attribute value */
                            unsigned int size    /* attribute size */ )
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_UMAC,
    .cmd      = G3PLC_CMD_MLME_SET
  };

  *(uint16_t  *)dat = g3plc_conf.htons(attr_id);  dat += sizeof(uint16_t);
  *(uint16_t  *)dat = g3plc_conf.htons(attr_idx); dat += sizeof(uint16_t);

  /* copy attribute value */
  memcpy(dat, attr, size);
  dat += size;

  return g3plc_command(cmd, dat - snd_cmdbuf);
}

static int mlme_start_request(uint8_t pan /* PAN ID */ )
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_UMAC,
    .cmd      = G3PLC_CMD_MLME_START
  };

  *(uint16_t *)dat = g3plc_conf.htons(pan); dat += sizeof(uint16_t);

  return g3plc_command(cmd, dat - snd_cmdbuf);
}

int g3plc_init(const struct g3plc_config *conf)
{
  int n;
  uint16_t u16;

  g3plc_conf = *conf;

  n = g3plc_reset();
  if(n)
    return n;

  /* init G3-PLC */
  x_(n, g3_init_request,
     500 /* neighbour tables */,
     500 /* device tables */,
     1   /* pan in scan */ );

  /* set config */
  x_(n, g3_setconfig_request,
     g3plc_conf.bandplan,
     g3plc_conf.ext_address );

  x_(n, mlme_reset_request, 1); /* MLME reset */

  /* configure short address */
  u16 = g3plc_conf.mac_address;
  x_(n, mlme_set_request,
     G3PLC_ATTR_SHORTADDR,
     0 /* attr idx */,
     (unsigned char *)&u16,
     sizeof(u16));

  /* configure PAN ID */
  u16 = g3plc_conf.pan_id;
  x_(n, mlme_set_request,
     G3PLC_ATTR_PANID,
     0 /* attr idx */,
     (unsigned char *)&u16,
     sizeof(u16));

  /* start MLME */
  x_(n, mlme_start_request, g3plc_conf.pan_id);

  return G3PLC_INIT_SUCCESS;
}

int g3plc_command(struct g3plc_cmd *cmd, unsigned int size)
{
  if(size < sizeof(struct g3plc_cmd))
    return G3PLC_SND_INVALID_HDR;

  hton_g3plc_cmd(cmd);                                        /* network order */
  append_crc(&g3plc_conf, (unsigned char *)cmd, &size);       /* apply CRC */
  size = pack(snd_cmdbuf_packed, (unsigned char *)cmd, size); /* HDLC */
  return g3plc_conf.uart_send(snd_cmdbuf_packed, size);       /* send command */
}

int g3plc_send(uint16_t dst, const void *payload, unsigned int payload_size, unsigned int *tx)
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)snd_cmdbuf;
  unsigned char    *dat = cmd->data;

  *cmd = (struct g3plc_cmd){
    .reserved = 0,
    .type     = G3PLC_TYPE_G3,
    .idc      = G3PLC_CHAN0,
    .ida      = G3PLC_IDA_REQUEST,
    .idp      = G3PLC_IDP_UMAC,
    .cmd      = G3PLC_CMD_MCPS_DATA
  };

  /* check payload length */
  if(payload_size > G3PLC_MAX_PAYLOAD)
    return G3PLC_SND_TOOLONG;

  /* assemble frame */
  *(uint8_t *)dat = 0x02; dat += sizeof(uint8_t); /* src addr type (16-bit short addr) */
  *(uint8_t *)dat = 0x02; dat += sizeof(uint8_t); /* dst addr type (16-bit short addr) */

  /* destination PAN ID */
  *(uint16_t *)dat = g3plc_conf.htons(g3plc_conf.pan_id);
  dat += sizeof(uint16_t);

  /* destination address */
  memset(dat, 0, 8 - sizeof(uint16_t));     dat += 8 - sizeof(uint16_t);
  *(uint16_t *)dat = g3plc_conf.htons(dst); dat += sizeof(uint16_t);

  /* MSDU length */
  *(uint16_t *)dat = g3plc_conf.htons(payload_size);
  dat += sizeof(uint16_t);

  *(uint8_t *)dat = 0x00; dat += sizeof(uint8_t); /* MSDU handle */

  /* TX options */
  *(uint8_t *)dat = g3plc_conf.flags & G3PLC_NOACK ? 0x00 : 0x01;
  dat += sizeof(uint8_t);

  /* security level
     key identification mode
     key source
     key index
     QoS */
  memset(dat, 0, 12); dat += 12;

  /* copy payload */
  memcpy(dat, payload, payload_size);
  dat += payload_size;

  /* send command to device */
  return g3plc_command(cmd, dat - snd_cmdbuf);
}

int g3plc_recv_frame(void)
{
  struct g3plc_cmd *cmd = (struct g3plc_cmd *)rcv_cmdbuf;
  unsigned int size = unpack(rcv_cmdbuf, rcv_cmdbuf_packed, rcv_size);
  int ret, status = G3PLC_RCV_SUCCESS;

  /* check that we at least have a valid command packet */
  if(size < (sizeof(struct g3plc_cmd) + sizeof(uint32_t))) {
    status = G3PLC_RCV_INVALID_HDR;
    goto PARSING_COMPLETE;
  }

  /* extract and check CRC */
  ret = extract_crc(&g3plc_conf, rcv_cmdbuf, &size);
  if(!ret) {
    status = G3PLC_RCV_INVALID_CRC;
    goto PARSING_COMPLETE;
  }

  /* convert command to host order */
  ntoh_g3plc_cmd(cmd);

PARSING_COMPLETE:
  /* based on parsing status and iface_flags
     we either return directly or pass the
     command packet to the dissectors */
  switch(status) {
  case G3PLC_RCV_INVALID_CRC:
  case G3PLC_RCV_INVALID_HDR:
    if(!(g3plc_conf.flags & G3PLC_INVALID))
      return status;
  }

  CB(raw, cmd, size, status, g3plc_conf.data);

  if(status == G3PLC_RCV_SUCCESS)
    status = dissector(&g3plc_conf, cmd, size);
  return status;
}

int g3plc_uart_putc(unsigned char c)
{
  /* Just writing out the FSM of what the code
     below actually does:

      (out-of-frame):
         0x7e -> write-to-buf; (in-frame)
         _    -> ignore; (out-of-frame)
      (in-frame):
         0x7e -> write-to-buf; message-received; (out-of-frame)
         _    -> write-to-buf; (in-frame)

     We make the distinction between the two states
     by checking whether we are at the beginning of the
     receive buffer or not. */
  static unsigned char *rcv_ptr = rcv_cmdbuf_packed;

  if(rcv_ptr == rcv_cmdbuf_packed) {
    /* state (out-of-frame) */

    if(c == 0x7e)
      *rcv_ptr++ = c;      /* write-to-buf; state <- (in-frame) */
    return G3PLC_RCV_CONT; /* ignore */
  }
  else {
    /* state (in-frame) */

    *rcv_ptr++ = c; /* write-to-buf */

    if(c == 0x7e) {
      /* message-received
         state <- (out-of-frame) */
      rcv_size = rcv_ptr - rcv_cmdbuf_packed;
      rcv_ptr  = rcv_cmdbuf_packed;
      return g3plc_conf.recv_frame();
    }
    else
      return G3PLC_RCV_CONT;
  }
}

/* Send a single byte through UART.
   We use this during the boot sequence to signal the device. */
#define xsend_byte(n, c) x_(n, send_byte, c)
static int send_byte(unsigned char c)
{
  return g3plc_conf.uart_send(&c, 1);
}

/* Wait for the reception of a specific character before continuation.
   We use this during the boot sequence to wait for signals from the device. */
#define xwait_for_byte(n, c) x_(n, wait_for_byte, c)
static int wait_for_byte(unsigned char c)
{
  unsigned char buf;
  int r = g3plc_conf.uart_read(&buf, 1);
  if(r < 0)
    return r;

  if(buf == c)
    return G3PLC_INIT_SUCCESS;
  else
    return G3PLC_INIT_BOOT_ERROR;
}

/* Send a program segment to the device. */
#define xsend_segment(n, segno) x_(n, send_segment, segno)
static int send_segment(unsigned int segno)
{
  int n;

  /* FIXME: Technically we need le32toh() and htole32() here.
     But we know that the renesas platform and Linux on RPi are LE.
     So we are OK. */
  const uint8_t *frmw_tbl    = cpx_firmware + 0x10;         /* firmware table */
  const uint8_t *info_tbl    = frmw_tbl + (segno << 4);     /* info table for this segment */
  uint32_t       offset      = *(uint32_t *)info_tbl;       /* program offset address in table */
  uint32_t       size        = *(uint32_t *)(info_tbl + 8); /* program size */

  /* send segment info table */
  n = g3plc_conf.uart_send(info_tbl + sizeof(uint32_t), 12);
  if(n < 0)
    return n;

  /* send segment */
  while(size) {
    unsigned int write_size = size > BOOT_SEGMENT_CHUNK ? BOOT_SEGMENT_CHUNK : size;

    n = g3plc_conf.uart_send(frmw_tbl + offset, write_size);
    if(n < 0)
      return n;

    size -= write_size;
  }

  return 0;
}

#define xset_uart_speed(n, speed) do {  \
  n = g3plc_conf.set_uart_speed(speed); \
  if(n < 0)                             \
    return n;                           \
} while(0)
int g3plc_reset(void)
{
  int n;

  /* speed for segment 0 */
  xset_uart_speed(n, 115200);

  /* hardware reset */
  g3plc_conf.reset_clear();
  g3plc_conf.usleep(30000); /* sleep 30ms */
  g3plc_conf.reset_set();

  xwait_for_byte(n, 0x80); /* program transmission request */
  xsend_segment(n, 0);     /* send segment 0 */

  /* switch to 1M/500k baudrate */
  xwait_for_byte(n, 0xa1);     /* baud rate change request */
  xsend_byte(n, 0xc1);         /* baud rate change command */
  xsend_byte(n, 0xc9);         /* baud rate (boot 1M / appl. 500k) */
  xwait_for_byte(n, 0xcf);     /* baud rate change accept */
  xset_uart_speed(n, 1000000); /* switch to boot baudrate */
  xsend_byte(n, 0xaa);         /* baud rate change response */

  /* send remaining segments */
  while(1) {
    unsigned char buf;
    unsigned int  segno;

    n = g3plc_conf.uart_read(&buf, 1);
    if(n < 0)
      return n;

    segno = buf & 0x0f;

    if(buf == 0xb0)
      break; /* boot completion */
    else if((buf & 0xf0) == 0x80)
      /* program transmission request for segment segno */
      xsend_segment(n, segno);
    else
      return G3PLC_INIT_BOOT_ERROR;
  }

  xset_uart_speed(n, 500000); /* switch to application baudrate */

  return G3PLC_INIT_SUCCESS;
}
