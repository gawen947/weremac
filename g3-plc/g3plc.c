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

#include "cmdbuf.h"
#include "pack.h"
#include "g3plc-cmd.h"
#include "g3plc.h"

#define CB(cb, ...) if(g3plc_conf.callbacks.cb) g3plc_conf.callbacks.cb(__VA_ARGS__)

/* G3PLC configuration with platform dependent functions,
   source mac address, callbacks and flags. */
static struct g3plc_config g3plc_conf;

/* Size of the received command packet.
   Glue between uart_putc() and recv_frame(). */
static unsigned int rcv_size;

int g3plc_init(const struct g3plc_config *conf)
{
  g3plc_conf = *conf;

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
