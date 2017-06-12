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

#include "g3plc.h"

const char * g3plc_flag2str(enum g3plc_flags flag)
{
  switch(flag) {
  case G3PLC_INVALID:
    return "invalid";
  case G3PLC_NOACK:
    return "no ACK";
  default:
    return "unknown flag";
  }
}

const char * g3plc_init2str(enum g3plc_init_status status)
{
  switch(status) {
  case G3PLC_INIT_SUCCESS:
    return "success";
  case G3PLC_INIT_BOOT_ERROR:
    return "boot error";
  default:
    return "unknown init status";
  }
}

const char * g3plc_rcv2str(enum g3plc_receive_status status)
{
  switch(status) {
  case G3PLC_RCV_SUCCESS:
    return "success";
  case G3PLC_RCV_CONT:
    return "no frame";
  case G3PLC_RCV_INVALID_CRC:
    return "invalid CRC";
  case G3PLC_RCV_INVALID_HDR:
    return "invalid header";
  case G3PLC_RCV_IGNORED:
    return "not supported";
  default:
    return "unknown receive status";
  }
}

const char * g3plc_send2str(enum g3plc_send_status status)
{
  switch(status) {
  case G3PLC_SND_SUCCESS:
    return "success";
  case G3PLC_SND_TOOLONG:
    return "too long";
  case G3PLC_SND_NOACK:
    return "max retransmit";
  case G3PLC_SND_INVALID_PARAM:
    return "invalid parameter";
  case G3PLC_SND_INVALID_HDR:
    return "invalid header";
  case G3PLC_SND_CONFIRM:
    return "cannot confirm";
  case G3PLC_SND_ACCESS:
    return "medium access failure";
  case G3PLC_SND_OOM:
    return "out of memory";
  case G3PLC_SND_FAILURE:
    return "failure";
  default:
    return "unknown send status";
  }
}

enum g3plc_flags g3plc_str2flag(const char *s)
{
  if(!strcmp("invalid", s))
    return G3PLC_INVALID;
  else if(!strcmp("no-ack", s))
    return G3PLC_NOACK;
  return 0;
}

