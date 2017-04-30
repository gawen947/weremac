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

#include <string.h>

#include "loramac.h"

const char * loramac_flag2str(enum loramac_flags flag)
{
  switch(flag) {
  case LORAMAC_PROMISCUOUS:
    return "promiscuous";
  case LORAMAC_INVALID:
    return "invalid";
  case LORAMAC_NOBROADCAST:
    return "ignore broadcasts";
  case LORAMAC_NOACK:
    return "no ACK";
  default:
    return "unknown flag";
  }
}

const char * loramac_init2str(enum loramac_init_status status)
{
  switch(status) {
  case LORAMAC_INIT_SUCCESS:
    return "success";
  case LORAMAC_INIT_TIMEVAL:
    return "invalid time value";
  case LORAMAC_INIT_ACK_FIFO:
    return "ACK FIFO too large";
  case LORAMAC_INIT_OOM:
    return "out of memory";
  default:
    return "unknown init status";
  }
}

const char * loramac_rcv2str(enum loramac_receive_status status)
{
  switch(status) {
  case LORAMAC_RCV_SUCCESS:
    return "success";
  case LORAMAC_RCV_CONT:
    return "no frame";
  case LORAMAC_RCV_INVALID_CRC:
    return "invalid CRC";
  case LORAMAC_RCV_INVALID_HDR:
    return "invalid header";
  case LORAMAC_RCV_DESTINATION:
    return "invalid destination";
  case LORAMAC_RCV_BROADCAST:
    return "broadcast frame";
  default:
    return "unknown receive status";
  }
}

const char * loramac_send2str(enum loramac_send_status status)
{
  switch(status) {
  case LORAMAC_SND_SUCCESS:
    return "success";
  case LORAMAC_SND_TOOLONG:
    return "too long";
  case LORAMAC_SND_NOACK:
    return "max retransmit";
  default:
    return "unknown send status";
  }
}

enum loramac_flags loramac_str2flag(const char *s)
{
  if(!strcmp("promiscuous", s))
    return LORAMAC_PROMISCUOUS;
  else if(!strcmp("invalid", s))
    return LORAMAC_INVALID;
  else if(!strcmp("no-broadcast", s))
    return LORAMAC_NOBROADCAST;
  else if(!strcmp("no-ack", s))
    return LORAMAC_NOACK;
  return 0;
}

enum loramac_init_status loramac_str2init(const char *s)
{
  if(!strcmp("success", s))
    return LORAMAC_INIT_SUCCESS;
  else if(!strcmp("timeval", s))
    return LORAMAC_INIT_TIMEVAL;
  else if(!strcmp("ack-fifo", s))
    return LORAMAC_INIT_ACK_FIFO;
  else if(!strcmp("oom", s))
    return LORAMAC_INIT_OOM;
  return 0;
}

enum loramac_receive_status loramac_str2rcv(const char *s)
{
  if(!strcmp("success", s))
    return LORAMAC_RCV_SUCCESS;
  else if(!strcmp("no-frame", s))
    return LORAMAC_RCV_CONT;
  else if(!strcmp("invalid-crc", s))
    return LORAMAC_RCV_INVALID_CRC;
  else if(!strcmp("invalid-hdr", s))
    return LORAMAC_RCV_INVALID_HDR;
  else if(!strcmp("destination", s))
    return LORAMAC_RCV_DESTINATION;
  else if(!strcmp("broadcast", s))
    return LORAMAC_RCV_BROADCAST;
  return 0;
}

enum loramac_send_status loramac_str2send(const char *s)
{
  if(!strcmp("success", s))
    return LORAMAC_SND_SUCCESS;
  else if(!strcmp("too-long", s))
    return LORAMAC_SND_TOOLONG;
  else if(!strcmp("no-ack", s))
    return LORAMAC_SND_NOACK;
  return 0;
}
