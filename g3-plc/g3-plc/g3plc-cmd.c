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

#include "g3plc-cmd.h"

void hton_g3plc_cmd(struct g3plc_cmd *cmd)
{
  unsigned char    *buf  = (unsigned char *)cmd;
  struct g3plc_cmd  copy = *cmd;

  buf[0] = copy.reserved;
  buf[1] = copy.type;

  buf[2] = copy.idc;
  buf[2] <<= 3; buf[2] |= copy.ida;
  buf[2] <<= 4; buf[2] |= copy.idp;

  buf[3] = copy.cmd;
}

void ntoh_g3plc_cmd(struct g3plc_cmd *cmd)
{
  unsigned char    *buf  = (unsigned char *)cmd;
  struct g3plc_cmd  copy = {
    .reserved = buf[0],
    .type     = buf[1],
    .idc      = buf[2]  >> 7,
    .ida      = (buf[2] >> 4) & 7,
    .idp      = buf[2] & 0x0f,
    .cmd      = buf[3]
  };

  *cmd = copy;
}
