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

#ifndef _CMDBUF_H_
#define _CMDBUF_H_

#include <stdint.h>

/* Maximum size for a command packet.

   We make the distinction between the command itself and
   the command escaped with HDLC between frame delimiters.
   The unescaped command can be appended with a CRC. */
#define G3PLC_MAX_CMD        1024 /* FIXME: depends on aMaxMACPayloadSize */
#define G3PLC_MAX_PACKED_CMD G3PLC_MAX_CMD * 2 /* HDLC */ + 2 /* frame delimiter */

/* Receive and send command buffers.
   Unpacked command (without HDLC and delimiters) are
   assembled/parsed in these two buffers. */
extern unsigned char snd_cmdbuf[];
extern unsigned char rcv_cmdbuf[];

/* Receive and send packed command buffers.
   Packed command (with HDLC and delimiters) are
   assembled/parsed in these two buffers. */
extern unsigned char snd_cmdbuf_packed[];
extern unsigned char rcv_cmdbuf_packed[];

#endif /* _CMDBUF_H_ */
