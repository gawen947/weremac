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

#ifndef _PACK_H_
#define _PACK_H_

#include "g3plc.h"

/* Append CRC to the end of the buffer and modify the size accordingly. */
void append_crc(const struct g3plc_config *g3plc, unsigned char *src, unsigned int *size);

/* Remove CRC from the end of the buffer and modify the size accordingly.
   The extracted CRC is compared to the expected CRC computed from the buffer itself.
   Returns true if both extracted and expected are equals, false otherwise. */
int extract_crc(const struct g3plc_config *g3plc, const unsigned char *src, unsigned int *size);

/* Escape the buffer using HDLC and add frame delimiters, store the result in destination buffer.
   Returns the size of the packed destination buffer. */
unsigned int pack(unsigned char *dst, const unsigned char *src, unsigned int size);

/* Remove frame delimiters and unescape the buffer using HDLC, store the result in destination buffer.
   Returns the size of the unpacked destination buffer. */
unsigned int unpack(unsigned char *dst, const unsigned char *src, unsigned int size);

#endif /* _PACK_H_ */
