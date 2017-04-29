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

#include "g3plc.h"
#include "crc32.h"

void append_crc(const struct g3plc_config *g3plc, unsigned char *src, unsigned int *size)
{
  uint32_t crc = crc32_G3PLC(src, *size, 0);

  src   += *size;
  *size += sizeof(uint32_t);

  /* All hail RFC1700! (network order is big endian) */
  *(uint32_t *)src = g3plc->htonl(crc);
}

int check_crc(const struct g3plc_config *g3plc, const unsigned char *src, unsigned int *size)
{
  uint32_t crc_expected, crc_found;

  *size -= sizeof(uint32_t);

  crc_found    = g3plc->ntohl(*(uint32_t *)(src + *size));
  crc_expected = crc32_G3PLC(src, *size, 0);

  return crc_found == crc_expected;
}

unsigned int pack(unsigned char *dst, const unsigned char *src, unsigned int size)
{
  unsigned char *d = dst;

  *d++ = 0x7e; /* frame delimiter */

  /* HDLC escaping:
      0x7e -> 0x7d 0x5e
      0x7d -> 0x7d 0x5d */
  while(size--) {
    unsigned char c = *src++;

    switch(c) {
    case 0x7e:
    case 0x7d:
      *d++ = 0x7d;
      *d++ = c ^ 0x20;
      break;
    default:
      *d++ = c;
      break;
    }
  }

  *d++ = 0x7e; /* frame delimiter */

  return (d - dst);
}

unsigned int unpack(unsigned char *dst, const unsigned char *src, unsigned int size)
{
  unsigned char *d = dst;
  int n = size; /* We use a signed integer to avoid a buffer overflow.
                   The negatives eat up an off by one in the while condition.
                   It helps in the case 0x7e 0x7d 0x7e. */

  /* skip frame delimiter */
  src++;
  n -= 2;

  /* HDLC unescaping:
      0x7d 0x5e -> 0x7e
      0x7d 0x5d -> 0x7d */
  while(n-- > 0) {
    unsigned char c = *src++;

    if(c == 0x7d) {
      /* escaped character */
      *d = *src++ ^ 0x20;
      n--;
    }
    else
      *d = c;

    d++;
  }

  return (d - dst);
}
