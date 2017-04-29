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

#include <stdio.h>
#include <string.h>

#include "dump.h"
#include "pack.h"

void test(const char *s)
{
  char packed[1024];
  char unpacked[1024];
  unsigned int orig_len = strlen(s);
  unsigned int packed_len, unpacked_len;

  printf("----------\n");

  printf("Orig:\n");
  hex_dump(s, orig_len);

  printf("Packed:\n");
  packed_len = pack(packed, s, orig_len);
  hex_dump(packed, packed_len);

  printf("Unpacked:\n");
  unpacked_len = unpack(unpacked, packed, packed_len);
  hex_dump(unpacked, unpacked_len);
}

void test_unpack(const char *s)
{
  char unpacked[1024];
  unsigned int orig_len = strlen(s);
  unsigned int unpacked_len;

  printf("---------\n");

  printf("Orig:\n");
  hex_dump(s, orig_len);

  printf("Unpacked:\n");
  unpacked_len = unpack(unpacked, s, orig_len);
  hex_dump(unpacked, unpacked_len);

}

int main(int argc, char *argv[])
{
  //test(argv[1]);

  test_unpack(argv[1]);

  return 0;
}
