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
#include <stdlib.h>

#include "dump.h"
#include "g3plc-cmd.h"

int main(int argc, char *argv[])
{
  struct g3plc_cmd cmd = {
    .reserved = '*',

    .type = 'T',

    /* 0b11011001 = 0xd9 */
    .idc  = 1, /* 0b1 */
    .ida  = 5, /* 0b101 */
    .idp  = 9, /* 0b1001 */

    .cmd  = 'C'
  };

  printf("Orig (%lu):\n", sizeof(struct g3plc_cmd));
  hex_dump((void *)&cmd, sizeof(struct g3plc_cmd));

  hton_g3plc_cmd(&cmd);
  printf("Network order:\n");
  hex_dump((void *)&cmd, sizeof(struct g3plc_cmd));

  ntoh_g3plc_cmd(&cmd);
  printf("Host order:\n");
  hex_dump((void *)&cmd, sizeof(struct g3plc_cmd));

  return 0;
}
