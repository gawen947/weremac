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

#include <stdlib.h>
#include <getopt.h>
#include <string.h>

#include "safe-call.h"

int opts_len(const struct option *opts)
{
  int len = 0;
  const struct option *o;

  for(o = opts ; o->name ; o++)
    len++;

  return len;
}

struct option * merge_opts(const struct option *a, const struct option *b)
{
  int len_a = opts_len(a);
  int len_b = opts_len(b);
  int len   = len_a + len_b; /* len without terminal zero struct */

  /* allocate memory for the merged array
     here we take into account the terminal
     zero struct */
  struct option *merged = xmalloc((len + 1) * sizeof(struct option));

  /* terminal zero struct */
  merged[len] = (struct option){ NULL, 0 , NULL, 0 };

  /* merge arrays */
  memcpy(merged,         a, len_a * sizeof(struct option));
  memcpy(merged + len_a, b, len_b * sizeof(struct option));

  return merged;
}
