/* Copyright (c) 2013-2016, David Hauweele <david@hauweele.net>
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

#include "help.h"

void help(const char *name, const char *usage, const struct opt_help opts[])
{
  const struct opt_help *opt;
  int size;
  int max = 0;

  fprintf(stderr, "usage: %s %s\n", name, usage);

  /* maximum option names size for padding */
  for(opt = opts ; opt->name_long ; opt++) {
    size = strlen(opt->name_long);
    if(size > max)
      max = size;
  }

  /* print options and help messages */
  for(opt = opts ; opt->name_long ; opt++) {
    if(opt->name_short != '\0')
      fprintf(stderr, "  -%c, --%s", opt->name_short, opt->name_long);
    else
      fprintf(stderr, "      --%s", opt->name_long);

    /* padding */
    size = strlen(opt->name_long);
    for(; size <= max ; size++)
      fputc(' ', stderr);
    fprintf(stderr, "%s\n", opt->help);
  }
}
