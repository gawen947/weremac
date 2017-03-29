/* Copyright (c) 2013-2017, David Hauweele <david@hauweele.net>
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
#include <string.h>
#include <sys/time.h>

void fill_with_random(unsigned char *buf, unsigned int size)
{
  int i;

  struct timeval tv;
  unsigned int seed;

  gettimeofday(&tv, NULL);

  /* David's magic hash */
  seed = (tv.tv_sec  + (tv.tv_sec  << 11)) ^ \
         (tv.tv_usec + (tv.tv_usec << 17));

  srand(seed);

  for(i = 0 ; i < size ; i++)
    buf[i] = rand() % 0xff;
}

const char * tv_to_str(const struct timeval *tv)
{
  static char buf[sizeof("111.111 minutes")];

  if(tv->tv_sec >= 86400)
    return "> 1 day";
  else if(tv->tv_sec >= 3600)
    sprintf(buf, "%3.3f hours", (double)tv->tv_sec / 3600);
  else if(tv->tv_sec >= 60)
    sprintf(buf, "%3.3f minutes", (double)tv->tv_sec / 60);
  else if(tv->tv_sec > 0)
    sprintf(buf, "%ld.%03ld s", tv->tv_sec, tv->tv_usec / 1000);
  else if(tv->tv_usec > 1000)
    sprintf(buf, "%ld.%03ld ms", tv->tv_usec / 1000, tv->tv_usec % 1000);
  else
    sprintf(buf, "%ld us", tv->tv_usec);

  return buf;
}

void * memdup(const void *buf, size_t size)
{
  void *new_buf = malloc(size);

  return memcpy(new_buf, buf, size);
}
