/* Copyright (c) 2016, David Hauweele <david@hauweele.net>
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
#include <stdint.h>

#define SCALE_BUFFER_SIZE UINT8_MAX

enum {
  SC_MICROS = 1000LLU,
  SC_MILLIS = 1000000LLU,
  SC_SECOND = 1000000000LLU,
  SC_MINUTE = 60LLU * SC_SECOND,
  SC_HOUR   = 60LLU * SC_MINUTE,
  SC_DAY    = 24LLU * SC_HOUR,
};

static char buffer[SCALE_BUFFER_SIZE];

const char * scale_time(uint64_t nsec)
{
  const char *unit = "ns";
  uint64_t factor = 1;

  if(nsec > SC_DAY) {
    unit   = "days";
    factor = SC_DAY;
  }
  else if(nsec > SC_HOUR) {
    unit   = "hours";
    factor = SC_HOUR;
  }
  else if(nsec > SC_MINUTE) {
    unit   = "minutes";
    factor = SC_MINUTE;
  }
  else if(nsec > SC_SECOND) {
    unit   = "s";
    factor = SC_SECOND;
  }
  else if(nsec > SC_MILLIS) {
    unit   = "ms";
    factor = SC_MILLIS;
  }
  else if(nsec > SC_MICROS) {
    unit   = "us";
    factor = SC_MICROS;
  }

  snprintf(buffer, SCALE_BUFFER_SIZE, "%.2f %s", (double)nsec / factor, unit);

  return buffer;
}
