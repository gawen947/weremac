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

#include <sys/types.h>
#include <stdint.h>
#include <time.h>
#include <assert.h>

#include "common.h"
#include "time-substract.h"

#define NSEC 1000000000LLU

int timespec_substract(struct timespec *result, struct timespec *x, struct timespec *y)
{
  /* Perform the carry for the later subraction by updating y. */
  if (x->tv_nsec < y->tv_nsec) {
    int sec = (y->tv_nsec - x->tv_nsec) / NSEC + 1;
    y->tv_nsec -= NSEC * sec;
    y->tv_sec  += sec;
  }
  if (x->tv_nsec - y->tv_nsec > NSEC) {
    int sec = (x->tv_nsec - y->tv_nsec) / NSEC;
    y->tv_nsec += NSEC * sec;
    y->tv_sec  -= sec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec  = x->tv_sec - y->tv_sec;
  result->tv_nsec = x->tv_nsec - y->tv_nsec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}

uint64_t substract_nsec(struct timespec *begin, struct timespec *end)
{
  struct timespec diff;
  uint64_t diff_nsec;

  /* Substract first and check that everything goes correctly. */
  int n = timespec_substract(&diff, end, begin);
  assert(!n);
  UNUSED(n);

  diff_nsec = (uint64_t)diff.tv_sec * NSEC + (uint64_t)diff.tv_nsec;

  return diff_nsec;
}
