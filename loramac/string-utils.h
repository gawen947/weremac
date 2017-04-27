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

#ifndef _STRING_UTILS_H_
#define _STRING_UTILS_H_

#include <stdlib.h>
#include <sys/time.h>

/* Write a string litteral. */
#define write_slit(fd, s) write(fd, s, sizeof(s) - 1)

/* Fill the size first bytes of the specified buffer with random bytes. */
void fill_with_random(unsigned char *buf, unsigned int size);

/* Convert a timeval structure seen as a duration to a string.
   The most common usage for this function is to represent latencies. */
const char * tv_to_str(const struct timeval *tv);

/* Duplicate a buffer. */
void * memdup(const void *buf, size_t size);

/* Concatenate two strings in a new buffer large
   enough to contain the two concatenated strings. */
char * strcat_dup(const char *a, const char *b);

/* Copy a string but fail if source is too large. */
void xstrcpy(char *dst, const char *src, size_t count);

#endif /* _STRING_UTILS_H_ */
