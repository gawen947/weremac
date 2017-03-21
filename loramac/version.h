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

#ifndef _VERSION_H_
#define _VERSION_H_

#include "loramac.h"

#define _stringify(s) #s
#define stringify(s) _stringify(s)

#define PACKAGE "LoRaMAC"

#define MAJOR 0
#define MINOR 9

#define VERSION         stringify(MAJOR) "." stringify(MINOR)
#define LORAMAC_VERSION stringify(LORAMAC_MAJOR) "." stringify(LORAMAC_MINOR)

#if !(defined COMMIT && defined PARTIAL_COMMIT)
# define PACKAGE_VERSION PACKAGE " v" VERSION \
         " (MAC-layer v" LORAMAC_VERSION ")"
#else
# define PACKAGE_VERSION PACKAGE " v" VERSION \
         " (MAC-layer v" LORAMAC_VERSION ")"  \
         " (commit: " PARTIAL_COMMIT ")"
#endif /* COMMIT */

void version(const char *target);

#ifdef COMMIT
void commit(void);
#endif /* COMMIT */

#endif /* _VERSION_H_ */
