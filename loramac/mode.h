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

#ifndef _MODES_H_
#define _MODES_H_

#include "loramac.h"
#include "main.h"

/* Description of an iface module.
   This describe how the program will
   interface itself to the LoRaMAC device,
   hence its behavior.
   The available modes are listed in "mode-list.h". */
struct iface_mode {
  const char *name;
  const char *description;

  /* executed before/after module execution */
  void (*before)(const struct context *ctx,
                 const struct loramac_config *loramac);
  void (*after)(const struct context *ctx);
};

/* Select a mode by its name. Returns null if mode is not found. */
const struct iface_mode * select_mode_by_name(const char *name);

/* List all modes. This is useful for example to display a list of
   all available modes along with their description. */
void walk_modes(void (*visit)(const struct iface_mode *mode, void *data), void *data);

#endif /* _MODES_H_ */
