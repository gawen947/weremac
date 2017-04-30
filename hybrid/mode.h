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

#include "g3plc/g3plc.h"
#include "main.h"

/* Description of an iface module.
   This describe how the program will
   interface itself to the G3-PLC device,
   hence its behavior.

   The actual iface_mode used is declared
   in the mode unit linked with the common
   code. */
extern struct iface_mode {
  const char *name;
  const char *description;

  /* argument parsing */
  const char      *optstring;      /* mode short options string */
  struct option   *long_opts;      /* mode long options structures */
  struct opt_help *extra_messages; /* mode help messages */

  /* Parse a getopt option for one of the mode specific options.
     Return 0 if the option is unknown by this module (which means
     that the next module or the common options can be parsed
     instead). Return 1 if the option was parsed (and the parsing
     should continue to the next option). */
  int (*parse_option)(const struct context *ctx, int c);

  /* Executed before/after module execution.
     These functions are used by the module to configure
     the G3-PLC layer (such as the recv function), and
     create/destroy mode specific structures. */
  void (*init)(const struct context *ctx, struct g3plc_config *g3plc);
  void (*destroy)(const struct context *ctx);

  /* Actual behavior of the module.
     This is the code that will eventually send data
     to the G3-PLC layer. */
  void (*start)(const struct context *ctx);
} iface_mode;

#endif /* _MODES_H_ */
