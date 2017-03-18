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

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <getopt.h>
#include <err.h>

#include "loramac.h"
#include "version.h"
#include "uart.h"
#include "help.h"

static const char * basename(const char *s)
{
  const char *base = (const char *)strrchr(s, '/');
  base = base ? (base + 1) : s;
  return base;
}

static void print_help(const char *name)
{
  struct opt_help messages[] = {
    { 'h', "help",         "Show this help message" },
    { 'V', "version",      "Show version information" },
#ifdef COMMIT
    { 0, "commit",         "Display commit information" }
#endif /* COMMIT */
    { 'p', "destination",  "Do not filter packets to another destination" },
    { 'i', "invalid",      "Do not filter invalid packets (packet header, CRC)" },
    { 'b', "no-broadcast", "Ignore broadcast messages" },
    { 'a', "no-ack",       "Do not answer nor expect ACKs" },
    { 'B', "baud",         "Specify the baud rate (default 9600)"},
    { 'd', "destination",  "Destination MAC (hex. short address, default to broadcast)" }
  };

  help(name, "[OPTIONS] source device", messages);
}

int main(int argc, char *argv[])
{
  const char *prog_name;
  struct loramac_config loramac = {
    .uart_send       = uart_send,
//    .start_ack_timer = start_ack_timer,
//    .stop_ack_timer  = stop_ack_timer,
//    .wait_ack_timer  = wait_ack_timer,
//    .lock            = lock,
//    .unlock          = unlock,
    .htons           = htons,
    .ntohs           = ntohs,
    .recv_frame      = loramac_recv_frame,
    .flags           = 0
  };
  speed_t speed   = B9600;
  int exit_status = EXIT_FAILURE;

  enum opt {
    OPT_COMMIT = 0x100
  };

  struct option opts[] = {
    { "help", no_argument, NULL, 'h' },
    { "version", no_argument, NULL, 'V' },
#ifdef COMMIT
    { "commit", no_argument, NULL, OPT_COMMIT },
#endif /* COMMIT */

    /* flags */
    { "promiscuous", no_argument, NULL, 'p' },
    { "invalid", no_argument, NULL, 'i' },
    { "no-broadcast", no_argument, NULL, 'b' },
    { "no-ack", no_argument, NULL, 'a' },

    { "baud", required_argument, NULL, 'B' },
    { "destination", required_argument, NULL, 'd' },
    { NULL, 0, NULL, 0 }
  };

  prog_name = basename(argv[0]);

  while(1) {
    int c = getopt_long(argc, argv, "hVpibad:", opts, NULL);

    if(c == 1)
      break;
    switch(c) {
    case 'p':
      loramac.flags |= LORAMAC_PROMISCUOUS;
      break;
    case 'i':
      loramac.flags |= LORAMAC_INVALID;
      break;
    case 'b':
      loramac.flags |= LORAMAC_NOBROADCAST;
      break;
    case 'a':
      loramac.flags |= LORAMAC_NOACK;
      break;
    case 'd':
      loramac.mac_address = strtol(optarg, NULL, 16);
      break;
    case 'B':
      speed = baud(optarg);
      break;
    case 'V':
      version(prog_name);
      exit_status = EXIT_SUCCESS;
      goto EXIT;
#ifdef COMMIT
    case OPT_COMMIT:
      commit();
      exit_status = EXIT_SUCCESS;
      goto EXIT;
#endif /* COMMIT */
    case 'h':
      exit_status = EXIT_SUCCESS;
    default:
      print_help(prog_name);
      goto EXIT;
    }
  }

  argc -= optind;
  argv += optind;

  if(argc < 1) {
    print_help(prog_name);
    goto EXIT;
  }

  exit_status = EXIT_SUCCESS;


EXIT:
  exit(exit_status);
}
