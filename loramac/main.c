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
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <getopt.h>
#include <err.h>

#include "loramac-str.h"
#include "loramac.h"
#include "stdio-mode.h"
#include "version.h"
#include "common.h"
#include "xatoi.h"
#include "timer.h"
#include "lock.h"
#include "uart.h"
#include "mode.h"
#include "help.h"
#include "main.h"

struct thread_data {
  const struct iface_mode *mode;
  const struct context    *ctx;
  struct loramac_config   *config;
};

static void * output_thread_func(void *p)
{
  struct thread_data *data = (struct thread_data *)p;

  data->mode->input(data->ctx);

  /* FIXME: should return correctly */
  return NULL;
}

static void * input_thread_func(void *p)
{
  UNUSED(p);

  uart_read_loop();

  /* FIXME: should return correctly */
  return NULL;
}

static void display_mode(const struct iface_mode *mode, void *data)
{
  UNUSED(data);
  printf("%s\t%s\n", mode->name, mode->description);
}

static void display_summary(const struct iface_mode *mode,
                            const struct loramac_config *conf,
                            uint16_t dst_mac,
                            const char *dev,
                            const char *speed)
{
  unsigned long flag;

  printf(PACKAGE_VERSION "\n");
  printf("Using %s mode on %s @%s bauds.\n", mode->name, dev, speed);
  printf(" iface (source) MAC address: %04X\n", conf->mac_address);
  printf(" destination MAC address   : %04X\n", dst_mac);
  printf(" ACK timeout               : %d us\n", conf->timeout);
  printf(" Max. retransmissions      : %d tries\n", conf->retrans);
  printf(" flags                     : 0x%08lx\n", conf->flags);
  for(flag = 0x1 ; flag <= LORAMAC_NOACK ; flag <<= 1) {
    if(conf->flags & flag)
      printf("  - %s\n", loramac_flag2str(flag));
  }
}

static void print_help(const char *name)
{
  struct opt_help messages[] = {
    { 'h', "help",          "Show this help message" },
    { 'V', "version",       "Show version information" },
    { 'v', "verbose",       "Enable verbose mode" },
#ifdef COMMIT
    { 0, "commit",          "Display commit information" }
#endif /* COMMIT */
    { 'p', "destination",     "Do not filter packets to another destination" },
    { 'i', "invalid",         "Do not filter invalid packets (packet header, CRC)" },
    { 'b', "no-broadcast",    "Ignore broadcast messages" },
    { 'a', "no-ack",          "Do not answer nor expect ACKs" },
    { 't', "timeout",         "ACK timeout in microseconds (default 50ms)" },
    { 'r', "retransmissions", "Maximum number of retransmissions (default 3)" },
    { 'm', "mode",            "Interface mode (use ? or list to display available modes)" },
    { 'B', "baud",            "Specify the baud rate (default 9600)"},
    { 'd', "destination",     "Destination MAC (hex. short address, default to broadcast)" },
    { 0, NULL, NULL }
  };

  help(name, "[OPTIONS] source device", messages);
}

int main(int argc, char *argv[])
{
  pthread_t output_thread, input_thread;
  const char *prog_name;
  const char *device;
  const char *speed_str = strdup("9600");
  const struct iface_mode *mode = &stdio_mode;
  struct context ctx = {
    .verbose = 0,
    .dst_mac = 0xffff
  };
  struct loramac_config loramac = {
    .uart_send       = uart_send,
    .start_ack_timer = start_timer,
    .stop_ack_timer  = stop_timer,
    .wait_ack_timer  = wait_timer,
    .lock            = lock,
    .unlock          = unlock,
    .htons           = htons,
    .ntohs           = ntohs,
    .recv_frame      = loramac_recv_frame,
    .retrans         = 3,
    .timeout         = 50000,
    .flags           = 0
  };
  struct thread_data data;
  speed_t speed    = B9600;
  int exit_status  = EXIT_FAILURE;
  int err;

  enum opt {
    OPT_COMMIT = 0x100
  };

  struct option opts[] = {
    { "help", no_argument, NULL, 'h' },
    { "version", no_argument, NULL, 'V' },
    { "verbose", no_argument, NULL, 'v' },
#ifdef COMMIT
    { "commit", no_argument, NULL, OPT_COMMIT },
#endif /* COMMIT */

    /* flags */
    { "promiscuous", no_argument, NULL, 'p' },
    { "invalid", no_argument, NULL, 'i' },
    { "no-broadcast", no_argument, NULL, 'b' },
    { "no-ack", no_argument, NULL, 'a' },

    { "timeout", required_argument, NULL, 't' },
    { "retransmissions", required_argument, NULL, 'r' },
    { "mode", required_argument, NULL, 'm' },
    { "baud", required_argument, NULL, 'B' },
    { "destination", required_argument, NULL, 'd' },
    { NULL, 0, NULL, 0 }
  };

  prog_name = basename(argv[0]);

  while(1) {
    int c = getopt_long(argc, argv, "hVvpibat:r:m:B:d:", opts, NULL);

    if(c == -1)
      break;
    switch(c) {
    case 'v':
      ctx.verbose = 1;
      break;
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
      ctx.dst_mac = strtol(optarg, NULL, 16);
      break;
    case 't':
      /* FIXME: should understand a time suffix */
      loramac.timeout = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse timeout value");
    case 'r':
      loramac.retrans = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse retransmissions value");
    case 'm':
      if(!strcmp(optarg, "list") || !strcmp(optarg, "?")) {
        walk_modes(display_mode, NULL);

        exit_status = EXIT_SUCCESS;
        goto EXIT;
      }

      mode = select_mode_by_name(optarg);
      if(!mode) {
        /* mode not found */
        fprintf(stderr, "error: Iface mode \"%s\" not found.\n"
                        "       Use list or ? to display available modes\n",
                optarg);
        goto EXIT;
      }
      break;
    case 'B':
      speed_str = strdup(optarg);
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

  if(argc != 2) {
    print_help(prog_name);
    goto EXIT;
  }

  loramac.mac_address = strtol(argv[0], NULL, 16);
  device              = argv[1];

  exit_status = EXIT_SUCCESS;

  /* display summary */
  IF_VERBOSE(&ctx, display_summary(mode, &loramac, ctx.dst_mac, device, speed_str));

  /* initialize serial */
  serial_init(device, speed);
  IF_VERBOSE(&ctx, printf("Serial initialized!\n"));

  /* initialize mode */
  mode->before(&ctx, &loramac);

  data = (struct thread_data){
    .mode   = mode,
    .ctx    = &ctx,
    .config = &loramac
  };
  err  = pthread_create(&output_thread, NULL, output_thread_func, &data);
  err |= pthread_create(&input_thread, NULL, input_thread_func, &data);
  if(err)
    errx(EXIT_FAILURE, "cannot create threads");

  /* FIXME: read return value */
  pthread_join(output_thread, NULL);
  pthread_join(input_thread, NULL);

EXIT:
  free((void *)speed_str);
  exit(exit_status);
}
