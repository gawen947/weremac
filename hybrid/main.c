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

#ifdef __linux__
# include <bsd/stdlib.h>
#endif /* __linux__ */

#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <err.h>

#include "hybrid/hybrid.h"
#include "hybrid/hybrid-str.h"
#include "g3plc/g3plc.h"
#include "string-utils.h"
#include "rpi-gpio.h"
#include "version.h"
#include "options.h"
#include "common.h"
#include "xatoi.h"
#include "timer.h"
#include "lock.h"
#include "uart.h"
#include "mode.h"
#include "help.h"
#include "main.h"

/* FIXME: avoid global */
static struct context ctx = {
  .verbose    = 0,
  .dst_mac    = 0xffff,
  .gpio_reset = -1,
};

struct io_thread_data {
  const struct iface_mode     *mode;
  const struct context        *ctx;
  const struct hybrid_config *config;
};

static void configure_gpio(const struct context *ctx)
{
  if(ctx->gpio_reset < 0)
    return;

  rpi_gpio_init();
  rpi_gpio_set_mode(ctx->gpio_reset, RPI_GPIO_OUT);
}

static void reset_clear(void)
{
  rpi_gpio_clr(ctx.gpio_reset);
}

static void reset_set(void)
{
  rpi_gpio_set(ctx.gpio_reset);
}

static void initialize_driver(struct context *ctx,
                              const char *lora_dev, const char *g3plc_dev,
                              speed_t speed)
{
  /* initialize serial */
  ctx->lora_uart_fd  = serial_init(&ctx->lora_tty, lora_dev, speed);
  ctx->g3plc_uart_fd = serial_init(&ctx->g3plc_tty, g3plc_dev, speed);
  IF_VERBOSE(ctx, printf("Serial initialized!\n"));

  /* configure GPIO */
  configure_gpio(ctx);
}

/* We must block those signals otherwise
   those might be selected when the SIGALRM
   is raised by the timers instead of the
   main thread where the handler is configured. */
static void thread_block_signals()
{
  sigset_t set;

  sigemptyset(&set);
  sigaddset(&set, SIGALRM);

  if(pthread_sigmask(SIG_BLOCK, &set, NULL))
    err(EXIT_FAILURE, "cannot block signals");
}

static void * output_thread_func(void *p)
{
  struct io_thread_data *data = (struct io_thread_data *)p;

  data->mode->start(data->ctx);

  return NULL; /* FIXME: return with error code */
}

static void * input_thread_lora_func(void *p)
{
  UNUSED(p);

  thread_block_signals();

  uart_read_loop(ctx.lora_uart_fd, hybrid_lora_uart_putc);

  return NULL; /* FIXME: return with error code */
}

static void * input_thread_g3plc_func(void *p)
{
  UNUSED(p);

  thread_block_signals();

  uart_read_loop(ctx.g3plc_uart_fd, hybrid_g3plc_uart_putc);

  return NULL; /* FIXME: return with error code */
}

static void start_io_threads(const struct context *ctx,
                             const struct hybrid_config *hybrid)
{
  pthread_t output_thread, input_thread_lora, input_thread_g3plc;
  int err;

  struct io_thread_data data = (struct io_thread_data){
    .mode   = &iface_mode,
    .ctx    = ctx,
    .config = hybrid
  };

  thread_block_signals();

  err  = pthread_create(&output_thread, NULL, output_thread_func, &data);
  err |= pthread_create(&input_thread_lora, NULL, input_thread_lora_func, &data);
  err |= pthread_create(&input_thread_g3plc, NULL, input_thread_g3plc_func, &data);
  if(err)
    errx(EXIT_FAILURE, "cannot create threads");

  pthread_join(output_thread, NULL);
}

static int uart_lora_send(const void *buf, unsigned int size)
{
  return uart_send(ctx.lora_uart_fd, buf, size);
}

static int uart_g3plc_send(const void *buf, unsigned int size)
{
  return uart_send(ctx.g3plc_uart_fd, buf, size);
}

static int uart_g3plc_read(void *buf, unsigned int size)
{
  return uart_read(ctx.g3plc_uart_fd, buf, size);
}

static int set_uart_g3plc_speed(unsigned int speed)
{
  return set_uart_speed(&ctx.g3plc_tty, ctx.g3plc_uart_fd, speed);
}

static void usleep_UL(unsigned long duration)
{
  usleep(duration);
}

static uint8_t rnd_seqno(void)
{
  return arc4random_uniform(0xff);
}

/* Display a summary of the MAC layer configuration. */
static void display_summary(const struct iface_mode *mode,
                            const struct hybrid_config *conf,
                            const struct context *ctx,
                            uint16_t dst_mac,
                            const char *lora_dev,
                            const char *g3plc_dev,
                            const char *speed)
{
  unsigned long flag;

  printf(PACKAGE_VERSION "\n");
  printf("Using %s mode on %s (LoRa) and %s (G3PLC) @%s bauds.\n", mode->name, lora_dev, g3plc_dev, speed);
  if((ctx->gpio_reset > 0)) {
    printf("GPIO configured on:\n");
    printf("  - RESET: %d\n", ctx->gpio_reset);
  }
  printf(" iface (source) MAC address: %04X\n", conf->mac_address);
  printf(" destination MAC address   : %04X\n", dst_mac);
  printf(" CMD timeout               : %d us\n", conf->g3plc.timeout);
  printf(" Max. G3PLC retransmissions: %d tries\n", conf->g3plc.retrans);
  printf(" LoRa ACK timeout          : %d us\n", conf->lora.timeout);
  printf(" LoRa SIFS                 : %d us\n", conf->lora.sifs);
  printf(" flags                     : 0x%08lx\n", conf->flags);
  for(flag = 0x1 ; flag <= HYBRID_NOACK ; flag <<= 1) {
    if(conf->flags & flag)
      printf("  - %s\n", hybrid_flag2str(flag));
  }
}

static void print_help(const char *name, const char *mode_name,
                       struct opt_help *extra_messages)
{
  struct opt_help common_messages[] = {
    { 'h', "help",            "Show this help message" },
    { 'V', "version",         "Show version information" },
    { 'v', "verbose",         "Enable verbose mode" },
#ifdef COMMIT
    { 0,   "commit",          "Display commit information" }
#endif /* COMMIT */
    { 'i', "invalid",         "Do not filter invalid packets (packet header, CRC)" },
    { 'a', "no-ack",          "Do not answer nor expect ACKs" },
    { 't', "ack-timeout",     "LoRa ACK timeout in microseconds (default 4s)" },
    { 'T', "cmd-timeout",     "G3-PLC command timeout (default 1s)" },
    { 's', "sifs",            "Short Inter Frame Spacing time in microseconds (default 2s)" },
    { 'S', "seqno",           "Initial sequence number (default: random)" },
    { 'r', "retransmissions", "Maximum number of retransmissions (default 3)" },
    { 'B', "baud",            "Specify the baud rate (default 9600)" },
    { 'd', "destination",     "Destination MAC (hex. short address, default to broadcast)" },
    { 0,   "reset",           "RESET RPi GPIO" },
    { 0, NULL, NULL }
  };

  help(name, "[OPTIONS] source lora-dev g3plc-dev", common_messages);

  if(extra_messages) {
    fprintf(stderr, "\nExtra options for %s mode:\n", mode_name);
    help(name, NULL, extra_messages);
  }
}

int main(int argc, char *argv[])
{
  const char *prog_name;
  const char *lora_dev, *g3plc_dev;
  const char *speed_str = strdup("9600");
  struct hybrid_config hybrid = {
    .start_timer          = start_timer,
    .stop_timer           = stop_timer,
    .wait_timer           = wait_timer,
    .uart_lora_send       = uart_lora_send,
    .uart_g3plc_send      = uart_g3plc_send,
    .uart_g3plc_read      = uart_g3plc_read,
    .lock                 = lock,
    .unlock               = unlock,
    .set_uart_g3plc_speed = set_uart_g3plc_speed,
    .lora_recv_frame      = hybrid_lora_recv_frame,
    .g3plc_recv_frame     = hybrid_g3plc_recv_frame,
    .reset_clear          = reset_clear,
    .reset_set            = reset_set,
    .htons                = htons,
    .ntohs                = ntohs,
    .htonl                = htonl,
    .ntohl                = ntohl,
    .usleep               = usleep_UL,

    .lora = (struct lora_opt){
      .seqno   = rnd_seqno(),
      .retrans = 3,
      .timeout = 4000000, /* 4 seconds */
      .sifs    = 2000000, /* 2 seconds */
    },
    .g3plc = (struct g3plc_opt){
      .bandplan       = G3PLC_BP_CENELEC_A, /* FIXME: option */
      .pan_id         = 0xAAAA,             /* FIXME: option */
      .ext_address    = 0,                  /* FIXME: option */
      .retrans        = 5,
      .timeout        = 1000000,  /* 1 second */
    },
    .flags          = 0,
    .data           = &ctx
  };
  speed_t speed    = B9600;
  int exit_status  = EXIT_FAILURE;
  int err, val;

  enum opt {
    OPT_COMMIT = 0x100,
    OPT_IRQ,
    OPT_CTS,
    OPT_RESET,
  };

  /* Common options used by all modes. */
  struct option common_opts[] = {
    { "help", no_argument, NULL, 'h' },
    { "version", no_argument, NULL, 'V' },
    { "verbose", no_argument, NULL, 'v' },
#ifdef COMMIT
    { "commit", no_argument, NULL, OPT_COMMIT },
#endif /* COMMIT */

    /* flags */
    { "invalid", no_argument, NULL, 'i' },
    { "no-ack", no_argument, NULL, 'a' },

    { "ack-timeout", required_argument, NULL, 't' },
    { "cmd-timeout", required_argument, NULL, 'T' },
    { "sifs", required_argument, NULL, 's' },
    { "seqno", required_argument, NULL, 'S' },
    { "retransmissions", required_argument, NULL, 'r' },
    { "baud", required_argument, NULL, 'B' },
    { "destination", required_argument, NULL, 'd' },

    /* GPIO configuration */
    { "reset", required_argument, NULL, OPT_RESET },
    { NULL, 0, NULL, 0 }
  };

  /* Options string and long options
     structure are merged from both
     the common options and the mode
     (stdio, ping, ...) options. */
  char * optstring_merged    = strcat_dup("hVviat:T:s:S:r:B:d:", iface_mode.optstring);
  struct option *opts_merged = merge_opts(common_opts, iface_mode.long_opts);

  prog_name = basename(argv[0]);

  while(1) {
    int c = getopt_long(argc, argv, optstring_merged, opts_merged, NULL);

    if(c == -1)
      break;

    /* parse mode options first */
    if(iface_mode.parse_option(&ctx, c))
      continue; /* this was a mode option, skip parsing common options */

    switch(c) {
    case OPT_RESET:
      ctx.gpio_reset = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse RESET GPIO");
      else if(!rpi_gpio_check(ctx.gpio_reset))
        errx(EXIT_FAILURE, "invalid RESET GPIO number");
      break;
    case 'v':
      ctx.verbose = 1;
      break;
    case 'i':
      hybrid.flags |= G3PLC_INVALID;
      break;
    case 'a':
      hybrid.flags |= G3PLC_NOACK;
      break;
    case 'd':
      ctx.dst_mac = strtol(optarg, NULL, 16);
      break;
    case 't':
      /* FIXME: should understand a time suffix */
      hybrid.lora.timeout = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse timeout value");
      break;
    case 'T':
      /* FIXME: should understand a time suffix */
      hybrid.g3plc.timeout = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse timeout value");
      break;
    case 's':
      hybrid.lora.sifs = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse SIFS value");
      break;
    case 'S':
      val = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse sequence number");
      else if(val > 0xff)
        errx(EXIT_FAILURE, "sequence number too large");
      hybrid.lora.seqno = val;
    case 'r':
      hybrid.lora.retrans = hybrid.g3plc.retrans = xatou(optarg, &err);
      if(err)
        errx(EXIT_FAILURE, "cannot parse retransmissions value");
      if(hybrid.g3plc.retrans < 1)
        errx(EXIT_FAILURE, "invalid number of retransmissions");
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
      print_help(prog_name, iface_mode.name, iface_mode.extra_messages);
      goto EXIT;
    }
  }

  argc -= optind;
  argv += optind;

  if(argc != 3) {
    print_help(prog_name, iface_mode.name, iface_mode.extra_messages);
    goto EXIT;
  }

  hybrid.mac_address = strtol(argv[0], NULL, 16);
  lora_dev           = argv[1];
  g3plc_dev          = argv[2];

  exit_status = EXIT_SUCCESS;

  /* display summary */
  IF_VERBOSE(&ctx, display_summary(&iface_mode,
                                   &hybrid,
                                   &ctx,
                                   ctx.dst_mac,
                                   lora_dev,
                                   g3plc_dev,
                                   speed_str));

  initialize_driver(&ctx, lora_dev, g3plc_dev, speed);
  iface_mode.init(&ctx, &hybrid);

  /* Block the SIGALRM signal so that it is
     not received by the main thread when
     a timer is triggered. */
  thread_block_signals();

  /* Initialize hybrid layer.
     The interface mode still has to configure
     the hybrid configuration structure. That
     is why we initialize the MAC layer after
     the mode. */
  err = hybrid_init(&hybrid);
  if(err < 0)
    errx(EXIT_FAILURE, "cannot initialize hybrid");

  /* Start the threads that will handle the IO
     with the hybrid layer. That is:
       - The input thread that read new messages from UART.
       - The output thread that send message according to iface_mode. */
  start_io_threads(&ctx, &hybrid);

  /* IO threads returned, this is the end.
     We can release everything. */
  iface_mode.destroy(&ctx);
EXIT:
  free((void *)speed_str);
  free(optstring_merged);
  free(opts_merged);
  exit(exit_status);
}
