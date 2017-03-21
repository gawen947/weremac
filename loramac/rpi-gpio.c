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

#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <err.h>

#include "safe-call.h"

/* FIXME: This is Linux specific, isn't it?
   We need a pragma to avoid compilation on
   other OS. */

#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE         (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE        4096

static volatile uint32_t *gpio_reg = MAP_FAILED;

void rpi_gpio_init(void)
{
  int fd = xopen("/dev/mem", O_RDWR | O_SYNC, 0);
  gpio_reg = mmap(0, BLOCK_SIZE,
                  PROT_READ | PROT_WRITE,
                  MAP_SHARED,
                  fd, GPIO_BASE);
  if(gpio_reg < 0)
    err(EXIT_FAILURE, "cannot mmap GPIO");
  close(fd);
}

void rpi_gpio_destroy(void)
{
  munmap((void *)gpio_reg, BLOCK_SIZE);
}

void rpi_gpio_set_mode(unsigned int gpio, unsigned int mode)
{
  int reg, shift;

  reg   = gpio / 10;
  shift = (gpio % 10) * 3;

  gpio_reg[reg] = (gpio_reg[reg] & ~(7 << shift)) | /* clear pin conf */
                  (mode << shift);                  /* apply mode */
}

void rpi_gpio_set(unsigned int gpio)
{
  *(gpio_reg + 7)  = 1 << gpio;
}

void rpi_gpio_clr(unsigned int gpio)
{
  *(gpio_reg + 10) = 1 << gpio;
}

int rpi_gpio_get(unsigned int gpio)
{
  return *(gpio_reg + 13) & (1 << gpio);
}

int rpi_gpio_check(unsigned int gpio)
{
  /* FIXME: not sure this is sufficient */
  if(gpio <= 27)
    return 1;
  return 0;
}
