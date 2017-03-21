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

/* FIXME: This is Linux specific, isn't it?
   We need a pragma to avoid compilation on
   other OS. */

enum rpi_gpio_mode {
  RPI_GPIO_OUT  = 0x0,
  RPI_GPIO_IN   = 0x1,
  RPI_GPIO_ALT0 = 0x4,
  RPI_GPIO_ALT1 = 0x5,
  RPI_GPIO_ALT2 = 0x6,
  RPI_GPIO_ALT3 = 0x7,
  RPI_GPIO_ALT4 = 0x3,
  RPI_GPIO_ALT5 = 0x2
};

/* Map GPIOs on the RPi */
void rpi_gpio_init(void);

/* Unmap GPIOs from the RPi */
void rpi_gpio_destroy(void);

/* Configure the mode for the GPIO (see rpi_gpio_mode).
   Note that contrary to some example that you can find
   on the Internet, you can directly configure the desired
   mode here. There is no need to configure as input beforehand. */
void rpi_gpio_set_mode(unsigned int gpio, unsigned int mode);

/* Set a GPIO */
void rpi_gpio_set(unsigned int gpio);

/* Clear a GPIO */
void rpi_gpio_clr(unsigned int gpio);

/* Get the state of a GPIO */
int rpi_gpio_get(unsigned int gpio);

/* Check that the GPIO number is valid */
int rpi_gpio_check(unsigned int gpio);
