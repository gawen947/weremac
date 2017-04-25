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
# define _POSIX_SOURCE 1
#endif /* __linux__ */

#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <err.h>

#include "common.h"
#include "timer.h"

static pthread_mutex_t  lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   cond = PTHREAD_COND_INITIALIZER;
static struct itimerval timer;

void timer_expired(int signum)
{
  UNUSED(signum);

  stop_timer();
}

void init_timer(void)
{
  struct sigaction sa;
  sigset_t set;

  /* unblock SIGALRM */
  sigemptyset(&set);
  sigaddset(&set, SIGALRM);

  if(pthread_sigmask(SIG_UNBLOCK, &set, NULL))
    err(EXIT_FAILURE, "cannot unblock signal");

  /* configure handler */
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &timer_expired;
  sigaction(SIGALRM, &sa, NULL);
}

void start_timer(unsigned int timeout)
{
  /* one shot timer */
  pthread_mutex_lock(&lock);
  {
    timer.it_value = (struct timeval){ .tv_sec  = timeout / 1000000,
                                       .tv_usec = timeout % 1000000 };
    setitimer(ITIMER_REAL, &timer, NULL);
  }
  pthread_mutex_unlock(&lock);
}

void wait_timer(void)
{
  pthread_mutex_lock(&lock);
  {
    if(timer.it_value.tv_sec || timer.it_value.tv_usec)
      pthread_cond_wait(&cond, &lock);
  }
  pthread_mutex_unlock(&lock);
}

void stop_timer(void)
{
  /* stop and unlock */
  pthread_mutex_lock(&lock);
  {
    timer.it_value.tv_usec = 0;
    pthread_cond_signal(&cond);
  }
  pthread_mutex_unlock(&lock);
}
