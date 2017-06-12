CFLAGS  := -std=c99 -O2 -fPIC -Wall -MMD -pipe
LDFLAGS := -lpthread -lrt

SRC  = $(wildcard *.c) $(wildcard g3-plc/*.c)
OBJS = $(SRC:.c=.o)
DEPS = $(SRC:.c=.d)

TARGETS = g3plc-stdio g3plc-send g3plc-unix

G3PLC_OBJS  = g3-plc/g3plc.o g3-plc/g3plc-cmd.o g3-plc/pack.o \
						g3-plc/cmdbuf.o g3-plc/crc32.o g3-plc/g3plc-str.o \
						g3-plc/g3plc-cmd-str.o
COMMON_OBJS = string-utils.o rpi-gpio.o timer.o uart.o lock.o \
						common.o xatoi.o version.o safe-call.o help.o \
						dump.o crc-ccitt.o options.o main.o
STDIO_OBJS  = stdio-mode.o $(COMMON_OBJS) $(G3PLC_OBJS)
SEND_OBJS   = send-mode.o time-substract.o scale.o $(COMMON_OBJS) $(G3PLC_OBJS)
UNIX_OBJS   = unix-mode.o $(COMMON_OBJS) $(G3PLC_OBJS)

ifeq ($(shell uname),Linux)
	CFLAGS  += -D_BSD_SOURCE=1
	LDFLAGS += -lbsd
endif

commit = $(shell ./hash.sh)
ifneq ($(commit), UNKNOWN)
	CFLAGS += -DCOMMIT="\"$(commit)\""
	CFLAGS += -DPARTIAL_COMMIT="\"$(shell echo $(commit) | cut -c1-8)\""
endif

ifndef DISABLE_DEBUG
	CFLAGS += -ggdb
else
	CFLAGS += -DNDEBUG=1
endif

ifdef VERBOSE
	Q :=
else
	Q := @
endif

.PHONY: all clean

all: $(TARGETS)

g3plc-stdio: $(STDIO_OBJS)
	@echo "===> LD $@"
	$(Q)$(CC) $(STDIO_OBJS) $(LDFLAGS) -o $@

g3plc-send: $(SEND_OBJS)
	@echo "===> LD $@"
	$(Q)$(CC) $(SEND_OBJS) $(LDFLAGS) -o $@

g3plc-unix: $(UNIX_OBJS)
	@echo "===> LD $@"
	$(Q)$(CC) $(UNIX_OBJS) $(LDFLAGS) -o $@

%.o: %.c
	@echo "===> CC $<"
	$(Q)$(CC) -c $(CFLAGS) -o $@ $<

clean:
	@echo "===> CLEAN"
	$(Q)rm -f $(DEPS)
	$(Q)rm -f $(OBJS)
	$(Q)rm -f $(TARGETS)

-include $(DEPS)
