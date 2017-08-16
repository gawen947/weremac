CFLAGS  := -std=c99 -O2 -fPIC -Wall -Wextra -MMD -pipe
LDFLAGS := -lpthread -lrt

SRC  = $(wildcard *.c) $(wildcard g3-plc/*.c)
OBJS = $(SRC:.c=.o)
DEPS = $(SRC:.c=.d)

TARGETS = set-acc

SET_ACC_OBJS = set-acc.o

ifeq ($(shell uname),Linux)
	CFLAGS  += -D_BSD_SOURCE=1
	LDFLAGS += -lbsd
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

set-acc: $(SET_ACC_OBJS)
	@echo "===> LD $@"
	$(Q)$(CC) $(SET_ACC_OBJS) $(LDFLAGS) -o $@

%.o: %.c
	@echo "===> CC $<"
	$(Q)$(CC) -c $(CFLAGS) -o $@ $<

clean:
	@echo "===> CLEAN"
	$(Q)rm -f $(DEPS)
	$(Q)rm -f $(OBJS)
	$(Q)rm -f $(TARGETS)

-include $(DEPS)
