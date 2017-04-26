include commands.mk

OPTS    := -O2
CFLAGS  := -std=c99 $(OPTS) -fPIC -Wall
LDFLAGS := -lpthread

SRC = $(wildcard *.c)
OBJ = $(foreach obj, $(SRC:.c=.o), $(notdir $(obj)))
DEP = $(SRC:.c=.d)

TARGETS = loramac-stdio loramac-send

COMMON_OBJ = string-utils.o rpi-gpio.o timer.o uart.o lock.o \
						 common.o xatoi.o version.o safe-call.o help.o \
						 loramac-str.o loramac.o dump.o crc-ccitt.o common.o \
						 options.o main.c
STDIO_OBJ  = stdio-mode.o $(COMMON_OBJ)
SEND_OBJ   = send-mode.o time-substract.o scale.o $(COMMON_OBJ)

PREFIX ?= /usr/local
BIN    ?= /bin

CFLAGS += -D_BSD_SOURCE=1

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

.PHONY: all clean

all: $(TARGETS)

loramac-stdio: $(STDIO_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

loramac-send: $(SEND_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -Wp,-MMD,$*.d -c $(CFLAGS) -o $@ $<

clean:
	$(RM) $(DEP)
	$(RM) $(OBJ)
	$(RM) $(CATALOGS)
	$(RM) $(TARGET)

install:
	$(MKDIR) -p $(DESTDIR)/$(PREFIX)/$(BIN)
	$(INSTALL_BIN) loramac-stdio $(DESTDIR)/$(PREFIX)/$(BIN)

uninstall:
	$(RM) $(DESTDIR)/$(PREFIX)/$(BIN)/$(TARGET)

-include $(DEP)
