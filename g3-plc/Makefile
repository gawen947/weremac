include commands.mk

OPTS    := -O2
CFLAGS  := -std=c99 $(OPTS) -fPIC -Wall
LDFLAGS := -lpthread -lrt

SRC = $(shell find . -path ./test -prune -o -name '*.c' )
OBJ = $(patsubst %.c,%.o,$(SRC))

TARGETS = g3plc-stdio g3plc-send g3plc-unix

G3PLC_OBJ  = g3-plc/g3plc.o g3-plc/g3plc-cmd.o g3-plc/pack.o \
						 g3-plc/cmdbuf.o g3-plc/crc32.o g3-plc/g3plc-str.o \
						 g3-plc/g3plc-cmd-str.o
COMMON_OBJ = string-utils.o rpi-gpio.o timer.o uart.o lock.o \
						 common.o xatoi.o version.o safe-call.o help.o \
						 dump.o crc-ccitt.o common.o options.o main.c
STDIO_OBJ  = stdio-mode.o $(COMMON_OBJ) $(G3PLC_OBJ)
SEND_OBJ   = send-mode.o time-substract.o scale.o $(COMMON_OBJ) $(G3PLC_OBJ)
UNIX_OBJ   = unix-mode.o $(COMMON_OBJ) $(G3PLC_OBJ)

PREFIX ?= /usr/local
BIN    ?= /bin

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

.PHONY: all clean g3-plc

all: $(TARGETS)

g3plc-stdio: $(STDIO_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

g3plc-send: $(SEND_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

g3plc-unix: $(UNIX_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -Wp,-MMD,$*.d -c $(CFLAGS) -o $@ $<

clean:
	$(RM) $(DEP)
	$(RM) $(OBJ)
	$(RM) $(CATALOGS)
	$(RM) $(TARGETS)

install:
	$(MKDIR) -p $(DESTDIR)/$(PREFIX)/$(BIN)
	$(INSTALL_BIN) g3plc-stdio $(DESTDIR)/$(PREFIX)/$(BIN)
	$(INSTALL_BIN) g3plc-send $(DESTDIR)/$(PREFIX)/$(BIN)
	$(INSTALL_BIN) g3plc-unix $(DESTDIR)/$(PREFIX)/$(BIN)

uninstall:
	$(RM) $(DESTDIR)/$(PREFIX)/$(BIN)/$(TARGET)

-include $(DEP)
