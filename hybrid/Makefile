include commands.mk

OPTS    := -O2
CFLAGS  := -std=c99 $(OPTS) -fPIC -Wall -I.
LDFLAGS := -lpthread -lrt

SRC = $(shell find . -path ./test -prune -o -name '*.c' )
OBJ = $(patsubst %.c,%.o,$(SRC))

TARGETS = hybrid-stdio hybrid-send hybrid-unix

HYBRID     = hybrid/hybrid.o
LORA_OBJ   = lora/loramac.o lora/loramac-str.o lora/crc-ccitt.o
G3PLC_OBJ  = g3plc/g3plc.o g3plc/g3plc-cmd.o g3plc/pack.o \
						 g3plc/cmdbuf.o g3plc/crc32.o g3plc/g3plc-str.o
COMMON_OBJ = string-utils.o rpi-gpio.o timer.o uart.o lock.o \
						 common.o xatoi.o version.o safe-call.o help.o \
						 dump.o common.o options.o main.c
STDIO_OBJ  = stdio-mode.o $(COMMON_OBJ) $(G3PLC_OBJ) $(LORA_OBJ) $(HYBRID)
SEND_OBJ   = send-mode.o time-substract.o scale.o $(COMMON_OBJ) $(G3PLC_OBJ) $(LORA_OBJ) $(HYBRID)
UNIX_OBJ   = unix-mode.o $(COMMON_OBJ) $(G3PLC_OBJ) $(LORA_OBJ) $(HYBRID)

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

.PHONY: all clean

all: $(TARGETS)

hybrid-stdio: $(STDIO_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

hybrid-send: $(SEND_OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

hybrid-unix: $(UNIX_OBJ)
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
