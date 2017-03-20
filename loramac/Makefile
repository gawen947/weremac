include commands.mk

OPTS    := -O2
CFLAGS  := -std=c99 $(OPTS) -fPIC -Wall
LDFLAGS := -lpthread

SRC = $(wildcard *.c)
OBJ = $(foreach obj, $(SRC:.c=.o), $(notdir $(obj)))
DEP = $(SRC:.c=.d)

TARGET = loramac

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

all: $(TARGET)

$(TARGET): $(OBJ)
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
	$(INSTALL_BIN) trace $(DESTDIR)/$(PREFIX)/$(BIN)

uninstall:
	$(RM) $(DESTDIR)/$(PREFIX)/$(BIN)/$(TARGET)

-include $(DEP)
