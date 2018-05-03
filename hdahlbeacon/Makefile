CONTIKI_PROJECT = application

PLATFORMS_ONLY = srf06-cc26xx

TARGET=srf06-cc26xx
BOARD=sensortag/cc2650

# Devpack fix
ifeq ($(BOARD), sensortag/cc2650)
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=1
else
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=0
endif

# Ignore unused functions
CFLAGS+=-Wno-unused-function	

all: $(CONTIKI_PROJECT)

CONTIKI = ../contiki-ng/
include $(CONTIKI)/Makefile.include
