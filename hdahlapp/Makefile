CONTIKI_PROJECT = hello-hdahl
all: $(CONTIKI_PROJECT)

TARGET=srf06-cc26xx
BOARD=sensortag/cc2650

# Devpack fix
ifeq ($(BOARD), sensortag/cc2650)
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=1
else
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=0
endif

CONTIKI = ../contiki-ng/
include $(CONTIKI)/Makefile.include
