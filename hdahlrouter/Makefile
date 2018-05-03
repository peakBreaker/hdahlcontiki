CONTIKI_PROJECT = border-router
all: $(CONTIKI_PROJECT)
CONTIKI = ../contiki-ng/

# The BR is either native or embedded, and in the latter case must support SLIP
PLATFORMS_EXCLUDE = nrf52dk

TARGET=srf06-cc26xx
BOARD=sensortag/cc2650

# Devpack fix
ifeq ($(BOARD), sensortag/cc2650)
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=1
else
	CFLAGS += -DBOARD_CONF_DEBUGGER_DEVPACK=0
endif

# Include RPL BR module
MODULES += os/services/rpl-border-router

# Include webserver module
MODULES_REL += webserver

# Include optional target-specific module
include $(CONTIKI)/Makefile.identify-target
MODULES_REL += $(TARGET)

include $(CONTIKI)/Makefile.include
