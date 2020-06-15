include ../Makefile.tests_common

USEMODULE += isbd
USEMODULE += at
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
#USEMODULE += periph_lpuart


# set board specific peripheral configurations
#ifeq (,$(filter nrf52840-mdk,$(BOARD)))
#  ISBD_PARAM_SLEEP_PIN  ?= GPIO_PIN\(0,25\) 
#  ISBD_PARAM_NET_AVAIL_PIN ?= GPIO_PIN\(0,21\)
#  ISBD_PARAM_RING_PIN ?= GPIO_PIN\(0,12\)
#endif

#ifeq (,$(filter nucleo-l031k6,$(BOARD)))
#  ISBD_PARAM_SLEEP_PIN  ?= GPIO_PIN\(0,25\) 
#  ISBD_PARAM_NET_AVAIL_PIN ?= GPIO_PIN\(0,21\)
#  ISBD_PARAM_RING_PIN ?= GPIO_PIN\(0,12\)
#endif

#Print AT echos + responses for debugging purposes
AT_PRINT_INCOMING ?= 1
#The ISBD is not always terminating the echo with \r\n but sometimes only with \r
AT_SEND_ECHO_NO_EOL_2 ?= 1

# fallback: set pins to default values
ISBD_PARAM_SLEEP_PIN  ?= GPIO_UNDEF 
ISBD_PARAM_NET_AVAIL_PIN ?= GPIO_UNDEF 
ISBD_PARAM_RING_PIN ?= GPIO_UNDEF 

# export parameters 
CFLAGS += -DISBD_PARAM_SLEEP_PIN=$(ISBD_PARAM_SLEEP_PIN)
CFLAGS += -DISBD_PARAM_NET_AVAIL_PIN=$(ISBD_PARAM_NET_AVAIL_PIN)
CFLAGS += -DISBD_PARAM_RING_PIN=$(ISBD_PARAM_RING_PIN)

CFLAGS += -DAT_PRINT_INCOMING=$(AT_PRINT_INCOMING)
CFLAGS += -DAT_SEND_ECHO_NO_EOL_2=$(AT_SEND_ECHO_NO_EOL_2)
CFLAGS += -O0 -g

include $(RIOTBASE)/Makefile.include
