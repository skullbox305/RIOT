include ../Makefile.tests_common

USEMODULE += isbd
USEMODULE += shell
USEMODULE += shell_commands
#USEMODULE += ps

#Print AT echos + responses for debugging purposes
AT_PRINT_INCOMING ?= 1
#The ISBD is not always terminating the echo with \r\n
AT_RECV_NO_EOL ?=1
CONFIG_ISBD_TEST_MODE ?=1

# export parameters 
CFLAGS += -DAT_PRINT_INCOMING=$(AT_PRINT_INCOMING)
CFLAGS += -DAT_RECV_NO_EOL=$(AT_RECV_NO_EOL)
CFLAGS += -DCONFIG_ISBD_TEST_MODE=$(CONFIG_ISBD_TEST_MODE)
CFLAGS += -O0 -g

include $(RIOTBASE)/Makefile.include
