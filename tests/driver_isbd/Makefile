include ../Makefile.tests_common

USEMODULE += isbd
USEMODULE += shell
USEMODULE += shell_commands

#Print AT echos + responses for debugging purposes
AT_PRINT_INCOMING ?= 1

# export parameters 
CFLAGS += -DAT_PRINT_INCOMING=$(AT_PRINT_INCOMING)
#CFLAGS += -O0 -g

include $(RIOTBASE)/Makefile.include
