include ../Makefile.tests_common

USEMODULE += co2_ezo

# Example for providing settings through CFLAGS. Change them based on your setup
CO2_EZO_PARAM_I2C  ?= I2C_DEV\(0\)
CO2_EZO_PARAM_ADDR ?= 0x69

# export parameters. Uncomment the one you want to use
#CFLAGS += -DCO2_EZO_PARAM_I2C=$(CO2_EZO_PARAM_I2C)
#CFLAGS += -DCO2_EZO_PARAM_ADDR=$(CO2_EZO_PARAM_ADDR)

include $(RIOTBASE)/Makefile.include
