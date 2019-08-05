include ../Makefile.tests_common

USEMODULE += co2_ezo
USEMODULE += event_callback

# Example for providing settings through CFLAGS. Change them based on your setup
CO2_EZO_PARAM_I2C  ?= I2C_DEV\(0\)
CO2_EZO_PARAM_ADDR ?= 0x68
CO2_EZO_PARAM_ALARM_INT_PIN ?= GPIO_PIN\(0,28\)

# export parameters. Uncomment the one you want to use
#CFLAGS += -DCO2_EZO_PARAM_I2C=$(CO2_EZO_PARAM_I2C)
#CFLAGS += -DCO2_EZO_PARAM_ADDR=$(CO2_EZO_PARAM_ADDR)
#CFLAGS += -DCO2_EZO_PARAM_ALARM_INT_PIN=$(CO2_EZO_PARAM_ALARM_INT_PIN)

include $(RIOTBASE)/Makefile.include
