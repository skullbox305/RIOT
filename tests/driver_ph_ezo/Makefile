include ../Makefile.tests_common

USEMODULE += ph_ezo
USEMODULE += event_callback

# Example for providing settings through CFLAGS. Change them based on your setup
PH_EZO_PARAM_I2C  ?= I2C_DEV\(0\)
PH_EZO_PARAM_ADDR ?= 0x64
PH_EZO_PARAM_ALARM_INT_PIN ?= GPIO_PIN\(0,28\)

# export parameters. Uncomment the one you want to use
#CFLAGS += -DPH_EZO_PARAM_I2C=$(PH_EZO_PARAM_I2C)
#CFLAGS += -DPH_EZO_PARAM_ADDR=$(PH_EZO_PARAM_ADDR)
#CFLAGS += -DPH_EZO_PARAM_ALARM_INT_PIN=$(PH_EZO_PARAM_ALARM_INT_PIN)

include $(RIOTBASE)/Makefile.include
