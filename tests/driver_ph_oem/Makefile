include ../Makefile.tests_common

USEMODULE += ph_oem

#Example for providing device settings thru CFLAGS. Change based on your setup
PH_OEM_PARAM_I2C  ?= I2C_DEV\(0\)
PH_OEM_PARAM_ADDR ?= 0x65
PH_OEM_PARAM_INTERRUPT_PIN ?= GPIO_PIN\(2,6\)

# export parameters. Uncomment the one you want to use
#CFLAGS += -DPH_OEM_PARAM_I2C=$(PH_OEM_PARAM_I2C)
#CFLAGS += -DPH_OEM_PARAM_ADDR=$(PH_OEM_PARAM_ADDR)
#CFLAGS += -DPH_OEM_PARAM_INTERRUPT_PIN=$(PH_OEM_PARAM_INTERRUPT_PIN)

include $(RIOTBASE)/Makefile.include
