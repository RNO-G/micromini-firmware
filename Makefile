#TLDR:  make mcu for micro, otherwise make install to install rno-g-control.h header  (optionally with PREFIX=dest) 


include config.mk


CFLAGS+= \
-x c -mthumb -Os -ffunction-sections -c -std=gnu99  \
-D__SAMD21J18A__ -mcpu=cortex-m0plus -MD -MP  --specs=nano.specs -g3 -D_GNU_SOURCE -fanalyzer

ifeq ($(LORA_DEBUG_GPIO),1)
	CFLAGS += -DUSE_RADIO_DEBUG
endif

ifeq ($(DEBUG_FLAG),1)
	CFLAGS += -DDEBUG
endif

ifeq ($(ENABLE_LORA),1)
	CFLAGS += -D_ENABLE_LORA_
endif


CFLAGS+=-D_RNO_G_MICROMINI_REV_$(REV)
VERSION_FLAGS+=-D_RNO_G_MICROMINI_REV_$(REV)



CFLAGS+= $(WARNINGS)


include asf4.mk
#include lorawan.mk


CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
OC=arm-none-eabi-objcopy
SZ=arm-none-eabi-size

#shared flags between application and bootloader
LD_FLAGS_PRE= -Wl,--start-group -lm -Wl,--end-group -mthumb
LD_FLAGS_POST=-Llinker/ --specs=nano.specs -mcpu=cortex-m0plus -Wl,--gc-sections


INCLUDES=$(ASF4_INCLUDES) -I./ -Iinclude/

APP_OBJS=spi_flash.o io.o printf.o driver_init.o main.o i2c_client.o measurement.o gpio.o time.o i2cbus.o

OBJS=$(ASF4_OBJS) $(LORAWAN_OBJS)
OBJS+=$(addprefix $(BUILD_DIR)/application/, $(APP_OBJS))

# List the dependency files
DEPS := $(OBJS:%.o=%.d)



MKDIRS:= $(BUILD_DIR) $(ASF4_MKDIRS) $(LORAWAN_MKDIRS) $(BUILD_DIR)/application 
OUTPUT_NAME := $(BUILD_DIR)/rno-G-micromini

.PHONY: help install mcu clean rev release

help:
	@echo "Targets: "
	@echo "  help:  print this message"
	@echo "  mcu:  build mcu firwmware (requires cross-compiler). Will be built in $(BUILD_DIR). Can specify REV if you want a different revision, e.g. make mcu REV=D "
	@echo "  clean: Clean everything"


# MCU
mcu: $(MKDIRS) $(OUTPUT_NAME).bin $(OUTPUT_NAME).combined.bin  $(OUTPUT_NAME).hex $(OUTPUT_NAME).uf2 rev

rev:
	echo REV_$(REV) > $(BUILD_DIR)/rev.txt

# Detect changes in the dependent files and recompile the respective object files.
ifneq ($(MAKECMDGOALS),clean)

ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif

endif


%.combined.bin: bootloader.bin %.bin
	cat $^ > $@

%.bin: %.elf
	$(OC) -O binary $< $@

%.uf2: %.bin
	-uf2conv.py -i $< -o $@

%.hex: %.elf
	$(OC) -O ihex -R .eeprom -R .fuse -R .lock -R .signature $< $@

%.eep: %.elf
	$(OC) -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma \
        .eeprom=0 --no-change-warnings -O binary $< \
        $@ || exit 0
$.lss: %.elf
	$(OC) -h -S $< > $@

config.mk: config.mk.default
	@echo "Copying config.mk.default to config.mk (backing up old if it exists)"
	@ if [ -f "$@" ] ; then  diff  -q $@ $< || cp $@ $@.`date -Is`.backup ;  fi
	@cp $< $@

$(OUTPUT_NAME).elf: $(OBJS)
	@echo Building target: $@
	$(CC) -o $@ $^ $(LD_FLAGS_PRE) -Wl,-Map=$(OUTPUT_NAME).map -Tlinker/main.ld  $(LD_FLAGS_POST)
	"arm-none-eabi-size" $@

# special case lorawan includes  (so we don't have to modify the reference implementation files)
$(BUILD_DIR)/lorawan/%.o: lorawan/%.c config.mk lorawan.mk
	@echo Building lorawan file: $<
	$(CC) $(CFLAGS) $(INCLUDES) $(LORAWAN_INCLUDES) -MF$(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o) -DREGION_US915 -o $@ $<
	@echo Finished building: $<

# Compiler targets
$(BUILD_DIR)/%.o: %.c Makefile config.mk
	@echo Building file: $<
	$(CC) $(CFLAGS) $(INCLUDES) -MF$(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o) -o $@ $<
	@echo Finished building: $<


$(BUILD_DIR)/%.o: %.s
	@echo Building file: $<
	$(AS) $(CFLAGS) -MF $(@:%.o=%.d) -MT$(@:%.o=%.d) -MT$(@:%.o=%.o)  -o $@ $<
	@echo Finished building: $<

$(BUILD_DIR)/%.o: %.S
	@echo Building file: $<
	$(CC) $(CFLAGS) -MF $(@:%.o=%.d) -MT$(@:%.o=%.d)  -MT$(@:%.o=%.o) -o $@ $<
	@echo Finished building: $<


$(BUILD_DIR)/test_base64: shared/base64.c | $(BUILD_DIR)
	gcc -o $@ -Os -D_TEST_ -D_HOST_ $<  -I./



$(MKDIRS):
	mkdir -p "$@"

clean:
	rm -rf $(BUILD_DIR)
