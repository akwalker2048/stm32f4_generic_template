SOURCES_PROJECT = main.c hardware_STM32F407G_DISC1.c stm32f4xx_it.c system_stm32f4xx.c lepton_functions.c
SOURCES_STD_PERIPH = misc.c stm32f4xx_rcc.c stm32f4xx_adc.c stm32f4xx_dac.c stm32f4xx_dma.c stm32f4xx_exti.c stm32f4xx_gpio.c stm32f4xx_tim.c stm32f4xx_usart.c stm32f4xx_syscfg.c stm32f4xx_spi.c stm32f4xx_i2c.c
SOURCES_ASSEMBLY = startup_stm32f40_41xxx.s

SOURCES = $(SOURCES_PROJECT) $(SOURCES_STD_PERIPH)
OBJECTS = $(SOURCES:.c=.o) $(SOURCES_ASSEMBLY:.s=.o)
#Where to put objects
OBJ_DIR = obj
OBJ_OBJECTS := $(addprefix $(OBJ_DIR)/, $(OBJECTS))

include makefile.local

#TOOL_PREFIX = /opt/ARM/arm-eabi
#PRG_PREFIX = $(TOOL_PREFIX)/bin/arm-none-eabi-
#LIB_PREFIX = $(TOOL_PREFIX)/arm-none-eabi/lib/thumb/cortex-m4/float-abi-hard/fpuv4-sp-d16/

#CC      = $(PRG_PREFIX)gcc
#LD      = $(PRG_PREFIX)gcc
#CP      = $(PRG_PREFIX)objcopy
#OD      = $(PRG_PREFIX)objdump

STM32FLASH = ./scripts/stm32_flash.pl


#Where to find sources
LOCAL_SRC_DIR = src
VPATH = $(LOCAL_SRC_DIR) $(ST_STD_PERIPH_SRC)



# -mfloat-abi=name
#    Specifies which floating-point ABI to use.
#    "soft": Causes GCC to generate output containing library calls for
#            floating-point operations. Same as -msoft-float.
#    "softfp": Allows the generation of code using hardware floating-point
#            instructions, but still uses the soft-float calling conventions.
#    "hard": Allows generation of floating-point instructions and uses
#            FPU-specific calling conventions. Same as -mhard-float
#    Note that the hard-float and soft-float ABIs are not link-compatible;
#    you must compile your entire program with the same ABI, and link with a
#    compatible set of libraries.
#
# -msoft-float, -mhard-float: Use -mfloat-abi=name instead.
#
# -mfpu=fpv4-sp-d16
#    Specify which FPU is available. I think, this is clear from the
#    -mcpu= but OTOH, we explicitly link to fpv4-sp-d16 libraries, so let's
#    keep it in there to be sure.
#
# -fsingle-precision-constant
#    Ensure that 1.0 is treated as 1.0f. Avoids that intermediate DOUBLE
#    floats are generated. Writing the 'f' suffix is better but error-prone.
#
# -mfix-cortex-m3-ldrd
#    Is automatically enabled for Cortex-M3.
#
# -fno-common : unclear if needed
#
# -lm -lc
#    Required during linking for sqrtf() etc.
#

#Need to put this pack in CFLAGS????
#-D"assert_param(x)="

CFLAGS  =  -I. -IInclude -Iinclude -Iinc \
	-I$(ST_STD_PERIPH_INCLUDE) -I$(ST_CMSIS_INCLUDE) -I$(GCC_INCLUDE) -I$(ST_CORE_INCLUDE) $(ST_STD_PERIPH_EVAL_INCLUDE) \
	-c -fno-common -O2 -g  \
	-mcpu=cortex-m4 -mthumb \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-fsingle-precision-constant \
	$(LOCAL_CFLAGS)
LFLAGS  = -TSTM32F417IG_FLASH.ld -nostartfiles -L$(LIB_PREFIX)
LFLAGS_END = $(LIB_M_C_PREFIX)/libm.a $(LIB_M_C_PREFIX)/libc.a
CPFLAGS = -Obinary
ODFLAGS = -S

all: main.bin

debug:
	@ echo "Sources:"  $(SOURCES)
	@ echo "Objects:"  $(OBJECTS)

connect:
	@ echo "/* ***************************************************** */"
	@ echo "/* ...connect to target with openocd...                  */"
	@ echo "/* ***************************************************** */"
	$(OPENOCD_CMD)

program: main.bin
	@ echo "/* ***************************************************** */"
	@ echo "/* ...flash main.bin to target...                        */"
	@ echo "/* ***************************************************** */"
	$(STM32FLASH) main.bin

clean:
	-rm -f main.lst $(OBJ_OBJECTS) main.elf main.lst main.bin

main.bin: main.elf
	@ echo "/* ***************************************************** */"
	@ echo "/* ...copying                                             */"
	@ echo "/* ***************************************************** */"
	$(CP) $(CPFLAGS) main.elf main.bin
	$(OD) $(ODFLAGS) main.elf > main.lst

main.elf: $(OBJ_OBJECTS)
	@ echo "/* ***************************************************** */"
	@ echo "/* ...linking                                            */"
	@ echo "/* ***************************************************** */"
	$(LD) $(LFLAGS) -o main.elf $(OBJ_OBJECTS) $(LFLAGS_END)


$(OBJ_DIR)/%.o: %.c
#%.o: %.c
	@ echo "/* ***************************************************** */"
	@ echo "/* ...compiling " $(notdir $<) "*/"
	@ echo "/* ***************************************************** */"
	$(CC) $(CFLAGS) $< -o $@

$(OBJ_DIR)/%.o: %.s
	@ echo "/* ***************************************************** */"
	@ echo "/* ...compiling assembly " $(notdir $<) "*/"
	@ echo "/* ***************************************************** */"
	$(AS) $< -o $@


#main.o: main.c mandelbrot.c mandelbrot.h
#	@ echo ".compiling"
#	$(CC) $(CFLAGS) main.c mandelbrot.c stm32f4xx_it.c  system_stm32f4xx.c startup_stm32f4xx.s \
		stm32f4xx_gpio.c stm32f4xx_rcc.c

run: main.bin
	$(STM32FLASH) main.bin

#mandelbrot: mandelbrot.c
#	gcc -O2 -DTEST_ON_HOST=1 mandelbrot.c -o mandelbrot
#	./mandelbrot

gdb:
	$(PRG_PREFIX)gdb -ex "target remote localhost:3333" \
		-ex "set remote hardware-breakpoint-limit 6" \
		-ex "set remote hardware-watchpoint-limit 4" main.elf
