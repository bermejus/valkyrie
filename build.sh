#!/bin/bash

set -e

print_help_message() {
    echo "build.sh is a script for building valkyrie code"
    echo
    echo "Usage: build.sh <arguments...>"
    echo
    echo "Possible arguments are:"
    echo
    echo $'\t'-h, --help$'\t\t'display "help" message
    echo $'\t'-u, --upload$'\t\t'upload generated code through sam-ba
    echo
    echo "Report bugs in https://github.com/bermejus/valkyrie/issues"
}

for arg in $@; do
    if [ "$arg" == "-u" ] || [ "$arg" == "--upload" ]; then
        UPLOAD=true
    elif [ "$arg" == "-h" ] || [ "$arg" == "--help" ]; then
        print_help_message
        exit
    fi
done

TOOLCHAIN_PATH="/opt/gcc-arm-none-eabi-9-2019-q4-major"
GPP="$TOOLCHAIN_PATH/bin/arm-none-eabi-g++"
GCC="$TOOLCHAIN_PATH/bin/arm-none-eabi-gcc"
LD="$TOOLCHAIN_PATH/bin/arm-none-eabi-ld"
OBJCOPY="$TOOLCHAIN_PATH/bin/arm-none-eabi-objcopy"

C_FLAGS="-O3 -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -D__SAME70J21__"
CPP_FLAGS="-O3 -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 --std=c++2a -D__SAME70J21__"

LINKER_SCRIPT="external/same70/same70j21_flash.ld"

C_FILES=(
    "external/same70/startup_same70j21.c"
    "external/same70/system_same70j21.c"
    "src/drivers/fatfs/diskio.c"
    "src/drivers/fatfs/ff.c"
    "src/drivers/fatfs/ffsystem.c"
    "src/drivers/fatfs/ffunicode.c"
    "src/utils/syscalls.c"
)

CPP_FILES=(
    "external/same70/init.cpp"
    "src/drivers/buzzer/buzzer.cpp"
    "src/drivers/clock/sysclk.cpp"
    "src/drivers/delay/delay.cpp"
    "src/drivers/fatfs/internal_io.cpp"
    "src/drivers/flash/flash.cpp"
    "src/drivers/hsmci/hsmci.cpp"
    "src/drivers/irq/irq.cpp"
    "src/drivers/led/led.cpp"
    "src/drivers/pio/pio.cpp"
    "src/drivers/pio/pio_handler.cpp"
    "src/drivers/pmc/pmc.cpp"
    "src/drivers/pwm/pwm.cpp"
    "src/drivers/sdcard/sdcard.cpp"
    "src/drivers/spi/spi.cpp"
    "src/drivers/tc/tc.cpp"
    "src/drivers/twihs/twihs.cpp"
    "src/drivers/xdmac/xdmac.cpp"
    "src/utils/timer.cpp"
    "src/main.cpp"
)

C_INCLUDES=(
    "-Iexternal/same70/include"
    "-Iexternal/CMSIS/include"
)

CPP_INCLUDES=(
    "-Iexternal/same70/include"
    "-Iexternal/CMSIS/include"
    "-Isrc"
)

# If build directory exists erase its content, create it otherwise.
if [[ -d "build" ]]; then
    rm -rf build/*
else
    mkdir build
fi

# Compile C files and pack them into a single drivers.o file
$GCC -s -c -Wl,--entry=Reset_Handler ${C_FLAGS} ${C_FILES[@]} ${C_INCLUDES[@]} --specs=nosys.specs
for file in ${C_FILES[@]}; do
    file=(${file//// })
    file=${file[-1]}
    objects="${objects}${file:0:-1}o "
done
$LD -s --entry=Reset_Handler -r ${objects} -o build/drivers.o
rm ${objects}

$GPP -s -Wl,--entry=Reset_Handler ${CPP_FLAGS} build/drivers.o ${CPP_FILES[@]} ${CPP_INCLUDES[@]} --specs=nosys.specs -T ${LINKER_SCRIPT} -o build/code.elf
$OBJCOPY -O binary build/code.elf build/code.bin

if [ "$UPLOAD" == true ]; then
    ../sam-ba/sam-ba -x same70.qml
fi