################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FlashPROM.c \
../Src/ds18b20.c \
../Src/dwt_stm32_delay.c \
../Src/fonts.c \
../Src/main.c \
../Src/ssd1306.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/FlashPROM.o \
./Src/ds18b20.o \
./Src/dwt_stm32_delay.o \
./Src/fonts.o \
./Src/main.o \
./Src/ssd1306.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/FlashPROM.d \
./Src/ds18b20.d \
./Src/dwt_stm32_delay.d \
./Src/fonts.d \
./Src/main.d \
./Src/ssd1306.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/workspace/Mi_termometrv3/Inc" -I"C:/workspace/Mi_termometrv3/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/workspace/Mi_termometrv3/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/workspace/Mi_termometrv3/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/workspace/Mi_termometrv3/Drivers/CMSIS/Include"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


