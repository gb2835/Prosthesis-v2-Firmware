################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/gpio.c \
../Core/Src/lptim.c \
../Core/Src/main.c \
../Core/Src/prosthesis_v2.c \
../Core/Src/spi.c \
../Core/Src/spi_hal.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/tim.c \
../Core/Src/utilities.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/gpio.o \
./Core/Src/lptim.o \
./Core/Src/main.o \
./Core/Src/prosthesis_v2.o \
./Core/Src/spi.o \
./Core/Src/spi_hal.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/tim.o \
./Core/Src/utilities.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/gpio.d \
./Core/Src/lptim.d \
./Core/Src/main.d \
./Core/Src/prosthesis_v2.d \
./Core/Src/spi.d \
./Core/Src/spi_hal.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/tim.d \
./Core/Src/utilities.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/SH2 -I../Drivers/BNO08x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/lptim.d ./Core/Src/lptim.o ./Core/Src/lptim.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/prosthesis_v2.d ./Core/Src/prosthesis_v2.o ./Core/Src/prosthesis_v2.su ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/spi_hal.d ./Core/Src/spi_hal.o ./Core/Src/spi_hal.su ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/utilities.d ./Core/Src/utilities.o ./Core/Src/utilities.su

.PHONY: clean-Core-2f-Src

