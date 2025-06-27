################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SH2/euler.c \
../Drivers/SH2/sh2.c \
../Drivers/SH2/sh2_SensorValue.c \
../Drivers/SH2/sh2_util.c \
../Drivers/SH2/shtp.c 

OBJS += \
./Drivers/SH2/euler.o \
./Drivers/SH2/sh2.o \
./Drivers/SH2/sh2_SensorValue.o \
./Drivers/SH2/sh2_util.o \
./Drivers/SH2/shtp.o 

C_DEPS += \
./Drivers/SH2/euler.d \
./Drivers/SH2/sh2.d \
./Drivers/SH2/sh2_SensorValue.d \
./Drivers/SH2/sh2_util.d \
./Drivers/SH2/shtp.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SH2/%.o Drivers/SH2/%.su: ../Drivers/SH2/%.c Drivers/SH2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/SH2 -I../Drivers/BNO08x -I../Drivers/AKxx-x -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SH2

clean-Drivers-2f-SH2:
	-$(RM) ./Drivers/SH2/euler.d ./Drivers/SH2/euler.o ./Drivers/SH2/euler.su ./Drivers/SH2/sh2.d ./Drivers/SH2/sh2.o ./Drivers/SH2/sh2.su ./Drivers/SH2/sh2_SensorValue.d ./Drivers/SH2/sh2_SensorValue.o ./Drivers/SH2/sh2_SensorValue.su ./Drivers/SH2/sh2_util.d ./Drivers/SH2/sh2_util.o ./Drivers/SH2/sh2_util.su ./Drivers/SH2/shtp.d ./Drivers/SH2/shtp.o ./Drivers/SH2/shtp.su

.PHONY: clean-Drivers-2f-SH2

