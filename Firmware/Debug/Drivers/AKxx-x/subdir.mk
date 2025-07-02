################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AKxx-x/akxx-x.c 

OBJS += \
./Drivers/AKxx-x/akxx-x.o 

C_DEPS += \
./Drivers/AKxx-x/akxx-x.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AKxx-x/%.o Drivers/AKxx-x/%.su: ../Drivers/AKxx-x/%.c Drivers/AKxx-x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/SH2 -I../Drivers/BNO08x -I../Drivers/AKxx-x -I../Drivers/MPU-925x -I../Drivers/LL_Drivers -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-AKxx-2d-x

clean-Drivers-2f-AKxx-2d-x:
	-$(RM) ./Drivers/AKxx-x/akxx-x.d ./Drivers/AKxx-x/akxx-x.o ./Drivers/AKxx-x/akxx-x.su

.PHONY: clean-Drivers-2f-AKxx-2d-x

