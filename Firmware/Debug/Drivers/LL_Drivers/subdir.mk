################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/LL_Drivers/stm32l4xx_ll_spi.c 

OBJS += \
./Drivers/LL_Drivers/stm32l4xx_ll_spi.o 

C_DEPS += \
./Drivers/LL_Drivers/stm32l4xx_ll_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/LL_Drivers/%.o Drivers/LL_Drivers/%.su: ../Drivers/LL_Drivers/%.c Drivers/LL_Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L476xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/BNO08x -I../Drivers/AKxx-x -I../Drivers/MPU-925x -I../Drivers/LL_Drivers -I../Drivers/BNO08x/SH2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-LL_Drivers

clean-Drivers-2f-LL_Drivers:
	-$(RM) ./Drivers/LL_Drivers/stm32l4xx_ll_spi.d ./Drivers/LL_Drivers/stm32l4xx_ll_spi.o ./Drivers/LL_Drivers/stm32l4xx_ll_spi.su

.PHONY: clean-Drivers-2f-LL_Drivers

