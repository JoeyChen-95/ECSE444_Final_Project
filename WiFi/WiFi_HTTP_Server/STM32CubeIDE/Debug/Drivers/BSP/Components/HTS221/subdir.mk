################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Drivers/BSP/Components/hts221/hts221.c 

OBJS += \
./Drivers/BSP/Components/HTS221/hts221.o 

C_DEPS += \
./Drivers/BSP/Components/HTS221/hts221.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/HTS221/hts221.o: C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Drivers/BSP/Components/hts221/hts221.c Drivers/BSP/Components/HTS221/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-HTS221

clean-Drivers-2f-BSP-2f-Components-2f-HTS221:
	-$(RM) ./Drivers/BSP/Components/HTS221/hts221.d ./Drivers/BSP/Components/HTS221/hts221.o ./Drivers/BSP/Components/HTS221/hts221.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-HTS221

