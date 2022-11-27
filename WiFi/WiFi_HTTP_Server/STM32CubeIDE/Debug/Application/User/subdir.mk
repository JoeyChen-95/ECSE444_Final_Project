################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/flash_l4.c \
C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/main.c \
../Application/User/stm32l4xx_hal_adc.c \
../Application/User/stm32l4xx_hal_adc_ex.c \
../Application/User/stm32l4xx_hal_msp.c \
C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/stm32l4xx_it.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c \
C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/system_stm32l4xx.c 

OBJS += \
./Application/User/flash_l4.o \
./Application/User/main.o \
./Application/User/stm32l4xx_hal_adc.o \
./Application/User/stm32l4xx_hal_adc_ex.o \
./Application/User/stm32l4xx_hal_msp.o \
./Application/User/stm32l4xx_it.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o \
./Application/User/system_stm32l4xx.o 

C_DEPS += \
./Application/User/flash_l4.d \
./Application/User/main.d \
./Application/User/stm32l4xx_hal_adc.d \
./Application/User/stm32l4xx_hal_adc_ex.d \
./Application/User/stm32l4xx_hal_msp.d \
./Application/User/stm32l4xx_it.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d \
./Application/User/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/flash_l4.o: C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/flash_l4.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32l4xx_it.o: C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/stm32l4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/system_stm32l4xx.o: C:/Users/Mr.Chimpanzee/STM32Cube/Repository/STM32Cube_FW_L4_V1.17.2/Projects/B-L4S5I-IOT01A/Applications/WiFi/WiFi_HTTP_Server/Src/system_stm32l4xx.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_STM32L4S5I_IOT01 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../../../../../../../Drivers/BSP/B-L4S5I-IOT01 -I../../../Common/Inc -I../../../../../../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../../../../../../Drivers/BSP/Components/Common -I../../../../../../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/flash_l4.d ./Application/User/flash_l4.o ./Application/User/flash_l4.su ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/stm32l4xx_hal_adc.d ./Application/User/stm32l4xx_hal_adc.o ./Application/User/stm32l4xx_hal_adc.su ./Application/User/stm32l4xx_hal_adc_ex.d ./Application/User/stm32l4xx_hal_adc_ex.o ./Application/User/stm32l4xx_hal_adc_ex.su ./Application/User/stm32l4xx_hal_msp.d ./Application/User/stm32l4xx_hal_msp.o ./Application/User/stm32l4xx_hal_msp.su ./Application/User/stm32l4xx_it.d ./Application/User/stm32l4xx_it.o ./Application/User/stm32l4xx_it.su ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su ./Application/User/system_stm32l4xx.d ./Application/User/system_stm32l4xx.o ./Application/User/system_stm32l4xx.su

.PHONY: clean-Application-2f-User

