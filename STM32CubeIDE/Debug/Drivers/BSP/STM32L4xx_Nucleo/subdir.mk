################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/maiv2/STM32CubeIDE/workspace_1.18.0/SensorDemo_BLE_l476rg-main/Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.o: C:/Users/maiv2/STM32CubeIDE/workspace_1.18.0/SensorDemo_BLE_l476rg-main/Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.c Drivers/BSP/STM32L4xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../../Inc -I../../Drivers/BSP/STM32L4xx_Nucleo -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../Middlewares/ST/BlueNRG-2/utils -I../../Middlewares/ST/BlueNRG-2/includes -I../Drivers/User -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32L4xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32L4xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.cyclo ./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.d ./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.o ./Drivers/BSP/STM32L4xx_Nucleo/stm32l4xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32L4xx_Nucleo

