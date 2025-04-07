################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/User/hts221_reg.c \
../Drivers/User/lps22hh_reg.c 

OBJS += \
./Drivers/User/hts221_reg.o \
./Drivers/User/lps22hh_reg.o 

C_DEPS += \
./Drivers/User/hts221_reg.d \
./Drivers/User/lps22hh_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/User/%.o Drivers/User/%.su Drivers/User/%.cyclo: ../Drivers/User/%.c Drivers/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../../Inc -I../../Drivers/BSP/STM32L4xx_Nucleo -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../../Middlewares/ST/BlueNRG-2/utils -I../../Middlewares/ST/BlueNRG-2/includes -I../Drivers/User -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-User

clean-Drivers-2f-User:
	-$(RM) ./Drivers/User/hts221_reg.cyclo ./Drivers/User/hts221_reg.d ./Drivers/User/hts221_reg.o ./Drivers/User/hts221_reg.su ./Drivers/User/lps22hh_reg.cyclo ./Drivers/User/lps22hh_reg.d ./Drivers/User/lps22hh_reg.o ./Drivers/User/lps22hh_reg.su

.PHONY: clean-Drivers-2f-User

