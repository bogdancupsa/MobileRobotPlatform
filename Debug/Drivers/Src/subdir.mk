################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/gpio_drivers.c \
../Drivers/Src/pwm_driver.c \
../Drivers/Src/rcc_config.c 

OBJS += \
./Drivers/Src/gpio_drivers.o \
./Drivers/Src/pwm_driver.o \
./Drivers/Src/rcc_config.o 

C_DEPS += \
./Drivers/Src/gpio_drivers.d \
./Drivers/Src/pwm_driver.d \
./Drivers/Src/rcc_config.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"D:/STM32_Workspace/ObstacleAvoidingRobot/ASW/Inc" -I"D:/STM32_Workspace/ObstacleAvoidingRobot/BSW/Inc" -I"D:/STM32_Workspace/ObstacleAvoidingRobot/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/gpio_drivers.cyclo ./Drivers/Src/gpio_drivers.d ./Drivers/Src/gpio_drivers.o ./Drivers/Src/gpio_drivers.su ./Drivers/Src/pwm_driver.cyclo ./Drivers/Src/pwm_driver.d ./Drivers/Src/pwm_driver.o ./Drivers/Src/pwm_driver.su ./Drivers/Src/rcc_config.cyclo ./Drivers/Src/rcc_config.d ./Drivers/Src/rcc_config.o ./Drivers/Src/rcc_config.su

.PHONY: clean-Drivers-2f-Src

