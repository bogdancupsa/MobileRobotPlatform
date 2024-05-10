################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSW/Src/HBridge.c \
../BSW/Src/HCSR04.c \
../BSW/Src/ServoSG90.c 

OBJS += \
./BSW/Src/HBridge.o \
./BSW/Src/HCSR04.o \
./BSW/Src/ServoSG90.o 

C_DEPS += \
./BSW/Src/HBridge.d \
./BSW/Src/HCSR04.d \
./BSW/Src/ServoSG90.d 


# Each subdirectory must supply rules for building sources it contributes
BSW/Src/%.o BSW/Src/%.su BSW/Src/%.cyclo: ../BSW/Src/%.c BSW/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"D:/STM32_Workspace/ObstacleAvoidingRobot/ASW/Inc" -I"D:/STM32_Workspace/ObstacleAvoidingRobot/BSW/Inc" -I"D:/STM32_Workspace/ObstacleAvoidingRobot/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSW-2f-Src

clean-BSW-2f-Src:
	-$(RM) ./BSW/Src/HBridge.cyclo ./BSW/Src/HBridge.d ./BSW/Src/HBridge.o ./BSW/Src/HBridge.su ./BSW/Src/HCSR04.cyclo ./BSW/Src/HCSR04.d ./BSW/Src/HCSR04.o ./BSW/Src/HCSR04.su ./BSW/Src/ServoSG90.cyclo ./BSW/Src/ServoSG90.d ./BSW/Src/ServoSG90.o ./BSW/Src/ServoSG90.su

.PHONY: clean-BSW-2f-Src

