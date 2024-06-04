################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ASW/Src/DrivingAPIs.c \
../ASW/Src/MotionController.c \
../ASW/Src/SensorMapping.c 

OBJS += \
./ASW/Src/DrivingAPIs.o \
./ASW/Src/MotionController.o \
./ASW/Src/SensorMapping.o 

C_DEPS += \
./ASW/Src/DrivingAPIs.d \
./ASW/Src/MotionController.d \
./ASW/Src/SensorMapping.d 


# Each subdirectory must supply rules for building sources it contributes
ASW/Src/%.o ASW/Src/%.su ASW/Src/%.cyclo: ../ASW/Src/%.c ASW/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"D:/STM32_Workspace/GitHub/MobileRobotPlatform/Test_Application/BSW/Inc" -I"D:/STM32_Workspace/GitHub/MobileRobotPlatform/Test_Application/ASW/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ASW-2f-Src

clean-ASW-2f-Src:
	-$(RM) ./ASW/Src/DrivingAPIs.cyclo ./ASW/Src/DrivingAPIs.d ./ASW/Src/DrivingAPIs.o ./ASW/Src/DrivingAPIs.su ./ASW/Src/MotionController.cyclo ./ASW/Src/MotionController.d ./ASW/Src/MotionController.o ./ASW/Src/MotionController.su ./ASW/Src/SensorMapping.cyclo ./ASW/Src/SensorMapping.d ./ASW/Src/SensorMapping.o ./ASW/Src/SensorMapping.su

.PHONY: clean-ASW-2f-Src

