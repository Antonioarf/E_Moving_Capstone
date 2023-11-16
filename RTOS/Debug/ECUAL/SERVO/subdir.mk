################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/SERVO/DWT_Delay.c \
../ECUAL/SERVO/SERVO.c \
../ECUAL/SERVO/SERVO_cfg.c 

C_DEPS += \
./ECUAL/SERVO/DWT_Delay.d \
./ECUAL/SERVO/SERVO.d \
./ECUAL/SERVO/SERVO_cfg.d 

OBJS += \
./ECUAL/SERVO/DWT_Delay.o \
./ECUAL/SERVO/SERVO.o \
./ECUAL/SERVO/SERVO_cfg.o 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/SERVO/%.o ECUAL/SERVO/%.su ECUAL/SERVO/%.cyclo: ../ECUAL/SERVO/%.c ECUAL/SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F302x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-SERVO

clean-ECUAL-2f-SERVO:
	-$(RM) ./ECUAL/SERVO/DWT_Delay.cyclo ./ECUAL/SERVO/DWT_Delay.d ./ECUAL/SERVO/DWT_Delay.o ./ECUAL/SERVO/DWT_Delay.su ./ECUAL/SERVO/SERVO.cyclo ./ECUAL/SERVO/SERVO.d ./ECUAL/SERVO/SERVO.o ./ECUAL/SERVO/SERVO.su ./ECUAL/SERVO/SERVO_cfg.cyclo ./ECUAL/SERVO/SERVO_cfg.d ./ECUAL/SERVO/SERVO_cfg.o ./ECUAL/SERVO/SERVO_cfg.su

.PHONY: clean-ECUAL-2f-SERVO

