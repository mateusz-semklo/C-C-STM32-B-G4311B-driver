################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.c \
../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.o \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.d \
./Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_common_tables.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_const_structs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_cos_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_init_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_pid_reset_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_cos_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sin_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.o: ../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include/arm_sqrt_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

