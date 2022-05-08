################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/fatfs/diskio.c \
../Core/fatfs/ff.c 

OBJS += \
./Core/fatfs/diskio.o \
./Core/fatfs/ff.o 

C_DEPS += \
./Core/fatfs/diskio.d \
./Core/fatfs/ff.d 


# Each subdirectory must supply rules for building sources it contributes
Core/fatfs/diskio.o: ../Core/fatfs/diskio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/engin/Documents/GitHub/Kies_HMI/sw/HMI_Core/Core/fatfs" -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/fatfs/diskio.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/fatfs/ff.o: ../Core/fatfs/ff.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"C:/Users/engin/Documents/GitHub/Kies_HMI/sw/HMI_Core/Core/fatfs" -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/fatfs/ff.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

