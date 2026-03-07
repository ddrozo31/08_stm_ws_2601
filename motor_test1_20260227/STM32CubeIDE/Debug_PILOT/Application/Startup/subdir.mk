################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/Startup/startup_stm32g431cbux.s 

OBJS += \
./Application/Startup/startup_stm32g431cbux.o 

S_DEPS += \
./Application/Startup/startup_stm32g431cbux.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Startup/%.o: ../Application/Startup/%.s Application/Startup/subdir.mk
	$(error unable to generate command line)

clean: clean-Application-2f-Startup

clean-Application-2f-Startup:
	-$(RM) ./Application/Startup/startup_stm32g431cbux.d ./Application/Startup/startup_stm32g431cbux.o

.PHONY: clean-Application-2f-Startup

