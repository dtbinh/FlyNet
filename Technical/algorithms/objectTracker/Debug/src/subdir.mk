################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/OBJECT.cpp \
../src/OBJECT_CONTAINER.cpp \
../src/TRACK.cpp \
../src/TRACKLIST.cpp \
../src/amm.cpp \
../src/trackerMain.cpp 

OBJS += \
./src/OBJECT.o \
./src/OBJECT_CONTAINER.o \
./src/TRACK.o \
./src/TRACKLIST.o \
./src/amm.o \
./src/trackerMain.o 

CPP_DEPS += \
./src/OBJECT.d \
./src/OBJECT_CONTAINER.d \
./src/TRACK.d \
./src/TRACKLIST.d \
./src/amm.d \
./src/trackerMain.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -I/usr/local/include/opencv2 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


