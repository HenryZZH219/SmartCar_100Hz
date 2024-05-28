################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AppSw/Tricore/Driver/LQ_ADC.c \
../src/AppSw/Tricore/Driver/LQ_CCU6.c \
../src/AppSw/Tricore/Driver/LQ_DMA.c \
../src/AppSw/Tricore/Driver/LQ_EEPROM.c \
../src/AppSw/Tricore/Driver/LQ_EMEM.c \
../src/AppSw/Tricore/Driver/LQ_FFT.c \
../src/AppSw/Tricore/Driver/LQ_GPIO.c \
../src/AppSw/Tricore/Driver/LQ_GPSR.c \
../src/AppSw/Tricore/Driver/LQ_GPT12_ENC.c \
../src/AppSw/Tricore/Driver/LQ_GTM.c \
../src/AppSw/Tricore/Driver/LQ_QSPI.c \
../src/AppSw/Tricore/Driver/LQ_SOFTI2C.c \
../src/AppSw/Tricore/Driver/LQ_SPI.c \
../src/AppSw/Tricore/Driver/LQ_STM.c \
../src/AppSw/Tricore/Driver/LQ_UART.c 

OBJS += \
./src/AppSw/Tricore/Driver/LQ_ADC.o \
./src/AppSw/Tricore/Driver/LQ_CCU6.o \
./src/AppSw/Tricore/Driver/LQ_DMA.o \
./src/AppSw/Tricore/Driver/LQ_EEPROM.o \
./src/AppSw/Tricore/Driver/LQ_EMEM.o \
./src/AppSw/Tricore/Driver/LQ_FFT.o \
./src/AppSw/Tricore/Driver/LQ_GPIO.o \
./src/AppSw/Tricore/Driver/LQ_GPSR.o \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.o \
./src/AppSw/Tricore/Driver/LQ_GTM.o \
./src/AppSw/Tricore/Driver/LQ_QSPI.o \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.o \
./src/AppSw/Tricore/Driver/LQ_SPI.o \
./src/AppSw/Tricore/Driver/LQ_STM.o \
./src/AppSw/Tricore/Driver/LQ_UART.o 

COMPILED_SRCS += \
./src/AppSw/Tricore/Driver/LQ_ADC.src \
./src/AppSw/Tricore/Driver/LQ_CCU6.src \
./src/AppSw/Tricore/Driver/LQ_DMA.src \
./src/AppSw/Tricore/Driver/LQ_EEPROM.src \
./src/AppSw/Tricore/Driver/LQ_EMEM.src \
./src/AppSw/Tricore/Driver/LQ_FFT.src \
./src/AppSw/Tricore/Driver/LQ_GPIO.src \
./src/AppSw/Tricore/Driver/LQ_GPSR.src \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.src \
./src/AppSw/Tricore/Driver/LQ_GTM.src \
./src/AppSw/Tricore/Driver/LQ_QSPI.src \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.src \
./src/AppSw/Tricore/Driver/LQ_SPI.src \
./src/AppSw/Tricore/Driver/LQ_STM.src \
./src/AppSw/Tricore/Driver/LQ_UART.src 

C_DEPS += \
./src/AppSw/Tricore/Driver/LQ_ADC.d \
./src/AppSw/Tricore/Driver/LQ_CCU6.d \
./src/AppSw/Tricore/Driver/LQ_DMA.d \
./src/AppSw/Tricore/Driver/LQ_EEPROM.d \
./src/AppSw/Tricore/Driver/LQ_EMEM.d \
./src/AppSw/Tricore/Driver/LQ_FFT.d \
./src/AppSw/Tricore/Driver/LQ_GPIO.d \
./src/AppSw/Tricore/Driver/LQ_GPSR.d \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.d \
./src/AppSw/Tricore/Driver/LQ_GTM.d \
./src/AppSw/Tricore/Driver/LQ_QSPI.d \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.d \
./src/AppSw/Tricore/Driver/LQ_SPI.d \
./src/AppSw/Tricore/Driver/LQ_STM.d \
./src/AppSw/Tricore/Driver/LQ_UART.d 


# Each subdirectory must supply rules for building sources it contributes
src/AppSw/Tricore/Driver/%.src: ../src/AppSw/Tricore/Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gpt12" -I"D:/AC_DESKTOP/ver600_2/src/AppSw/Tricore/Driver" -I"D:/AC_DESKTOP/ver600_2/src/AppSw/Tricore/Main" -I"D:/AC_DESKTOP/ver600_2/src/AppSw/Tricore/User" -I"D:/AC_DESKTOP/ver600_2/src/AppSw/Tricore/APP" -I"D:/AC_DESKTOP/ver600_2/src/AppSw" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Platform/Tricore/Compilers" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Multican/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Platform" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cif/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Hssl/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cpu/Trap" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/If/Ccu6If" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dsadc/Dsadc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Port" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Stm/Timer" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dts/Dts" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eth" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Flash" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Vadc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Msc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Qspi/SpiMaster" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Scu/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe/Comm" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe/Math" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Platform/Tricore" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Trig" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tim" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/TimerWithTrigger" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Emem" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Mtu" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fft" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/I2c/I2c" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Asclin/Asc" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Flash/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/If" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cpu" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fce/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Stm/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Msc/Msc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Vadc/Adc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Asclin" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Pwm" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Atom" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Port/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5/Psi5" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eray" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Qspi/SpiSlave" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/Icu" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cpu/CStart" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Hssl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cif" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eth/Phy_Pef7071" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Hssl/Hssl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Iom/Driver" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Multican/Can" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5s/Psi5s" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fft/Fft" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmHl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Iom/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/_Lib" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Timer" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Sent" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eray/Eray" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gpt12/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dma" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fce/Crc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Qspi" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Sfr" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Sfr/TC26B" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe/Bsp" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe/General" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cpu/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dts" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Src" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dma/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cif/Cam" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Src/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Asclin/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/I2c/Std" -I"D:/AC_DESKTOP/ver600_2/Configurations" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/_Lib/DataHandling" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Sent/Sent" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/Timer" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5s" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Emem/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmBc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Iom" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/TPwm" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Multican" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Mtu/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Infra/Sfr/TC26B/_Reg" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/PwmHl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dma/Dma" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Timer" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/SysSe/Time" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dsadc/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Cpu/Irq" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Ccu6" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gpt12/IncrEnc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Psi5s/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Scu" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/_Lib/InternalMux" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Stm" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dsadc/Rdc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Vadc/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dts/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eth/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Smu" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/_PinMap" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Asclin/Lin" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/StdIf" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Dsadc" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fce" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/PwmHl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Qspi/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tom" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Tim/In" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Msc/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Fft/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Pwm" -I"D:/AC_DESKTOP/ver600_2/Libraries/Service/CpuGeneric/_Utilities" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Gtm/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Smu/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/I2c" -I"D:/AC_DESKTOP/ver600_2/Libraries" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Asclin/Spi" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Eray/Std" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Port/Io" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/_Impl" -I"D:/AC_DESKTOP/ver600_2/Libraries/iLLD/TC26B/Tricore/Sent/Std" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\LQ_TC26xB_LIB_ADS\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

src/AppSw/Tricore/Driver/%.o: ./src/AppSw/Tricore/Driver/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


