################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/iLLD/TC26B/Tricore/Fft/Fft/IfxFft_Fft.c 

OBJS += \
./Libraries/iLLD/TC26B/Tricore/Fft/Fft/IfxFft_Fft.o 

COMPILED_SRCS += \
./Libraries/iLLD/TC26B/Tricore/Fft/Fft/IfxFft_Fft.src 

C_DEPS += \
./Libraries/iLLD/TC26B/Tricore/Fft/Fft/IfxFft_Fft.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/iLLD/TC26B/Tricore/Fft/Fft/%.src: ../Libraries/iLLD/TC26B/Tricore/Fft/Fft/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gpt12" -I"D:/AC_DESKTOP/100hz/src/AppSw/Tricore/Driver" -I"D:/AC_DESKTOP/100hz/src/AppSw/Tricore/Main" -I"D:/AC_DESKTOP/100hz/src/AppSw/Tricore/User" -I"D:/AC_DESKTOP/100hz/src/AppSw/Tricore/APP" -I"D:/AC_DESKTOP/100hz/src/AppSw" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Platform/Tricore/Compilers" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Multican/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Platform" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cif/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Hssl/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cpu/Trap" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/If/Ccu6If" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dsadc/Dsadc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Port" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Stm/Timer" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dts/Dts" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eth" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Flash" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Vadc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Msc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Qspi/SpiMaster" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Scu/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe/Comm" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe/Math" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Platform/Tricore" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Trig" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tim" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/TimerWithTrigger" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Emem" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Mtu" -I"D:/AC_DESKTOP/100hz/Libraries/Infra" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fft" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/I2c/I2c" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Asclin/Asc" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Flash/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/If" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cpu" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fce/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Stm/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Msc/Msc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Vadc/Adc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Asclin" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Pwm" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Atom" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Port/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5/Psi5" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eray" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Qspi/SpiSlave" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/Icu" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cpu/CStart" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Hssl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cif" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eth/Phy_Pef7071" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Hssl/Hssl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Iom/Driver" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Multican/Can" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5s/Psi5s" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fft/Fft" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmHl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Iom/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/_Lib" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Timer" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Sent" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eray/Eray" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gpt12/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dma" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fce/Crc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Qspi" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Sfr" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Sfr/TC26B" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe/Bsp" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe/General" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cpu/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dts" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Src" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dma/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cif/Cam" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Src/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Asclin/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/I2c/Std" -I"D:/AC_DESKTOP/100hz/Configurations" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/_Lib/DataHandling" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Sent/Sent" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/Timer" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5s" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Emem/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmBc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Iom" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/TPwm" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Multican" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Mtu/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Infra/Sfr/TC26B/_Reg" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/PwmHl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dma/Dma" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Timer" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/SysSe/Time" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dsadc/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Cpu/Irq" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Ccu6" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gpt12/IncrEnc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Psi5s/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Scu" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/_Lib/InternalMux" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Stm" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dsadc/Rdc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Vadc/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dts/Std" -I"D:/AC_DESKTOP/100hz/Libraries/Service" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eth/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Smu" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/_PinMap" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Asclin/Lin" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/StdIf" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Dsadc" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fce" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/PwmHl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Qspi/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tom" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Tim/In" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Msc/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Fft/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Pwm" -I"D:/AC_DESKTOP/100hz/Libraries/Service/CpuGeneric/_Utilities" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Gtm/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Smu/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/I2c" -I"D:/AC_DESKTOP/100hz/Libraries" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Asclin/Spi" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Eray/Std" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Port/Io" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/_Impl" -I"D:/AC_DESKTOP/100hz/Libraries/iLLD/TC26B/Tricore/Sent/Std" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\LQ_TC26xB_LIB_ADS\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

Libraries/iLLD/TC26B/Tricore/Fft/Fft/%.o: ./Libraries/iLLD/TC26B/Tricore/Fft/Fft/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


