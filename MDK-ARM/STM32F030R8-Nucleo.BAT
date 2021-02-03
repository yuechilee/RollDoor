SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Windows\System32\OpenSSH\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\NVIDIA Corporation\NVIDIA NvDLISR;C:\Cadence\SPB_16.6\tools\pcb\bin;C:\Cadence\SPB_16.6\tools\bin;C:\Cadence\SPB_16.6\tools\libutil\bin;C:\Cadence\SPB_16.6\tools\fet\bin;C:\Cadence\SPB_16.6\tools\specctra\bin;C:\Cadence\SPB_16.6\tools\pspice;C:\Cadence\SPB_16.6\tools\capture;C:\Cadence\SPB_16.6\openaccess\bin\win32\opt;C:\Users\kid05\AppData\Local\Microsoft\WindowsApps
SET CPU_TYPE=STM32F030R8
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32F030R8-Nucleo
SET CPU_CLOCK=0x007A1200
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\output\startup_stm32f030x8._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_nucleo.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\system_stm32f0xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_adc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_uart.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_i2c.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\output\stm32f0xx_hal_i2c_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\Output\H001.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\Output\H001.axf" --i32combined --output "..\Output\H001.hex"
