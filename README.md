# stm32_servo_control

Use the included files to help you recreate the setup in STM32 CUBE IDE.
- Make sure to set up PA0 and TIM2 to match the settings in ioc_config.png
- Make sure to make System Core -> SYS match the settings in SYS_config.png
- Add the files servo_fxns.h and servo_fxns.c to your Core -> Src folder
- Replace your main.c with the included main.c, or configure the values based on the PNG files and add the code into main.c your self

Will Craychee's old code is included in the wills_code folder for future reference when we start implementing USB.
