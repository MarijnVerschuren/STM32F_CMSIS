> # TODO
> * implement transmit irq
>
> * create function for other timer pins
> * add more options to fconfig_UART
> * add complementary support to PWM
> * move non PWM pindefs from pwm.h to the right header file
> * 
> * make it possible to disable the PLL and HSE
> * implement printf: https://gitlab.com/mcturra2000/rpi/-/blob/master/stm32f411re/cmsis/printf.c https://gitlab.com/mcturra2000/rpi/-/blob/master/stm32f411re/cmsis/printf.h
> * add config locks
> * use the fpu: https://gitlab.com/mcturra2000/rpi/-/blob/master/stm32f411re/cmsis/12-fpu/main.c
> * External counters? (tim.h)
> * 
> ## DOCS
> * what does BSRR do
> * what is the difference between speed modes
> ## VISUAL
> * TypeDef -> _t
> ## CAR
> * connection guard
> * drive back in self driving mode on disconnect