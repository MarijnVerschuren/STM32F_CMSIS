//
// Created by marijn on 2/13/23.
//
#ifndef STM32F_TIM_H
#define STM32F_TIM_H
#include "main.h"
#include "base.h"


/* data layout:
 * struct:
 *		dev_id						: 16;  // MSB
 *			num							: 5;	// MSB
 *			clock						: 5;
 *			sub							: 6;	// LSB	// (reserved)
 *		alternate_function_number	: 4;
 *		port_number					: 4;
 *		pin_number					: 4;
 *		misc						: 4;  // LSB
 *			channel 					: 2;	// MSB
 *			reserved					: 2;	// LSB
 * functions:
 * 		TIMx_ETR -> external trigger (used in external counters)
 * 		TIMx_CHx -> channel output (used in pwm)
 * 		TIMx_CHxN -> channel inverse out
 * 		TIMx_BKIN -> ?
 * */
typedef enum {
	TIM_PIN_DISABLE =	0x00000000,
	// TIM1
	TIM1_BKIN_A6 =		0x60001060,	TIM1_CH1N_A7 =		0x60001070,
	TIM1_CH1_A8 =		0x60001080,	TIM1_CH2_A9 =		0x60001094,
	TIM1_CH3_A10 =		0x600010a8,	TIM1_CH4_A11 =		0x600010bc,
	TIM1_ETR_A12 =		0x600010c0,	TIM1_CH2N_B0 =		0x60001104,
	TIM1_CH3N_B1 =		0x60001118,	TIM1_BKIN_B12 =		0x600011c0,
	TIM1_CH1N_B13 =		0x600011d0,	TIM1_CH2N_B14 =		0x600011e4,
	TIM1_CH3N_B15 =		0x600011f8,	TIM1_ETR_E7 =		0x60001470,
	TIM1_CH1N_E8 =		0x60001480,	TIM1_CH1_E9 =		0x60001490,
	TIM1_CH2N_E10 =		0x600014a4,	TIM1_CH2_E11 =		0x600014b4,
	TIM1_CH3N_E12 =		0x600014c8,	TIM1_CH3_E13 =		0x600014d8,
	TIM1_CH4_E14 =		0x600014ec,	TIM1_BKIN_E15 =		0x600014f0,
	// TIM2
	TIM2_CH1_A0 =		0x40001000,	TIM2_ETR_A0 =		0x40001000,
	TIM2_CH2_A1 =		0x40001014,	TIM2_CH3_A2 =		0x40001028,
	TIM2_CH4_A3 =		0x4000103c,	TIM2_CH1_A5 =		0x40001050,
	TIM2_ETR_A5 =		0x40001050,	TIM2_CH1_A15 =		0x400010f0,
	TIM2_ETR_A15 =		0x400010f0,	TIM2_CH2N_B0 =		0x40001104,
	TIM2_CH3N_B1 =		0x40001118,	TIM2_CH2_B3 =		0x40001134,
	TIM2_CH3_B10 =		0x400011a8,	TIM2_CH4_B11 =		0x400011bc,
	// TIM3
	TIM3_CH1_A6 =		0x41002060,	TIM3_CH2_A7 =		0x41002074,
	TIM3_CH3_B0 =		0x41002108,	TIM3_CH4_B1 =		0x4100211c,
	TIM3_CH1_B4 =		0x41002140,	TIM3_CH2_B5 =		0x41002154,
	TIM3_CH1_C6 =		0x41002260,	TIM3_CH2_C7 =		0x41002274,
	TIM3_CH3_C8 =		0x41002288,	TIM3_CH4_C9 =		0x4100229c,
	TIM3_ETR_D2 =		0x41002320,
	// TIM4
	TIM4_CH1_B6 =		0x42002160,	TIM4_CH2_B7 =		0x42002174,
	TIM4_CH3_B8 =		0x42002188,	TIM4_CH4_B9 =		0x4200219c,
	TIM4_CH1_D12 =		0x420023c0,	TIM4_CH2_D13 =		0x420023d4,
	TIM4_CH3_D14 =		0x420023e8,	TIM4_CH4_D15 =		0x420023fc,
	TIM4_ETR_E0 =		0x42002400,
	// TIM5
	TIM5_CH1_A0 =		0x43002000,	TIM5_CH2_A1 =		0x43002014,
	TIM5_CH3_A2 =		0x43002028,	TIM5_CH4_A3 =		0x4300203c,
	// TIM9
	TIM9_CH1_A2 =		0x70003020,	TIM9_CH2_A3 =		0x70003034,
	TIM9_CH1_E5 =		0x70003450,	TIM9_CH2_E6 =		0x70003464,
	// TIM10
	TIM10_CH1_B8 =		0x71003180,
	// TIM11
	TIM11_CH1_B9 =		0x72003190
} TIM_GPIO_t;


/*!< init / enable / disable */
void config_TIM(TIM_TypeDef* tim, uint32_t prescaler, uint32_t limit);
void disable_TIM(TIM_TypeDef* tim);
/*!< actions */
void start_TIM(TIM_TypeDef* tim);
void stop_TIM(TIM_TypeDef* tim);
/*!< irq */
void start_TIM_update_irq(TIM_TypeDef* tim);
void stop_TIM_update_irq(TIM_TypeDef* tim);
void start_TIM_capture_compare_irq(TIM_TypeDef* tim);
void stop_TIM_capture_compare_irq(TIM_TypeDef* tim);
void start_TIM_break_irq(TIM_TypeDef* tim);
void stop_TIM_break_irq(TIM_TypeDef* tim);
void start_TIM_trigger_commutation_irq(TIM_TypeDef* tim);
void stop_TIM_trigger_commutation_irq(TIM_TypeDef* tim);


#endif //STM32F_TIM_H
