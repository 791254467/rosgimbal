#include "drv_pwm_out.h"

PWM_OUT::PWM_OUT(){}

void PWM_OUT::init(const pwm_hardware_struct_t* pwm_init, uint16_t frequency, uint32_t max_us, uint32_t min_us) {
	GPIO_InitTypeDef 		gpio_init_struct;
	TIM_TimeBaseInitTypeDef tim_init_struct;
	TIM_OCInitTypeDef 		tim_oc_init_struct;

	port_ = pwm_init->gpio;
	pin_  = pwm_init->gpio_pin;

	GPIO_PinAFConfig(port_, pwm_init->gpio_pin_source, pwm_init->tim_af_config);

	gpio_init_struct.GPIO_Pin 	= pin_;
	gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;
	gpio_init_struct.GPIO_Speed	= GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_OType = GPIO_OType_PP;
	gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init(port_, &gpio_init_struct);

	//disable();

	uint32_t timer_prescaler = 42; //This is dependent on how fast the SystemCoreClock is. Not sure what to set to...
	uint32_t timer_freq_hz = SystemCoreClock / timer_prescaler;

	uint16_t cycles_per_us = timer_freq_hz / 1000000;//E^6
	max_cyc_ = max_us * cycles_per_us;
	min_cyc_ = min_us * cycles_per_us;

	uint16_t period_cyc = timer_freq_hz / frequency;

	TIM_TimeBaseStructInit(&tim_init_struct);
	tim_init_struct.TIM_Period 		  = period_cyc - 1; // 0 indexed
	tim_init_struct.TIM_Prescaler 	  = timer_prescaler - 1; //0 indexed
	tim_init_struct.TIM_ClockDivision = TIM_CKD_DIV1; //0x0000
	tim_init_struct.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(pwm_init->tim, &tim_init_struct);

	TIM_OCStructInit(&tim_oc_init_struct);
	tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM2;
	tim_oc_init_struct.TIM_OutputState 	= TIM_OutputState_Enable;
	tim_oc_init_struct.TIM_OutputNState = TIM_OutputNState_Disable;
	tim_oc_init_struct.TIM_Pulse 		= min_cyc_ - 1;
	tim_oc_init_struct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
	tim_oc_init_struct.TIM_OCIdleState 	= TIM_OCIdleState_Set;

	TIM_TypeDef* TIMPtr = pwm_init->tim;

  	switch (pwm_init->tim_channel) {
		case TIM_Channel_1:
			TIM_OC1Init(TIMPtr, &tim_oc_init_struct);
			TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
			CCR_ = &TIMPtr->CCR1;
			break;
		case TIM_Channel_2:
			TIM_OC2Init(TIMPtr, &tim_oc_init_struct);
			TIM_OC2PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
			CCR_ = &TIMPtr->CCR2;
			break;
		case TIM_Channel_3:
			TIM_OC3Init(TIMPtr, &tim_oc_init_struct);
			TIM_OC3PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
			CCR_ = &TIMPtr->CCR3;
			break;
		case TIM_Channel_4:
			TIM_OC4Init(TIMPtr, &tim_oc_init_struct);
			TIM_OC4PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
			CCR_ = &TIMPtr->CCR4;
			break;
	}

	TIM_Cmd(TIMPtr, ENABLE);
}

void PWM_OUT::enable() {
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF;
	gpio_init_struct.GPIO_Pin = pin_;

	GPIO_Init(port_, &gpio_init_struct);
}

void PWM_OUT::disable() {
	//This could conflict with the GPIO_PinAFConfig above
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init_struct.GPIO_Pin = pin_;

	GPIO_Init(port_, &gpio_init_struct);
	GPIO_ResetBits(port_, pin_);
}

void PWM_OUT::write(float value) {
	*CCR_ = min_cyc_ + (uint16_t)((max_cyc_ - min_cyc_) * value);
}