/*
 * 002led_toggle_interrupt.c
 *
 *  Created on: Sep 28, 2023
 *      Author: UF825MBO
 */

#include <stdio.h>
#include "stm32wb55rgvx.h"
#include "stm32wb55rgvx_gpio_driver.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);
}

int main ()
{
	GPIO_Handle_t GpioLed, GpioPB;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioPB,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioPB.pGPIOx = GPIOD;
	GpioPB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioPB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioPB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioPB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioPB);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0,ENABLE);

	while(1);


	return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_5);
}
