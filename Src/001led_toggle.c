/*
 * 001led_toggle.c
 *
 *  Created on: Sep 26, 2023
 *      Author: UF825MBO
 */
#include <stdio.h>
#include "stm32wb55rgvx.h"
#include "stm32wb55rgvx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);
}

int main ()
{
	GPIO_Handle_t GpioLed, GpioPB;
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GpioPB.pGPIOx = GPIOD;
	GpioPB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioPB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioPB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioPB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;



	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioPB);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_0) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);
		}

	}
	return 0;
}


