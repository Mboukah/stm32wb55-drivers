/*
 * stm32wb55rgvx_gpio_driver.c
 *
 *  Created on: Sep 25, 2023
 *      Author: UF825MBO
 */

#include "stm32wb55rgvx_gpio_driver.h"
#include "stm32wb55rgvx.h"

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1 . mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// clear the corresponding RTSR bit
			EXTI->FTSR1 &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1. configure the FTSR and the RTSR
			EXTI->FTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// clear the corresponding RTSR bit
			EXTI->RTSR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR1 |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//2 . configure the speed
	temp=0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3 . configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4 . configure the output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5 . configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Conf and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{		// ISER enable interrupt
		if(IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//ISER1
			*NVIC_ISER1 |= ( 1 << (IRQNumber%32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//ISER2
			*NVIC_ISER2 |= ( 1 << (IRQNumber%64) );
		}
	}else    // ICER disable the interrupt ISER no effect on disabling interrupt
	{
		if(IRQNumber <= 31)
		{
			//ICER0
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//ICER1
			*NVIC_ICER1 |= ( 1 << (IRQNumber%32) );
		}else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//ICER2
			*NVIC_ICER2 |= ( 1 << (IRQNumber%64) );
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber %4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx )) |= ( IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR1 & (1<<PinNumber))
	{
		EXTI->PR1 |= (1<<PinNumber);
	}

}
