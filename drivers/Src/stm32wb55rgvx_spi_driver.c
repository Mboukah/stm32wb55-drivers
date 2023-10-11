/*
 * stm32wb55rgvx_spi_driver.c
 *
 *  Created on: Oct 2, 2023
 *      Author: UF825MBO
 */

#include "stm32wb55rgvx_spi_driver.h"


/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
		}
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temreg = 0;

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. configure the device mode
	temreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPI_PinConfig.SPI_BUSConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		temreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_PinConfig.SPI_BUSConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		temreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_PinConfig.SPI_BUSConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		temreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY must be set
		temreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	temreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF or CRCL
	temreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. configure the CPOL
	temreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. configure the CPHA
	temreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA;

	temreg |= pSPIHandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temreg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) // & 1 << 1
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);
		//while(!SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG));

		//wait until TXE is set
		while(!(pSPIx->SR & (1 << 1)));

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);
			//while(!SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG));

			//wait until RXE is set
			//while(!(pSPIx->SR & (1 << 1)));
			while(!(pSPIx->SR & (1 << 0)));

			//2. check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
			{
				//16 bit DFF
				//1. load the data in to the DR
				*((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}

		}
}


/*
 * IRQ Conf and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1|=(1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1|=(1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
