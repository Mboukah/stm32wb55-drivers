/*
 * 005spi_rx_slave.c
 *
 *  Created on: Oct 6, 2023
 *      Author: UF825MBO
 */



#include "stm32wb55rgvx.h"
#include <string.h>
#include "stm32wb55rgvx_gpio_driver.h"
#include "stm32wb55rgvx_spi_driver.h"


void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits()
{
	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPI_PinConfig.SPI_BUSConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPI1handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI1handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI;  //software slave management enabled for NSS pin

	SPI_Init(&SPI1handle);

}

/*
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK PB13
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI PB15
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	//GPIO_Init(&SPIPins);
}

void SPI2_Inits()
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_PinConfig.SPI_BUSConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN;  //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

*/

void delay()
{
	for (uint32_t i = 0 ; i < 500000/4 ; i++);
}

int main ()
{
	uint8_t data_received=0;
	char send_data[] = "OK";
	uint8_t user_data = 0x55;
	//this function initialise the gpio pins to behave as spi pins
	SPI1_GPIOInits();
	//this function initialise the spi peripheral parameters
	SPI1_Inits();
	//SPI_SSOEConfig(SPI1, ENABLE);
	//SPI_SSIConfig(SPI1,ENABLE);
	SPI_PeripheralControl(SPI1, ENABLE);
	while(1)
	{
		//SPI_SendData(SPI1,&user_data,1);
			SPI_ReceiveData(SPI1, &data_received, 1);
			//if(data_received == 0xAA)
			//{
				//delay();
				SPI_SendData(SPI1,(uint8_t*)send_data,strlen(send_data));
				SPI_SendData(SPI1,&user_data,1);
			//}

			//SPI_PeripheralControl(SPI1, DISABLE);
	}
	return 0;
}
