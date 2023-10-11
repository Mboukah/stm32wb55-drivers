/*
 * stm32wb55rgvx.h
 *
 *  Created on: Sep 22, 2023
 *      Author: UF825MBO
 */

#ifndef INC_STM32WB55RGVX_H_
#define INC_STM32WB55RGVX_H_

#include <stdint.h>

#define __vo volatile

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 *  ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0		( (__vo uint8_t*)0xE000E100)
#define NVIC_ISER1		( (__vo uint8_t*)0xE000E104)
#define NVIC_ISER2		( (__vo uint8_t*)0xE000E108)
#define NVIC_ISER3		( (__vo uint8_t*)0xE000E10C)

/*
 *  ARM Cortex Mx Processor priority register addresses
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

/*
 *  ARM Cortex Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0		( (__vo uint8_t*)0xE000E180)
#define NVIC_ICER1		( (__vo uint8_t*)0xE000E184)
#define NVIC_ICER2		( (__vo uint8_t*)0xE000E188)
#define NVIC_ICER3		( (__vo uint8_t*)0xE000E18C)

/*
 *  ARM Cortex Mx Processor NVIC IPRx register addresses
 */

#define NVIC_IPR0		( (__vo uint8_t*)0xE000E400)
#define NVIC_IPR1		( (__vo uint8_t*)0xE000E404)
#define NVIC_IPR2		( (__vo uint8_t*)0xE000E408)
#define NVIC_IPR3		( (__vo uint8_t*)0xE000E40C)


/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 			0x08000000U
#define SRAM1_BASEADDR 			0x20000000U
#define SRAM2A_BASEADDR			0x20030000U
#define SRAM2B_BASEADDR			0x20038000U
#define ROM						0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define APB3PERIPH_BASE			0x60000000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x48000000U
#define AHB3PERIPH_BASE			0x90000000U
#define AHB4PERIPH_BASE			0x58000000U

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */
#define GPIOA_BASEADDR			(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR			(AHB2PERIPH_BASE + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define EXTI_BASEADDR			(AHB4PERIPH_BASE + 0x0800)


/*
 * Base addresses of peripherals which are hanging on AHB4 bus
 */
#define RCC_BASEADDR			(AHB4PERIPH_BASE + 0x0000)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x3800)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE )  //offset 0x100


typedef struct
{
	__vo uint32_t MODER;  			//GPIO port mode register
	__vo uint32_t OTYPER;			//GPIO port output type register
	__vo uint32_t OSPEEDR;			//GPIO port output speed register
	__vo uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//GPIO port input data register
	__vo uint32_t ODR;				//GPIO port output data register
	__vo uint32_t BSRR;				//GPIO port bit set/reset register
	__vo uint32_t LCKR;				//GPIO port configuration lock register
	__vo uint32_t AFR[2];			//GPIO alternate function low register AFR[0] GPIO alternate function high register AFR[1]
	__vo uint32_t BRR;				//GPIO port bit reset register
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t ICSCR;
	__vo uint32_t CFGR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t PLLSAI1CFGR;
	__vo uint32_t RESERVED0;
	__vo uint32_t CIER;
	__vo uint32_t CIFR;
	__vo uint32_t CICR;
	__vo uint32_t SIMPSCR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED1;
	__vo uint32_t APB1RSTR1;
	__vo uint32_t APB1RSTR2;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB3RSTR;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR1;
	__vo uint32_t APB1ENR2;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3;
	__vo uint32_t AHB1SMENR;
	__vo uint32_t AHB2SMENR;
	__vo uint32_t AHB3SMENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1SMENR1;
	__vo uint32_t APB1SMENR2;
	__vo uint32_t APB2SMENR;
	__vo uint32_t RESERVED5;
	__vo uint32_t CCIPR;
	__vo uint32_t RESERVED6;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t CRRCR;
	__vo uint32_t HSECR;
	__vo uint32_t RESERVED7[26];
	__vo uint32_t EXTCFGR;
	__vo uint32_t RESERVED8[10];
	__vo uint32_t C2AHB1ENR;
	__vo uint32_t C2AHB2ENR;
	__vo uint32_t C2AHB3ENR;
	__vo uint32_t RESERVED9;
	__vo uint32_t C2APB1ENR1;
	__vo uint32_t C2APB1ENR2;
	__vo uint32_t C2APB2ENR;
	__vo uint32_t C2APB3ENR;
	__vo uint32_t C2AHB1SMENR;
	__vo uint32_t C2AHB2SMENR;
	__vo uint32_t C2AHB33MENR;
	__vo uint32_t RESERVED10;
	__vo uint32_t C2APB1SMENR1;
	__vo uint32_t C2APB1SMENR2;
	__vo uint32_t C2APB2SMENR;
	__vo uint32_t C2APB3SMENR;
	__vo uint32_t RESERVED11[91];
}RCC_RegDef_t;


typedef struct
{
	__vo uint32_t RTSR1;
	__vo uint32_t FTSR1;
	__vo uint32_t SWIER1;
	__vo uint32_t PR1;
	__vo uint32_t RESERVED0[4];
	__vo uint32_t RTSR2;
	__vo uint32_t FTSR2;
	__vo uint32_t SWIER2;
	__vo uint32_t PR2;
	__vo uint32_t RESERVED1[20];
	__vo uint32_t IMR1;
	__vo uint32_t EMR1;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t IMR2;
	__vo uint32_t EMR2;
	__vo uint32_t RESERVED3[10];
	__vo uint32_t C2IMR1;
	__vo uint32_t C2EMR1;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t C2IMR2;
	__vo uint32_t C2EMR2;
}EXTI_RegDef_t;


typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t SCSR;
	__vo uint32_t CFGR2;
	__vo uint32_t SWPR;
	__vo uint32_t SKR;
	__vo uint32_t SWPR2;
	__vo uint32_t IMR1;
	__vo uint32_t IMR2;
	__vo uint32_t C2IMR1;
	__vo uint32_t C2IMR2;
	__vo uint32_t SIPCR;
}SYSCFG_RegDef_t;

#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*
 *  Peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
}SPI_RegDef_t;

/*
 * 	Peripheral register definition structure for I2C
 */

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t TIMINGR;
	__vo uint32_t TIMEOUTR;
	__vo uint32_t ISR;
	__vo uint32_t ICR;
	__vo uint32_t PECR;
	__vo uint32_t RXDR;
	__vo uint32_t TXDR;

}I2C_RegDef_t;


/*
 *	I2C peripheral definition
 */

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)


/*
 * spi peripheral definition
 */

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)

/*
 * peripheral definitions => peripherals base address casted to xxxxRegDef_t
 */

#define GPIOA					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*) EXTI_BASEADDR)
/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			( RCC->AHB2ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()			( RCC->AHB2ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()			( RCC->AHB2ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB2ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB2ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()			( RCC->AHB2ENR |= (1 << 7) )


/*
 * macros for GPIOx reset peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7));}while(0)

/*
 * macros for SPIx reset peripherals
 */

#define SPI1_REG_RESET()		(RCC->APB2RSTR |= (1 << 12))
#define SPI2_REG_RESET()		(RCC->APB1RSTR1 |= (1 << 14))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			( RCC->APB1ENR1 |= (1 << 21) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR1 |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR1 |= (1 << 14) )

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock enable macros for SYSCFG
 */


/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 4) )
#define GPIOH_PCLK_DI()			( RCC->AHB2ENR &= ~(1 << 7) )

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 21) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 23) )

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR1 &= ~(1 << 14) )

/*
 * Clock disable macros for USARTx peripherals
 */
#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14) )

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


#define GPIO_BASEADDR_TO_CODE(x)			((x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:\
											(x == GPIOH)?7:0)

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15



/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2   //not for stm32wb55
#define SPI_SR_UDR					 	3   //not for stm32wb55
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */

#define I2C_CR1_PE						0

/*
 * Bit position definitions I2C_CR2
 */

#define I2C_CR2_START					13
#define I2C_CR2_STOP					14
#define I2C_CR2_NACK					15



#endif /* INC_STM32WB55RGVX_H_ */
