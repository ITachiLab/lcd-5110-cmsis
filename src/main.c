/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"

#define SCREEN_BUFFER_SIZE 504

uint8_t screen_buffer[SCREEN_BUFFER_SIZE];

void delay_ms(uint16_t delay) {
	TIM2->CNT = 0;
	TIM2->EGR |= TIM_EGR_UG;

	uint16_t clk_value = TIM2->CNT;
	while (TIM2->CNT < (clk_value + delay));
}

void lcd_send_cmd(uint8_t data) {
	GPIOB->BRR = GPIO_Pin_8 | GPIO_Pin_6;

	SPI1->DR = data;

	while (!(SPI1->SR & SPI_SR_TXE));
	while (SPI1->SR & SPI_SR_BSY);

	GPIOB->BSRR = GPIO_Pin_6;
}

int main(void)
{
	// Enable clocks for GPIOA, GPIOB and SPI1
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_SPI1EN);

	// Enable clocks for TIM2 and USART2
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_USART2EN);

	// Enable clock for DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Enable timer and set its frequency to: (64000000 / 64000) = 1000 Hz (1 ms timer tick)
	TIM2->PSC = 63999;
	TIM2->CR1 |= TIM_CR1_CEN;

	// DC (8), RST (9), SS (6) as 50 MHz general push-pull output
	GPIOB->CRL |= (GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1);
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1);

	GPIOB->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1);
	GPIOB->CRH &= ~(GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1);

	// Set high on RST line
	GPIOB->BSRR = GPIO_Pin_9;

	// CLK (5), MOSI (7) as 50 MHz alternate function push-pull output
	GPIOA->CRL |= (GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_0 | GPIO_CRL_CNF7_0);

	// SPI enabled, fclk / 16, master
	SPI1->CR1 = (SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_MSTR);
	SPI1->CR1 |= SPI_CR1_SPE;

	// Enable USART2, DMA receiver, 230400 BPS, enable receiver
	USART2->CR1 = USART_CR1_UE;
	USART2->CR3 = USART_CR3_DMAR;
	USART2->BRR = 0x8B; // Change to 0x3415 for 2400 BPS, so we can observe screen refreshing
	USART2->CR1 |= USART_CR1_RE;

	// Enable 6th DMA channel for USART2 RX
	DMA1_Channel6->CNDTR = SCREEN_BUFFER_SIZE;
	DMA1_Channel6->CPAR = (uint32_t) &USART2->DR;
	DMA1_Channel6->CMAR = (uint32_t) screen_buffer;
	DMA1_Channel6->CCR = (DMA_CCR6_PL_0 | DMA_CCR6_MINC | DMA_CCR6_CIRC | DMA_CCR6_EN | DMA_CCR6_TCIE);

	// Enable 3rd DMA channel for SPI TX
	DMA1_Channel3->CNDTR = SCREEN_BUFFER_SIZE;
	DMA1_Channel3->CPAR = (uint32_t) &SPI1->DR;
	DMA1_Channel3->CMAR = (uint32_t) screen_buffer;
	DMA1_Channel3->CCR = (DMA_CCR3_PL_1 | DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_EN | DMA_CCR3_DIR | DMA_CCR3_TCIE);

	// Reset LCD
	GPIOB->BRR = GPIO_Pin_9;
	delay_ms(1);
	GPIOB->BSRR = GPIO_Pin_9;

	// Switch LCD to command mode and select slave
	GPIOB->BRR = GPIO_Pin_8;
	GPIOB->BRR = GPIO_Pin_6;

	// Prepare LCD for displaying
	lcd_send_cmd(0x21);
	lcd_send_cmd(0x14);
	lcd_send_cmd(0x80 | 0x35);
	lcd_send_cmd(0x20);

	// This mode will display 0's as white and 1's as black.
	// Change to: 0b00001101, to achieve opposite effect.
	lcd_send_cmd(0b00001100);

	// Set RAM address to (0, 0)
	lcd_send_cmd(0b01000000);
	lcd_send_cmd(0b10000000);

	// Unselect slave
	GPIOB->BSRR = GPIO_Pin_6;

	// Disable SPI before changing settings
	SPI1->CR1 &= ~SPI_CR1_SPE;

	// Switch LCD to DATA mode and select slave
	GPIOB->BSRR = GPIO_Pin_8;
	GPIOB->BRR = GPIO_Pin_6;

	// Switch SPI to DMA mode and enable it
	SPI1->CR2 |= SPI_CR2_TXDMAEN;
	SPI1->CR1 |= SPI_CR1_SPE;

	while (1) {
		// Nothing to do here!
		// DMA reads and writes everything behind a scene.
	}
}
