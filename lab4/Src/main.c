#include <stdint.h>
#include <stdio.h>
#include <stm32f446xx.h>
#include <math.h>

/* Входная частота контроллера*/
int inp_freq = 14000000;
int int_freq = 17000;

/* Значения коэффициентов PLL;вычисленные */
int PLLN = 160;
int PLLM = 7;
int PLLP = 2;

/* Рассчитываем main clock value*/
#define SysClk (inp_freq / PLLM * PLLN / PLLP ) //160000000
#define APB1 4
#define AHB1 1
#define HCLK (SysClk/AHB1) //160000000
#define SysTicks (HCLK/int_freq)

//#define PCLK1 (HCLK/APB1)
//#define PCLK2 (HCLK/APB2)

//оПРЕДЕЛЯЕМ КОЭФФИЦИЕНТЫ ДЛЯ ТАЙМЕРА
#define TIM_PSC 7999 //значение делителя
#define TIM_ARR 100 //прерывание 100 раз в секунду

/* Определяем частоту работы USART receiver and transmitter*/
#define APB1_FREQ (HCLK/APB1) // 40MHz
#define BAUDRATE 19200

double tx_data[2]; //transfer x data
double rx_data[4]; //Receiver x data
double timer = 0.0;
double result = 0.0;

//Создаем DMA1_Stream5 receiver Interrupt Handler
void DMA1_Stream5_IRQHandler(void) {
	//Задаем вводные переменные
	double bias = rx_data[0];
	double amp = rx_data[1];
	double freq = rx_data[2];
	double phase = rx_data[3];

	result = amp*sin(freq*timer+phase)+bias; //formula matlab

	DMA1_Stream5->NDTR = 32; //число данных для приема - number of data register
	DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;
	DMA1_Stream5->CR |= DMA_SxCR_EN; //Switch on Stream
}
//Создаем Timker Interrupt Handler
void TIM3_IRQHandler(void){
	TIM3->SR &= ~(TIM_SR_UIF); //ОБНУЛЯЕМ
	tx_data [0] = result; //а осцилограф
	rx_data [1] = timer;// на таймер
	timer +=0.01;
	if (timer >= 60000.0){
		timer = 0.0;
	}

	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6; // high interrupt flag clear register
		DMA1_Stream6->NDTR = 16; //число данных для передачи - number of data register
		DMA1_Stream6->CR |= DMA_SxCR_EN; //Switch on Stream
}

int main(void){
	SCB->CPACR |=((3UL << 10*2)|(3UL << 11*2));

	FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // Set flash latency (задержка для памяти)
	RCC->CR |= RCC_CR_HSEON;	 // Включаем hse


	//Запускаем GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//Задаем пины и функцию
	GPIOA->MODER &= ~(1<<4)|(1<<5);
	GPIOA->MODER |= (1<<5);
	GPIOA->MODER &= ~(1<<6)|(1<<7);
	GPIOA->MODER |= (1<<7);


	/* Значения коэффициентов Делителей; */

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;


	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; //Включаем PLL Source в режим Hse
	  while (!(RCC->CR & RCC_CR_HSERDY)); //Enable HSE. Wait until HSE Ready.

	/* Значения коэффициентов PLL */

	// Чистим регистр от заранее заданных значений
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk |  RCC_PLLCFGR_PLLP_Msk );

	// Передаем новые значение в регистр
	RCC->PLLCFGR |= PLLN << RCC_PLLCFGR_PLLN_Pos ;
	RCC->PLLCFGR |= PLLM << RCC_PLLCFGR_PLLM_Pos ;

	RCC->CR |= RCC_CR_PLLON ;  // Включаем pll
	  while (!(RCC->CR & RCC_CR_PLLRDY)); //Enable PLL. Wait until PLL Ready.

	RCC->CFGR |= RCC_CFGR_SW_PLL; //Включаем system clock  в режим PLL_p

	//Режим АФ7
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2 ;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2 ;

	//Включаем USART
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// Включаем DMA
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//Задаем частоту работы USART2
	USART2->BRR = APB1_FREQ/BAUDRATE;

	//обнулили
	USART2->CR1 = 0x00;

	//Настроить USART на прием и передачу данных, генерацию требуемых прерываний

	USART2->CR1 |= USART_CR1_TE; //Включаем transm
	USART2->CR1 |= USART_CR1_RE; //Включаем receiver
	USART2->CR1 |= USART_CR1_UE; //Включаем прерывания USART

	//Включить прерывания в контроллере прерываний на прием
	//USART2->CR1 |= USART_CR1_RXNEIE;

	//Включаем Timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM3->DIER |= TIM_DIER_UIE; //Update interrupt enable
	TIM3->PSC |= TIM_PSC; //prescaler
	TIM3->ARR |= TIM_ARR; //auto-reload register
	TIM3->CR1 |= TIM_CR1_CEN;//sWITCH ON

	//Включение контрольных регистров - receiver
	USART2->CR3 = USART_CR3_DMAR;

	// STREAM 5 - receiver
	DMA1_Stream5->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_CHSEL_2;
	DMA1_Stream5->CR |= DMA_SxCR_TCIE;
	DMA1_Stream5->PAR = (uint32_t) &USART2->DR;// peripheral address register
	DMA1_Stream5->M0AR = (uint32_t) &rx_data; // memory 0 address register for receiver
	DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5; // high interrupt flag clear register
	DMA1_Stream5->NDTR = 32; //число данных для приема - number of data register
	DMA1_Stream5->CR |= DMA_SxCR_EN; //Switch on Stream

	NVIC_EnableIRQ(DMA1_Stream5_IRQn); //включение прерываний на прием

	//Включение контрольных регистров - transfer
	USART2->CR3 |= USART_CR3_DMAT;

	// STREAM 6 - transfer
	DMA1_Stream6->CR =  DMA_SxCR_MINC | DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0 ; //DMA_SxCR_TCIE - transfer complete
	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;// peripheral address register
	DMA1_Stream6->M0AR = (uint32_t) &tx_data; // memory 0 address register for transfer
	DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6; // high interrupt flag clear register
	DMA1_Stream6->NDTR = 16; //число данных для передачи - number of data register


	NVIC_EnableIRQ(TIM3_IRQn); //включение прерываний на отправку


    /* Loop forever */
	for(;;);
}

//1. Таймер (прескейлер и коунтеры )
