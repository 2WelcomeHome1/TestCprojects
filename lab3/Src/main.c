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

//#define PCLK1 (HCLK/APB1)
//#define PCLK2 (HCLK/APB2)


/* Определяем частоту работы USART receiver and transmitter*/
#define APB1_FREQ (HCLK/APB1) // 40MHz
#define BAUDRATE 19200

//Принимаем сообщение
int count_read = 0; //Для расчета числа знаков в принимаемом сообщении
char text[5];// Принимаемый текст

//Расчитываем значение ф-ции
int number = 0; //Для считываемого числа
float function = 0; // Расчет косинуса

//Отправляем сообщение
int count_send = 0;
char message [16]; //Текст отправляемого сообщения

//Создаем USART2 Interrupt Handler
void USART2_IRQHandler(void) {

	 //Принимаем сообщение
	 if (USART2->SR & USART_SR_RXNE) {
		 text[count_read] = USART2->DR; //Data register
		 //Регистрируем
	 	 if (text[0] == 'N' ){
	 		 count_read +=1;
	 	 }
	 	 if (count_read == 6){
	 		 count_read = 0;
	 		//Расчитываем значение ф-ции
		 	 if (text[5] == 'E' ){
		 		number = (text[3] - 48) * 10 + (text[4] - 48); //вытаскиваем число
		 		function = cos(number);//расчитываем косинус полученного значения
		 		sprintf(message,"\ncos(x)=%6.3f\r", function);
		 		USART2->CR1 |= USART_CR1_TXEIE;
		 	 }

	 	 }
	 }
	 //Отправляем сообщение
 	 if(USART2->SR && USART_SR_TXE &&(USART2->CR1  & USART_CR1_TXEIE)){
 		USART2->DR = message[count_send];
 		count_send +=1;
 		if (count_send > 14) {
 			count_send = 0;
 			USART2->CR1 &= ~(USART_CR1_TXEIE);
 		}
 	 }
}

int main(void){

	SCB->CPACR |=((3UL << 10*2)|(3UL << 11*2));

	FLASH->ACR |= FLASH_ACR_LATENCY_5WS; // Set flash latency (задержка для памяти)
	RCC->CR |= RCC_CR_HSEON;	 // Включаем hse

	/* Значения коэффициентов Делителей; */

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;


	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; //Включаем PLL Source в режим Hse

	/* Значения коэффициентов PLL */

	// Чистим регистр от заранее заданных значений
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk |  RCC_PLLCFGR_PLLP_Msk );

	// Передаем новые значение в регистр
	RCC->PLLCFGR |= PLLN << RCC_PLLCFGR_PLLN_Pos ;
	RCC->PLLCFGR |= PLLM << RCC_PLLCFGR_PLLM_Pos ;


	RCC->CR |= RCC_CR_PLLON ;  // Включаем pll

	RCC->CFGR |= RCC_CFGR_SW_PLL; //Включаем system clock  в режим PLL_p

	//Запускаем GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


	//Запускаем GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//Задаем пины и функцию
	GPIOA->MODER &= ~(1<<4)|(1<<5);
	GPIOA->MODER |= (1<<5);
	GPIOA->MODER &= ~(1<<6)|(1<<7);
	GPIOA->MODER |= (1<<7);

	//Режим АФ7
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2 ;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2 ;

	//Управляем Светодиодом
	GPIOC->MODER &= ~(1<<17)|(1<<16);
	GPIOC->MODER |= (1<<16); //GPIO_MODER_MODER8_1  /* Установили значение 1 в нужном бите - 01. Сотояние: General purpose output mode */


	//Включаем USART
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	//Задаем частоту работы USART2
	USART2->BRR = APB1_FREQ/BAUDRATE;

	//обнулили
	USART2->CR1 = 0x00;

	//Настроить USART на прием и передачу данных, генерацию требуемых прерываний

	USART2->CR1 |= USART_CR1_TE; //Включаем transm
	USART2->CR1 |= USART_CR1_RE; //Включаем receiver
	USART2->CR1 |= USART_CR1_UE; //Включаем прерывания USART

	//Включить прерывания в контроллере прерываний на прием
	USART2->CR1 |= USART_CR1_RXNEIE;

	//Включение контрольных регистров
	//USART->CR3 |= USART_CR3_DMAR;
	//USART->CR3 |= USART_CR3_DMAT;

	NVIC_EnableIRQ(USART2_IRQn); //включение прерываний


    /* Loop forever */
	for(;;);
}
