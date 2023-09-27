#include <stdint.h>
#include <stm32f446xx.h>


/* Входная частота контроллера*/
int inp_freq = 14000000;
int int_freq = 17000;

#define PC8 *((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(GPIOC->ODR) - PERIPH_BASE)*32 + 8*4))/* Управление «Синим» светодиодом (метод BitBanding) */

/* Рассчитываем main clock value*/
#define SysTicksClk (inp_freq / PLLM * PLLN / PLLP ) //160000000
#define HCLK (SysTicks/AHB1) //160000000
#define PCLK1 (HCLK/APB1)
#define PCLK2 (HCLK/APB2)
#define SysTicks (160000000/17000)

/* Значения коэффициентов PLL;вычисленные */
int PLLN = 160;
int PLLM = 7;
int PLLP = 2;

uint16_t count = 0;
int wait = 0;

// Задаем функцию системного таймера для переключения светодиода с частотой 1Гц (~1 сек)

void SysTick_Handler (void){
	if(count == int_freq) {
		if (wait){
			PC8 = 1;
			wait = 0;
		} else {
			PC8 = 0;
			wait = 1;
		}
	count = 0;
	} else {
	count += 1;
	}
}

uint32_t rcc_pll_m, rcc_pll_n, rcc_pll_p;


int main(void)
{
	//Запускаем GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//Управляем Светодиодом
	GPIOC->MODER &= ~(1<<17)|(1<<16);
	GPIOC->MODER |= (1<<16); //GPIO_MODER_MODER8_1  /* Установили значение 1 в нужном бите - 01. Сотояние: General purpose output mode */

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

	SysTick_Config(SysTicks); //Запускаем системный таймер

	rcc_pll_m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos;
	rcc_pll_n = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
	rcc_pll_p = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk) >> RCC_PLLCFGR_PLLP_Pos;



    /* Loop forever */
	for(;;);
}
