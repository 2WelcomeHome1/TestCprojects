#include <stdint.h>

/* Этап 1 - Определяем основные области памяти */

#define PERIPH_BASE 0x40000000  //Базовый адрес Peripheral
#define PERIPH_BB_BASE (PERIPH_BASE + 0x02000000)  //Базовый адрес Peripheral bit-band region
#define AHB1_BASE (PERIPH_BASE + 0x00020000) //Базовый адрес Advanced Hardware Bus 1
#define GPIOC_BASE (AHB1_BASE + 0x00000800) // Базовый адрес General-purpose I/O для порта С


/* Этап 2 - Определяем структуру General-purpose I/O для порта С */

typedef struct
{
uint32_t GPIOC_MODER;
uint32_t GPIOC_OTYPER;
uint32_t GPIOC_OSPEEDR;
uint32_t GPIOC_PUPDR;
uint32_t GPIOC_IDR;
uint32_t GPIOC_ODR;
uint32_t GPIOC_BSRR;
uint32_t GPIOC_LCKR;
uint32_t GPIOC_AFR;

} GPIOCstr_TypeDef;


/* Этап 3 - Тело программы */

#define GPIOC ((GPIOCstr_TypeDef *) GPIOC_BASE) /* Задаем структуру GPIOC */
#define PC8 *((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(GPIOC->GPIOC_ODR) - PERIPH_BASE)*32 + 8*4)) /* Управление «Синим» светодиодом (метод BitBanding) */
#define PC9 *((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(GPIOC->GPIOC_ODR) - PERIPH_BASE)*32 + 9*4)) /* Управление «Red» светодиодом (метод BitBanding) */


int main(void)
{
*((uint32_t *) 0x40023830) |= 1 | 4; //Включает Gpioc


/* Этап 3.1 - Чтение состояния светодиодов */

GPIOC->GPIOC_MODER &= ~(1<<19)|(1<<18); /*  Изменили в регистре moder битовое написание на 00 */
GPIOC->GPIOC_MODER &= ~(1<<17)|(1<<16); /*  Изменили в регистре moder битовое написание на 00 */
GPIOC->GPIOC_MODER |= (1<<16); /* Установили значение 1 в нужном бите - 01. Сотояние: General purpose output mode */
GPIOC->GPIOC_MODER |= (1<<18); /* Установили значение 1 в бите - 01. Сотояние: General purpose output mode */


/*Задержка переключения*/

void my_sleep(int secs) {
  #define STEPS_PER_SEC 120000
  unsigned int i,s;
  for (s=0; s < secs; s++) {
    for (i=0; i < STEPS_PER_SEC; i++) {
       // skip CPU cycle or any other statement(s) for making loop
       // untouched by C compiler code optimizations
       asm("nop");
    }
  }
}


int count = 0;


while (1){

	/* Этап 3.2 - Чтение состояния кнопок */

	uint8_t button_RED = GPIOC->GPIOC_IDR & (1 << 5); /*  Присволили красной кнопке значение бита (пина) */
	uint8_t button_BLUE = GPIOC->GPIOC_IDR & (1 << 6); /*  Присволили красной кнопке значение бита (пина) */

	/* Этап 3.3 - Выполняем заданное условие */

	if(!(button_RED))
	{
		count = count + 1;
	}
	if(!(button_BLUE))
	{
		count = count - 1;
	}
	if (count < 0)
	{
		PC8 = 0;
		PC9 = 0;
		count = 0;
	}
	if (count == 0 || count == 4 || count == 8)
	{
		PC8 = 0;
		PC9 = 0;
	}

	if (count == 1 || count == 5 || count == 9)
	{
		PC8 = 1;
		PC9 = 0;
	}
	if (count == 2 || count == 6 || count == 10)
	{
		PC8 = 0;
		PC9 = 1;
	}

	if (count == 3 || count == 7 || count == 11)
	{
		PC8 = 1;
		PC9 = 1;
	}

	if (count >= 12)
	{
		PC8 = 0;
		PC9 = 0;
		count = 0;
	}

	my_sleep(1);
}


for(;;);
}
