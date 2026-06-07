/**
 ******************************************************************************
 * @file           : Tarea2_TallerV_MABG.c
 * @author         : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief          : Tarea 2 - Siete segmentos y fotocompuertas
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f4xx.h>

/* Varbiables */

uint16_t counter_EXTI = 0;

/* Definición de funciones*/

void init_hardware(void);
void blinky(void);

/* Main */

int main(void){
	blinky();


	while(1){


	}

	return 0;
}

/* Funciones */

void init_hardware(void){
	//Activando las señales de reloj/
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; //Limpiando el registro
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Activando las señal de reloj para GPIOA
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; //Limpiando el registro
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //Activando las señal de reloj para GPIOB
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; //Limpiando el registro
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Activando las señal de reloj para GPIOC
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN; //Limpiando el registro
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; //Activando las señal de reloj para GPIOD

	/* Configuración de puertos GPIO para los LED del 7 segmentos */

	/*Configuracion del pin B8 -> Segmento B */
	//Configurción como salida
	GPIOB->MODER &= ~GPIO_MODER_MODE8; //Limpiando el registro
	GPIOB->MODER |= GPIO_MODER_MODE8_0;

	//Configuracion como salida push-pull/
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT8);

	//Configuracion de la velocidad como alta/
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8; //Limpiando el registro
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1;

	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8; //No pull up, no pull down

	//Encendido del LED/
	GPIOB->ODR |= GPIO_ODR_OD8;

	/*Configuracion del pin C8 -> Segmento F */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE8; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE8_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT8);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD8; //No pull up, no pull down

	//Encendido del LED/
	GPIOC->ODR |= GPIO_ODR_OD8;

	/*Configuracion del pin C9 -> Segmento A */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE9; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE9_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT9);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD9; //No pull up, no pull down

	//Encendido del LED/
	GPIOC->ODR |= GPIO_ODR_OD9;

	/*Configuracion del pin C10 -> Segmento E */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE10; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE10_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD10; //No pull up, no pull down

	//Encendido del LED/
	GPIOC->ODR |= GPIO_ODR_OD10;

	/*Configuracion del pin C11 -> Segmento C */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE11; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE11_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT11);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED11; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11; //No pull up, no pull down

	//Encendido del LED/
	GPIOC->ODR |= GPIO_ODR_OD11;

	/*Configuracion del pin C12 -> Segmento D */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE12; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE12_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT12);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED12; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED12_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD12; //No pull up, no pull down

	//Encendido del LED/
	GPIOC->ODR |= GPIO_ODR_OD12;

	/*Configuracion del pin D2 -> Segmento G */
	//Configurción como salida
	GPIOD->MODER &= ~GPIO_MODER_MODE2; //Limpiando el registro
	GPIOD->MODER |= GPIO_MODER_MODE2_0;

	//Configuracion como salida push-pull/
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT2);

	//Configuracion de la velocidad como alta/
	GPIOD->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2; //Limpiando el registro
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_1;

	GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD2; //No pull up, no pull down

	//Encendido del LED/
	GPIOD->ODR |= GPIO_ODR_OD2;


	/* Configuración de puertos GPIO para los Transistores */

	/*Configuracion del pin C6 -> Digito D1 */
	//Configurción como salida
	GPIOC->MODER &= ~GPIO_MODER_MODE6; //Limpiando el registro
	GPIOC->MODER |= GPIO_MODER_MODE6_0;

	//Configuracion como salida push-pull/
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT6);

	//Configuracion de la velocidad como alta/
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6; //Limpiando el registro
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1;

	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD6; //No pull up, no pull down

	//Activación/encendido del transistor
	GPIOC->ODR |= GPIO_ODR_OD6;

	/*Configuracion del pin B9 -> Digito D2 */
	//Configurción como salida
	GPIOB->MODER &= ~GPIO_MODER_MODE9; //Limpiando el registro
	GPIOB->MODER |= GPIO_MODER_MODE9_0;

	//Configuracion como salida push-pull/
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT9);

	//Configuracion de la velocidad como alta/
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9; //Limpiando el registro
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_1;

	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD9; //No pull up, no pull down

	//Activación/encendido del transistor
	GPIOB->ODR |= GPIO_ODR_OD9;

	/*Configuracion del pin H0 -> Digito D3 */
	//Configurción como salida
	GPIOH->MODER &= ~GPIO_MODER_MODE0; //Limpiando el registro
	GPIOH->MODER |= GPIO_MODER_MODE0_0;

	//Configuracion como salida push-pull/
	GPIOH->OTYPER &= ~(GPIO_OTYPER_OT0);

	//Configuracion de la velocidad como alta/
	GPIOH->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0; //Limpiando el registro
	GPIOH->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_1;

	GPIOH->PUPDR &= ~GPIO_PUPDR_PUPD0; //No pull up, no pull down

	//Activación/encendido del transistor
	GPIOH->ODR |= GPIO_ODR_OD0;

	/*Configuracion del pin B7 -> Digito D4 */
	//Configurción como salida
	GPIOB->MODER &= ~GPIO_MODER_MODE7; //Limpiando el registro
	GPIOB->MODER |= GPIO_MODER_MODE7_0;

	//Configuracion como salida push-pull/
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);

	//Configuracion de la velocidad como alta/
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7; //Limpiando el registro
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_1;

	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7; //No pull up, no pull down

	//Activación/encendido del transistor
	GPIOB->ODR |= GPIO_ODR_OD7;


	/* Configuración de puertos GPIO para las Fotocompuertas */

	/*Configuracion del pin C2 -> Fotocompuerta F1 */
	//Configuracion como entrada simple
	GPIOC->MODER &= ~GPIO_MODER_MODE2;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD2; //No resistencia pull up, ni pull down

	/*Configuracion del pin A0 -> Fotocompuerta F2 */
	//Configuracion como entrada simple
	GPIOA->MODER &= ~GPIO_MODER_MODE0;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0; //No resistencia pull up, ni pull down


}

void blinky(void){
	//Encendiendo la señal de reloj para el bus AHB1 donde esta el GPIOH
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN; //Limpiando el registro del RCC en la posición del GPIOH
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;  //Encendiendo la señal de reloj para el GPIOH

	/* Configuracion del pin H1 */
	GPIOH->MODER &= ~GPIO_MODER_MODE1; //Limpiando el registro
	GPIOH->MODER |= GPIO_MODER_MODE1_0; //Asignando el pin H1 como salida

	//Configuracion del pin H1 como salida push-pull/
	GPIOH->OTYPER &= ~(GPIO_OTYPER_OT1);

	//Configuracion de la velocidad como alta/
	GPIOH->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1; //Limpiando el registro
	GPIOH->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1; //Asignando la velocidad de alta

	GPIOH->PUPDR &= ~GPIO_PUPDR_PUPD1; //No pull up, no pull down

	//Encendido del LED/
	GPIOH->ODR |= GPIO_ODR_OD1;


	/* Configuración del Timer 2 */
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN; //Limpieza de la posición del registro
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  //Encendiendo la señal de reloj para TIM2

	//Configurando el Prescaler
	TIM2->PSC = 1600 - 1; // 0.1 ms O 10 kHz ya que el TIM2 es a 16 MHz / 1600 = 0.1 ms

	//Configurando el ARR
	TIM2->ARR = 5000 - 1; // 0.1 ms * 5000 = 500 ms para que genere un ciclo

	//Reinicio del contador
	TIM2->CNT = 0; //Inicializa en 0

	//Limpieza de la bandera de la interrupción
	TIM2->SR &= ~TIM_SR_UIF;

	//Asignando el tipo de interrupcion como update-event
	TIM2->DIER &= ~ TIM_DIER_UIE; //Limpiando la posición del registro
	TIM2->DIER |=  TIM_DIER_UIE;

	//Matriculando la interrupción generada por TIM2 en el NVIC
	__NVIC_EnableIRQ(TIM2_IRQn);

	//Configuracion del contador
	TIM2->CR1 &= ~TIM_CR1_DIR; //Indicando la dirección de conteo para el contador

	//Estableciendo la precarga del ARR
	TIM2->CR1 &= ~TIM_CR1_ARPE; //Limpiando la posición del registro antes de asignar
	TIM2->CR1 |= TIM_CR1_ARPE; //Activando la precarga

	//Activando el contador para que el reloj inicie la propagación
	TIM2->CR1 |= TIM_CR1_CEN;
}

/* Funcion ISR para el TIM2 (Startup) */
void TIM2_IRQHandler(void){
	//Confirmar que se ha generado una interrupcion
	if(TIM2->SR && TIM_SR_UIF){
		GPIOH->ODR ^= GPIO_ODR_OD1; //Hace el toggle del LED
		TIM2->SR &= ~TIM_SR_UIF; //Baja la bandera para que la interrupción se pueda volver a generar
	}
}




