/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief          : Introduccion Timers
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

uint8_t state = 0;
uint8_t numeroled = 0;


void color (void);

int main(void)
{

	/* definicion de variables del sistema */
	//Activando la señal de reloj/
		RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; //Limpiando el registro
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN; //Limpiando el registro
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	    //Configuracion del pin A6 como salida/
	    GPIOA->MODER &= ~GPIO_MODER_MODE6; //Limpiando el registro
	    GPIOA->MODER |= GPIO_MODER_MODE6_0;

	    //Configuracion deel pin A6 como salida push-pull/
	    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT6);

	    //Configuracion de la velocidad como alta/
	    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6; //Limpiando el registro
	    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1;

	    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6; //No pull up, no pull down

	    //Encendido del LED/
	    GPIOA->ODR |= GPIO_ODR_OD6;

	    //Configuracion del pin A7 como salida/
	    GPIOA->MODER &= ~GPIO_MODER_MODE7; //Limpiando el registro
	    GPIOA->MODER |= GPIO_MODER_MODE7_0;

		//Configuracion deel pin A7 como salida push-pull/
		GPIOA->OTYPER &= ~GPIO_OTYPER_OT7;

		//Configuracion de la velocidad como alta/
		GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7;
		GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7_1);

		GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7; //No pull up, no pull down

		//Encendido del LED
		GPIOA->ODR |= GPIO_ODR_OD7;

		//Configuracion del pin B6 como salida/
		GPIOB->MODER &= ~GPIO_MODER_MODE6; //Limpiando el registro
		GPIOB->MODER |= GPIO_MODER_MODE6_0;

		//Configuracion deel pin B6 como salida push-pull/
		GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6);

		//Configuracion de la velocidad como alta/
		GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED6; //Limpiando el registro
		GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_1;

		GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6; //No pull up, no pull down

		//Encendido del LED/
		GPIOB->ODR |= GPIO_ODR_OD6;

	    //Configurando el TIM3
	    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM3EN); //Limpiando la posición
	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	    //Configurando el TIM3 internamente
	    TIM3->PSC = 1600 - 1;  //se incrementa en 0.1ms o 10kHz
	    TIM3->ARR = 20000 - 1; //Genera interrupciones cada 2000 ms
	    TIM3->CNT = 0; //Inicia el contador en 0
	    TIM3->SR &= ~(TIM_SR_UIF); //Limpiando el status register
	    TIM3->DIER &= ~(TIM_DIER_UIE);  //Limpiando la posicion
	    TIM3->DIER |= TIM_DIER_UIE; //Activamos la interrupcion

	    __NVIC_EnableIRQ(TIM3_IRQn); //Matriculando la interrupcion TIM3 en el NVIC, para que sea reconocida

	    //Configurando la direccion en la que el contador cuenta
	    TIM3->CR1 &= ~(TIM_CR1_DIR); //Limpiando la direccion, queda de forma ascendente

	    //Activamos la precarga de ARR
	    TIM3->CR1 &= ~(TIM_CR1_ARPE); //Limpiamos la posicion del ARPE
	    TIM3->CR1 |= TIM_CR1_ARPE; //Activamos la precarga

	    //Activamos el contador, para que la señal del reloj se comience a propagar
	    TIM3->CR1 |= TIM_CR1_CEN;

	/* Loop forever */
	while(1) {

		//Toggle LED
		//GPIOA->ODR ^= GPIO_ODR_OD5;

	//	delay_Ms(160000);

		//SI hay peticion, entra al if y ejecuta la función color() y luego devuelve state a 0
		if (state){
			color();
			state = 0;
		}

	}

	return 0;
}

//Funcion ISR para el TIM3
	//En general, toda funcoina ISR no retorna nada (void) y no recibe parametros
	void TIM3_IRQHandler(void){

//		//Verificar que genero la interrupcion (la flag)
//		if (TIM3->SR && TIM_SR_UIF){
//			GPIOA->ODR ^= GPIO_ODR_OD5; //Realizamos una accion en respuesta a la interrupcion (toggle de led)
//
//			TIM3->SR &= ~(TIM_SR_UIF); //Bajamos la bandera de la interrupcion

		//Configuración semaforo

			if (TIM3->SR && TIM_SR_UIF){

				state = 1;
				TIM3->SR &= ~(TIM_SR_UIF); //Bajamos la bandera de la interrupcion


		}
	}

	void color (void){
		switch (numeroled){
		case (0): //Encender el led verde
			GPIOA->ODR &= ~GPIO_ODR_OD6;
			GPIOA->ODR &= ~GPIO_ODR_OD7;
			GPIOB->ODR |= GPIO_ODR_OD6; //Pin led verde
			numeroled ++;
			break;

		case (1): //Encender el led amarillo
			GPIOA->ODR &= ~GPIO_ODR_OD6;
			GPIOA->ODR |= GPIO_ODR_OD7; //Pin led amarillo
			GPIOB->ODR &= ~GPIO_ODR_OD6;
			numeroled ++;
			break;

		case (2): //Encender el led rojo
			GPIOA->ODR |= GPIO_ODR_OD6; //Pin led rojo
			GPIOA->ODR &= ~GPIO_ODR_OD7;
			GPIOB->ODR &= ~GPIO_ODR_OD6;
			numeroled = 0;
			break;

		default:
			numeroled = 0;
			break;

		}
	}



	//Funcion para el delay_Ms
//	void delay_Ms (uint32_7 time_delay)
//	{
//		for(counter = 0; counter < time_delay; counter++);
//	}

