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

//Definición de variables

uint8_t state = 0;
uint8_t numeroled = 0;

//Definición de funciones

void color (void);

//Main

int main(void){

	/* definicion de variables del sistema */
	//Activando la señal de reloj/
		RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN; //Limpiando el registro
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;


	    //Configuracion del pin A5 como salida/
	    GPIOA->MODER &= ~GPIO_MODER_MODE5; //Limpiando el registro
	    GPIOA->MODER |= GPIO_MODER_MODE5_0;

	    //Configuracion del pin A6 como salida push-pull/
	    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);

	    //Configuracion de la velocidad como alta/
	    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5; //Limpiando el registro
	    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5_1;

	    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; //No pull up, no pull down

	    //Encendido del LED/
	    GPIOA->ODR |= GPIO_ODR_OD5;


	    //Configuración PA13
	    GPIOC->MODER &= ~GPIO_MODER_MODE13; //Limpiando el registro - Asignación de entrada
	    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD13; //Limpiando el registro
	    GPIOC->PUPDR |= GPIO_PUPDR_PUPD13_0; //Asignación con resistor pull up.


	    while(1){

	    	//Tarea 4.3
//	    	GPIOA->ODR |= GPIO_ODR_OD5;
//	    	for (uint32_t i = 0; i < 100000; i++);
//	    	GPIOA->ODR &= ~GPIO_ODR_OD5;
//	    	for (uint32_t i = 0; i < 1000000; i++);

	    	//Tarea 4.4 - LA logica se invierte porque hay resistencia pull up, por lo que si el pulsador esta sin presionar este está conectado a VDD (3.3 V)
	    	if((GPIOC->IDR & GPIO_IDR_ID13) == 0){ //Lee el registro - SI es 0 el boton está presionado, cierra a gnd (0)
	    		GPIOA->ODR |= GPIO_ODR_OD5; // boton presionado - enciende el led PA5

	    	}
	    	else
	    	{
	    		//Boton sin presionar
	    		GPIOA->ODR &= ~GPIO_ODR_OD5; //Apaga el led
	    	}


	    }

	    return 0;

}
