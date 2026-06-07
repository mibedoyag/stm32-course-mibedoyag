/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief          : Introduccion EXTI
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

uint16_t counter_exti1 = 0;
volatile uint8_t increment_counter = 0;


int main(void)
{

	/* definicion de variables del sistema */
	//Activando la señal de reloj/
		RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN; //Limpiando el registro
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	    RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN; //Limpiando el registro
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //Encender el registro del EXTI (SYSCONFIG)

	    //Configuracion del pin H1 como salida/
	    GPIOH->MODER &= ~GPIO_MODER_MODE1; //Limpiando el registro
	    GPIOH->MODER |= GPIO_MODER_MODE1_0;

	    //Configuracion deel pin H1 como salida push-pull/
	    GPIOH->OTYPER &= ~(GPIO_OTYPER_OT1);

	    //Configuracion de la velocidad como alta/
	    GPIOH->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1; //Limpiando el registro
	    GPIOH->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_1;

	    GPIOH->PUPDR &= ~GPIO_PUPDR_PUPD1; //No pull up, no pull down

	    //Encendido del LED/
	    GPIOH->ODR |= GPIO_ODR_OD1;

	    //COFIGURANDO EL PIN_C1 como entrada simple
	    GPIOC->MODER &= ~GPIO_MODER_MODE1;
	    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD1; //Configurando sin resistencia de PUPD

	    //Configurando el EXTI
	    //Configurando el MUX del EXTI

	    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;  //Limpiamos el registro
	    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;  //Configurando el EXTI 1 para que funcione en el puerto C (pin1)

	    //Configurando para detectar flancos de bajada en el EXTI

	    EXTI->FTSR |= EXTI_FTSR_TR1; //Seleccionando flancos de bajada


	    __NVIC_EnableIRQ(EXTI1_IRQn); //Matriculando la interrupcion en el NVIC, para que sea reconocida

	    EXTI->PR |= EXTI_PR_PR1; //Limpiando la bandera

	    //Activamos la interrupcion (IMR)
	    EXTI->IMR |= EXTI_IMR_IM1;



	/* Loop forever */
	while(1) {

		if (increment_counter == 1) {
			counter_exti1++;
			increment_counter = 0;
		}
		}

	return 0;
}

// Funcion ISR (Buscar en startup) para el EXTI1

void EXTI1_IRQHandler(void){
	if(EXTI->PR && EXTI_PR_PR1){
		//limpiamos la bandera relacionada con el EXTI1
		EXTI->PR |= EXTI_PR_PR1;
		__NOP();
		increment_counter = 1;

	}

}



