/*
 * stm32f4xx_it.c
 * Rutinas de servicio de interrupción
 * Autor: Miguel Angel Bedoya G. --> mibedoyag@unal.edu.co
 */

#include "stm32f4xx_hal.h"

/* Declarar el handle de TIM4 — definido en main.c */
extern TIM_HandleTypeDef htim4;

/* Declarar el handle de ADC1 — definido en main.c */
extern ADC_HandleTypeDef hadc1;

/* Manejador de SysTick — requerido por HAL para HAL_Delay() y timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Manejador del evento de actualización de TIM4 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

// ADC conversion ISR

void ADC_IRQHandler (void){
	HAL_ADC_IRQHandler(&hadc1);
}
