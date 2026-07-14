/**
 * @file    : stm32f4xx_it.c
 * @brief   : Rutinas de servicio de interrupción (ISR)
 * @author  : Miguel Angel Bedoya G. --> mibedoyag@unal.edu.co
 */

#include "stm32f4xx_hal.h"

/* ====================================================================
 * NOTA DE FASE 1:
 * Los Handles para TIM4 (Encoder), USART1 (GPS) y USART2 (PC)
 * se irán agregando aquí mediante 'extern' conforme vayamos
 * construyendo esos módulos. Por ahora, los omitimos para
 * evitar errores de "Undefined Reference" en el enlazador.
 * ==================================================================== */

/* Declarar el handle que configuramos en sensores.c */
extern UART_HandleTypeDef huart1;

/* Declarar el handle de TIM4 — definido en main.c */
extern TIM_HandleTypeDef htim4;

/* Manejador de SysTick — requerido por la HAL para HAL_Delay() y timeouts base */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* --------------------------------------------------------------------
 * INTERRUPCIONES EXTERNAS (EXTI) - Botones de la Interfaz
 * -------------------------------------------------------------------- */

/* Manejador para la línea EXTI 0 (Ej. Botón SYNC en PA0, PB0 o PC0) */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* Manejador para la línea EXTI 1 (Ej. Botón SELECT en PA1, PB1 o PC1) */
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/* Manejador para la línea EXTI 2 (Ej. Botón SPEED en PA2, PB2 o PC2) */
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/* Manejador del evento de actualización de TIM4 */

void TIM4_IRQHandler(void)
{

HAL_TIM_IRQHandler(&htim4);

}


/* Despachador de la interrupción del GPS */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* * NOTA SOBRE EL ADC:
 * Como implementamos la lectura del Joystick mediante Polling manual optimizado
 * en motores.c (sin bloqueos de sistema), la rutina ADC_IRQHandler ya no
 * es necesaria y la eliminamos para ahorrar ciclos de reloj y memoria.
 */
