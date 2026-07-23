/**
 * @file    : stm32f4xx_it.c
 * @brief   : Rutinas de servicio de interrupción (ISR) y DMA
 * @author  : Miguel Angel Bedoya G. --> mibedoyag@unal.edu.co
 */

#include "stm32f4xx_hal.h"

/* ====================================================================
 * HANDLES EXTERNOS DE LOS PERIFÉRICOS
 * Referencias a las configuraciones creadas en motores.c y sensores.c
 * ==================================================================== */

/* Handles del GPS (UART + DMA) -> Vienen de sensores.c */
extern UART_HandleTypeDef huart2;


/* Handle del Encoder Rotativo -> Viene de interfaz.c */
extern TIM_HandleTypeDef htim4;



/* ====================================================================
 * INTERRUPCIONES DEL SISTEMA BASE
 * ==================================================================== */

/* Manejador de SysTick — requerido por la HAL*/
void SysTick_Handler(void) {
    HAL_IncTick();
}

/* ====================================================================
 * INTERRUPCIONES EXTERNAS (EXTI) - Botones de la Interfaz
 * ==================================================================== */

/**
 * @brief Manejador de interrupciones para pines del 10 al 15.
 * Aquí caen los tres botones de control agrupados en el puerto B.
 */


/* ====================================================================
 * INTERRUPCIONES DE PERIFÉRICOS (UART / TIMERS)
 * ==================================================================== */

/**
 * @brief Despachador de la interrupción del GPS (Útil para control de errores de línea)
 */
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief Manejador del evento de actualización de TIM4 (Encoder Rotativo)
 */
void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}



