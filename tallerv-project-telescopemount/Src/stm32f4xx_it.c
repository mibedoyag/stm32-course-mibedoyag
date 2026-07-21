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
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/* Handles del Joystick (ADC + DMA) -> Vienen de motores.c */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/* Handle del Encoder Rotativo -> Viene de interfaz.c */
extern TIM_HandleTypeDef htim4;

/* Handle del TIM10 para el blinky en main.c */
extern TIM_HandleTypeDef htim10;

extern TIM_HandleTypeDef htim2; // Manejador del Timer 2 (Eje Azimut)
extern TIM_HandleTypeDef htim3; // Manejador del Timer 3 (Eje Altitud)

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
void EXTI15_10_IRQHandler(void) {
    // PB12: Botón SPEED
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    // PB13: Botón SYNC
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    // PB14: Botón SELECT (Hundir el Encoder)
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
}

/* ====================================================================
 * INTERRUPCIONES DE ACCESO DIRECTO A MEMORIA (DMA)
 * ==================================================================== */

/**
 * @brief Vector del DMA2 Stream 0 -> Encargado de actualizar el Joystick (ADC1)
 */
void DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
 * @brief Vector del DMA2 Stream 5 -> Encargado de recibir el GPS (USART1_RX)
 */
void DMA2_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/* ====================================================================
 * INTERRUPCIONES DE PERIFÉRICOS (UART / TIMERS)
 * ==================================================================== */

/**
 * @brief Despachador de la interrupción del GPS (Útil para control de errores de línea)
 */
void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief Manejador del evento de actualización de TIM4 (Encoder Rotativo)
 */
void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief This function handles TIM1 Update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  // Le pasamos el control a la HAL para que limpie banderas y llame al Callback
  HAL_TIM_IRQHandler(&htim10);
}

/* =========================================================================
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISR) PARA LOS MOTORES
 * ========================================================================= */

/**
 * @brief Función ISR para la interrupción global de TIM2.
 * @note  Se ejecuta en cada flanco del PWM del motor de Azimut.
 */
void TIM2_IRQHandler(void)
{
    // La función de la capa HAL se encarga de limpiar las banderas de hardware
    // y redirige el flujo automáticamente hacia nuestro HAL_TIM_PWM_PulseFinishedCallback()
    HAL_TIM_IRQHandler(&htim2);
}

/**
 * @brief Función ISR para la interrupción global de TIM3.
 * @note  Se ejecuta en cada flanco del PWM del motor de Altitud.
 */
void TIM3_IRQHandler(void)
{
    // La función de la capa HAL se encarga de limpiar las banderas de hardware
    // y redirige el flujo automáticamente hacia nuestro HAL_TIM_PWM_PulseFinishedCallback()
    HAL_TIM_IRQHandler(&htim3);
}
