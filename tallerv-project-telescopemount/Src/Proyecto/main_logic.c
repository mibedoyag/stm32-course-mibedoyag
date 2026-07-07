/**
 * @file    : main_logic.c
 * @author  : Miguel Angel Bedoya Gonzalez -> mibedoyag@unal.edu.co
 * @brief   : Implementación del bucle principal del sistema.
 */
#include "Proyecto/main_logic.h"
#include "Proyecto/interfaz.h"
#include "Proyecto/motores.h"
#include "Proyecto/sensores.h"
#include "Proyecto/astronomia.h"

/**
 * @brief Inicializa todos los subsistemas lógicos.
 * Se debe llamar antes del while(1).
 */
void Montura_Init(void) {
    // Inicialización lógica de cada módulo (el hardware se configura en main.c)
    Interfaz_InitLogica();
    Motores_InitLogica();
    Sensores_InitLogica();
    Astronomia_InitLogica();
}

/**
 * @brief Bucle principal infinito.
 * Procesa las banderas levantadas por las interrupciones.
 */
void Montura_Loop(void) {
    // 1. Actualizar máquina de estados de la interfaz
    Interfaz_UpdateFSM();

    // 2. Procesar tramas de sensores si hay datos nuevos
    Sensores_ProcesarDatos();

    // 3. Actualizar cálculos de motores (Tracking o GoTo)
    Motores_UpdateLogica();
}


/**
 * @brief  Callback de interrupción externa (Pulsadores)
 * @param  GPIO_Pin: Pin que disparó la interrupción
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Ejemplo: Si el pin del botón SELECT se disparó
    if (GPIO_Pin == BTN_SELECT_Pin) {
        flag_btn_select = 1; // Bandera definida en interfaz.c
    }
    // Ejemplo: Si el pin del botón SYNC se disparó
    else if (GPIO_Pin == BTN_SYNC_Pin) {
        flag_btn_sync = 1; // Bandera definida en interfaz.c
    }
    // Ejemplo: Si el pin del botón SPEED se disparó
    else if (GPIO_Pin == BTN_SPEED_Pin) {
        flag_btn_speed = 1; // Bandera definida en interfaz.c
    }
}

/**
 * @brief  Callback de finalización de conversión ADC (Joystick)
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // Obtenemos los valores convertidos por DMA o registro
        // joystick_actual.eje_x = HAL_ADC_GetValue(&hadc1);
        // joystick_actual.eje_y = ...

        flag_adc_joystick_ready = 1; // Bandera definida en motores.c
    }
}

/**
 * @brief  Callback de recepción de UART (GPS)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // GPS
        // Lógica de llenado de buffer circular
        // Si detectamos '\n':
        flag_gps_trama_lista = 1; // Bandera definida en sensores.c

        // Re-activar la interrupción para el siguiente caracter
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
