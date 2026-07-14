/**
 * @file    : motores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de la lógica de movimiento, escalado de pasos y mapeo analógico.
 * Configuración manual de TIM2 (Azimut), TIM3 (Altitud), ADC1 (Joystick) y pines GPIO.
 */

#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include "stm32f4xx_hal.h"

/* * Instancias globales de los periféricos.
 * El control absoluto del hardware recae en este módulo, no en el autogenerador del IDE.
 */
TIM_HandleTypeDef htim2; // Timer 2 - PWM Eje Azimut
TIM_HandleTypeDef htim3; // Timer 3 - PWM Eje Altitud
ADC_HandleTypeDef hadc1; // ADC 1 - Lectura del Joystick

/* Definición de Pines Físicos */
#define AZ_DIR_PORT  GPIOA
#define AZ_DIR_PIN   GPIO_PIN_4 // PA4
#define ALT_DIR_PORT GPIOA
#define ALT_DIR_PIN  GPIO_PIN_5 // PA5

/* Variables lógicas del módulo */
JoystickData_t joystick_actual;
VelocidadModo_t velocidad_actual;

/* Frecuencias base para los registros ARR a 16 MHz de reloj (HCLK) */
uint32_t arr_buscar  = 999;   // 1000 Hz (GoTo Rápido)
uint32_t arr_centrar = 3999;  // 250 Hz  (Joystick Normal)
uint32_t arr_guiar   = 19999; // 50 Hz   (Ajuste Fino - Seguimiento Sideral)

/* =========================================================================
 * 1. CONFIGURACIÓN DE HARDWARE A BAJO NIVEL (ESTILO TAREA 3)
 * ========================================================================= */

/**
 * @brief Configuración de los pines GPIO para Dirección y Timers (Funciones Alternativas).
 */
static void Motores_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Activar señal de reloj para el Puerto A

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Pines de Dirección (Salida Digital) -> PA4 y PA5
    GPIO_InitStruct.Pin = AZ_DIR_PIN | ALT_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   // Salida Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // Sin resistencias internas
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. Pin PWM Azimut (TIM2_CH1) -> PA0
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // Modo Función Alternativa
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;    // AF01 multiplexa el TIM2 al PA0
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Pin PWM Altitud (TIM3_CH1) -> PA6
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;    // AF02 multiplexa el TIM3 al PA6
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Configuración del ADC1 en modo manual (sin DMA) para PA1 y PA2.
 */
static void Motores_ADC_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();  // Activar señal de reloj para ADC1

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar PA1 (IN1) y PA2 (IN2) en modo entrada analógica pura
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configurar el ADC1 para disparos individuales por software
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // Dividir reloj para estabilidad
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;           // Resolución 0 - 4095
    hadc1.Init.ScanConvMode = DISABLE;                    // Se escanea un canal a la vez a mano
    hadc1.Init.ContinuousConvMode = DISABLE;              // Solo una lectura por trigger
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;     // Se dispara por código
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;           // Ajuste a la derecha (estándar)
    hadc1.Init.NbrOfConversion = 1;                       // Una conversión por ciclo
    hadc1.Init.DMAContinuousRequests = DISABLE;           // Sin DMA
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);
}

/**
 * @brief Configuración de los Timers 2 y 3 para generar PWM a 1 MHz de base.
 */
static void Motores_TIM_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* --- Configuración TIM2 (Azimut) --- */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16 - 1;                // 16MHz / 16 = 1MHz -> El timer cuenta cada 1 us
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr_buscar;               // Valor de Auto-Reload (ARR) inicial
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;           // Modo PWM estándar
    sConfigOC.Pulse = arr_buscar / 2;             // 50% de Duty Cycle (Onda cuadrada perfecta)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    /* --- Configuración TIM3 (Altitud) --- */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16 - 1;                // Misma base de tiempo de 1MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = arr_buscar;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

/* =========================================================================
 * 2. FUNCIONES PRIVADAS DE CONTROL
 * ========================================================================= */

/**
 * @brief Lee los dos ejes del joystick cambiando el canal del ADC manualmente.
 * Al usar Polling optimizado, esta función toma aprox ~10 microsegundos y no bloquea.
 */
static void Motores_LeerJoystick(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // Tiempo de estabilización del capacitor

    // 1. Leer Eje X (Canal 1 - Pin PA1)
    sConfig.Channel = ADC_CHANNEL_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); // Seleccionar el canal del eje X
    HAL_ADC_Start(&hadc1);                   // Disparar conversión
    HAL_ADC_PollForConversion(&hadc1, 5);    // Esperar máximo 5ms a que termine
    joystick_actual.eje_x = HAL_ADC_GetValue(&hadc1); // Guardar valor
    HAL_ADC_Stop(&hadc1);

    // 2. Leer Eje Y (Canal 2 - Pin PA2)
    sConfig.Channel = ADC_CHANNEL_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); // Cambiar multiplexor al eje Y
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5);
    joystick_actual.eje_y = HAL_ADC_GetValue(&hadc1); // Guardar valor
    HAL_ADC_Stop(&hadc1);
}

/* =========================================================================
 * 3. LÓGICA DEL MÓDULO (API PÚBLICA)
 * ========================================================================= */

/**
 * @brief Inicializa el hardware y pone el sistema mecánico en un estado seguro.
 */
void Motores_InitLogica(void) {
    // 1. Iniciar hardware a bajo nivel
    Motores_GPIO_Init();
    Motores_ADC_Init();
    Motores_TIM_Init();

    // 2. Inicialización segura del Joystick en la zona muerta (Centro mecánico = 2048)
    joystick_actual.eje_x = 2048;
    joystick_actual.eje_y = 2048;
    velocidad_actual = SPEED_BUSCAR;

    // 3. Asegurar que los motores arrancan totalmente apagados
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief Modifica el registro ARR (Auto-Reload Register) en tiempo real
 * para cambiar la frecuencia del PWM y, por ende, la velocidad de los motores.
 */
void Motores_SetVelocidadGlobal(VelocidadModo_t nueva_velocidad) {
    velocidad_actual = nueva_velocidad;
    uint32_t nuevo_arr = arr_buscar;

    switch(velocidad_actual) {
        case SPEED_BUSCAR:  nuevo_arr = arr_buscar;  break;
        case SPEED_CENTRAR: nuevo_arr = arr_centrar; break;
        case SPEED_GUIAR:   nuevo_arr = arr_guiar;   break;
    }

    // Actualizar Timer de Azimut
    __HAL_TIM_SET_AUTORELOAD(&htim2, nuevo_arr);                 // Frecuencia
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, nuevo_arr / 2); // 50% Duty Cycle

    // Actualizar Timer de Altitud
    __HAL_TIM_SET_AUTORELOAD(&htim3, nuevo_arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nuevo_arr / 2);
}

/**
 * @brief Lógica de evaluación periódica del sistema de movimiento.
 * Debe ejecutarse continuamente dentro del super-loop (Montura_Loop).
 */
void Motores_UpdateLogica(void) {

    // Solo permitimos el control directo del joystick si la FSM está en Modo Manual
    if (currentState == STATE_MANUAL) {

        // 1. Capturar los valores actuales del Joystick analógico
        Motores_LeerJoystick();

        // ---------------------------------------------------------
        // 2. EVALUACIÓN EJE X (AZIMUT - TIM2)
        // ---------------------------------------------------------
        // Si el valor cae por debajo del umbral de centro (Movimiento Izquierda)
        if (joystick_actual.eje_x < (2048 - JOYSTICK_DEADZONE)) {
            HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        }
        // Si el valor supera el umbral superior (Movimiento Derecha)
        else if (joystick_actual.eje_x > (2048 + JOYSTICK_DEADZONE)) {
            HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        }
        // Si el valor está en la zona muerta (Centro)
        else {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Detener pulsos instantáneamente
        }

        // ---------------------------------------------------------
        // 3. EVALUACIÓN EJE Y (ALTITUD - TIM3)
        // ---------------------------------------------------------
        if (joystick_actual.eje_y < (2048 - JOYSTICK_DEADZONE)) {
            HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        }
        else if (joystick_actual.eje_y > (2048 + JOYSTICK_DEADZONE)) {
            HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);
            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        }
        else {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        }
    }

    // NOTA: Si el currentState es STATE_OFFLINE o STATE_ONLINE (Tracking activo),
    // la lógica de activación del PWM se gobernará por el error entre target_actual
    // y los encoders AS5600, no por el joystick. (A implementar en la fase 3).
}
