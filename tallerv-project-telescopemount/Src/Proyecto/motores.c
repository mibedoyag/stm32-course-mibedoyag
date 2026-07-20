/**
 * @file    : motores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de la lógica de movimiento, escalado de pasos y mapeo analógico.
 * Configuración de TIM2 (Azimut), TIM3 (Altitud) y ADC1 con DMA (Joystick).
 */

#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include "stm32f4xx_hal.h"

/* Instancias globales de los periféricos.
 * El control absoluto del hardware recae en este módulo, no en el autogenerador del IDE.
 */
TIM_HandleTypeDef htim2; // Timer 2 - PWM Eje Azimut
TIM_HandleTypeDef htim3; // Timer 3 - PWM Eje Altitud
ADC_HandleTypeDef hadc1; // ADC 1 - Lectura del Joystick
DMA_HandleTypeDef hdma_adc1; // DMA para el ADC1

/* Buffer donde el hardware DMA depositará continuamente las lecturas del Joystick (X e Y) */
volatile uint16_t adc_dma_buffer[2] = {2048, 2048};

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
 * 1. CONFIGURACIÓN DE HARDWARE
 * ========================================================================= */

/**
 * @brief Configuración de los pines GPIO para Dirección y Timers.
 */
static void Motores_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Pines de Dirección (Salida Digital) -> PA4 y PA5
    GPIO_InitStruct.Pin = AZ_DIR_PIN | ALT_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. Pin PWM Azimut (TIM2_CH1) -> PA0
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Pin PWM Altitud (TIM3_CH1) -> PA6
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Configuración del ADC1 en modo Scan Continuo con DMA para PA1 y PA2.
 */
static void Motores_ADC_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE(); // El ADC1 en el F411 utiliza el DMA2

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar PA1 (IN1, Eje X) y PA2 (IN2, Eje Y) en modo analógico
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- Configuración del DMA (Stream 0, Canal 0) --- */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE; // El registro del ADC es fijo
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;     // Avanzar memoria para X y luego Y
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 16 bits
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 16 bits
    hdma_adc1.Init.Mode = DMA_CIRCULAR;          // Buffer circular automático
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* --- Configuración del ADC1 --- */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;                     // Activar lectura multicanal
    hadc1.Init.ContinuousConvMode = ENABLE;               // Nunca detenerse
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;                       // Leeremos Eje X y Eje Y
    hadc1.Init.DMAContinuousRequests = ENABLE;            // Conectar con el DMA activamente
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);

    /* --- Configurar Canales Secuenciales --- */
    ADC_ChannelConfTypeDef sConfig = {0};

    // Rango 1: Canal 1 (PA1 - Eje X)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Rango 2: Canal 2 (PA2 - Eje Y)
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Configurar interrupción del DMA (útil si hay errores de transmisión)
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
    htim2.Init.Prescaler = 16 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr_buscar;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = arr_buscar / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    /* --- Configuración TIM3 (Altitud) --- */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16 - 1;
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
 * @brief Lee los dos ejes del joystick.
 * Con el DMA activo, ya no hay tiempos de espera (Polling). Solo copiamos datos.
 */
static void Motores_LeerJoystick(void) {
    // El hardware escribe constantemente en adc_dma_buffer de fondo.
    joystick_actual.eje_x = adc_dma_buffer[0];
    joystick_actual.eje_y = adc_dma_buffer[1];
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

    // 2. Inicialización segura del Joystick en la zona muerta
    joystick_actual.eje_x = 2048;
    joystick_actual.eje_y = 2048;
    velocidad_actual = SPEED_BUSCAR;

    // 3. Asegurar que los motores arrancan totalmente apagados
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

    // 4. Iniciar la recolección asíncrona de datos del Joystick vía DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 2);
}

/**
 * @brief Modifica el registro ARR (Auto-Reload Register) en tiempo real
 */
void Motores_SetVelocidadGlobal(VelocidadModo_t nueva_velocidad) {
    velocidad_actual = nueva_velocidad;
    uint32_t nuevo_arr = arr_buscar;

    switch(velocidad_actual) {
        case SPEED_BUSCAR:  nuevo_arr = arr_buscar;  break;
        case SPEED_CENTRAR: nuevo_arr = arr_centrar; break;
        case SPEED_GUIAR:   nuevo_arr = arr_guiar;   break;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim2, nuevo_arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, nuevo_arr / 2);

    __HAL_TIM_SET_AUTORELOAD(&htim3, nuevo_arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nuevo_arr / 2);
}

/**
 * @brief Lógica de evaluación periódica del sistema de movimiento.
 */
void Motores_UpdateLogica(void) {

    // [IMPORTANTE: Se asume que currentState se lee globalmente o se inyecta.
    // Para compilar este fragmento aislado, omito el if(currentState == STATE_MANUAL)
    // y proceso directamente. Ajusta esto según cómo inyectes la variable de estado].

    Motores_LeerJoystick();

    // ---------------------------------------------------------
    // EVALUACIÓN EJE X (AZIMUT - TIM2)
    // ---------------------------------------------------------
    if (joystick_actual.eje_x < (2048 - JOYSTICK_DEADZONE)) {
        HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
    else if (joystick_actual.eje_x > (2048 + JOYSTICK_DEADZONE)) {
        HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    }
    else {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    }

    // ---------------------------------------------------------
    // EVALUACIÓN EJE Y (ALTITUD - TIM3)
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

