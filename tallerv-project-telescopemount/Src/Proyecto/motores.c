/**
 * @file    : motores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de la lógica de movimiento, escalado de pasos y mapeo analógico.
 * Configuración de TIM2 (Azimut), TIM3 (Altitud) y ADC1 con DMA (Joystick).
 */

#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include "stm32f4xx_hal.h"
#include <math.h>

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

/* Definición de Pines Físicos para Finales de Carrera (Compatibles Nucleo/Blackpill) */
#define LIMIT_AZ_PORT  GPIOB
#define LIMIT_AZ_PIN   GPIO_PIN_1  // PB1 - Final de Carrera Azimut (Filtro RC -> GND)
#define LIMIT_ALT_PORT GPIOB
#define LIMIT_ALT_PIN  GPIO_PIN_2  // PB2 - Final de Carrera Altitud (Filtro RC -> GND)

/* =========================================================================
 * VARIABLES DE LA SUB-MÁQUINA DE HOMING
 * ========================================================================= */
volatile uint8_t flag_homing_ok = 0; // Inicia en 0 (Descalibrado)

typedef enum {
    HOME_IDLE = 0,
    HOME_AZ_FAST,     // Acercamiento rápido Eje Azimut
    HOME_AZ_BACKOFF,  // Retroceso para liberar el contacto
    HOME_AZ_SLOW,     // Acercamiento micrométrico Eje Azimut
    HOME_ALT_FAST,    // Acercamiento rápido Eje Altitud
    HOME_ALT_BACKOFF, // Retroceso
    HOME_ALT_SLOW,    // Acercamiento micrométrico Eje Altitud
    HOME_DONE
} HomingState_t;

static HomingState_t estado_homing = HOME_IDLE;

/* Variables lógicas del módulo */
JoystickData_t joystick_actual;
VelocidadModo_t velocidad_actual;

/* Variables de estado interno para la cinemática */
static float posicion_actual_az = 0.0f;  // Equivalente a 'preaz' del .ino
static float posicion_actual_alt = 0.0f; // Equivalente a 'prealt' del .ino

volatile uint32_t pasos_restantes_az = 0;
volatile uint32_t pasos_restantes_alt = 0;

volatile uint8_t flag_goto_terminado_az = 1;
volatile uint8_t flag_goto_terminado_alt = 1;

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

    // 4. Pines de los Finales de Carrera (Entradas con Pull-Up)
    __HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = LIMIT_AZ_PIN | LIMIT_ALT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief Configuración del ADC1 en modo Scan Continuo con DMA para PA1 y PA2.
 */
static void Motores_ADC_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE(); // El ADC1 en el F411 utiliza el DMA2

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configurar PA1 (IN1, Eje X) y PA7 (IN2, Eje Y) en modo analógico
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_7;
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

    // Rango 2: Canal 7 (PA2 - Eje Y)
    sConfig.Channel = ADC_CHANNEL_7;
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

    // Habilitar Interrupciones globales para contar los pasos de los motores
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

    // 5. Arrancar el proceso de calibración absoluta al finalizar la inicialización
    Motores_IniciarHoming();
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

	// PROTECCIÓN DE SISTEMA: Si el Homing está activo, delegar el control y salir
	if (estado_homing != HOME_DONE) {
		Motores_UpdateHoming();
		return;
	}

	// Si ya terminó el homing, el usuario recupera el control analógico
	Motores_LeerJoystick();

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

/**
 * @brief Algoritmo principal de movimiento GoTo (Equivalente a 'steptep' mejorado)
 * @param azimut_target  Ángulo objetivo en Azimut (0 a 360)
 * @param altitud_target Ángulo objetivo en Altitud (-90 a 90)
 */
void Motores_Apuntar(float azimut_target, float altitud_target) {

    // 1. Calcular deltas (Diferencia de ángulos)
    float delta_az = azimut_target - posicion_actual_az;
    float delta_alt = altitud_target - posicion_actual_alt;

    // 2. Optimización: Buscar el camino más corto en Azimut (Círculo de 360°)
    if (delta_az > 180.0f)  delta_az -= 360.0f;
    if (delta_az < -180.0f) delta_az += 360.0f;

    // 3. Determinar dirección de los ejes (Set/Reset de los pines DIR)
    if (delta_az >= 0) HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);

    if (delta_alt >= 0) HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);

    // 4. Convertir grados a número de pasos absolutos y cargar los contadores
    pasos_restantes_az = (uint32_t)(fabsf(delta_az) * PULSOS_POR_GRADO_AZIMUT);
    pasos_restantes_alt = (uint32_t)(fabsf(delta_alt) * PULSOS_POR_GRADO_ALTITUD);

    // 5. Actualizar la memoria del telescopio para el próximo viaje
    posicion_actual_az = azimut_target;
    posicion_actual_alt = altitud_target;

    // 6. Arrancar los Timers en modo INTERRUPCIÓN (No bloqueante)
    if (pasos_restantes_az > 0) {
        flag_goto_terminado_az = 0; // Bajamos la bandera
        HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    }

    if (pasos_restantes_alt > 0) {
        flag_goto_terminado_alt = 0; // Bajamos la bandera
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    }
}

/**
 * @brief Detiene los motores inmediatamente en caso de emergencia o cancelación
 */
void Motores_DetenerGoTo(void) {
    HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    pasos_restantes_az = 0;
    pasos_restantes_alt = 0;
    flag_goto_terminado_az = 1;
    flag_goto_terminado_alt = 1;
}


/**
 * @brief Prepara las variables y arranca la calibración del eje Azimut.
 */
void Motores_IniciarHoming(void) {
    flag_homing_ok = 0;
    estado_homing = HOME_AZ_FAST;

    // Configurar dirección hacia el final de carrera (Asumimos RESET es avanzar)
    HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);

    // Configurar velocidad rápida y arrancar generador de PWM (sin interrupción)
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr_centrar);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr_centrar / 2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief Sub-máquina de estados no bloqueante para el Homing de doble etapa.
 */
void Motores_UpdateHoming(void) {
    if (estado_homing == HOME_IDLE || estado_homing == HOME_DONE) return;

    // Leemos el estado eléctrico de ambos sensores (0V = Tocado)
    uint8_t limit_az = HAL_GPIO_ReadPin(LIMIT_AZ_PORT, LIMIT_AZ_PIN);
    uint8_t limit_alt = HAL_GPIO_ReadPin(LIMIT_ALT_PORT, LIMIT_ALT_PIN);

    switch (estado_homing) {

        /* --- SECUENCIA EJE AZIMUT --- */
        case HOME_AZ_FAST:
            if (limit_az == GPIO_PIN_RESET) { // Contacto detectado a alta velocidad
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Freno de emergencia
                HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET); // Reversa
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                estado_homing = HOME_AZ_BACKOFF;
            }
            break;

        case HOME_AZ_BACKOFF:
            if (limit_az == GPIO_PIN_SET) { // El switch se liberó eléctricamente
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET); // Hacia adelante

                // Disminuimos la velocidad al mínimo absoluto (Ajuste Fino)
                __HAL_TIM_SET_AUTORELOAD(&htim2, arr_guiar);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr_guiar / 2);
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

                estado_homing = HOME_AZ_SLOW;
            }
            break;

        case HOME_AZ_SLOW:
            if (limit_az == GPIO_PIN_RESET) { // Contacto micrométrico detectado
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                posicion_actual_az = 0.0f; // ¡Azimut Calibrado! Cero absoluto fijado.

                // Arrancamos el mismo proceso para la Altitud
                HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
                __HAL_TIM_SET_AUTORELOAD(&htim3, arr_buscar);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_buscar / 2);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                estado_homing = HOME_ALT_FAST;
            }
            break;

        /* --- SECUENCIA EJE ALTITUD --- */
        case HOME_ALT_FAST:
            if (limit_alt == GPIO_PIN_RESET) {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                estado_homing = HOME_ALT_BACKOFF;
            }
            break;

        case HOME_ALT_BACKOFF:
            if (limit_alt == GPIO_PIN_SET) {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);

                __HAL_TIM_SET_AUTORELOAD(&htim3, arr_guiar);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_guiar / 2);
                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

                estado_homing = HOME_ALT_SLOW;
            }
            break;

        case HOME_ALT_SLOW:
            if (limit_alt == GPIO_PIN_RESET) {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                posicion_actual_alt = 0.0f; // ¡Altitud Calibrada!

                // ¡Homing Exitoso! Levantamos la bandera para la pantalla LCD
                flag_homing_ok = 1;
                estado_homing = HOME_DONE;
            }
            break;

        default:
            break;
    }
}


/* =========================================================================
 * CALLBACKS DE INTERRUPCIÓN (HAL)
 * ========================================================================= */
/**
 * @brief Se ejecuta cada vez que el Timer PWM termina un pulso.
 * @note  Esta lógica DEBE ir aquí para precisión micrométrica de hardware.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

    if (htim->Instance == TIM2) { // Pulso del motor de Azimut
        if (pasos_restantes_az > 0) {
            pasos_restantes_az--;
            if (pasos_restantes_az == 0) {
                // Se acabó el viaje de este eje: Frenamos el hardware inmediatamente
                HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
                // Levantamos la bandera para que la FSM tome decisiones
                flag_goto_terminado_az = 1;
            }
        }
    }

    else if (htim->Instance == TIM3) { // Pulso del motor de Altitud
        if (pasos_restantes_alt > 0) {
            pasos_restantes_alt--;
            if (pasos_restantes_alt == 0) {
                HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
                flag_goto_terminado_alt = 1;
            }
        }
    }
}
