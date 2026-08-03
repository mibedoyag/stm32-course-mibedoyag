/**
 * @file    : motores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de la lógica de movimiento, escalado de pasos y mapeo analógico.
 * Configuración de TIM2 (Azimut), TIM3 (Altitud) y ADC1 con DMA (Joystick).
 */

#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include "stm32f4xx_hal.h"
#include "Proyecto/sensores.h"
#include <math.h>
#include <stdlib.h>

/* Instancias globales de los periféricos.
 * El control absoluto del hardware recae en este módulo, no en el autogenerador del IDE.
 */
TIM_HandleTypeDef htim2; // Timer 2 - PWM Eje Azimut
TIM_HandleTypeDef htim3; // Timer 3 - PWM Eje Altitud
ADC_HandleTypeDef hadc1; // ADC 1 - Lectura del Joystick
DMA_HandleTypeDef hdma_adc1; // DMA para el ADC1

/* Buffer donde el hardware DMA depositará continuamente las lecturas del Joystick (X e Y) */
volatile uint16_t adc_dma_buffer[2] = {2048, 2048};

// NUEVO: Variables para guardar el centro físico real del joystick
static uint16_t joy_centro_x = 2048;
static uint16_t joy_centro_y = 2048;

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


/* Acumuladores de error fraccional para no perder precisión por redondeo */
static float error_acumulado_az = 0.0f;
static float error_acumulado_alt = 0.0f;

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
	HOME_ALIGN_IMU,  //Alineación post finales de carrera con los angulos euler de la IMU
	HOME_WAIT_GOTO,
    HOME_DONE
} HomingState_t;

static HomingState_t estado_homing = HOME_IDLE;

/* Variables lógicas del módulo */
JoystickData_t joystick_actual;
VelocidadModo_t velocidad_actual;

/* Variables de estado interno para la cinemática */
float posicion_actual_az = 0.0f;  // Equivalente a 'preaz' del .ino
float posicion_actual_alt = 0.0f; // Equivalente a 'prealt' del .ino

volatile uint32_t pasos_restantes_az = 0;
volatile uint32_t pasos_restantes_alt = 0;

volatile uint8_t flag_goto_terminado_az = 1;
volatile uint8_t flag_goto_terminado_alt = 1;

/* Frecuencias base para los registros ARR a 16 MHz de reloj (HCLK) */
uint32_t arr_buscar  = 999;   // 1000 Hz (GoTo Rápido)
uint32_t arr_centrar = 3999;  // 250 Hz  (Joystick Normal)
uint32_t arr_guiar   = 19999; // 50 Hz   (Ajuste Fino - Seguimiento Sideral)

/* =========================================================================
 * SISTEMA DE RAMPAS DE ACELERACIÓN (16 MHz BASE)
 * ========================================================================= */
typedef struct {
    uint32_t freq_actual;    // Frecuencia instantánea (Hz)
    uint32_t freq_objetivo;  // Destino (Hz)
    uint32_t paso_acel;      // Agresividad (Hz por cada 5ms). 10 = Suave (0.5s en llegar a max)
    uint8_t  activo;         // 1 = Motor en rampa, 0 = Libre
} MotorRampa_t;

volatile MotorRampa_t rampa_az =  {0, 0, 10, 0};
volatile MotorRampa_t rampa_alt = {0, 0, 10, 0};

// Frecuencias traducidas de tus ARR originales a 16MHz (Prescaler 16)
#define FREQ_BUSCAR   1000 // Equivale a ARR 999
#define FREQ_CENTRAR  250  // Equivale a ARR 3999
#define FREQ_GUIAR    50   // Equivale a ARR 19999
#define FREQ_MINIMA   50   // Velocidad de arranque/frenado seguro

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
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Pin PWM Altitud (TIM3_CH1) -> PA6
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
 * 3. LÓGICA DEL MÓDULO (API PÚBLICA)
 * ========================================================================= */

/**
 * @brief Inicializa el hardware y pone el sistema mecánico en un estado seguro.
 */
void Motores_InitLogica(void) {
    Motores_GPIO_Init();
    Motores_ADC_Init();
    Motores_TIM_Init();

	HAL_Delay(2000);

    joystick_actual.eje_x = 2048;
    joystick_actual.eje_y = 2048;
    velocidad_actual = SPEED_BUSCAR;

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, 2);

	HAL_Delay(100);
	joy_centro_x = adc_dma_buffer[0];
	joy_centro_y = adc_dma_buffer[1];

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
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	__HAL_TIM_SET_AUTORELOAD(&htim3, nuevo_arr);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nuevo_arr / 2);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief Gestiona la aceleración suave de los motores.
 */
static void Motores_ProcesarRampas(void) {
    static uint32_t ultimo_tick_rampa = 0;

    if (HAL_GetTick() - ultimo_tick_rampa < 5) return;
    ultimo_tick_rampa = HAL_GetTick();

    MotorRampa_t *rampas[2] = { (MotorRampa_t*)&rampa_az, (MotorRampa_t*)&rampa_alt };
    TIM_HandleTypeDef *timers[2] = { &htim2, &htim3 };
    volatile uint32_t *pasos[2] = { &pasos_restantes_az, &pasos_restantes_alt };

    for (int i = 0; i < 2; i++) {
        if (rampas[i]->activo) {

            if (*pasos[i] > 0 && *pasos[i] <= 150) {
                rampas[i]->freq_objetivo = FREQ_MINIMA;
            }

            if (rampas[i]->freq_actual < rampas[i]->freq_objetivo) {
                rampas[i]->freq_actual += rampas[i]->paso_acel;
                if (rampas[i]->freq_actual > rampas[i]->freq_objetivo)
                    rampas[i]->freq_actual = rampas[i]->freq_objetivo;
            }
            else if (rampas[i]->freq_actual > rampas[i]->freq_objetivo) {
                if (rampas[i]->freq_actual > rampas[i]->paso_acel + FREQ_MINIMA)
                    rampas[i]->freq_actual -= rampas[i]->paso_acel;
                else
                    rampas[i]->freq_actual = rampas[i]->freq_objetivo;
            }

			if (rampas[i]->freq_actual >= FREQ_MINIMA) {
				uint32_t nuevo_arr = (1000000 / rampas[i]->freq_actual) - 1;
				__HAL_TIM_SET_AUTORELOAD(timers[i], nuevo_arr);
				__HAL_TIM_SET_COMPARE(timers[i], TIM_CHANNEL_1, nuevo_arr / 2);

				if (__HAL_TIM_GET_COUNTER(timers[i]) > nuevo_arr) {
					__HAL_TIM_SET_COUNTER(timers[i], 0);
				}
			}

            if (rampas[i]->freq_actual <= FREQ_MINIMA && rampas[i]->freq_objetivo == 0 && *pasos[i] == 0) {
                HAL_TIM_PWM_Stop(timers[i], TIM_CHANNEL_1);
                rampas[i]->activo = 0;
            }
        }
    }
}

/**
 * @brief Lógica de evaluación periódica del sistema de movimiento con suavizado ADC y velocidad global.
 */
void Motores_UpdateLogica(void) {
	if (estado_homing != HOME_DONE) {
	        Motores_UpdateHoming();
	        return;
	    }

	Motores_ProcesarRampas();

	static SystemState_t estado_anterior = STATE_BOOTING;

	if (currentState != STATE_MANUAL && currentState != STATE_CALIBRACION_FINA) {
	    if (estado_anterior == STATE_MANUAL || estado_anterior == STATE_CALIBRACION_FINA) {
	        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	        rampa_az.activo = 0;
	        rampa_alt.activo = 0;
	    }
	    estado_anterior = currentState;
	    return;
	}

	estado_anterior = currentState;

	static uint16_t filtro_x = 2048;
	static uint16_t filtro_y = 2048;

	filtro_x = (uint16_t) (((uint32_t) filtro_x * 9 + adc_dma_buffer[0]) / 10);
	filtro_y = (uint16_t) (((uint32_t) filtro_y * 9 + adc_dma_buffer[1]) / 10);

	joystick_actual.eje_x = filtro_x;
	joystick_actual.eje_y = filtro_y;

	static uint8_t az_estado = 0;
	static uint8_t alt_estado = 0;

	// ------------------ PARCHE DE SINCRONIZACIÓN FÍSICA (DELTA IMU INVERTIDO) ------------------
	static uint8_t joystick_en_uso = 0;
	static float imu_az_inicio_joy = 0.0f;
	static float imu_alt_inicio_joy = 0.0f;
	static float pos_az_inicio_joy = 0.0f;
	static float pos_alt_inicio_joy = 0.0f;

	if (rampa_az.activo || rampa_alt.activo) {
		if (!joystick_en_uso) {
			joystick_en_uso = 1;
			imu_az_inicio_joy = imu_actual.orientacion_z;
			imu_alt_inicio_joy = imu_actual.inclinacion_y;
			pos_az_inicio_joy = posicion_actual_az;
			pos_alt_inicio_joy = posicion_actual_alt;
		} else {
			if (rampa_az.activo) {
				// Invertimos la polaridad del delta de IMU para alinearla con los motores
				float delta_az_imu = imu_az_inicio_joy
						- imu_actual.orientacion_z;
				if (delta_az_imu > 180.0f)
					delta_az_imu -= 360.0f;
				if (delta_az_imu < -180.0f)
					delta_az_imu += 360.0f;

				float nueva_pos_az = pos_az_inicio_joy + delta_az_imu;
				if (nueva_pos_az >= 360.0f)
					nueva_pos_az -= 360.0f;
				if (nueva_pos_az < 0.0f)
					nueva_pos_az += 360.0f;

				posicion_actual_az = nueva_pos_az;
			}

			if (rampa_alt.activo) {
				// Invertimos la polaridad del delta en Altitud
				float delta_alt_imu = imu_alt_inicio_joy
						- imu_actual.inclinacion_y;
				posicion_actual_alt = pos_alt_inicio_joy + delta_alt_imu;
			}
		}
	} else {
		joystick_en_uso = 0;

	}
	// ---------------------------------------------------------------------------------

	uint32_t obj_freq = (velocidad_actual == SPEED_BUSCAR) ? FREQ_BUSCAR : ((velocidad_actual == SPEED_CENTRAR) ? FREQ_CENTRAR : FREQ_GUIAR);

	// ---------------------------------------------------------
	// EVALUACIÓN EJE X (AZIMUT - TIM2)
	// ---------------------------------------------------------
	if (joystick_actual.eje_x < (joy_centro_x - JOYSTICK_DEADZONE)) {
		if (az_estado != 1) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);
			rampa_az.freq_actual = FREQ_MINIMA;
			rampa_az.activo = 1;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			az_estado = 1;
		}
		rampa_az.freq_objetivo = obj_freq;
	} else if (joystick_actual.eje_x > (joy_centro_x + JOYSTICK_DEADZONE)) {
		if (az_estado != 2) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);
			rampa_az.freq_actual = FREQ_MINIMA;
			rampa_az.activo = 1;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			az_estado = 2;
		}
		rampa_az.freq_objetivo = obj_freq;
	} else {
		if (az_estado != 0) {
			rampa_az.freq_objetivo = 0;
			az_estado = 0;
		}
	}

	// ---------------------------------------------------------
	// EVALUACIÓN EJE Y (ALTITUD - TIM3)
	// ---------------------------------------------------------
	if (joystick_actual.eje_y < (joy_centro_y - JOYSTICK_DEADZONE)) {
		if (alt_estado != 1) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
			rampa_alt.freq_actual = FREQ_MINIMA;
			rampa_alt.activo = 1;
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			alt_estado = 1;
		}
		rampa_alt.freq_objetivo = obj_freq;
	} else if (joystick_actual.eje_y > (joy_centro_y + JOYSTICK_DEADZONE)) {
		if (alt_estado != 2) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);
			rampa_alt.freq_actual = FREQ_MINIMA;
			rampa_alt.activo = 1;
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			alt_estado = 2;
		}
		rampa_alt.freq_objetivo = obj_freq;
	} else {
		if (alt_estado != 0) {
			rampa_alt.freq_objetivo = 0;
			alt_estado = 0;
		}
	}
}

/**
 * @brief Algoritmo principal de movimiento GoTo (Equivalente a 'steptep' mejorado)
 * @param azimut_target  Ángulo objetivo en Azimut (0 a 360)
 * @param altitud_target Ángulo objetivo en Altitud (-90 a 90)
 */
void Motores_Apuntar(float azimut_target, float altitud_target) {
    float delta_az = azimut_target - posicion_actual_az;
    float delta_alt = altitud_target - posicion_actual_alt;

    if (delta_az > 180.0f)  delta_az -= 360.0f;
    if (delta_az < -180.0f) delta_az += 360.0f;

    /* INVERSIÓN EXCLUSIVA PARA GOTO (ONLINE / OFFLINE) */
    if (delta_az >= 0) HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);

    if (delta_alt >= 0) HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);

    pasos_restantes_az = (uint32_t)(fabsf(delta_az) * PULSOS_POR_GRADO_AZIMUT);
    pasos_restantes_alt = (uint32_t)(fabsf(delta_alt) * PULSOS_POR_GRADO_ALTITUD);

    posicion_actual_az = azimut_target;
    posicion_actual_alt = altitud_target;

    if (pasos_restantes_az > 0) {
        flag_goto_terminado_az = 0;
        rampa_az.freq_actual = FREQ_MINIMA;
        rampa_az.freq_objetivo = FREQ_BUSCAR;
        rampa_az.activo = 1;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    }

    if (pasos_restantes_alt > 0) {
        flag_goto_terminado_alt = 0;
        rampa_alt.freq_actual = FREQ_MINIMA;
        rampa_alt.freq_objetivo = FREQ_BUSCAR;
        rampa_alt.activo = 1;
        __HAL_TIM_SET_COUNTER(&htim3, 0);
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
    rampa_az.activo = 0;
    rampa_alt.activo = 0;
}

static uint32_t temporizador_homing = 0;
static uint32_t tiempo_inicio_homing = 0;

void Motores_IniciarHoming(void) {
	flag_homing_ok = 0;
	estado_homing = HOME_ALT_FAST;
	temporizador_homing = HAL_GetTick();
	tiempo_inicio_homing = HAL_GetTick();

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
	__HAL_TIM_SET_AUTORELOAD(&htim3, arr_centrar);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_centrar / 2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief Sub-máquina de estados no bloqueante para el Homing Híbrido.
 */
void Motores_UpdateHoming(void) {
	if (estado_homing == HOME_IDLE || estado_homing == HOME_DONE)
		return;

	uint8_t limit_alt = HAL_GPIO_ReadPin(LIMIT_ALT_PORT, LIMIT_ALT_PIN);

	switch (estado_homing) {

	/* --- SECUENCIA EJE ALTITUD (Interruptor Físico) --- */
	case HOME_ALT_FAST:
		if (limit_alt == GPIO_PIN_RESET && (HAL_GetTick() - temporizador_homing > 200)) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			temporizador_homing = HAL_GetTick();
			estado_homing = HOME_ALT_BACKOFF;
		}
		break;

	case HOME_ALT_BACKOFF:
		if (limit_alt == GPIO_PIN_SET && (HAL_GetTick() - temporizador_homing > 200)) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);

			__HAL_TIM_SET_AUTORELOAD(&htim3, arr_guiar);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr_guiar / 2);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

			temporizador_homing = HAL_GetTick();
			estado_homing = HOME_ALT_SLOW;
		}
		break;

	case HOME_ALT_SLOW:
		if (limit_alt == GPIO_PIN_RESET && (HAL_GetTick() - temporizador_homing > 200)) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_Delay(300); // Estabilización mecánica

			posicion_actual_alt = imu_actual.inclinacion_y;
			estado_homing = HOME_ALIGN_IMU;
		}
		break;

	/* --- ALINEACIÓN POST FINALES DE CARRERA HACIA EL CERO ABSOLUTO --- */
	case HOME_ALIGN_IMU: {
		uint32_t tiempo_transcurrido = HAL_GetTick() - tiempo_inicio_homing;
		uint8_t condicion_mag = (imu_actual.estado_calibracion == 3);
		uint8_t timeout_cumplido = (tiempo_transcurrido > 30000); // 30s timeout

		if (condicion_mag || timeout_cumplido) {
			if (condicion_mag) {
				posicion_actual_az = imu_actual.orientacion_z;
			} else {
				posicion_actual_az = 0.0f;
			}
			posicion_actual_alt = imu_actual.inclinacion_y;

			estado_homing = HOME_WAIT_GOTO;

			// 1. Ajustar velocidad
			Motores_SetVelocidadGlobal(SPEED_CENTRAR);

			// 2. Calcular pasos absolutos necesarios para volver a (0.0, 0.0)
			float delta_az = 0.0f - posicion_actual_az;
			float delta_alt = 0.0f - posicion_actual_alt;

			if (delta_az > 180.0f)  delta_az -= 360.0f;
			if (delta_az < -180.0f) delta_az += 360.0f;

			pasos_restantes_az = (uint32_t)(fabsf(delta_az) * PULSOS_POR_GRADO_AZIMUT);
			pasos_restantes_alt = (uint32_t)(fabsf(delta_alt) * PULSOS_POR_GRADO_ALTITUD);

			// 3. DIRECCIÓN DIRECTA DE HOMING (Fuerza al motor de Altitud a SUBIR alejándose del switch)
			HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, (delta_az >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET); // GPIO_PIN_SET obliga a subir mecánicamente

			// 4. Actualizar memoria de posición a 0.0f
			posicion_actual_az = 0.0f;
			posicion_actual_alt = 0.0f;

			// 5. Arrancar temporizadores
			if (pasos_restantes_az > 0) {
				flag_goto_terminado_az = 0;
				__HAL_TIM_SET_COUNTER(&htim2, 0);
				HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
			} else {
				flag_goto_terminado_az = 1;
			}

			if (pasos_restantes_alt > 0) {
				flag_goto_terminado_alt = 0;
				__HAL_TIM_SET_COUNTER(&htim3, 0);
				HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
			} else {
				flag_goto_terminado_alt = 1;
			}
		}
		break;
	}

	case HOME_WAIT_GOTO:
		if (flag_goto_terminado_az && flag_goto_terminado_alt) {
			posicion_actual_az = 0.0f;
			posicion_actual_alt = 0.0f;

			Motores_SetVelocidadGlobal(SPEED_BUSCAR);
			flag_homing_ok = 1; // Liberar menú principal
			estado_homing = HOME_DONE;
		}
		break;

	case HOME_DONE:
		break;

	default:
		break;
	}
}

/**
 * @brief Ejecuta el movimiento milimétrico compensando la rotación de la Tierra.
 */
void Motores_PasoSideral(float az_nuevo, float alt_nuevo) {
    float delta_az = az_nuevo - posicion_actual_az;
    float delta_alt = alt_nuevo - posicion_actual_alt;

    if (delta_az > 180.0f) delta_az -= 360.0f;
    if (delta_az < -180.0f) delta_az += 360.0f;

    float pasos_teoricos_az = delta_az * PULSOS_POR_GRADO_AZIMUT;
    float pasos_teoricos_alt = delta_alt * PULSOS_POR_GRADO_ALTITUD;

    error_acumulado_az += pasos_teoricos_az;
    error_acumulado_alt += pasos_teoricos_alt;

    int32_t pasos_a_dar_az = (int32_t)error_acumulado_az;
    int32_t pasos_a_dar_alt = (int32_t)error_acumulado_alt;

    error_acumulado_az -= (float)pasos_a_dar_az;
    error_acumulado_alt -= (float)pasos_a_dar_alt;

    /* INVERSIÓN EXCLUSIVA PARA TRACKING SIDERAL */
    if (pasos_a_dar_az > 0) HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_RESET);
    else if (pasos_a_dar_az < 0) HAL_GPIO_WritePin(AZ_DIR_PORT, AZ_DIR_PIN, GPIO_PIN_SET);

    if (pasos_a_dar_alt > 0) HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_RESET);
    else if (pasos_a_dar_alt < 0) HAL_GPIO_WritePin(ALT_DIR_PORT, ALT_DIR_PIN, GPIO_PIN_SET);

    posicion_actual_az = az_nuevo;
    posicion_actual_alt = alt_nuevo;

    Motores_SetVelocidadGlobal(SPEED_GUIAR);

    pasos_restantes_az += labs(pasos_a_dar_az);
    pasos_restantes_alt += labs(pasos_a_dar_alt);

    if (pasos_restantes_az > 0) {
        flag_goto_terminado_az = 0;
        rampa_az.freq_actual = FREQ_GUIAR;
        rampa_az.freq_objetivo = FREQ_GUIAR;
        rampa_az.activo = 1;
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    }
    if (pasos_restantes_alt > 0) {
        flag_goto_terminado_alt = 0;
        rampa_alt.freq_actual = FREQ_GUIAR;
        rampa_alt.freq_objetivo = FREQ_GUIAR;
        rampa_alt.activo = 1;
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    }
}

/* =========================================================================
 * CALLBACKS DE INTERRUPCIÓN (HAL)
 * ========================================================================= */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (pasos_restantes_az > 0) {
            pasos_restantes_az--;
            if (pasos_restantes_az == 0) {
                HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
                flag_goto_terminado_az = 1;
            }
        }
    }
    else if (htim->Instance == TIM3) {
        if (pasos_restantes_alt > 0) {
            pasos_restantes_alt--;
            if (pasos_restantes_alt == 0) {
                HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
                flag_goto_terminado_alt = 1;
            }
        }
    }
}
