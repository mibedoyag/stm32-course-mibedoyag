/**
 * @file    : sensores.c
 * @brief   : Implementación BNO055 (UART6), LCD (I2C1) y NEO-M8N (UART1 + DMA).
 */
#include "Proyecto/sensores.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

extern uint32_t currentState;
#define STATUS_STATE_ERROR 3

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

char gps_dma_buffer[GPS_BUFFER_SIZE] = {0};
char gps_rx_buffer[GPS_BUFFER_SIZE] = {0};

DatosIMU_t imu_actual = {0.0f, 0.0f, 0.0f, 0};
DatosGPS_t gps_actual = {0.0f, 0.0f, 0.0f, -6.5f, 0, 0, 0};

/* =========================================================================
 * 1. CONFIGURACIÓN DE HARDWARE
 * ========================================================================= */
static void Sensores_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

static void Sensores_UART1_Init(void) {
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

static void Sensores_UART6_Init(void) {
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PA11 = TX6, PA12 = RX6
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200; // Baudrate oficial del BNO055
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);
}

/* =========================================================================
 * 2. MÉTODOS DE COMUNICACIÓN UART ROBUSTOS
 * ========================================================================= */
static uint8_t IMU_EscribirRegistro(uint8_t registro, uint8_t valor) {
    uint8_t cmd[5] = {0xAA, 0x00, registro, 0x01, valor};
    uint8_t respuesta[2] = {0, 0};

    // Purgar buffer RX por si hay ruido o respuestas previas
    __HAL_UART_FLUSH_DRREGISTER(&huart6);

    HAL_UART_Transmit(&huart6, cmd, 5, 20);

    if (HAL_UART_Receive(&huart6, respuesta, 2, 50) == HAL_OK) {
        // 0xEE es cabecera de estado, 0x01 es escritura exitosa
        if (respuesta[0] == 0xEE && respuesta[1] == 0x01) {
            return 1; // Éxito
        }
    }
    return 0; // Fallo
}

// NUEVO: Función para leer el estado interno del procesador BNO055
static uint8_t IMU_LeerCalibracion(void) {
    uint8_t cmd_leer[4] = {0xAA, 0x01, 0x35, 0x01};
    uint8_t respuesta[3] = {0};

    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    HAL_UART_Transmit(&huart6, cmd_leer, 4, 10);

    if (HAL_UART_Receive(&huart6, respuesta, 3, 20) == HAL_OK) {
        if (respuesta[0] == 0xBB && respuesta[1] == 0x01) {
            return respuesta[2]; // Retorna el byte de calibración
        }
    }
    return 0;
}



/* =========================================================================
 * 3. LÓGICA DE INICIALIZACIÓN
 * ========================================================================= */
void Sensores_InitLogica(void) {
    Sensores_I2C1_Init();
    Sensores_UART1_Init();
    Sensores_UART6_Init();

    memset(gps_dma_buffer, 0, GPS_BUFFER_SIZE);

    uint8_t inicializacion_ok = 0;
    uint32_t intentos = 0;

    while (!inicializacion_ok && intentos < 3) {
        HAL_Delay(800 + (intentos * 300));

        // Soft-Reset para limpiar el BNO055
        IMU_EscribirRegistro(0x3F, 0x20);
        HAL_Delay(800);

        IMU_EscribirRegistro(0x07, 0x00); // Página 0
        HAL_Delay(30);
        IMU_EscribirRegistro(0x3D, 0x00); // Modo Config
        HAL_Delay(40);

        // Oscilador interno para clones GY-BNO055
        IMU_EscribirRegistro(0x3F, 0x00);
        HAL_Delay(30);

        // Arrancamos en modo Fusión NDOF para permitir la lectura del magnetómetro si llega a 3
        if (IMU_EscribirRegistro(0x3D, 0x0C)) {
            inicializacion_ok = 1;
            break;
        }

        intentos++;
        HAL_Delay(100);
    }

    if (!inicializacion_ok) {
        currentState = STATUS_STATE_ERROR;
    }

    // Iniciar GPS
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)gps_dma_buffer, GPS_BUFFER_SIZE);
}

/* =========================================================================
 * 4. PROCESAMIENTO DE DATOS EN LAZO
 * ========================================================================= */
/**
 * @brief Convierte grados/minutos NMEA a grados decimales usando doble precisión.
 * @note Se pasa el string directamente para aprovechar atof() que devuelve un double.
 */
static float ConvertirCoordenada(const char* nmea_str, char direccion) {
    if (nmea_str == NULL || strlen(nmea_str) == 0) return 0.0f;

    double nmea_raw = atof(nmea_str); // Mantiene todos los decimales
    int grados = (int)(nmea_raw / 100.0);
    double minutos = nmea_raw - (grados * 100.0);
    double decimal = (double)grados + (minutos / 60.0);

    if (direccion == 'S' || direccion == 'W') {
        decimal *= -1.0;
    }

    return (float)decimal; // Retornamos a float para la FPU de astronomia.c
}
/**
 * @brief Extrae datos de la trama GPRMC de forma robusta, soportando campos vacíos.
 * Permite capturar la hora del RTC interno del GPS sin necesidad de satélites.
 */
// Variables estáticas para el candado inteligente de posición propuesto
volatile uint8_t gps_coordenadas_fijadas = 0;
static uint8_t gps_lecturas_validas = 0;

static void Parser_NMEA_GPRMC(char *trama_limpia) {
    char *campos[13] = {NULL};
    uint8_t num_campos = 0;

    // 1. Fragmentación segura de la trama NMEA
    campos[num_campos++] = trama_limpia;
    for (int i = 0; trama_limpia[i] != '\0'; i++) {
        if (trama_limpia[i] == ',' || trama_limpia[i] == '*') {
            trama_limpia[i] = '\0';
            if (num_campos < 13) {
                campos[num_campos++] = &trama_limpia[i + 1];
            }
        }
    }

    if (num_campos >= 10) {

		// 2. HORA Y FECHA (Se actualizan continuamente)
		if (strlen(campos[1]) >= 6) {
			double raw_time = atof(campos[1]);
			int hh = (int) (raw_time / 10000.0);
			int mm = (int) ((raw_time - (hh * 10000.0)) / 100.0);
			double ss = raw_time - (hh * 10000.0) - (mm * 100.0);

			// Guardamos en horas decimales perfectas para astronomia.c
			gps_actual.ut_horas = (float) hh + ((float) mm / 60.0f)
					+ ((float) ss / 3600.0f);
		}

		if (strlen(campos[9]) == 6) {
			uint32_t fecha = atoi(campos[9]);
			gps_actual.dia = fecha / 10000;
			gps_actual.mes = (fecha % 10000) / 100;
			gps_actual.anio = (fecha % 100) + 2000;
		}

        // 3. LATITUD Y LONGITUD (Bloqueo Estático para Telescopio)
        if (campos[2][0] == 'A') { // Si hay Fix válido de los satélites

            if (gps_coordenadas_fijadas == 0) {
                gps_lecturas_validas++; // Filtro de confianza

                // Esperamos 5 tramas seguidas para asegurar que la señal es estable
                if (gps_lecturas_validas > 5) {
                    if (strlen(campos[3]) > 0) {
                        gps_actual.latitud = ConvertirCoordenada(campos[3], campos[4][0]);
                    }
                    if (strlen(campos[5]) > 0) {
                        gps_actual.longitud = ConvertirCoordenada(campos[5], campos[6][0]);
                    }
                    // ¡Candado activado! No volveremos a sobreescribir la posición.
                    gps_coordenadas_fijadas = 1;
                }
            }
        } else {
            // Si perdemos señal antes de fijarla, reiniciamos el contador de seguridad
            gps_lecturas_validas = 0;
        }
    }
}


/* =========================================================================
 * 4. PROCESAMIENTO DE DATOS EN LAZO
 * ========================================================================= */

// Variables estáticas para la persecución del DMA (Ring Buffer)
static uint32_t rx_tail = 0;
static char linea_actual[120];
static uint8_t indice_linea = 0;
void Sensores_ProcesarDatos(void) {
	// A. PROCESAMIENTO GPS (DMA Circular)
	uint32_t rx_head = GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

	while (rx_tail != rx_head) {
		char c = gps_dma_buffer[rx_tail];
		rx_tail = (rx_tail + 1) % GPS_BUFFER_SIZE;

		if (c == '$') {
			indice_linea = 0;
		}

		if (indice_linea < sizeof(linea_actual) - 1) {
			linea_actual[indice_linea++] = c;
		}

		if (c == '\n') {
			linea_actual[indice_linea] = '\0';
			if (strncmp(linea_actual, "$GPRMC", 6) == 0 || strncmp(linea_actual, "$GNRMC", 6) == 0) {
				Parser_NMEA_GPRMC(linea_actual);
			}
		}
	}

	// B. PROCESAMIENTO IMU (Norte Físico + Odometría Relativa Anti-Saltos)
	static uint32_t ultimo_tick_imu = 0;

	if (HAL_GetTick() - ultimo_tick_imu >= 20) { // 50 Hz
		ultimo_tick_imu = HAL_GetTick();

		uint8_t cmd_leer[4] = {0xAA, 0x01, 0x1A, 0x06};
		uint8_t respuesta[8] = {0};

		__HAL_UART_FLUSH_DRREGISTER(&huart6);

		if (HAL_UART_Transmit(&huart6, cmd_leer, 4, 10) == HAL_OK) {
			if (HAL_UART_Receive(&huart6, respuesta, 8, 20) == HAL_OK) {
				if (respuesta[0] == 0xBB && respuesta[1] == 0x06) {

					int16_t yaw_raw   = (int16_t)((respuesta[3] << 8) | respuesta[2]);
					int16_t roll_raw  = (int16_t)((respuesta[5] << 8) | respuesta[4]);
					int16_t pitch_raw = (int16_t)((respuesta[7] << 8) | respuesta[6]);

					float yaw_actual_imu = (float)yaw_raw / 16.0f;

					// --- LÓGICA DE FIJACIÓN DE NORTE Y RELATIVIDAD ---
					static uint8_t modo_fijado = 0;
					static uint8_t es_absoluto = 0;
					static float yaw_referencia_imu = 0.0f;
					static float z_acumulado_relativo = 0.0f; // Arranca en 0.0 (Norte Físico)

					// 1. En la primerísima lectura válida, elegimos la estrategia de trabajo
					if (!modo_fijado) {
						uint8_t byte_calib = IMU_LeerCalibracion();
						imu_actual.estado_calibracion = byte_calib & 0x03;

						yaw_referencia_imu = yaw_actual_imu;

						if (imu_actual.estado_calibracion > 0) {
							// Caso A: La brújula está calibrada -> Usamos Norte Magnético Real
							es_absoluto = 1;
						} else {
							// Caso B: Brújula ciega -> Asumimos Norte Físico Manual (0.0 grados)
							es_absoluto = 0;
							z_acumulado_relativo = 0.0f;
						}
						modo_fijado = 1;
					}

					// 2. Procesamiento de ángulo según la estrategia elegida
					if (es_absoluto) {
						// Modo Absoluto (Lectura directa de brújula + declinación)
						float azimut_verdadero = yaw_actual_imu + gps_actual.declinacion_mag;
						if (azimut_verdadero < 0.0f) azimut_verdadero += 360.0f;
						if (azimut_verdadero >= 360.0f) azimut_verdadero -= 360.0f;
						imu_actual.orientacion_z = azimut_verdadero;
					}
					else {
						// Modo Relativo (Calcula solo Deltas desde el Norte Físico 0.0°)
						float delta_yaw = yaw_actual_imu - yaw_referencia_imu;

						// Corrección del paso por el límite de 0/360 grados
						if (delta_yaw > 180.0f)  delta_yaw -= 360.0f;
						if (delta_yaw < -180.0f) delta_yaw += 360.0f;

						z_acumulado_relativo += delta_yaw;
						yaw_referencia_imu = yaw_actual_imu; // Avanzamos la referencia

						// Normalizar a rango 0 - 360°
						if (z_acumulado_relativo >= 360.0f) z_acumulado_relativo -= 360.0f;
						if (z_acumulado_relativo < 0.0f)   z_acumulado_relativo += 360.0f;

						imu_actual.orientacion_z = z_acumulado_relativo;
					}

					imu_actual.roll_x        = (float)roll_raw / 16.0f;
					imu_actual.inclinacion_y = (float)pitch_raw / 16.0f;
				}
			}
		}

		// Refresco periódico de estado de calibración (Cada 2 segundos)
		static uint32_t ultimo_tick_calib = 0;
		if (HAL_GetTick() - ultimo_tick_calib >= 2000) {
			ultimo_tick_calib = HAL_GetTick();
			uint8_t byte_calibracion = IMU_LeerCalibracion();
			imu_actual.estado_calibracion = byte_calibracion & 0x03;
		}
	}
}
