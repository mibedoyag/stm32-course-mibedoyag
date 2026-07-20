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

    	// 1. Tiempo indispensable para que el micro del BNO055 arranque
    	HAL_Delay(800 + (intentos * 300));

        // A. Forzar a Página 0 de memoria
        IMU_EscribirRegistro(0x07, 0x00);
        HAL_Delay(30);

        // B. Forzar Modo Configuración (Obligatorio antes de cambiar cualquier cosa)
        IMU_EscribirRegistro(0x3D, 0x00);
        HAL_Delay(40);

        // C. Configurar para usar el cristal externo (Más precisión para NDOF)
        IMU_EscribirRegistro(0x3F, 0x80);
        HAL_Delay(30);

        // D. Cambiar a modo Fusión NDOF (0x0C)
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
static float ConvertirCoordenada(float nmea_raw, char direccion) {
    int grados = (int)(nmea_raw / 100);
    float minutos = nmea_raw - (grados * 100);
    float decimal = grados + (minutos / 60.0f);
    if (direccion == 'S' || direccion == 'W') decimal *= -1.0f;
    return decimal;
}

static void Parser_NMEA_GPRMC(char *trama_limpia) {
    char temp_trama[GPS_BUFFER_SIZE];
    strncpy(temp_trama, trama_limpia, GPS_BUFFER_SIZE);
    char *token = strtok(temp_trama, ",");
    token = strtok(NULL, ",");
    if (token) gps_actual.ut_horas = atof(token) / 10000.0f;
    token = strtok(NULL, ",");
    if (token && token[0] == 'A') {
        token = strtok(NULL, ",");
        float raw_lat = token ? atof(token) : 0.0f;
        token = strtok(NULL, ",");
        char dir_lat = token ? token[0] : 'N';
        gps_actual.latitud = ConvertirCoordenada(raw_lat, dir_lat);
        token = strtok(NULL, ",");
        float raw_lon = token ? atof(token) : 0.0f;
        token = strtok(NULL, ",");
        char dir_lon = token ? token[0] : 'W';
        gps_actual.longitud = ConvertirCoordenada(raw_lon, dir_lon);
        token = strtok(NULL, ",");
        token = strtok(NULL, ",");
        token = strtok(NULL, ",");
        if (token) {
            uint32_t fecha = atoi(token);
            gps_actual.dia = fecha / 10000;
            gps_actual.mes = (fecha % 10000) / 100;
            gps_actual.anio = (fecha % 100) + 2000;
        }
    }
}

void Sensores_ProcesarDatos(void) {
    // A. PROCESAMIENTO GPS
    char *ptr_inicio = strstr(gps_dma_buffer, "$GPRMC");
    if (ptr_inicio == NULL) { ptr_inicio = strstr(gps_dma_buffer, "$GNRMC"); }
    if (ptr_inicio != NULL) {
        char *ptr_fin = strchr(ptr_inicio, '\n');
        if (ptr_fin != NULL) {
            size_t longitud_trama = ptr_fin - ptr_inicio;
            if (longitud_trama < GPS_BUFFER_SIZE && longitud_trama > 10) {
                memcpy(gps_rx_buffer, ptr_inicio, longitud_trama);
                gps_rx_buffer[longitud_trama] = '\0';
                ptr_inicio[0] = '0';
                Parser_NMEA_GPRMC(gps_rx_buffer);
            }
        }
    }

    // B. PROCESAMIENTO IMU (UART sin bloqueos)
    // Trama lectura: Header(0xAA) | Read(0x01) | Reg(0x1A - Euler Z) | Len(0x06 bytes)
    uint8_t cmd_leer[4] = {0xAA, 0x01, 0x1A, 0x06};
    uint8_t respuesta[8] = {0};

    __HAL_UART_FLUSH_DRREGISTER(&huart6);
    HAL_UART_Transmit(&huart6, cmd_leer, 4, 10);

    // Esperamos 8 bytes: Header(0xBB) + Len(0x06) + 6 bytes de datos
    if (HAL_UART_Receive(&huart6, respuesta, 8, 20) == HAL_OK) {
        if (respuesta[0] == 0xBB && respuesta[1] == 0x06) {

            int16_t yaw_raw   = (int16_t)((respuesta[3] << 8) | respuesta[2]);
            int16_t roll_raw  = (int16_t)((respuesta[5] << 8) | respuesta[4]);
            int16_t pitch_raw = (int16_t)((respuesta[7] << 8) | respuesta[6]);

            float azimut_magnetico = (float)yaw_raw / 16.0f;
            float azimut_verdadero = azimut_magnetico + gps_actual.declinacion_mag;

            if (azimut_verdadero < 0.0f) azimut_verdadero += 360.0f;
            if (azimut_verdadero >= 360.0f) azimut_verdadero -= 360.0f;

            imu_actual.orientacion_z = azimut_verdadero;
            imu_actual.roll_x        = (float)roll_raw / 16.0f;
            imu_actual.inclinacion_y = (float)pitch_raw / 16.0f;
        }
    }
}
