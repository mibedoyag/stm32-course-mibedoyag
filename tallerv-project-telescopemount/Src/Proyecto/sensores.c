/**
 * @file    : sensores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación Bare-Metal del BNO055 (I2C2), LCD (I2C1) y NEO-M8N (UART1).
 */
#include "Proyecto/sensores.h"
#include "Proyecto/astronomia.h"
#include <string.h>
#include <stdlib.h>

/* =========================================================================
 * VARIABLES GLOBALES Y HARDWARE
 * ========================================================================= */
I2C_HandleTypeDef hi2c1;   // Controla físicamente el bus I2C1 (LCD)
I2C_HandleTypeDef hi2c2;   // Controla físicamente el bus I2C2 (BNO055)
UART_HandleTypeDef huart1; // Controla físicamente el UART1 (GPS)

volatile uint8_t flag_gps_trama_lista = 0;
volatile uint8_t flag_imu_datos_listos = 0;

char gps_rx_buffer[GPS_BUFFER_SIZE] = {0};
uint8_t gps_rx_index = 0;
uint8_t rx_byte = 0;

DatosIMU_t imu_actual = {0.0f, 0.0f, 0.0f};

/* Dirección del módulo BNO055 (Clon GY-BNO055 con pin ADD al aire = 0x29) */
#define BNO055_ADDR (0x29 << 1)

/* =========================================================================
 * PARÁMETROS GEOGRÁFICOS LOCALES
 * ========================================================================= */
/* Declinación magnética (~-6.5° Oeste en Medellín).
 * Se suma al norte magnético para hallar el norte verdadero. */

#define DECLINACION_MAG_LOCAL (-6.5f)

/* =========================================================================
 * 1. CONFIGURACIÓN DE HARDWARE DE BAJO NIVEL
 * ========================================================================= */

static void Sensores_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PB8 (SCL) y PB9 (SDA) -> Para LCD
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 50000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

static void Sensores_I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. Configuración exclusiva de PB10 (SCL) -> Usa AF4 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;    // AF4 para SCL
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 2. Configuración exclusiva de PB3 (SDA) -> Usa AF9 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    // Las configuraciones de Mode, Pull y Speed se mantienen igual
    GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;    // AF9 para SDA (¡Crucial!)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Inicialización del periférico I2C2
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c2);
}

static void Sensores_UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PA9 (TX) y PA10 (RX) -> Para GPS
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Sensores_InitLogica(void) {
    Sensores_I2C1_Init();  // Inicia bus LCD
    Sensores_I2C2_Init();  // Inicia bus IMU
    Sensores_UART1_Init(); // Inicia bus GPS

    memset(gps_rx_buffer, 0, GPS_BUFFER_SIZE);
    gps_rx_index = 0;

    // Despertar la IMU usando el puerto hi2c2
    uint8_t mode = 0x0C;
    HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR, 0x3D, 1, &mode, 1, 100);
    HAL_Delay(100);

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* =========================================================================
 * 2. PROCESAMIENTO DE DATOS
 * ========================================================================= */

static float ConvertirCoordenada(float nmea_raw, char direccion) {
    int grados = (int)(nmea_raw / 100);
    float minutos = nmea_raw - (grados * 100);
    float decimal = grados + (minutos / 60.0f);

    if (direccion == 'S' || direccion == 'W') {
        decimal = decimal * -1.0f;
    }
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
    }
}

void Sensores_ProcesarDatos(void) {
    // ----------------------------------------------------------------
    // A. PROCESAMIENTO GPS
    // ----------------------------------------------------------------
    if (flag_gps_trama_lista) {
        HAL_NVIC_DisableIRQ(USART1_IRQn);
        char *ptr_inicio = strstr((char*)gps_rx_buffer, "RMC");
        if (ptr_inicio != NULL) {
            Parser_NMEA_GPRMC(ptr_inicio);
        }
        memset(gps_rx_buffer, 0, GPS_BUFFER_SIZE);
        gps_rx_index = 0;
        flag_gps_trama_lista = 0;
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }

    // ----------------------------------------------------------------
    // B. PROCESAMIENTO IMU (BNO055)
    // ----------------------------------------------------------------
    uint8_t data_euler[6];
    uint8_t temp_raw = 0;

    // 1. Lectura de los ángulos de Euler
    if (HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR, 0x1A, 1, data_euler, 6, 10) == HAL_OK) {
        int16_t yaw_raw   = (int16_t)((data_euler[1] << 8) | data_euler[0]);
        int16_t roll_raw  = (int16_t)((data_euler[3] << 8) | data_euler[2]);
        int16_t pitch_raw = (int16_t)((data_euler[5] << 8) | data_euler[4]);

        float azimut_magnetico = (float)yaw_raw / 16.0f;

        // CÁLCULO DEL NORTE VERDADERO
        float azimut_verdadero = azimut_magnetico + DECLINACION_MAG_LOCAL;

        // Normalizamos el ángulo para que siempre esté entre 0° y 360°
        if (azimut_verdadero < 0.0f) {
            azimut_verdadero += 360.0f;
        } else if (azimut_verdadero >= 360.0f) {
            azimut_verdadero -= 360.0f;
        }

        imu_actual.orientacion_z = azimut_verdadero;
        imu_actual.roll_x        = (float)roll_raw / 16.0f;
        imu_actual.inclinacion_y = (float)pitch_raw / 16.0f;
    }

    // 2. Lectura de la temperatura interna (Registro 0x34)
    if (HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR, 0x34, 1, &temp_raw, 1, 10) == HAL_OK) {
        imu_actual.temperatura = (int8_t)temp_raw;
    }
}
