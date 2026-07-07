/**
 * @file    : sensores.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación del parser NMEA y el procesamiento de la IMU.
 */
#include "Proyecto/sensores.h"
#include "Proyecto/astronomia.h" // Para sobreescribir la estructura gps_actual
#include <string.h>
#include <stdlib.h>

/* Inicialización estricta de variables y banderas en 0 */
volatile uint8_t flag_gps_trama_lista = 0;
volatile uint8_t flag_imu_datos_listos = 0;

char gps_rx_buffer[GPS_BUFFER_SIZE] = {0};
uint8_t gps_rx_index = 0;

DatosIMU_t imu_actual = {0.0f, 0.0f, 0.0f};

/**
 * @brief Inicializa las variables, banderas y limpia el buffer de recepción.
 */
void Sensores_InitLogica(void) {
    flag_gps_trama_lista = 0;
    flag_imu_datos_listos = 0;
    gps_rx_index = 0;

    imu_actual.orientacion_z = 0.0f;
    imu_actual.inclinacion_y = 0.0f;
    imu_actual.roll_x = 0.0f;

    for (uint8_t i = 0; i < GPS_BUFFER_SIZE; i++) {
        gps_rx_buffer[i] = 0;
    }
}

/**
 * @brief Analiza una cadena NMEA ($GPRMC) para extraer UTC, Latitud, Longitud y Fecha.
 * @param trama Puntero a la cadena de texto recibida por UART.
 */
static void Parser_NMEA_GPRMC(char *trama) {
    // La trama típica GPRMC se ve así:
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

    // Verificamos que sea realmente una trama GPRMC
    if (strstr(trama, "$GPRMC") != NULL) {

        // TODO: Implementar strtok() o conteo de comas ',' para aislar los tokens.
        // Ejemplo de extracción lógica a implementar posteriormente:

        // 1. Extraer Hora UTC (Formato HHMMSS o HHMMSS.SS)
        // char *token_hora = obtener_token(trama, 1);
        // gps_actual.ut_horas = convertir_hora_a_decimal(token_hora);

        // 2. Extraer Latitud (Formato DDMM.MMMM y N/S)
        // char *token_lat = obtener_token(trama, 3);
        // char *token_lat_dir = obtener_token(trama, 4);
        // gps_actual.latitud = convertir_coordenada(token_lat, token_lat_dir);

        // 3. Extraer Longitud (Formato DDDMM.MMMM y E/W)
        // char *token_lon = obtener_token(trama, 5);
        // char *token_lon_dir = obtener_token(trama, 6);
        // gps_actual.longitud = convertir_coordenada(token_lon, token_lon_dir);

        // 4. Extraer Fecha (Formato DDMMYY)
        // char *token_fecha = obtener_token(trama, 9);
        // gps_actual.dia = extraer_dia(token_fecha);
        // gps_actual.mes = extraer_mes(token_fecha);
        // gps_actual.anio = extraer_anio(token_fecha) + 2000; // Asumiendo post-año 2000
    }
}

/**
 * @brief Rutina periódica ejecutada en Montura_Loop() para procesar sensores.
 */
void Sensores_ProcesarDatos(void) {

    // ----------------------------------------------------------------
    // 1. Procesamiento de la Trama GPS
    // ----------------------------------------------------------------
    if (flag_gps_trama_lista == 1) {
        // Apagamos la bandera inmediatamente
        flag_gps_trama_lista = 0;

        // Procesamos la trama que está en el buffer
        Parser_NMEA_GPRMC(gps_rx_buffer);

        // Limpiamos el buffer y reiniciamos el índice para la siguiente trama
        for (uint8_t i = 0; i < GPS_BUFFER_SIZE; i++) {
            gps_rx_buffer[i] = 0;
        }
        gps_rx_index = 0;
    }

    // ----------------------------------------------------------------
    // 2. Procesamiento de datos de la IMU (I2C)
    // ----------------------------------------------------------------
    if (flag_imu_datos_listos == 1) {
        flag_imu_datos_listos = 0;

        // TODO: Leer los registros I2C abstractos donde el HAL o DMA guardó los bytes
        // y convertirlos a grados flotantes para actualizar la estructura imu_actual.
        // Esto servirá para la rutina de Homing en interfaz.c (STATE_BOOTING).
    }
}
