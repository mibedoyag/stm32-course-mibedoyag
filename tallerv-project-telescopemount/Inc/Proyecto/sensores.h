/**
 * @file    : sensores.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Cabecera del módulo de telemetría y orientación espacial.
 */
#ifndef PROYECTO_SENSORES_H
#define PROYECTO_SENSORES_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define GPS_BUFFER_SIZE 128

/* Estructura para almacenar los ángulos de orientación de la IMU (BNO055) */
typedef struct {
    float orientacion_z; // Yaw (Azimut Verdadero compensado: 0 a 360 grados)
    float inclinacion_y; // Pitch (Altitud del tubo: -180 a +180 grados)
    float roll_x;        // Roll (Alabeo: -90 a +90)
    int8_t temperatura;  // Temperatura interna del chip en °C
} DatosIMU_t;

/* Estructura para almacenar los datos vitales del GPS */
typedef struct {
    float latitud;
    float longitud;
    float ut_horas;
    float declinacion_mag;
    int16_t anio; // Agregado para los cálculos astronómicos
    int8_t mes;   // Agregado para los cálculos astronómicos
    int8_t dia;   // Agregado para los cálculos astronómicos
} DatosGPS_t;

/* ====================================================================
 * VARIABLES GLOBALES EXTERNAS
 * ==================================================================== */
extern I2C_HandleTypeDef hi2c1;   // I2C1 -> Pantalla LCD
extern UART_HandleTypeDef huart1; // UART1 -> GPS (DMA)
extern UART_HandleTypeDef huart6; // UART6 -> IMU BNO055 (Polling con Timeout

/* Banderas de estado del GPS */
extern volatile uint8_t gps_coordenadas_fijadas;

extern DatosIMU_t imu_actual;
extern DatosGPS_t gps_actual;

/* Prototipos de funciones públicas */
void Sensores_InitLogica(void);
void Sensores_ProcesarDatos(void);

#endif // PROYECTO_SENSORES_H
