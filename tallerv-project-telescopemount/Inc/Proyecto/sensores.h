/**
 * @file    : sensores.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Cabecera del módulo de telemetría y orientación espacial.
 * Define las estructuras y expone las variables globales al sistema.
 */
#ifndef PROYECTO_SENSORES_H
#define PROYECTO_SENSORES_H

#include <stdint.h>
#include "stm32f4xx_hal.h" // Necesario para que el compilador reconozca los Handles (UART_HandleTypeDef, etc.)

/* Tamaño máximo del buffer de recepción serial del GPS.
 * Una trama NMEA estándar rara vez supera los 82 caracteres. */
#define GPS_BUFFER_SIZE 128

/* Estructura para almacenar los ángulos de orientación de la IMU (BNO055) */
typedef struct {
    float orientacion_z; // Yaw (Azimut magnético: 0 a 360 grados)
    float inclinacion_y; // Pitch (Altitud del tubo: -180 a +180 grados)
    float roll_x;        // Roll (Alabeo: útil para nivelación de la base: -90 a +90)
} DatosIMU_t;

/* ====================================================================
 * VARIABLES GLOBALES EXTERNAS
 * Al usar 'extern', le decimos al compilador: "Estas variables existen
 * en la memoria de sensores.c, pero permíteme leerlas desde otros archivos".
 * ==================================================================== */

/* Banderas de estado (Volátiles para evitar que el compilador las borre al optimizar) */
extern volatile uint8_t flag_gps_trama_lista;
extern volatile uint8_t flag_imu_datos_listos;

/* Variables para el manejo de la interrupción UART del GPS */
extern char gps_rx_buffer[GPS_BUFFER_SIZE];
extern uint8_t gps_rx_index;
extern uint8_t rx_byte; // Byte temporal donde aterriza cada letra que llega del GPS

/* ====================================================================
 * INSTANCIAS DE HARDWARE GLOBALES
 * ==================================================================== */
extern I2C_HandleTypeDef hi2c1; // Canal físico 1 -> Exclusivo para la Pantalla LCD
extern I2C_HandleTypeDef hi2c2; // Canal físico 2 -> Exclusivo para la IMU BNO055
extern UART_HandleTypeDef huart1; // Canal físico UART -> Exclusivo para el GPS

/* Estructura global con los datos listos para ser consumidos por la montura */
extern DatosIMU_t imu_actual;

/* ====================================================================
 * PROTOTIPOS DE FUNCIONES PÚBLICAS
 * ==================================================================== */
void Sensores_InitLogica(void);
void Sensores_ProcesarDatos(void);

#endif // PROYECTO_SENSORES_H
