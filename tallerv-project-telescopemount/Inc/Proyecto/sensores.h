/**
 * @file    : sensores.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Gestión de periféricos externos de posicionamiento y orientación.
 * Manejo de tramas NMEA (GPS NEO-M8N) y ángulos de Euler (IMU BNO055).
 */
#ifndef PROYECTO_SENSORES_H
#define PROYECTO_SENSORES_H

#include <stdint.h>

/* Tamaño máximo del buffer para almacenar una trama NMEA completa */
#define GPS_BUFFER_SIZE 128

/* Estructura para almacenar temporalmente los ángulos de orientación del IMU */
typedef struct {
    float orientacion_z; // Azimut magnético/verdadero (0-360)
    float inclinacion_y; // Altitud inicial / Nivelación (-90 a +90)
    float roll_x;        // Alabeo (Útil para saber si la montura está chueca)
} DatosIMU_t;

/* Banderas volátiles externas (se levantan en las ISR de UART e I2C) */
extern volatile uint8_t flag_gps_trama_lista; // 1 cuando llega el '\n' del GPS
extern volatile uint8_t flag_imu_datos_listos; // 1 cuando finaliza la lectura I2C del BNO055

/* Buffer global abstracto donde la ISR del UART1 guardará los caracteres */
extern char gps_rx_buffer[GPS_BUFFER_SIZE];
extern uint8_t gps_rx_index;

/* Prototipos de funciones */
void Sensores_InitLogica(void);
void Sensores_ProcesarDatos(void);

#endif // PROYECTO_SENSORES_H
