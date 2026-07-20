/**
 * @file    : astronomia.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Motor matemático y de transformaciones de coordenadas.
 * Calcula trigonometría esférica aprovechando la FPU del microcontrolador.
 */
#ifndef PROYECTO_ASTRONOMIA_H
#define PROYECTO_ASTRONOMIA_H

#include <stdint.h>
#include <math.h>

/* Constantes matemáticas en precisión simple (float) */
#define PI_F        3.14159265f
#define DEG2RAD_F   0.01745329f // (PI / 180)
#define RAD2DEG_F   57.2957795f // (180 / PI)

/* Estructura para almacenar coordenadas ecuatoriales y horizontales */
typedef struct {
    float ra;       // Ascensión Recta en horas decimales (0.0 - 24.0)
    float dec;      // Declinación en grados decimales (-90.0 a +90.0)
    float altitud;  // Altitud local calculada en grados
    float azimut;   // Azimut local calculado en grados
} Coordenadas_t;


/* Estructura para objetos celestes del catálogo */
typedef struct {
    char nombre[16];   // Nombre del objeto (ej. "M42")
    float ra;          // RA en horas decimales (0.0 - 24.0)
    float dec;         // Dec en grados (-90.0 a 90.0)
} ObjetoCeleste_t;

/* Prototipos para acceder al catálogo */
const ObjetoCeleste_t* Astronomia_ObtenerObjeto(uint8_t categoria, uint8_t indice);
uint8_t Astronomia_ObtenerTotalObjetos(void);

/* Variables globales externas para ser leídas por motores.c e interfaz.c */
extern Coordenadas_t target_actual;
extern Coordenadas_t offset_calibracion;

/* Prototipos de funciones */
void Astronomia_InitLogica(void);
void Astronomia_CalcularAltAz(void);
void Astronomia_SyncOffset(float encoder_alt_actual, float encoder_az_actual);

#endif // PROYECTO_ASTRONOMIA_H
