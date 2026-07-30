/**
 * @file    : motores.h
 * @author  : Miguel Angel Bedoya G. -> mibedoyag@unal.edu.co
 * @brief   : Capa de abstracción para el control de actuadores (TMC2208) y lectura de Joystick (ADC).
 * Calcula la cinemática de los ejes Altitud y Azimut.
 */
#ifndef PROYECTO_MOTORES_H
#define PROYECTO_MOTORES_H

#include <stdint.h>

/* Constantes mecánicas calculadas para correas 3GT y motor 1.8° con 16 micro-pasos */
#define PULSOS_POR_GRADO_AZIMUT  49.38f  // Relación 100/18
#define PULSOS_POR_GRADO_ALTITUD 98.76f  // Relación (50/18) * (100/25)

/* Umbral de zona muerta para el joystick analógico (0 - 4095 en STM32 de 12 bits, centro en ~2048) */
#define JOYSTICK_DEADZONE 200

/* Enumeración para los perfiles de velocidad de los motores */
typedef enum {
    SPEED_GUIAR = 0,   // Velocidad mínima (Tracking fine)
    SPEED_CENTRAR = 1, // Velocidad media (Joystick normal)
    SPEED_BUSCAR = 2   // Velocidad máxima (GoTo rápido)
} VelocidadModo_t;

extern VelocidadModo_t velocidad_actual; //SE hace pública la variable para que desde interfaz.c la pantalla LCD la pueda leer para mostrarla

/* Estructura para almacenar las lecturas del ADC del Joystick */
typedef struct {
    uint16_t eje_x; // Lectura cruda ADC (Azimut)
    uint16_t eje_y; // Lectura cruda ADC (Altitud)
} JoystickData_t;

/* Banderas volátiles externas compartidas (se levantan en las ISR) */
extern volatile uint8_t flag_adc_joystick_ready;

/* Prototipos de funciones */
void Motores_InitLogica(void);
void Motores_UpdateLogica(void);
void Motores_SetVelocidadGlobal(VelocidadModo_t nueva_velocidad);

/* =========================================================================
 * VARIABLES GLOBALES DEL CONTROL DE POSICIÓN (GoTo)
 * ========================================================================= */
// Banderas para la FSM (1 = Terminado / En reposo, 0 = Moviéndose)
extern volatile uint8_t flag_goto_terminado_az;
extern volatile uint8_t flag_goto_terminado_alt;

// API de Posicionamiento
void Motores_Apuntar(float azimut_target, float altitud_target);
void Motores_DetenerGoTo(void);

/* =========================================================================
 * RUTINA DE CALIBRACIÓN INICIAL (HOMING)
 * ========================================================================= */
// Esta bandera ahora pertenece a los motores y será leída por la FSM de la interfaz
extern volatile uint8_t flag_homing_ok;

// Funciones de Homing
void Motores_IniciarHoming(void);
void Motores_UpdateHoming(void);

#endif // PROYECTO_MOTORES_H
