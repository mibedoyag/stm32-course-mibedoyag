/**
 * @file    : interfaz.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Cabecera del gestor de Interfaz (Pantalla LCD, Encoder, Botones y FSM).
 */

#ifndef PROYECTO_INTERFAZ_H
#define PROYECTO_INTERFAZ_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Definición de todos los estados de la interfaz
typedef enum {
    STATE_BOOTING = 0,         // Buscando GPS y Homing
    STATE_BOOT_SUCCESS,        // Muestra satélites y hora
    STATE_MAIN_MENU,           // Menú Principal
    STATE_MANUAL,              // Modo Manual (Joystick)
    STATE_OFFLINE_CATALOGO,    // Selección de Catálogo (Messier/Planetas)
    STATE_OFFLINE_OBJETO,      // Selección de Objeto celeste
    STATE_OFFLINE_ACCION,      // Selección de acción (Apuntar/Seguir)
	STATE_MOVIENDO,
    STATE_TRACKING,            // Modo de seguimiento activo
    STATE_ONLINE,              // Comunicación UART con Stellarium/SkySafari
    STATE_INFO,                // Pantalla de información (GPS + Euler)
    STATE_ERROR                // Error crítico de hardware
} SystemState_t;

// Variables globales exportadas
extern SystemState_t currentState;
extern volatile uint8_t flag_homing_ok; // Definida en motores.c (1 cuando termine el home)

// Prototipos de funciones
void Interfaz_InitLogica(void);
void Interfaz_UpdateFSM(void);
void Interfaz_Controles_Init(void);
void Interfaz_LeerEncoder(void);

// Funciones nativas de la LCD
void LCD_Clear(void);
void LCD_Print(uint8_t row, uint8_t col, char *str);

#endif // PROYECTO_INTERFAZ_H
