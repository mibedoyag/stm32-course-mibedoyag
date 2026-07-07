/**
 * @file    : interfaz.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Máquina de estados (FSM) de la interfaz de usuario, menús y botones.
 * Define los modos de operación (Offline, Online, Manual).
 */
#ifndef PROYECTO_INTERFAZ_H
#define PROYECTO_INTERFAZ_H

#include <stdint.h>

/* Enumeración de los estados principales de la aplicación */
typedef enum {
    STATE_BOOTING = 0,     // Estado inicial: Homing y espera de sensores (GPS/IMU)
    STATE_MENU_MAIN = 1,   // Menú principal (Selección de modos)
    STATE_OFFLINE = 2,     // Modo autónomo: Navegación por catálogo interno (GoTo)
    STATE_ONLINE = 3,      // Modo esclavo: Controlado por Stellarium (LX200)
    STATE_MANUAL = 4       // Modo libre: Control por Joystick y Tracking inverso
} AppState_t;

/* Banderas volátiles externas compartidas con stm32f4xx_it.c (Interrupciones) */
extern volatile uint8_t flag_btn_select; // Se levanta a 1 en la ISR del pulsador del Encoder
extern volatile uint8_t flag_btn_sync;   // Se levanta a 1 en la ISR del pulsador SYNC
extern volatile uint8_t flag_btn_speed;  // Se levanta a 1 en la ISR del pulsador SPEED

/* Variable para capturar el diferencial de giro del encoder desde el TIM4 */
extern volatile int16_t encoder_diff;    // Diferencia de pasos para navegar en los menús

/* Estado global de la máquina para ser consultado por otros módulos (ej. motores) */
extern AppState_t currentState;

/* Prototipos de funciones públicas */
void Interfaz_InitLogica(void);
void Interfaz_UpdateFSM(void);

#endif // PROYECTO_INTERFAZ_H
