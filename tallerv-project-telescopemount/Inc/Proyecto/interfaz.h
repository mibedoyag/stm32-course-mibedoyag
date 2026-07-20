/**
 * @file    : interfaz.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Cabecera del gestor de Interfaz (Pantalla LCD, Encoder, Botones y FSM).
 */
#ifndef PROYECTO_INTERFAZ_H
#define PROYECTO_INTERFAZ_H

#include "stm32f4xx_hal.h"
#include "Proyecto/astronomia.h" // ¡Crucial para que reconozca gps_actual!

/* ====================================================================
 * 1. MÁQUINA DE ESTADOS DEL SISTEMA
 * ==================================================================== */
typedef enum {
    STATE_BOOTING,
    STATE_MANUAL,
    STATE_TRACKING,
    STATE_ERROR
} SystemState_t;

extern SystemState_t currentState;

/* ====================================================================
 * 2. VARIABLES DE CONTROLES (ENCODER Y BOTONES)
 * ==================================================================== */
/* El valor actual del encoder rotativo (Aumenta o disminuye al girar) */
extern volatile int32_t encoder_contador;

/* Banderas volátiles externas compartidas (se levantan en las ISR) */
extern volatile uint8_t flag_btn_select;
extern volatile uint8_t flag_btn_sync;
extern volatile uint8_t flag_btn_speed;

/* ====================================================================
 * 3. PROTOTIPOS DE LA INTERFAZ
 * ==================================================================== */
void Interfaz_Controles_Init(void); // Inicializa Timer del Encoder y EXTI
void Interfaz_LeerEncoder(void);    // Lee el registro del TIM4

void Interfaz_InitLogica(void);
void Interfaz_UpdateFSM(void);

void LCD_Print(uint8_t row, uint8_t col, char *str);
void LCD_Clear(void);

#endif // PROYECTO_INTERFAZ_H
