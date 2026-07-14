/**
 * @file    : interfaz.h
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Cabecera del gestor de Interfaz (Pantalla LCD y Máquina de Estados).
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
 * 2. BANDERAS DE BOTONES EXTERNOS (Para main_logic.c)
 * ==================================================================== */
extern volatile uint8_t flag_btn_select;
extern volatile uint8_t flag_btn_sync;
extern volatile uint8_t flag_btn_speed;

/* ====================================================================
 * 3. PROTOTIPOS DE LA INTERFAZ
 * ==================================================================== */

// Nombres actualizados para coincidir con main_logic.c
void Interfaz_InitLogica(void);
void Interfaz_UpdateFSM(void);

void LCD_Print(uint8_t row, uint8_t col, char *str);
void LCD_Clear(void);

#endif // PROYECTO_INTERFAZ_H
