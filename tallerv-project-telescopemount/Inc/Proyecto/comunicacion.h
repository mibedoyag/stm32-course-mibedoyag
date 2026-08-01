/**
 * @file    : comunicacion.h
 * @brief   : Interfaz UART2 para conexión con PC (Stellarium - Protocolo LX200)
 */
#ifndef PROYECTO_COMUNICACION_H
#define PROYECTO_COMUNICACION_H

#include "stm32f4xx_hal.h"

// Inicia el hardware UART2 y habilita las interrupciones
void Comunicacion_InitLogica(void);

// Lógica que evalúa el buffer buscando comandos válidos de Stellarium
void Comunicacion_ProcesarComandos(void);

#endif // PROYECTO_COMUNICACION_H
