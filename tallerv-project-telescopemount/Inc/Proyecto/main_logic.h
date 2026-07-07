/* @file    : main_logic.h
 * @author  : Miguel Angel Bedoya Gonzalez -> mibedoyag@unal.edu.co
 * @brief   : Orquestador principal de la montura Alt-Az.
 * Conecta la FSM de la interfaz con los motores y astronomía.
 */
#ifndef PROYECTO_MAIN_LOGIC_H
#define PROYECTO_MAIN_LOGIC_H

#include <stdint.h>

/* Prototipos de funciones principales */
void Montura_Init(void);  // Función de inicialización global
void Montura_Loop(void);  // Bucle infinito super-loop

#endif // PROYECTO_MAIN_LOGIC_H
