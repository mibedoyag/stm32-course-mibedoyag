/**
 * @file    : main_logic.c
 * @author  : Miguel Angel Bedoya Gonzalez -> mibedoyag@unal.edu.co
 * @brief   : Implementación del bucle principal del sistema - puente entre el main.c y la lógica del proyecto
 */
#include "Proyecto/main_logic.h"
#include "Proyecto/interfaz.h"
#include "Proyecto/motores.h"
#include "Proyecto/sensores.h"
#include "Proyecto/astronomia.h"
#include "Proyecto/comunicacion.h"
#include "stm32f4xx_hal.h"


/**
 * @brief Inicializa todos los subsistemas lógicos en el orden de dependencias de hardware.
 */
void Montura_Init(void) {
    // --- SEGURO DE ARRANQUE EN FRÍO ---
    // Detiene el microcontrolador 1000ms para permitir que las fuentes de alimentación
    // de 5V y 3.3V se estabilicen y los chips de la LCD e IMU terminen su Power-On Reset.
    HAL_Delay(1000);

    // 1. Iniciar hardware base: I2C (LCD), UART (GPS - IMU), DMA circular para ADC y GPS
    Sensores_InitLogica();

    // 2. Iniciar hardware de control: ADC (Joystick), Timers PWM para motores, DMA
    Motores_InitLogica();

    // 3. Iniciar la Interfaz de usuario en pantalla y botones físicos
    Interfaz_InitLogica();

    // 4. Iniciar variables matemáticas y compensaciones
    Astronomia_InitLogica();

    // 5. Iniciar la comunicación serial que escuchará Stellarium por protocolo LX200
    Comunicacion_InitLogica();
}


/**
 * @brief Bucle principal infinito.
 * Procesa las banderas levantadas por las interrupciones y actualizaciones de hardware.
 */
void Montura_Loop(void) {
    // 1. Actualizar máquina de estados de la interfaz
    Interfaz_UpdateFSM();

    // 2. Procesar tramas de sensores si hay datos nuevos para GPS e IMU
    Sensores_ProcesarDatos();

    // 3. Actualizar cálculos de motores (Tracking o GoTo)
    Motores_UpdateLogica();

    // 4. Escucha al PC de fondo para stellarium y el protocolo LX200
    Comunicacion_ProcesarComandos();
}


