/**
 * @file    : interfaz.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Implementación de la máquina de estados y control de menús LCD.
 */
#include "Proyecto/interfaz.h"
// #include "proyecto/motores.h"    // (Se incluirá cuando se implemente la lógica de cambio de velocidad)
// #include "proyecto/astronomia.h" // (Se incluirá cuando se implemente la compensación SYNC)

/* Inicialización estricta de variables globales y banderas en 0 */
AppState_t currentState = STATE_BOOTING;

volatile uint8_t flag_btn_select = 0;
volatile uint8_t flag_btn_sync = 0;
volatile uint8_t flag_btn_speed = 0;
volatile int16_t encoder_diff = 0;

/* Variables estáticas locales para el control interno de menús */
static uint8_t menu_main_index = 0;    // Índice del menú principal (0:Offline, 1:Online, 2:Manual)
static uint8_t menu_offline_index = 0; // Índice del submenú de catálogos
static uint8_t tracking_activo = 0;    // Bandera lógica de tracking (0: OFF, 1: ON)

/**
 * @brief Inicializa las variables lógicas de la interfaz a sus valores seguros.
 */
void Interfaz_InitLogica(void) {
    currentState = STATE_BOOTING;
    flag_btn_select = 0;
    flag_btn_sync = 0;
    flag_btn_speed = 0;
    encoder_diff = 0;
    menu_main_index = 0;
    menu_offline_index = 0;
    tracking_activo = 0;
}

/**
 * @brief Actualiza la máquina de estados evaluando las banderas de hardware.
 * Debe ejecutarse constantemente dentro del Montura_Loop().
 */
void Interfaz_UpdateFSM(void) {

    // ------------------------------------------------------------------
    // 1. Procesamiento de botones globales (Independientes del estado)
    // ------------------------------------------------------------------
    if (flag_btn_speed == 1) {
        flag_btn_speed = 0; // Limpiar bandera inmediatamente

        // TODO: Llamar a Motores_SetVelocidadGlobal() para ciclar perfiles.
        // Si estamos en STATE_MANUAL, actualizar la LCD con "Vel: MEDIA/ALTA".
    }

    // ------------------------------------------------------------------
    // 2. Máquina de Estados Principal
    // ------------------------------------------------------------------
    switch (currentState) {

        case STATE_BOOTING:
            // TODO: Mostrar "Homing..." en la pantalla LCD.
            // Una vez los motores toquen los finales de carrera y el GPS tenga Fix:
            // Por ahora, simulamos una transición inmediata al menú principal.
            currentState = STATE_MENU_MAIN;
            break;

        case STATE_MENU_MAIN:
            // Navegación del menú con el Encoder (TIM4)
            if (encoder_diff != 0) {
                // TODO: Limitar y ajustar menu_main_index entre 0 y 2.
                encoder_diff = 0; // Consumir el diferencial
                // TODO: Refrescar la LCD con la opción seleccionada.
            }

            // Ingreso a un submodo con el botón SELECT
            if (flag_btn_select == 1) {
                flag_btn_select = 0; // Limpiar bandera

                if (menu_main_index == 0) {
                    currentState = STATE_OFFLINE;
                    menu_offline_index = 0; // Reiniciar vista del submenú
                    // TODO: Dibujar menú de catálogo (Messier, etc.) en LCD.
                }
                else if (menu_main_index == 1) {
                    currentState = STATE_ONLINE;
                    // TODO: Dibujar "Esperando LX200..." en LCD.
                }
                else if (menu_main_index == 2) {
                    currentState = STATE_MANUAL;
                    tracking_activo = 0; // Seguridad: Ingresar con motores apagados.
                    // TODO: Dibujar vista Manual (Trk:OFF Vel:XXX) en LCD.
                }
            }
            break;

        case STATE_OFFLINE:
            // Navegación por el catálogo de objetos astronómicos
            if (encoder_diff != 0) {
                // TODO: Avanzar retroceder en el arreglo de estrellas/Messier
                encoder_diff = 0;
            }

            // Ejecutar GoTo al objeto seleccionado o retornar al Menú Principal
            if (flag_btn_select == 1) {
                flag_btn_select = 0;
                // TODO: Si seleccionó "< Volver", currentState = STATE_MENU_MAIN;
                // TODO: Sino, llamar Astronomia_CalcularAltAz() y mover motores.
            }

            // Compensación geométrica en lazo cerrado
            if (flag_btn_sync == 1) {
                flag_btn_sync = 0;
                // TODO: Llamar a Astronomia_SyncOffset() comparando encoders AS5600.
                // TODO: Mostrar "[Sincronizado!]" temporalmente.
            }
            break;

        case STATE_ONLINE:
            // Modo esclavo: El flujo lo dicta el puerto UART2 (LX200)

            // Abortar modo online y volver al inicio
            if (flag_btn_select == 1) {
                flag_btn_select = 0;
                currentState = STATE_MENU_MAIN;
                // TODO: Refrescar la LCD.
            }

            // Corrección de error de apuntado enviada físicamente (útil si Stellarium falla por unos grados)
            if (flag_btn_sync == 1) {
                flag_btn_sync = 0;
                // TODO: Llamar a Astronomia_SyncOffset().
            }
            break;

        case STATE_MANUAL:
            // Movimiento puro comandado por el Joystick y JoystickData_t.

            // Alternar Tracking Inverso con el botón SELECT (Menú Contextual)
            if (flag_btn_select == 1) {
                flag_btn_select = 0;

                if (tracking_activo == 0) {
                    tracking_activo = 1;
                    // TODO: Calcular RA/Dec inverso según posición actual y encender TIMers.
                } else {
                    tracking_activo = 0;
                    // TODO: Apagar seguimiento.
                }
                // TODO: Actualizar texto "Trk:ON" o "Trk:OFF" en LCD.
            }

            // Guardar punto de interés manual
            if (flag_btn_sync == 1) {
                flag_btn_sync = 0;
                // TODO: Ejecutar corrección local.
            }
            break;

        default:
            // Mecanismo de seguridad (Failsafe) en caso de corrupción de estado
            currentState = STATE_BOOTING;
            break;
    }
}
