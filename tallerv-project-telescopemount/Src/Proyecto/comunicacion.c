/**
 * @file    : comunicacion.c
 * @brief   : Implementación del Protocolo LX200 para Stellarium (UART2 con Ring Buffer).
 */
#include "Proyecto/comunicacion.h"
#include "Proyecto/astronomia.h"
#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

UART_HandleTypeDef huart2;

// --- NUEVO: RING BUFFER DE RECEPCIÓN ---
#define RX_RING_SIZE 256
static uint8_t rx_ring[RX_RING_SIZE];
static uint32_t rx_head = 0;
static uint32_t rx_tail = 0;
static uint8_t rx_byte;

/* =========================================================================
 * INICIALIZACIÓN DE HARDWARE (USART2 -> ST-LINK USB)
 * ========================================================================= */
void Comunicacion_InitLogica(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0); // Prioridad alta para no perder bytes
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    // Iniciar recepción del primer byte por interrupción
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

static void Transmitir_Respuesta(const char* respuesta) {
    HAL_UART_Transmit(&huart2, (uint8_t*)respuesta, strlen(respuesta), 100);
}

/* =========================================================================
 * PARSER LX200 CON PROCESAMIENTO EN COLA
 * ========================================================================= */
void Comunicacion_ProcesarComandos(void) {
    if (currentState != STATE_ONLINE) return;

    static char cmd_buffer[64];
    static uint8_t cmd_idx = 0;

    // Procesamos todos los bytes atrapados en el anillo
    while (rx_tail != rx_head) {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % RX_RING_SIZE;

        if (c == ':') {
            cmd_idx = 0; // Inicio de un comando limpio
        }

        if (cmd_idx < 63) {
            cmd_buffer[cmd_idx++] = c;
        }

		// Si encontramos el delimitador de cierre, ejecutamos el parser
		if (c == '#') {
			cmd_buffer[cmd_idx] = '\0';

			// ==========================================================
			// DEPURADOR VISUAL EN PANTALLA LCD
			// Ignoramos los pings de rutina (:GR y :GD) para no saturar la pantalla.
			// Solo atrapamos los comandos de coordenadas (:Sr, :Sd) y el de viaje (:MS)
			if (strncmp(cmd_buffer, ":Sr", 3) == 0
					|| strncmp(cmd_buffer, ":Sd", 3) == 0
					|| strncmp(cmd_buffer, ":MS", 3) == 0) {

				LCD_Clear();
				char debug_str[17];
				// Imprimimos "RX:" seguido de los primeros 12 caracteres del comando
				snprintf(debug_str, 17, "RX:%-13s", cmd_buffer);
				LCD_Print(0, 0, "Intercepcion UART");
				LCD_Print(1, 0, debug_str);

				// Congelamos TODO el sistema 3 segundos para que alcances a leerlo
				HAL_Delay(3000);
				LCD_Clear();
				LCD_Print(0, 0, "LINK: STELLARIUM");
				LCD_Print(1, 0, "Escuchando UART.");
			}


            // 1. Stellarium pregunta RA (:GR#)
            if (strncmp(cmd_buffer, ":GR", 3) == 0) {
                char respuesta[15];
                float ra = target_actual.ra;
                int h = (int)ra;
                int m = (int)((ra - h) * 60.0f);
                int s = (int)((ra - h - (m / 60.0f)) * 3600.0f);
                sprintf(respuesta, "%02d:%02d:%02d#", h, m, s);
                Transmitir_Respuesta(respuesta);
            }

            // 2. Stellarium pregunta DEC (:GD#)
            else if (strncmp(cmd_buffer, ":GD", 3) == 0) {
                char respuesta[15];
                float dec = target_actual.dec;
                char signo = (dec >= 0) ? '+' : '-';
                dec = fabsf(dec);
                int d = (int)dec;
                int m = (int)((dec - d) * 60.0f);
                int s = (int)((dec - d - (m / 60.0f)) * 3600.0f);
                sprintf(respuesta, "%c%02d*%02d:%02d#", signo, d, m, s);
                Transmitir_Respuesta(respuesta);
            }

            // 3. Recepción de Coordenada RA (:Sr...)
            else if (strncmp(cmd_buffer, ":Sr", 3) == 0) {
                char *p = cmd_buffer + 3;
                int h = atoi(p);
                while (*p >= '0' && *p <= '9') p++;
                while (*p && (*p < '0' || *p > '9')) p++;
                int m = atoi(p);
                while (*p >= '0' && *p <= '9') p++;
                while (*p && (*p < '0' || *p > '9')) p++;
                int s = atoi(p);

                target_actual.ra = (float)h + ((float)m / 60.0f) + ((float)s / 3600.0f);
                Transmitir_Respuesta("1");
            }

            // 4. Recepción de Coordenada DEC (:Sd...)
            else if (strncmp(cmd_buffer, ":Sd", 3) == 0) {
                char *p = cmd_buffer + 3;
                char signo = '+';
                if (*p == '+' || *p == '-') { signo = *p; p++; }

                int d = atoi(p);
                while (*p >= '0' && *p <= '9') p++;
                while (*p && (*p < '0' || *p > '9')) p++;
                int m = atoi(p);
                while (*p >= '0' && *p <= '9') p++;
                while (*p && (*p < '0' || *p > '9')) p++;
                int s = atoi(p);

                float dec_val = (float)d + ((float)m / 60.0f) + ((float)s / 3600.0f);
                if (signo == '-') dec_val *= -1.0f;

                target_actual.dec = dec_val;
                Transmitir_Respuesta("1");
            }

            // 5. Disparo de Viaje GoTo (:MS#)
            else if (strncmp(cmd_buffer, ":MS", 3) == 0) {
                Transmitir_Respuesta("0");

                LCD_Clear();
                LCD_Print(0, 0, "Orden Recibida!");
                HAL_Delay(500);

                Astronomia_CalcularAltAz();
                Motores_SetVelocidadGlobal(SPEED_BUSCAR);
                Motores_Apuntar(target_actual.azimut, target_actual.altitud);
                currentState = STATE_MOVIENDO;
            }

            // 6. Parada de Emergencia (:Q#)
            else if (strncmp(cmd_buffer, ":Q", 2) == 0) {
                Motores_DetenerGoTo();
            }
        }
    }
}

/* =========================================================================
 * CALLBACK DE INTERRUPCIÓN UART (Escritura veloz en el Ring Buffer)
 * ========================================================================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {

        // Guardar byte y avanzar la cabeza del anillo
        rx_ring[rx_head] = rx_byte;
        rx_head = (rx_head + 1) % RX_RING_SIZE;

        // Armar la trampa para el siguiente byte inmediatamente
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
