/**
 * @file    : comunicacion.c
 * @author  : Miguel Angel Bedoya G. --> mibedoyag@unal.edu.co
 * @brief   : Parser LX200 protocolo de comunicación serial
 */
#include "Proyecto/comunicacion.h"
#include "Proyecto/astronomia.h"
#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

UART_HandleTypeDef huart2; //Handle de manipulación de USART2 para comunicación serial con Stellarium


/*Si el microcontrolador está ocupado calculando trigonometría o moviendo la pantalla, podría perder letras del comando.
 * Este Ring Buffer actúa como una sala de espera: la interrupción mete los caracteres rápidamente por la "cabeza", y luego el lazo principal los procesa con calma */
#define RX_RING_SIZE 256
static uint8_t rx_ring[RX_RING_SIZE];
static uint32_t rx_head = 0;
static uint32_t rx_tail = 0;
static uint8_t rx_byte;

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
    // 9600 baudios es el estándar del protocolo Meade LX200
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}


/* Toma una cadena de texto (string) y la envía de vuelta al PC usando HAL_UART_Transmit
 * YA que el protocolo lo primero que hace es solicitar información de RA y DEC actual del telescopio */
static void Transmitir_Respuesta(const char* respuesta) {
    HAL_UART_Transmit(&huart2, (uint8_t*)respuesta, strlen(respuesta), 100);
}


/* Extrae caracteres del Ring Buffer y los va pegando en cmd_buffer hasta que encuentra un símbolo de numeral (#),
 * que es el carácter oficial que indica "Fin de Comando" en el protocolo LX200. Una vez que tiene un comando completo,
 * utiliza la función strstr de C para buscar subcadenas y saber qué orden dio el PC */
void Comunicacion_ProcesarComandos(void) {
    if (currentState != STATE_ONLINE) return;

    static char cmd_buffer[128];
    static uint8_t cmd_idx = 0;

    while (rx_tail != rx_head) {
        char c = (char)rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % RX_RING_SIZE;

        // Metemos todo al buffer ignorando desbordamientos, EXCEPTO el cierre
        if (cmd_idx < 127 && c != '#') {
            cmd_buffer[cmd_idx++] = c;
            cmd_buffer[cmd_idx] = '\0';
        }
        else if (c == '#') {

            // 1. Stellarium pregunta Ascensión Recta (Busca "GR")
            if (strstr(cmd_buffer, "GR") != NULL) {
                char respuesta[16];
                float ra = target_actual.ra;
                int h = (int)ra;
                float rem_m = (ra - h) * 60.0f;
                int m = (int)rem_m;
                int s = (int)((rem_m - m) * 60.0f);
                sprintf(respuesta, "%02d:%02d:%02d#", h, m, s);
                Transmitir_Respuesta(respuesta);
            }

            // 2. Stellarium pregunta Declinación (Busca "GD")
            else if (strstr(cmd_buffer, "GD") != NULL) {
                char respuesta[16];
                float dec = target_actual.dec;
                char signo = (dec >= 0) ? '+' : '-';
                dec = fabsf(dec);
                int d = (int)dec;
                float rem_m = (dec - d) * 60.0f;
                int m = (int)rem_m;
                int s = (int)((rem_m - m) * 60.0f);
                // Casteo directo a char del valor 223 para evitar fusiones de compilador
                sprintf(respuesta, "%c%02d%c%02d:%02d#", signo, d, (char)223, m, s);
                Transmitir_Respuesta(respuesta);
            }

            // 3. Recepción de coordenada RA para viajar (Busca "Sr")
            else if (strstr(cmd_buffer, "Sr") != NULL) {
                char *p = strstr(cmd_buffer, "Sr") + 2;
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

            // 4. Recepción de coordenada DEC para viajar (Busca "Sd")
            else if (strstr(cmd_buffer, "Sd") != NULL) {
                char *p = strstr(cmd_buffer, "Sd") + 2;
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

            // 5. Iniciar movimiento GoTo (Busca "MS")
            else if (strstr(cmd_buffer, "MS") != NULL) {
                Transmitir_Respuesta("0");
                Astronomia_CalcularAltAz();
                Motores_SetVelocidadGlobal(SPEED_BUSCAR);
                Motores_Apuntar(target_actual.azimut, target_actual.altitud);
                currentState = STATE_MOVIENDO;
            }

            // 6. Comandos de sincronización y parada de emergencia
            else if (strstr(cmd_buffer, "CM") != NULL) { Transmitir_Respuesta("OK#"); }
            else if (strstr(cmd_buffer, "Q") != NULL) { Motores_DetenerGoTo(); }

            // Limpiamos el buffer por completo para recibir el próximo comando
            cmd_idx = 0;
            cmd_buffer[0] = '\0';
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        rx_ring[rx_head] = rx_byte;
        rx_head = (rx_head + 1) % RX_RING_SIZE;
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
