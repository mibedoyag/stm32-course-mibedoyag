#include "Proyecto/interfaz.h"
#include "Proyecto/sensores.h"
#include <stdio.h>
#include <string.h>

#define LCD_ADDR (0x27 << 1)

static I2C_HandleTypeDef *lcd_i2c;
TIM_HandleTypeDef htim4; // Handle para el Timer 4 (Modo Encoder)

SystemState_t currentState = STATE_BOOTING;

/* Definición en memoria de las variables de los controles */
volatile int32_t encoder_contador = 0;
volatile uint8_t flag_btn_select = 0;
volatile uint8_t flag_btn_sync = 0;
volatile uint8_t flag_btn_speed = 0;

/* =========================================================================
 * DRIVER LCD
 * ========================================================================= */

// NUEVA FUNCIÓN: Envía solo 4 bits. Obligatorio para el despertar del LCD.
static void LCD_SendNibble(uint8_t nibble) {
    uint8_t data_t[2];
    data_t[0] = (nibble & 0xF0) | 0x0C; // EN=1, RS=0
    data_t[1] = (nibble & 0xF0) | 0x08; // EN=0, RS=0
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_t, 2, 100);
}

static void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;
    data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C;
    data_t[3] = data_l | 0x08;
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_t, 4, 100);
}

static void LCD_SendData(uint8_t data) {
    uint8_t data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;
    data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D;
    data_t[3] = data_l | 0x09;
    HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_t, 4, 100);
}

void LCD_Clear(void) {
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

void LCD_Print(uint8_t row, uint8_t col, char *str) {
    uint8_t pos = (row == 0) ? (0x80 | col) : (0xC0 | col);
    LCD_SendCommand(pos);
    while (*str) {
        LCD_SendData(*str++);
    }
}

/* =========================================================================
 * CONFIGURACIÓN DE HARDWARE: ENCODER Y BOTONES
 * ========================================================================= */
void Interfaz_Controles_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. ENCODER ROTATIVO (TIM4 - Pines PB6 y PB7) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // Configuración TI12: Cuenta en todos los flancos para máxima precisión
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 15; // Filtro de rebotes de hardware elevado

    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 15;

    HAL_TIM_Encoder_Init(&htim4, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 2. BOTONES POR INTERRUPCIÓN (PB12, PB13, PB14) */
    // PB12 = SPEED | PB13 = SYNC | PB14 = SELECT (Encoder Btn)
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupción al soltar a GND
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Interfaz_LeerEncoder(void) {
    // Se divide por 4 debido al modo TI12 que cuenta los 4 estados de la cuadratura
    encoder_contador = (int32_t)(__HAL_TIM_GET_COUNTER(&htim4)) / 4;
}

/* =========================================================================
 * LÓGICA PRINCIPAL (FSM E INICIALIZACIÓN)
 * ========================================================================= */
void Interfaz_InitLogica(void) {
    // 1. Inicializar hardware del usuario (Encoder y Botones)
    Interfaz_Controles_Init();

    // 2. Inicializar Pantalla LCD
    lcd_i2c = &hi2c1;

    // Espera crítica para que el voltaje de 5V de la pantalla se estabilice
    HAL_Delay(100);

    // Secuencia oficial de hardware (HD44780) para forzar el paso de 8-bit a 4-bit
    LCD_SendNibble(0x30); HAL_Delay(5);
    LCD_SendNibble(0x30); HAL_Delay(1);
    LCD_SendNibble(0x30); HAL_Delay(1);
    LCD_SendNibble(0x20); HAL_Delay(1); // ¡A partir de aquí ya estamos en 4-bit!

    // Ahora sí podemos usar la función normal de comandos
    LCD_SendCommand(0x28); HAL_Delay(2); // Función Set: 4-bit, 2 líneas, 5x8
    LCD_SendCommand(0x08); HAL_Delay(2); // Display OFF
    LCD_SendCommand(0x01); HAL_Delay(5); // Clear Display
    LCD_SendCommand(0x06); HAL_Delay(2); // Entry Mode Set
    LCD_SendCommand(0x0C); HAL_Delay(2); // Display ON, Cursor OFF
}

void Interfaz_UpdateFSM(void) {
    char linea1[20];
    char linea2[20];

    // Actualizamos el valor del encoder en cada ciclo
    Interfaz_LeerEncoder();

    static uint32_t ultimo_refresco = 0;
    if (HAL_GetTick() - ultimo_refresco < 250) {
        return;
    }
    ultimo_refresco = HAL_GetTick();

    switch (currentState) {
        case STATE_BOOTING:
            LCD_Print(0, 0, "ASTRO-MOUNT v1.0");
            if (gps_actual.latitud != 0.0f) {
                currentState = STATE_MANUAL;
                LCD_Clear();
            } else {
                LCD_Print(1, 0, "Buscando GPS... ");
            }
            break;

        case STATE_MANUAL:
            sprintf(linea1, "Z:%5.1f  Y:%5.1f", imu_actual.orientacion_z, imu_actual.inclinacion_y);
            LCD_Print(0, 0, linea1);

            {
                char dir_lat = (gps_actual.latitud >= 0) ? 'N' : 'S';
                char dir_lon = (gps_actual.longitud >= 0) ? 'E' : 'W';

                float lat_abs = (gps_actual.latitud < 0) ? -gps_actual.latitud : gps_actual.latitud;
                float lon_abs = (gps_actual.longitud < 0) ? -gps_actual.longitud : gps_actual.longitud;

                sprintf(linea2, "M:%4.1f%c  %5.1f%c", lat_abs, dir_lat, lon_abs, dir_lon);
                LCD_Print(1, 0, linea2);
            }
            break;

        case STATE_TRACKING:
            LCD_Print(0, 0, "MODO SEGUIMIENTO");
            LCD_Print(1, 0, "Objetivo fijado.");
            break;

        case STATE_ERROR:
            LCD_Print(0, 0, "ERROR DE SISTEMA");
            LCD_Print(1, 0, "Revise sensores ");
            break;
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_12) {
        flag_btn_speed = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_13) {
        flag_btn_sync = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_14) {
        // Se oprimió el botón central del Encoder Rotativo
        flag_btn_select = 1;
    }
}
