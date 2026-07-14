#include "Proyecto/interfaz.h"
#include "Proyecto/sensores.h" // Para leer imu_actual y obtener hi2c1
#include <stdio.h>
#include <string.h>

#define LCD_ADDR (0x27 << 1)

static I2C_HandleTypeDef *lcd_i2c;
SystemState_t currentState = STATE_BOOTING;

/* Definición en memoria de las banderas de los botones */
volatile uint8_t flag_btn_select = 0;
volatile uint8_t flag_btn_sync = 0;
volatile uint8_t flag_btn_speed = 0;

/* =========================================================================
 * DRIVER LCD (Se mantiene igual)
 * ========================================================================= */
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
 * RENOMBRADO PARA COINCIDIR CON main_logic.c
 * ========================================================================= */
void Interfaz_InitLogica(void) {
    lcd_i2c = &hi2c1;

    // Incrementamos el delay inicial drásticamente (de 50ms a 150ms)
    // para darle tiempo al regulador de la pantalla de estabilizar sus 5V.
    HAL_Delay(150);

    // Secuencia de inicialización robusta de 4 bits con tiempos de espera holgados
    LCD_SendCommand(0x30); HAL_Delay(10); // Comando de reinicio
    LCD_SendCommand(0x30); HAL_Delay(5);  // Repetición del comando
    LCD_SendCommand(0x30); HAL_Delay(5);  // Confirmación
    LCD_SendCommand(0x20); HAL_Delay(10); // Forzar cambio físico a modo 4-bits

    // Ahora que está en 4-bits seguros, enviamos la parametrización
    LCD_SendCommand(0x28); HAL_Delay(2);  // 2 líneas, fuente 5x8
    LCD_SendCommand(0x08); HAL_Delay(2);  // Apagar display para configurar
    LCD_SendCommand(0x01); HAL_Delay(5);  // Limpiar memoria DDRAM (requiere bastante tiempo)
    LCD_SendCommand(0x06); HAL_Delay(2);  // Dirección del cursor: Incremento
    LCD_SendCommand(0x0C); HAL_Delay(2);  // Encender pantalla, apagar cursor
}

void Interfaz_UpdateFSM(void) {
    char linea1[20];
    char linea2[20];

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
		// Línea 1: "Z:359.9 Y:-179.9" -> Exactamente 16 caracteres
		// Z = Azimut (orientacion_z), Y = Altura (inclinacion_y)
		sprintf(linea1, "Z:%5.1f  Y:%5.1f", imu_actual.orientacion_z,
				imu_actual.inclinacion_y);
		LCD_Print(0, 0, linea1);

		// Línea 2: "M:06.2N  -075.5W" -> Exactamente 16 caracteres
		// M = Manual, seguido de las coordenadas formateadas
		{
			char dir_lat = (gps_actual.latitud >= 0) ? 'N' : 'S';
			char dir_lon = (gps_actual.longitud >= 0) ? 'E' : 'W';

			// Usamos fabs() para mostrar el valor absoluto ya que la dirección 'S' o 'W' da el contexto del signo
			float lat_abs =
					(gps_actual.latitud < 0) ?
							-gps_actual.latitud : gps_actual.latitud;
			float lon_abs =
					(gps_actual.longitud < 0) ?
							-gps_actual.longitud : gps_actual.longitud;

			sprintf(linea2, "M:%4.1f%c  %5.1f%c", lat_abs, dir_lat, lon_abs,
					dir_lon);
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
