#include "Proyecto/interfaz.h"
#include "Proyecto/sensores.h"
#include "Proyecto/astronomia.h"
#include "Proyecto/motores.h"
#include <stdio.h>
#include <string.h>

#define LCD_ADDR (0x27 << 1)

static I2C_HandleTypeDef *lcd_i2c;
TIM_HandleTypeDef htim4;

SystemState_t currentState = STATE_BOOTING;
//volatile uint8_t flag_homing_ok = 1; // Simulamos la variable que vendrá de motores.c

/* Variables volátiles de los controles */
volatile int32_t encoder_contador = 0;
//volatile uint8_t flag_btn_select = 0;
volatile uint8_t flag_btn_sync = 0;
volatile uint8_t flag_btn_speed = 0;

/* Variables para la navegación de la FSM */
static int32_t ultimo_encoder = 0; // Para calcular diferencias de movimiento
static uint32_t boot_timer = 0;    // Temporizador para la pantalla de éxito
static uint8_t menu_index = 0;     // Índice genérico de menús
//static uint32_t ultimo_boton_tick = 0; // Para el Anti-Rebote del botón

/* Variables para recordar selecciones de Astronomía */
static uint8_t catalogo_seleccionado = 0; // 0 = Messier, 1 = Estrellas
static uint8_t objeto_seleccionado = 0;   // Índice del 0 al 19 (o 9)
static uint8_t total_objetos_actual = 20; // Tamaño del catálogo actual


/* =========================================================================
 * DRIVERS DE LA LCD (Se mantienen idénticos a los tuyos)
 * ========================================================================= */
static void LCD_SendNibble(uint8_t nibble) {
	uint8_t data_t[2];
	data_t[0] = (nibble & 0xF0) | 0x0C;
	data_t[1] = (nibble & 0xF0) | 0x08;
	HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_t, 2, 100);
}

static void LCD_SendCommand(uint8_t cmd) {
	uint8_t data_u, data_l, data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;
	data_t[1] = data_u | 0x08;
	data_t[2] = data_l | 0x0C;
	data_t[3] = data_l | 0x08;
	HAL_I2C_Master_Transmit(lcd_i2c, LCD_ADDR, data_t, 4, 100);
}

static void LCD_SendData(uint8_t data) {
	uint8_t data_u, data_l, data_t[4];
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
 * FUNCIONES AUXILIARES DE NAVEGACIÓN
 * ========================================================================= */

/**
 * @brief  Procesa el movimiento del encoder para navegar entre un rango definido.
 * @param  max_opciones: Cantidad total de opciones en el menú actual.
 */
static void Procesar_Navegacion_Encoder(uint8_t max_opciones) {
	int32_t delta = encoder_contador - ultimo_encoder;
	if (delta > 0) {
		menu_index++;
		if (menu_index >= max_opciones)
			menu_index = 0; // Rollover hacia arriba
		ultimo_encoder = encoder_contador;
		LCD_Clear(); // Limpiamos pantalla al cambiar para no dejar rastros
	} else if (delta < 0) {
		if (menu_index == 0)
			menu_index = max_opciones - 1; // Rollover hacia abajo
		else
			menu_index--;
		ultimo_encoder = encoder_contador;
		LCD_Clear();
	}
}

/**
 * @brief Renderiza un menú desplazable de 16x2.
 * @param titulo_menu: Arreglo de strings con los nombres.
 * @param total: Número total de opciones.
 * @param index: Índice actual seleccionado.
 */
static void LCD_MostrarMenu(const char *opciones[], uint8_t total,
		uint8_t index) {
	char buffer[17];

	// Fila 0: Muestra la opción seleccionada con una flecha
	sprintf(buffer, "> %-14s", opciones[index]);
	LCD_Print(0, 0, buffer);

	// Fila 1: Muestra la siguiente opción como contexto (si existe)
	uint8_t next_index = (index + 1) % total;
	sprintf(buffer, "  %-14s", opciones[next_index]);
	LCD_Print(1, 0, buffer);
}

/**
 * @brief Lógica para leer el botón Select (PB14) optimizada para filtros RC físicos.
 * @retval 1 si el botón fue presionado de forma válida, 0 en caso contrario.
 */
static uint8_t Leer_Boton_Select(void) {
	static uint8_t boton_presionado_anterior = 0;

	// FILTRO DE SEGURIDAD: Ignorar transitorios lógicos en el microsegundo de arranque
	if (HAL_GetTick() < 500) {
		return 0;
	}

	// Leemos el estado eléctrico actual en el pin PB14
	uint8_t estado_pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

	// Si el pin está en RESET (0V) significa que el botón está activado físicamente
	if (estado_pin == GPIO_PIN_RESET) {
		// Candado lógico: Solo dispara el clic la primera vez que detecta el cambio
		if (!boton_presionado_anterior) {
			boton_presionado_anterior = 1; // Bloqueamos para que no repita en el lazo
			return 1; // Retorna ÉXITO inmediatemente
		}
	} else {
		// Cuando la rampa del capacitor sube y supera el umbral de 3.3V (botón suelto)
		boton_presionado_anterior = 0; // Liberamos el candado para el próximo clic
	}

	return 0; // Sin clics nuevos
}

/**
 * @brief Lógica para leer el botón Speed (PB12) con filtro RC.
 */
static uint8_t Leer_Boton_Speed(void) {
	static uint8_t boton_speed_anterior = 0;

	if (HAL_GetTick() < 500) return 0; // Filtro de arranque

	uint8_t estado_pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); // PB12

	if (estado_pin == GPIO_PIN_RESET) {
		if (!boton_speed_anterior) {
			boton_speed_anterior = 1;
			return 1;
		}
	} else {
		boton_speed_anterior = 0;
	}
	return 0;
}

static uint8_t Leer_Boton_Sync(void) {
	static uint8_t boton_sync_anterior = 0;
	if (HAL_GetTick() < 500) return 0;

	uint8_t estado_pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); // PB13

	if (estado_pin == GPIO_PIN_RESET) {
		if (!boton_sync_anterior) {
			boton_sync_anterior = 1;
			return 1;
		}
	} else {
		boton_sync_anterior = 0;
	}
	return 0;
}

/* =========================================================================
 * CONFIGURACIÓN DE HARDWARE: ENCODER Y BOTONES
 * ========================================================================= */
void Interfaz_Controles_Init(void) {
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* 1. ENCODER ROTATIVO (TIM4 - Pines PB6 y PB7) */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;

	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;

	HAL_TIM_Encoder_Init(&htim4, &sConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// PB14 (Select), PB13 (Sync) y PB12 (Speed) pasan a entrada pura para sus filtros RC físicos
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Interfaz_LeerEncoder(void) {
	// Se divide por 4 debido al modo TI12 que cuenta los 4 estados de la cuadratura
	encoder_contador = (int32_t) (__HAL_TIM_GET_COUNTER(&htim4)) / 4;
}

/* =========================================================================
 * LÓGICA PRINCIPAL DE INICIALIZACIÓN DE LA INTERFAZ
 * ========================================================================= */
void Interfaz_InitLogica(void) {
	// 1. Inicializar hardware del usuario (Encoder y Botones)
	Interfaz_Controles_Init();

	// 2. Inicializar Pantalla LCD
	lcd_i2c = &hi2c1;

	// Espera crítica para que el voltaje de 5V de la pantalla se estabilice
	HAL_Delay(100);

	// Secuencia oficial de hardware (HD44780) para forzar el paso de 8-bit a 4-bit
	LCD_SendNibble(0x30);
	HAL_Delay(5);
	LCD_SendNibble(0x30);
	HAL_Delay(1);
	LCD_SendNibble(0x30);
	HAL_Delay(1);
	LCD_SendNibble(0x20);
	HAL_Delay(1); // ¡A partir de aquí ya estamos en 4-bit!

	// Ahora sí podemos usar la función normal de comandos
	LCD_SendCommand(0x28);
	HAL_Delay(2); // Función Set: 4-bit, 2 líneas, 5x8
	LCD_SendCommand(0x08);
	HAL_Delay(2); // Display OFF
	LCD_SendCommand(0x01);
	HAL_Delay(5); // Clear Display
	LCD_SendCommand(0x06);
	HAL_Delay(2); // Entry Mode Set
	LCD_SendCommand(0x0C);
	HAL_Delay(2); // Display ON, Cursor OFF
}

/* =========================================================================
 * MÁQUINA DE ESTADOS PRINCIPAL (FSM)
 * ========================================================================= */
void Interfaz_UpdateFSM(void) {
	// 1. Lectura obligatoria de hardware
	Interfaz_LeerEncoder();

	// --- SISTEMA DE LATCHING (CANDADO DE MEMORIA) ---
	static uint8_t flag_clic_pendiente = 0;
	static uint8_t flag_speed_pendiente = 0;
	static uint8_t flag_sync_pendiente = 0;

	if (Leer_Boton_Select()) flag_clic_pendiente = 1;
	if (Leer_Boton_Speed()) flag_speed_pendiente = 1;
	if (Leer_Boton_Sync()) flag_sync_pendiente = 1;


	char buffer1[20];
	char buffer2[20];

	// 2. Control de Tasa de Refresco (200ms)
	static uint32_t ultimo_refresco = 0;
	if (HAL_GetTick() - ultimo_refresco < 200) {
		return;
	}
	ultimo_refresco = HAL_GetTick();

	// 3. Descargamos los clics pendientes
	uint8_t btn_presionado = flag_clic_pendiente;
	flag_clic_pendiente = 0;

	uint8_t btn_speed_pres = flag_speed_pendiente;
	flag_speed_pendiente = 0;

	uint8_t btn_sync_pres = flag_sync_pendiente;
	flag_sync_pendiente = 0;

	// 4. Evaluación de Estados
	switch (currentState) {

	/* --------------------------------------------------
	 * ARRANQUE Y VERIFICACIÓN
	 * -------------------------------------------------- */
	case STATE_BOOTING:
		LCD_Print(0, 0, "ASTRO-MOUNT v1.0");

		// Revisamos dos condiciones: GPS válido y Homing de motores terminado
		if (gps_actual.latitud != 0.0f && flag_homing_ok) {
			currentState = STATE_BOOT_SUCCESS;
			boot_timer = HAL_GetTick(); // Guardamos el tiempo de inicio
			LCD_Clear();
		} else {
			LCD_Print(1, 0, "Wait GPS & Home.");
		}
		break;

	case STATE_BOOT_SUCCESS:
		// Mostramos señal exitosa por 3 segundos
		sprintf(buffer1, "GPS OK! SATS: 0 "); // TODO: Poner variable de satélites real
		sprintf(buffer2, "Hora: %02d:%02d:%02d",
				(int) gps_actual.ut_horas / 10000,
				((int) gps_actual.ut_horas % 10000) / 100,
				(int) gps_actual.ut_horas % 100);

		LCD_Print(0, 0, buffer1);
		LCD_Print(1, 0, buffer2);

		if (HAL_GetTick() - boot_timer > 3000) {
			currentState = STATE_MAIN_MENU;
			menu_index = 0;
			LCD_Clear();
		}
		break;

		/* --------------------------------------------------
		 * MENÚ PRINCIPAL
		 * -------------------------------------------------- */
	case STATE_MAIN_MENU: {
		const char *opciones_main[] = { "Modo Manual", "Modo Offline",
				"Modo Online", "Informacion" };
		Procesar_Navegacion_Encoder(4);
		LCD_MostrarMenu(opciones_main, 4, menu_index);

		if (btn_presionado) {
			LCD_Clear();
			if (menu_index == 0)
				currentState = STATE_MANUAL;
			if (menu_index == 1) {
				currentState = STATE_OFFLINE_CATALOGO;
				menu_index = 0;
			}
			if (menu_index == 2)
				currentState = STATE_ONLINE;
			if (menu_index == 3) {
				currentState = STATE_INFO;
				menu_index = 0;
			}
		}
		break;
	}

		/* --------------------------------------------------
		 * MODO 1: MANUAL (Joystick Activo)
		 * -------------------------------------------------- */
	case STATE_MANUAL: {
		// 1. Si se presiona el botón SPEED, ciclamos la velocidad
		if (btn_speed_pres) {
			VelocidadModo_t nueva_vel;
			if (velocidad_actual == SPEED_GUIAR)
				nueva_vel = SPEED_CENTRAR;
			else if (velocidad_actual == SPEED_CENTRAR)
				nueva_vel = SPEED_BUSCAR;
			else
				nueva_vel = SPEED_GUIAR;

			Motores_SetVelocidadGlobal(nueva_vel);
		}

		// 2. Determinamos el texto para la LCD según la variable global
		const char *str_vel;
		if (velocidad_actual == SPEED_GUIAR)
			str_vel = "GUIAR";
		else if (velocidad_actual == SPEED_CENTRAR)
			str_vel = "CENTR";
		else
			str_vel = "BUSCA";

		// 3. Imprimimos en pantalla (Fila 0: IMU, Fila 1: Velocidad y Salida)
		sprintf(buffer1, "Z:%5.1f Y:%5.1f ", imu_actual.orientacion_z,
				imu_actual.inclinacion_y);
		LCD_Print(0, 0, buffer1);

		sprintf(buffer2, "V:%s [SEL=OUT]", str_vel); // Ej: "V:BUSCA [SEL=OUT]"
		LCD_Print(1, 0, buffer2);

		// 4. Si se presiona el SELECT, salimos al menú
		if (btn_presionado) {
			currentState = STATE_MAIN_MENU;
			menu_index = 0;
			LCD_Clear();
		}
		break;
	}

		/* --------------------------------------------------
		 * MODO 2: OFFLINE (Base de Datos)
		 * -------------------------------------------------- */
	case STATE_OFFLINE_CATALOGO: {
		const char *opciones_cat[] = { "Messier", "Estrellas", "< Volver" };
		Procesar_Navegacion_Encoder(3);
		LCD_MostrarMenu(opciones_cat, 3, menu_index);

		if (btn_presionado) {
			LCD_Clear();
			if (menu_index == 2) {
				currentState = STATE_MAIN_MENU;
				menu_index = 1;
			} else {
				catalogo_seleccionado = menu_index; // 0 o 1
				total_objetos_actual = (catalogo_seleccionado == 0) ? 20 : 10;
				currentState = STATE_OFFLINE_OBJETO;
				menu_index = 0;
			}
		}
		break;
	}

	case STATE_OFFLINE_OBJETO: {
		// El número de opciones es el total de la base de datos + 1 (Botón Volver)
		uint8_t opciones_totales = total_objetos_actual + 1;
		Procesar_Navegacion_Encoder(opciones_totales);

		// Renderizado dinámico leyendo directamente de astronomia.c (ROM)
		char buf_fila0[17];
		char buf_fila1[17];

		// Fila 0
		if (menu_index < total_objetos_actual) {
			sprintf(buf_fila0, "> %-14s",
					Astronomia_ObtenerObjeto(catalogo_seleccionado, menu_index)->nombre);
		} else {
			sprintf(buf_fila0, "> %-14s", "< Volver");
		}

		// Fila 1
		uint8_t next_idx = (menu_index + 1) % opciones_totales;
		if (next_idx < total_objetos_actual) {
			sprintf(buf_fila1, "  %-14s",
					Astronomia_ObtenerObjeto(catalogo_seleccionado, next_idx)->nombre);
		} else {
			sprintf(buf_fila1, "  %-14s", "< Volver");
		}

		LCD_Print(0, 0, buf_fila0);
		LCD_Print(1, 0, buf_fila1);

		if (btn_presionado) {
			LCD_Clear();
			if (menu_index == total_objetos_actual) { // < Volver
				currentState = STATE_OFFLINE_CATALOGO;
				menu_index = catalogo_seleccionado;
			} else {
				objeto_seleccionado = menu_index; // Guardamos en RAM el objeto elegido
				currentState = STATE_OFFLINE_ACCION;
				menu_index = 0;
			}
		}
		break;
	}

	case STATE_OFFLINE_ACCION: {
			const char *opciones_acc[] = { "Apuntar", "Iniciar Track", "< Volver" };
			Procesar_Navegacion_Encoder(3);
			LCD_MostrarMenu(opciones_acc, 3, menu_index);

			if (btn_presionado) {
				LCD_Clear();
				if (menu_index == 2) {
					currentState = STATE_OFFLINE_OBJETO;
					menu_index = objeto_seleccionado;
				} else {
					const ObjetoCeleste_t *obj = Astronomia_ObtenerObjeto(
							catalogo_seleccionado, objeto_seleccionado);
					target_actual.ra = obj->ra;
					target_actual.dec = obj->dec;

					Astronomia_CalcularAltAz();

					if (menu_index == 0) { // Opcion: Apuntar (GoTo)
						LCD_Print(0, 0, "Calculando...   ");
						sprintf(buffer2, "Z:%.0f Y:%.0f     ", target_actual.azimut,
								target_actual.altitud);
						LCD_Print(1, 0, buffer2);
						HAL_Delay(1000);
						LCD_Clear();

						Motores_Apuntar(target_actual.azimut, target_actual.altitud);
						currentState = STATE_MOVIENDO;
					} else if (menu_index == 1) { // Opcion: Iniciar Track directo
						currentState = STATE_TRACKING;
					}
				}
			}
			break;
		}

	/* --------------------------------------------------
		 * 1. LLEGADA DEL GOTO (STATE_MOVIENDO)
		 * -------------------------------------------------- */
		case STATE_MOVIENDO:
			LCD_Print(0, 0, "Moviendo Tubo...");
			LCD_Print(1, 0, "[SELECT = STOP]");

			if (btn_presionado) {
				Motores_DetenerGoTo();
				LCD_Clear();
				LCD_Print(0, 0, "Viaje Cancelado ");
				HAL_Delay(1500);
				currentState = STATE_MAIN_MENU;
				menu_index = 0;
				LCD_Clear();
			}
			else if (flag_goto_terminado_az && flag_goto_terminado_alt) {
				LCD_Clear();
				LCD_Print(0, 0, "Llegada Exitosa!");
				HAL_Delay(1000);
				LCD_Clear();

				// Al llegar, entramos a la fase de encuadre y sincronización
				currentState = STATE_CALIBRACION_FINA;
				menu_index = 0;
			}
			break;

		/* --------------------------------------------------
		 * 2. CALIBRACIÓN FINA (Joystick + Speed + Botón SYNC)
		 * -------------------------------------------------- */
		case STATE_CALIBRACION_FINA: {
			// Permitir cambiar velocidad con el botón SPEED
			if (btn_speed_pres) {
				VelocidadModo_t nueva_vel;
				if (velocidad_actual == SPEED_GUIAR) nueva_vel = SPEED_CENTRAR;
				else if (velocidad_actual == SPEED_CENTRAR) nueva_vel = SPEED_BUSCAR;
				else nueva_vel = SPEED_GUIAR;

				Motores_SetVelocidadGlobal(nueva_vel);
			}

			const char *str_vel;
			if (velocidad_actual == SPEED_GUIAR) str_vel = "GUIAR";
			else if (velocidad_actual == SPEED_CENTRAR) str_vel = "CENTR";
			else str_vel = "BUSCA";

			// Mostrar interfaz de sincronización
			LCD_Print(0, 0, "Centre & [SYNC] ");
			sprintf(buffer2, "V:%s [SYNC=OK] ", str_vel);
			LCD_Print(1, 0, buffer2);

			// Si el usuario centra con el joystick y oprime el botón SYNC (PB13)
			if (btn_sync_pres) {
				Astronomia_SyncOffset(imu_actual.inclinacion_y, imu_actual.orientacion_z);

				LCD_Clear();
				LCD_Print(0, 0, "Sincronizado OK!");
				HAL_Delay(1200);
				LCD_Clear();

				// Tras sincronizar con éxito, pasamos al menú de decisión de tracking
				currentState = STATE_POST_SYNC;
				menu_index = 0;
			}

			// Si presiona SELECT sin hacer sync, pasa directo al menú post-sync o menú principal
			if (btn_presionado) {
				currentState = STATE_POST_SYNC;
				menu_index = 0;
				LCD_Clear();
			}
			break;
		}

		/* --------------------------------------------------
		 * 3. MENÚ DE DECISIÓN POST-SYNC (STATE_POST_SYNC)
		 * -------------------------------------------------- */
		case STATE_POST_SYNC: {
			const char *opciones_post[] = { "Iniciar Tracking", "< Volver" };
			Procesar_Navegacion_Encoder(2);
			LCD_MostrarMenu(opciones_post, 2, menu_index);

			if (btn_presionado) {
				LCD_Clear();
				if (menu_index == 0) {
					currentState = STATE_TRACKING;
					LCD_Clear();
				} else {
					currentState = STATE_MAIN_MENU;
					menu_index = 0;
					LCD_Clear();
				}
			}
			break;
		}

		/* --------------------------------------------------
		 * 4. TRACKING ACTIVO (STATE_TRACKING_ACTIVO)
		 * -------------------------------------------------- */
	case STATE_TRACKING: {
		// Mostrar coordenadas y estado en vivo
		LCD_Print(0, 0, ">> TRACKING <<  ");
		sprintf(buffer2, "Z:%4.0f Y:%4.0f", imu_actual.orientacion_z,
				imu_actual.inclinacion_y);
		LCD_Print(1, 0, buffer2);

		// Ejecutar la rutina de seguimiento 1 vez por segundo (sin bloquear la CPU)
		static uint32_t ultimo_tick_tracking = 0;
		uint32_t tick_actual = HAL_GetTick();
		uint32_t delta_t = tick_actual - ultimo_tick_tracking;

		// Refresco cada 1000 ms (1 segundo)
		if (delta_t >= 1000) {
			ultimo_tick_tracking = tick_actual;

			// 1. Avanzar la simulación del reloj interno de la Tierra
			Astronomia_AvanzarTiempo(delta_t);

			// 2. Recalcular matemáticamente las coordenadas teóricas actualizadas
			Astronomia_CalcularAltAz();

			// 3. Inyectar la diferencia en los motores (El telescopio se mueve suavemente)
			Motores_PasoSideral(target_actual.azimut, target_actual.altitud);
		}

		// Si el usuario cancela con SELECT
		if (btn_presionado) {
			// Limpiar acumuladores y regresar
			Motores_DetenerGoTo();
			currentState = STATE_MAIN_MENU;
			menu_index = 0;
			LCD_Clear();
		}
		break;
	}
		/* --------------------------------------------------
		 * MODO 3: ONLINE (Serial)
		 * -------------------------------------------------- */
		case STATE_ONLINE: {
			    char buf_debug[17];
			    LCD_Print(0, 0, "LINK: STELLARIUM");
			    LCD_Print(1, 0, "Sync:[SYNC Btn] ");

			    // Si el usuario centra la estrella manualmente tras el GoTo de Stellarium y oprime SYNC
			    if (btn_sync_pres) {
			        Astronomia_SyncOffset(imu_actual.inclinacion_y, imu_actual.orientacion_z);

			        LCD_Clear();
			        LCD_Print(0, 0, "Sync Online OK!");
			        HAL_Delay(1200);
			        LCD_Clear();
			    }

			    if (btn_presionado) {
			        currentState = STATE_MAIN_MENU;
			        menu_index = 2;
			        LCD_Clear();
			    }
			    break;
			}

		/* --------------------------------------------------
		 * INFO EXTRA (Coordenadas y Euler)
		 * -------------------------------------------------- */
	case STATE_INFO: {
		// Ahora usamos el encoder para scrollear 3 páginas
		Procesar_Navegacion_Encoder(3);

		if (menu_index == 0) { // Página 1: Lat/Lon
			sprintf(buffer1, "Lat:%5.2f %c  ",
					(gps_actual.latitud >= 0) ?
							gps_actual.latitud : -gps_actual.latitud,
					(gps_actual.latitud >= 0) ? 'N' : 'S');
			sprintf(buffer2, "Lon:%5.2f %c  ",
					(gps_actual.longitud >= 0) ?
							gps_actual.longitud : -gps_actual.longitud,
					(gps_actual.longitud >= 0) ? 'E' : 'W');
			LCD_Print(0, 0, buffer1);
			LCD_Print(1, 0, buffer2);

		} else if (menu_index == 1) { // Página 2: Ángulos Euler y Calibración
					sprintf(buffer1, "Z:%5.1f Y:%5.1f ", imu_actual.orientacion_z,
							imu_actual.inclinacion_y);
					sprintf(buffer2, "Mag Calib: %d    ", imu_actual.estado_calibracion);
					LCD_Print(0, 0, buffer1);
					LCD_Print(1, 0, buffer2);
				}

		 else { // Página 3: Fecha, Hora UTC y Estado GPS
			uint8_t horas = (uint8_t) gps_actual.ut_horas;
			float temp_min = (gps_actual.ut_horas - horas) * 60.0f;
			uint8_t minutos = (uint8_t) temp_min;
			uint8_t segundos = (uint8_t) ((temp_min - minutos) * 60.0f);

			if (gps_coordenadas_fijadas) {
				sprintf(buffer1, "D:%02d/%02d/%02d [3D]", gps_actual.dia,
						gps_actual.mes, (gps_actual.anio % 100));
			} else {
				sprintf(buffer1, "D:%02d/%02d/%02d [WT]", gps_actual.dia,
						gps_actual.mes, (gps_actual.anio % 100));
			}
			sprintf(buffer2, "T:%02d:%02d:%02d UTC ", horas, minutos, segundos);

			LCD_Print(0, 0, buffer1);
			LCD_Print(1, 0, buffer2);
		}

		if (btn_presionado) {
			currentState = STATE_MAIN_MENU;
			menu_index = 3; // Para que al volver, el cursor siga sobre "Informacion"
			LCD_Clear();
		}
		break;
	}

		/* --------------------------------------------------
		 * ESTADO DE ERROR
		 * -------------------------------------------------- */
	case STATE_ERROR:
		LCD_Print(0, 0, "ERROR CRITICO   ");
		LCD_Print(1, 0, "Revise sensores ");
		break;
	}
}

