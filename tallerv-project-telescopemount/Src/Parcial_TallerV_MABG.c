/**
 ******************************************************************************
 * @file           : Parcial_TallerV_MABG.c
 * @author         : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief          : Parcial - Pantalla LCD 20X4 y Acelerometro MPU 6050
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *
 *** Mapeo de Pines:
 ** BLINKY TIM4 -> PC13
 *
 ** USART2 - AF01:
 * - Tx -> PA2
 * - Rx -> PA3
 *
 ** I2C1 - AF04:
 * - SCL -> PB8
 * - SDA -> PB9
 *
 ** MCO1 - AF00 -> PA8
 *
 ** Parámetros de Configuración para Terminal Serial:
 * - Baudrate : 19200
 * - Bits de Datos : 8 --\
 * - Paridad : No      -- |--> 8N1
 * - Bit de Parada : 1 --/
 *
 ** Caracteres de Recepción o Envío desde la Terminal Serial:
 * - 'H' : Cambia la salida del MCO al HSI
 * - 'L' : Cambia la salida del MCO al LSE
 * - 'P' : Cambia la salida del MCO al PLL
 * - 'F' : Cambia el formato de la hora entre formato de 12 h y 24 h
 * - 'U' : Cambia las unidades de la aceleración mostrada y transmitida entre g y m/s2
 *
 * ===================== FUNCIONAMIENTO =====================
 *
 * El siguiente programa implementa un sistema embebido de monitoreo en tiempo real
 * integrando la lectura de sensores, medición de tiempo, despliegue visual y
 * control por puerto serial mediante varios periféricos:
 *
 *  - Un bus I2C1 (PB8-SCL, PB9-SDA), compartido por dos dispositivos:
 *      1. Un expansor PCF8574T (dirección 0x20) que controla una pantalla LCD 20x4
 *         en modo de 4 bits para desplegar la fecha, la hora, las aceleraciones en
 *         los tres ejes (X, Y, Z) y la fuente actual del reloj MCO.
 *      2. Un sensor IMU MPU6050 (dirección 0x68) del cual se extraen y convierten
 *         los datos crudos de aceleración en lecturas físicas (g o m/s2).
 *
 *  - Un periférico RTC (Real-Time Clock), alimentado por el cristal LSE (32.768 kHz),
 *         encargado de mantener la hora y la fecha actualizadas continuamente. Hace uso
 *         de un registro de respaldo (Backup Register DR0) para preservar la hora
 *         configurada tras reinicios del sistema.
 *
 *  - Una salida de reloj MCO1 (PA8), utilizada para conmutar y monitorear la señal
 *         del reloj del sistema seleccionando dinámicamente entre HSI, LSE y PLL.
 *
 *  - Una comunicación serial USART2 (PA2-Tx, PA3-Rx a 19200 baudios):
 *      1. Vía recepción (Rx por interrupción): Procesa comandos ingresados por teclado
 *         ('H', 'L', 'P' para cambiar la fuente del MCO1; 'F' para alternar formato de
 *         hora 12h/24h; 'U' para cambiar las unidades de aceleración entre g y m/s2).
 *      2. Vía transmisión (Tx por interrupción): Transmite periódicamente hacia la
 *         consola serial la trama completa de datos formateados reflejados en el LCD.
 *
 *  - Un temporizador TIM4, configurado para generar interrupciones periódicas cada
 *         250 ms utilizadas para controlar el LED testigo (Blinky en PC13) y servir
 *         como base de tiempo.
 *
 * Durante la inicialización se configuran el reloj del sistema (PLL a 100 MHz), los
 * GPIOs y periféricos, se verifica la presencia del MPU6050 mediante WHO_AM_I, se
 * inicializa el LCD 20x4 y se transmite un menú de bienvenida por USART2.
 * Posteriormente, el programa ejecuta una Máquina de Estados Finita (FSM) dentro del
 * bucle principal while(1) que alterna entre reposo (IDLE), lectura de sensores/RTC
 * (READ_DATA) y actualización del display junto con el envío serial por interrupción
 * (UPDATE_LCD) cada segundo
 *
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


/* TIM4 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim4;

/* I2C1 handle — must be global so stm32f4xx_it.c can access it */
I2C_HandleTypeDef hi2c1;

/* USART2 handle — must be global so stm32f4xx_it.c can access it */
UART_HandleTypeDef huart2 = {0};

/* RTC handle — must be global so stm32f4xx_it.c can access it */
RTC_HandleTypeDef hrtc;


uint8_t msg_buffer[256] = {0}; //Arerglo donde estará el mensaje dinámico a transmitir USART2 Tx - max 256 caracteres
uint8_t startupMsg[] =
"\r\n"
"=====================================\r\n"
"      MPU6050 + LCD 20x4 + RTC\r\n"
"=====================================\r\n"
"Comandos disponibles:\r\n"
"\r\n"
"H --> MCO = HSI\r\n"
"L --> MCO = LSE\r\n"
"P --> MCO = PLL\r\n"
"F --> Hora 12h / 24h\r\n"
"U --> g / m/s^2\r\n"
"\r\n"
"=====================================\r\n"; //Mensaje Inicial del UART Tx

uint8_t rx_data = 0; //Variable donde se almacena el caracter recibido de la consola para su interpretación

/* Dirección I2C MPU 6050 */
#define MPU6050_ADDR              (0x68 << 1)

/* Registros MPU 6050 */
#define MPU6050_WHO_AM_I          0x75  //Registro donde se encuentra la identificación (Address 0x68)
#define MPU6050_PWR_MGMT_1        0x6B  //Registro donde se escribe 0x00 para sacar la MPU de estado en reposo

#define MPU6050_ACCEL_XOUT_H      0x3B  //Registro donde se encuentra la medición en X de la aceleración realizada por el MPU
#define MPU6050_ACCEL_YOUT_H      0x3D  //Registro donde se encuentra la medición en Y de la aceleración realizada por el MPU
#define MPU6050_ACCEL_ZOUT_H      0x3F  //Registro donde se encuentra la medición en Z de la aceleración realizada por el MPU

#define MPU6050_TEMP_OUT_H        0x41  //Registro donde se encuentra la medición de temperatura realizada por el MPU

#define MPU6050_GYRO_XOUT_H       0x43  //Registro donde se encuentra la medición en X del giroscopio realizada por el MPU
#define MPU6050_GYRO_YOUT_H       0x45  //Registro donde se encuentra la medición en Y del giroscopio realizada por el MPU
#define MPU6050_GYRO_ZOUT_H       0x47  //Registro donde se encuentra la medición en Z del giroscopio realizada por el MPU


/* Buffer para comunicación I2C */

uint8_t mpu_buffer[14] = {0}; //Arreglo donde se guarda los valosres leidos de la acelaración de la MPU

/* Variables de acelerómetro */

int16_t accel_x_raw = 0;
int16_t accel_y_raw = 0;
int16_t accel_z_raw = 0;

/* Variables de giroscopio */

int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;

/* Temperatura */

int16_t temp_raw = 0;

/* Valores convertidos */

float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;

float gyro_x = 0.0f;
float gyro_y = 0.0f;
float gyro_z = 0.0f;

float temperature = 0.0f;

/* WHO_AM_I */

uint8_t who_am_i = 0;

/* Estado de la comunicación  para la MPU*/

HAL_StatusTypeDef mpu_status = HAL_OK;

static HAL_StatusTypeDef MPU6050_Init(void); //Funcion que despierta el sensor y verifica que responda a la direccion

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value); //Función de escritura en los registros de la MPU

static HAL_StatusTypeDef MPU6050_ReadRegister(uint8_t reg, uint8_t *value); //Función de lectura de los registros de la MPU

static HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t reg, uint8_t *buffer,  //Función que permite leer varios registros al tiempo y guardalos en un arreglo que se indica con el puntero *buffer
		uint8_t length);

static void MPU6050_ReadAccel(void); //Función que lee y procesa los datos de aceleración que se leen de los registros de la MPU

static void MPU6050_ReadGyro(void);  //Función que lee y procesa los datos del giroscopio que se leen de los registros de la MPU

static void MPU6050_ReadTemperature(void); //Función que lee y procesa el dato de temperatura que se lee del registro de la MPU


/* Dirección I2C del PCF8574T */
#define LCD_ADDR            (0x20 << 1)

/*
 * LCD_RS (Register Select) -> Conectado al pin P0 la placa que comunica I2C con la pantalla LCD
 * Si RS = 0: Le decimos a la pantalla que vamos a enviarle un "Comando" (ej. limpiar pantalla).
 * Si RS = 1: Le decimos que vamos a enviarle "Datos" (ej. imprimir la letra 'A').
 */
#define LCD_RS              0x01

/*
 * LCD_RW (Read/Write) -> Conectado al pin P1
 * Si RW = 0: Escribimos en la pantalla.
 * Si RW = 1: Leemos de la pantalla.
 */
#define LCD_RW              0x02

/*
 * LCD_EN (Enable) -> Conectado al pin P2
 */
#define LCD_EN              0x04

/*
 * LCD_BACKLIGHT -> Conectado al pin P3
 * Controla el LED de retroiluminación de la pantalla.
 */
#define LCD_BACKLIGHT       0x08


static void LCD_SendCommand(uint8_t cmd);             // Envía una instrucción de control, ya sea mover el cursor o apagar la pantalla.
static void LCD_SendData(uint8_t data);               // Envía un solo carácter (letra o número) para que se dibuje en la pantalla.
static void LCD_Init(void);                           // Despierta y configura la pantalla por primera vez al arrancar el sistema.
static void LCD_Clear(void);                          // Borra el texto visible y devuelve el cursor al inicio.
static void LCD_SetCursor(uint8_t row, uint8_t col);  // Ubica el cursor invisible en la fila y columna exacta donde queremos escribir.
static void LCD_Print(char *text);                    // Toma una frase completa y la imprime letra por letra en la pantalla.
static void LCD_Write4Bits(uint8_t data);             // Empaqueta los datos en trozos de 4 bits

// Arreglos para formateo de texto
char lcd_line1[21] = {0};  // Texto preparado para la fila 1 de la pantalla. 20 Caracteres más el carácter nulo
char lcd_line2[21] = {0};  // Texto preparado para la fila 2 de la pantalla.
char lcd_line3[21] = {0};  // Texto preparado para la fila 3 de la pantalla.
char lcd_line4[21] = {0};  // Texto preparado para la fila 4 de la pantalla.

char uart_msg[150] = {0};  // Texto preparado para ser transmitido por el puerto serial (UART).
/* Bandera que se activa cada 1 segundo desde la interrupción del TIM4 */
volatile uint8_t flag_send_uart = 0;

/*=========================================================
 * RTC
 *=========================================================*/

RTC_TimeTypeDef rtc_time = {0};
RTC_DateTypeDef rtc_date = {0};

typedef enum
{
    CLOCK_HSI = 0,
    CLOCK_LSE,
    CLOCK_PLL
} ClockSource_t;

ClockSource_t currentClock = CLOCK_PLL;

uint8_t hourFormat = 24;      //24 o 12

uint8_t accelUnit = 0;        //0 = g   1 = m/s2

// --- VARIABLES PARA LA FSM ---
typedef enum {
    FSM_IDLE = 0, //Modo reposo
    FSM_READ_DATA, //Comienza el modo de lectura y despliegue de la FSM
    FSM_UPDATE_LCD //Actualiza la LCD y la tramisión Serial con los nuevos datos y reinicia la FSM
} SystemState_t;

SystemState_t currentState = FSM_IDLE; // Estado inicial
uint32_t previousTick = 0;             // Almacena el último tiempo registrado
const uint32_t UPDATE_INTERVAL = 500;  // Tiempo en ms para actualizar los datos (500 ms)

/* Private function prototypes */
static void SystemClock_Config(void);
static void gpio_Init(void); //Función que inicializa y configura el puerto PH1 del blinky
static void tim4_Init(void); //Función asociada a la inicialización y configuración del TIM4(Blinky)
static void usart2_Init(void); //Función asociada a la inicialización y configuración del USART2 (Rx, Tx)
static void mco1_Init(void); //Función asociada a la configuración del PA8 para el MCO1
static void i2c1_Init(void); //Función asociada a la incialización y configuración del periferico I2C
static void rtc_Init(void); //Función asociada a la incialización y configuración del periferico RTC
static void RTC_Read(void); //Función asociada a la lectura de los datos del RTC
static void Update_Display_And_UART(void);  //Función asociada al procesamiento y actualizción de los datos que se enviarán al display y a la terminal serial

int main(void)
{
    HAL_Init();           /* initialize HAL: SysTick, cache, priority grouping */
    SystemClock_Config(); /* configure clock tree: HSI at 16 MHz               */
    gpio_Init();          /* configure PH1 as push-pull output                  */
    tim4_Init();          /* configure TIM4: update event every 250 ms          */

    usart2_Init();
    mco1_Init ();
    i2c1_Init();
    rtc_Init();
    mpu_status = MPU6050_Init();

    if(mpu_status == HAL_OK)
    {
        sprintf((char*)msg_buffer,
                 "WHO_AM_I = 0x%02X\r\n",
                 who_am_i);

        HAL_UART_Transmit(&huart2,
                          msg_buffer,
                          strlen((char*)msg_buffer),
                          100);

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"MPU6050 Inicializado\r\n",
                          23,
                          100);
    }
    else
    {
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)"Error Inicializando MPU6050\r\n",
                          30,
                          100);
    }

   /* HAL_UART_Transmit(&huart2,
                         (uint8_t *)"\r\nEscaneando I2C...\r\n",
                         21,
                         100);

       for(uint8_t addr = 1; addr < 128; addr++)
       {
           if(HAL_I2C_IsDeviceReady(&hi2c1,
                                    addr << 1,
                                    2,
                                    10) == HAL_OK)
           {
               sprintf((char*)msg_buffer,
                       "Dispositivo encontrado: 0x%02X\r\n",
                       addr);

               HAL_UART_Transmit(&huart2,
                                 msg_buffer,
                                 strlen((char*)msg_buffer),
                                 100);
           }
       }

       HAL_UART_Transmit(&huart2,
                         (uint8_t *)"Escaneo finalizado\r\n",
                         20,
                         100);  */

    HAL_UART_Transmit(&huart2, (uint8_t *)startupMsg,strlen((char *)startupMsg), 500);  //Mensaje Inicial de Funcionamiento de la Recepción Rx via Tx

    LCD_Init(); //Inicializa la LCD

    LCD_Clear(); //Limpia todo caracter que esté en pantalla apra empezar a mostrar lo nuevo


	while (1) {
		// 1. Condición de transición de tiempo (Reemplazo del HAL_Delay)
		// Comprueba si han pasado 500 ms desde la última vez
		if ((HAL_GetTick() - previousTick) >= UPDATE_INTERVAL) {
			previousTick = HAL_GetTick(); // Reinicia el cronómetro

			// Si el sistema estaba descansando, inicia un nuevo ciclo de lectura
			if (currentState == FSM_IDLE) {
				currentState = FSM_READ_DATA;
			}
		}

		// 2. Ejecución de la Máquina de Estados
		switch (currentState) {

		case FSM_READ_DATA:
			RTC_Read();
			MPU6050_ReadAccel();
			currentState = FSM_UPDATE_LCD;
			break;

		case FSM_UPDATE_LCD:
			Update_Display_And_UART(); // Llamamos a nuestra nueva superfunción
			currentState = FSM_IDLE; // Terminamos y volvemos al estado de reposo
			break;


		case FSM_IDLE:
		default:
			break;
		}
	}
}

/*
 * SystemClock_Config
 * Uses HSI internal oscillator at 16 MHz
 * No PLL — simplest possible clock configuration
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI is already on at reset — confirm and use it */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON; //Activa el PLL encargado de llevar la MCU a 100 MHz a partir del HSI

    //Configuración del PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; //Tomará como fuente para el PLL el HSI
    RCC_OscInitStruct.PLL.PLLM = 16; // 1er divisor 16 MHz(HSI) / 16 = 1 MHz
    RCC_OscInitStruct.PLL.PLLN = 400; // 1er multiplicador VCO -> 1 MHz * 400 = 400 MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 2do divisor para reloj principal SYSCLK -> 400 MHz / 4 = 100 MHz

    HAL_RCC_OscConfig(&RCC_OscInitStruct); //Cargano la configuración en los registros FSR del MCU

    /* Select PLL as SYSCLK — all bus dividers set to 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | //Todos estos relojes tendrán esta configuracion, reloj principal SYSCLK
                                       RCC_CLOCKTYPE_HCLK   | //reloj AHB HCLK
                                       RCC_CLOCKTYPE_PCLK1  | //reloj APB1 PCLK1
                                       RCC_CLOCKTYPE_PCLK2;   //reloj APB2 PCLK2
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;  // EL reloj principal del sistema (SYSCLK) utilizará como fuente el PLL, PLLCLK si se usará PLL
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 100 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     /* APB1  = 50 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 100 MHz */
    RCC_OscInitStruct.PLL.PLLQ = 7;

    // FLASH_LATENCY: Para  100 MHz, por ejemplo, se debe agregar ciclos de espera ya que la Flash no puede ir tan rapido como el CLK (3 ciclos -> Tabla 5 Manual de Referencia, CAP 3)
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

/*
 * gpio_Init
 * Configures PH1 as push-pull output — onboard LED on Nucleo board (PH1 tactic board)
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOC clock on AHB1 bus
       Same as bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure PC13 */
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); //Cargamos la configuracion en los registros FSR del MCU
    __NOP();
}


//USART2 init configuracion
//19200 8N1
static void usart2_Init(void){

	/* Enable GPIOA clock on AHB1 bus */
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	 GPIO_InitTypeDef GPIO_Init_Tx = {0};

	 GPIO_Init_Tx.Pin = GPIO_PIN_2; //Pin PA2 (Tx)
	 GPIO_Init_Tx.Mode = GPIO_MODE_AF_PP;
	 GPIO_Init_Tx.Pull = GPIO_NOPULL;
	 GPIO_Init_Tx.Speed = GPIO_SPEED_FREQ_HIGH;
	 GPIO_Init_Tx.Alternate = GPIO_AF7_USART2;

	 HAL_GPIO_Init(GPIOA, &GPIO_Init_Tx);

	 GPIO_InitTypeDef GPIO_Init_Rx = {0};

	GPIO_Init_Rx.Pin = GPIO_PIN_3; //Pin PA3 (Rx)
	GPIO_Init_Rx.Mode = GPIO_MODE_AF_PP;
	GPIO_Init_Rx.Pull = GPIO_PULLUP; //Configurando resistencia PullUp para que luego de recibir, el pin quede un estado alto.
	GPIO_Init_Rx.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init_Rx.Alternate = GPIO_AF7_USART2;


	 //Cargar la configuracion en los registros FSR del MCU
	 HAL_GPIO_Init(GPIOA, &GPIO_Init_Rx);

	 __NOP();


	/* Enable USART2 clock on APB1 bus */
	 __HAL_RCC_USART2_CLK_ENABLE();

	 huart2.Instance = USART2;
	 huart2.Init.BaudRate = 19200; //19200 bit / segundo, para 8N1 serían aprox 1920 bytes / s (Ya que es formato 8N1 son 1 Start, 8 Datos y 1 Stop (10 Bit)
	 huart2.Init.Mode = USART_MODE_TX_RX; //Modo transmisión (Tx) y recepcion (Rx)
	 huart2.Init.Parity = USART_PARITY_NONE;
	 huart2.Init.StopBits = USART_STOPBITS_1; //1 bit de parada
	 huart2.Init.WordLength = USART_WORDLENGTH_8B; //8N1

	 //Cargando la configuracion en los registros FSR del MCU
	 HAL_UART_Init(&huart2);
	 //Registrar la interrupcion en el NVIC para la recepcion Rx
	 HAL_NVIC_EnableIRQ(USART2_IRQn);

	 //Iniciando la Recepcion
	 HAL_UART_Receive_IT(&huart2, &rx_data, 1); //Guardará el dato en rx_data de a 1 byte

}


/*
 * tim4_Init
 * Configures TIM4 to generate an update event every 250 ms
 *
 * Clock chain:
 *   PLL (100 MHz) → APB1 (100 MHz) → TIM4 clock (100 MHz)
 *
 * PSC = 9999 →  tick = 16,000,000 / (9999 + 1) = 10 000 Hz  (0.1 ms per tick)
 * ARR = 2499    →  period = (2499 + 1) x 0.1 ms = 250 ms
 */
static void tim4_Init(void)
{
    /* Enable TIM4 clock on APB1 bus */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Configure TIM4 base */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 9999; // 100 MHz / (9999 + 1) = 10 kHz (un tick cada 0.1 ms)
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 2499; // 0.1 ms * (2499 + 1) = 250 ms exactos
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim4); //Cargando la configuracion en los registros FSR del MCU

    /* Enable TIM4 interrupt line in the NVIC */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    /* Start TIM4 in interrupt mode — enables the update event interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
    __NOP();

}

static void i2c1_Init(void){

	//Configuración de los pines GPIO PB8 y PB9 quienes serán SCL y SDA respectivamente
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_I2C_Init = {0};

	GPIO_I2C_Init.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_I2C_Init.Mode = GPIO_MODE_AF_OD;
	GPIO_I2C_Init.Pull = GPIO_PULLUP;
	GPIO_I2C_Init.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_I2C_Init.Alternate = GPIO_AF4_I2C1;

	HAL_GPIO_Init(GPIOB,&GPIO_I2C_Init); //Cargamos la configuración en los registros FSR del MCU

	//Configuración del I2C1
	__HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000; //Configuración para 100 kHz, velocidad standar
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; //Relaión 2:1 al 50%
	hi2c1.Init.OwnAddress1 = 0; //Como el micro es el master, no tenemos dirección
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c1) != HAL_OK){
		__NOP(); //Cargamos la configuración en los registros FSR del MCU
	}
}

/*
 * ============================================================
 * MPU6050_Init
 *
 * Inicializa el acelerómetro.
 *
 * 1. Verifica WHO_AM_I.
 * 2. Despierta el sensor.
 * ============================================================
 */

static HAL_StatusTypeDef MPU6050_Init(void)
{
    /* Leer identificación */

    mpu_status =
        MPU6050_ReadRegister(MPU6050_WHO_AM_I,
                             &who_am_i);  //Pregunta al sensor cual es su dirección

    if(mpu_status != HAL_OK)
        return mpu_status;

    /* Debe devolver 0x68 */

    if(who_am_i != 0x68)
        return HAL_ERROR;

    /* Despertar el MPU */

    mpu_status =
        MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x00); // Despierta el sensor de su modo reposo enviando ceros al registro PWR_MGMT

    HAL_Delay(100);

    return mpu_status;
}

/*
 * ============================================================
 * MPU6050_WriteRegister
 *
 * Escribe un único byte dentro de un registro del MPU6050.
 *
 * Parámetros:
 * reg   -> registro donde escribir.
 * value -> dato a almacenar.
 *
 * Retorna:
 * HAL_OK si la escritura fue correcta o HAL_ERROR de acuerdo a si HAL_I2C_Mem_Write es exitoso o no quien es la encargada como tal de escribir en los registros de la MPU.
 * ============================================================
 */

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/*
 * ============================================================
 * MPU6050_ReadRegister
 *
 * Lee un byte desde cualquier registro interno de la MPU hacia el MCU.
 *
 * reg -> registro a leer.
 *
 * value -> variable donde guardar el dato.
 * SE usa HAL_I2C_Mem_Read ya que se puede obtener un (solo uno) dato exacto de un registro específico de la MPU.
 * ============================================================
 */

static HAL_StatusTypeDef MPU6050_ReadRegister(uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

/*
 * ============================================================
 * MPU6050_ReadBytes
 *
 * Lee varios registros consecutivos del MPU6050.
 *
 * reg    -> primer registro donde empezará la lectura.
 * buffer -> donde guardará los datos.
 * length -> cantidad de bytes a leer.
 * ============================================================
 */

static HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
	return HAL_I2C_Mem_Read(&hi2c1,
	MPU6050_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, buffer, length, 100);
}

/*
 * ============================================================
 * Función: MPU6050_ReadAccel
 * Descripción: Extrae los datos crudos del acelerómetro para
 *              los 3 ejes (X, Y, Z) mediante una lectura en ráfaga,
 *              los reconstruye a 16 bits y los convierte a
 *              fuerza G (gravedad) real.
 * ============================================================
 */
static void MPU6050_ReadAccel(void)
{
    MPU6050_ReadBytes(MPU6050_ACCEL_XOUT_H,
                      mpu_buffer,
                      6);

    accel_x_raw =
            (mpu_buffer[0]<<8) | mpu_buffer[1]; //Coge los primeros 8 bits almacenados en mpu_buffer [0] los shiftea 8 posiciones y hace un OR con mpu_buffer[1] para reconstruir el tamaño completo de la lectura en X que son 16 bits y se guarda en accel_x_raw

    accel_y_raw =
            (mpu_buffer[2]<<8) | mpu_buffer[3]; //Coge los primeros 8 bits almacenados en mpu_buffer [2] los shiftea 8 posiciones y hace un OR con mpu_buffer[3] para reconstruir el tamaño completo de la lectura en Y que son 16 bits y se guarda en accel_y_raw

    accel_z_raw =
            (mpu_buffer[4]<<8) | mpu_buffer[5]; //Coge los primeros 8 bits almacenados en mpu_buffer [4] los shiftea 8 posiciones y hace un OR con mpu_buffer[6] para reconstruir el tamaño completo de la lectura en Z que son 16 bits y se guarda en accel_z_raw

    accel_x =
            accel_x_raw / 16384.0f; //Según su hoja de datos (Datasheet), en este rango, 1 'g' equivale a 16384

    accel_y =
            accel_y_raw / 16384.0f; //Según su hoja de datos (Datasheet), en este rango, 1 'g' equivale a 16384

    accel_z =
            accel_z_raw / 16384.0f; //Según su hoja de datos (Datasheet), en este rango, 1 'g' equivale a 16384
}

static void MPU6050_ReadGyro(void)
{
    MPU6050_ReadBytes(MPU6050_GYRO_XOUT_H,
                      mpu_buffer,
                      6);

    gyro_x_raw =
        (mpu_buffer[0]<<8) | mpu_buffer[1];

    gyro_y_raw =
        (mpu_buffer[2]<<8) | mpu_buffer[3];

    gyro_z_raw =
        (mpu_buffer[4]<<8) | mpu_buffer[5];

    gyro_x =
        gyro_x_raw / 131.0f;

    gyro_y =
        gyro_y_raw / 131.0f;

    gyro_z =
        gyro_z_raw / 131.0f;
}

static void MPU6050_ReadTemperature(void)
{
    MPU6050_ReadBytes(MPU6050_TEMP_OUT_H,
                      mpu_buffer,
                      2);

    temp_raw =
        (mpu_buffer[0]<<8) | mpu_buffer[1];

    temperature =
        (temp_raw / 340.0f) + 36.53f;
}


/*
 * ============================================================
 * LCD_Write4Bits
 *
 * Envía un nibble al PCF8574.
 *
 * Se genera el pulso Enable.
 * ============================================================
 */

static void LCD_Write4Bits(uint8_t data) {
	uint8_t value;

	value = data | LCD_BACKLIGHT; //Prepara el valor del bit que se encarga de encender la pantalla

	HAL_I2C_Master_Transmit(&hi2c1,
	LCD_ADDR, &value, 1, 100); //Envia el comando que enciende la pantalla

	value |= LCD_EN; //SE le agrega el valor del bit que se encarga de habilitar la pantalla para funcionar

	HAL_I2C_Master_Transmit(&hi2c1,
	LCD_ADDR, &value, 1, 100);  //Envia el comando que habilita la pantalla

	HAL_Delay(1);

	value &= ~LCD_EN;

	HAL_I2C_Master_Transmit(&hi2c1,
	LCD_ADDR, &value, 1, 100);

	HAL_Delay(1);
}

/*
 * ============================================================
 * LCD_SendCommand
 *
 * Envía un comando al controlador HD44780.
 *
 * RS = 0 -> Se trata de un comando.
 * El byte se envía en dos nibbles:
 *      primero los 4 bits altos
 *      luego los 4 bits bajos
 * ============================================================
 */

static void LCD_SendCommand(uint8_t cmd)
{
    /* Parte alta */

    LCD_Write4Bits(cmd & 0xF0);  //Con la mascara 0xF0 y un & se garantiza que mantenga los primeros 4 bits  desde el más significativo en el valor de cmd y los demás esten limpios en 0

    /* Parte baja */

    LCD_Write4Bits((cmd << 4) & 0xF0);

    HAL_Delay(2);
}

/*
 * ============================================================
 * LCD_SendData
 *
 * Envía un carácter ASCII a la LCD. También lo divide en dos nibbles de 4 bits,
 * pero enciende el pin RS para avisarle a la LCD que es texto.
 *
 * RS = 1 -> Se trata de un dato.
 * ============================================================
 */

static void LCD_SendData(uint8_t data)
{
    /* Parte alta */

    LCD_Write4Bits((data & 0xF0) | LCD_RS);

    /* Parte baja */

    LCD_Write4Bits(((data << 4) & 0xF0) | LCD_RS);

    HAL_Delay(2);
}

/*
 * ============================================================
 * LCD_Init
 *
 * Inicializa la pantalla LCD HD44780 en modo de 4 bits.
 *
 * Secuencia tomada de la hoja de datos del controlador HD44780.
 *
 * Configuración final:
 *  - Interfaz de 4 bits.
 *  - 2 líneas (válido también para LCD 20x4).
 *  - Cursor apagado.
 *  - Display encendido.
 *  - Incremento automático del cursor.
 * ============================================================
 */

static void LCD_Init(void)
{
    /* Esperar a que la LCD termine su encendido */

    HAL_Delay(50);

    /*----------------------------------------------------------
     * La LCD inicialmente está en modo de 8 bits.
     * Se envía tres veces el comando 0x30.
     *---------------------------------------------------------*/

    LCD_Write4Bits(0x30);
    HAL_Delay(5);

    LCD_Write4Bits(0x30);
    HAL_Delay(5);

    LCD_Write4Bits(0x30);
    HAL_Delay(5);

    /*----------------------------------------------------------
     * Cambiar definitivamente a modo de 4 bits.
     *---------------------------------------------------------*/

    LCD_Write4Bits(0x20);
    HAL_Delay(5);

    /*----------------------------------------------------------
     * Function Set
     *
     * DL = 0  -> 4 bits
     * N  = 1  -> 2 líneas
     * F  = 0  -> fuente 5x8
     *---------------------------------------------------------*/

    LCD_SendCommand(0x28);

    /*----------------------------------------------------------
     * Display OFF
     *---------------------------------------------------------*/

    LCD_SendCommand(0x08);

    /*----------------------------------------------------------
     * Clear Display
     *---------------------------------------------------------*/

    LCD_SendCommand(0x01);
    HAL_Delay(2);

    /*----------------------------------------------------------
     * Entry Mode Set
     *
     * Cursor avanza automáticamente.
     *---------------------------------------------------------*/

    LCD_SendCommand(0x06);

    /*----------------------------------------------------------
     * Display ON
     *
     * Display encendido
     * Cursor apagado
     * Blink apagado
     *---------------------------------------------------------*/

    LCD_SendCommand(0x0C);
}

/*
 * ============================================================
 * LCD_Clear
 *
 * Limpia completamente la pantalla y posiciona el cursor
 * en la primera fila y primera columna.
 * ============================================================
 */

static void LCD_Clear(void)
{
    LCD_SendCommand(0x01); //LImpia los registros donde etá la información que muestra la pantalla

    HAL_Delay(2);
}

/*
 * ============================================================
 * LCD_SetCursor
 *
 * Posiciona el cursor en la fila y columna indicadas.
 *
 * row:
 *      0 -> fila 1
 *      1 -> fila 2
 *      2 -> fila 3
 *      3 -> fila 4
 * ============================================================
 */

static void LCD_SetCursor(uint8_t row,
                          uint8_t col)
{
    uint8_t address;

    switch(row)
    {
        case 0:
            address = 0x00 + col;
            break;

        case 1:
            address = 0x40 + col;
            break;

        case 2:
            address = 0x14 + col;
            break;

        case 3:
            address = 0x54 + col;
            break;

        default:
            address = 0x00;
            break;
    }

    LCD_SendCommand(0x80 | address);
}

/*
 * ============================================================
 * LCD_Print
 *
 * Escribe una cadena de caracteres en la posición actual
 * del cursor.
 * ============================================================
 */

static void LCD_Print(char *text)
{
    while(*text)
    {
        LCD_SendData(*text);

        text++;
    }
}


/*
 * mco1_Init
 *
 * Configura PA8 como salida MCO1
 */
static void mco1_Init(void)
{
    /* Habilitar reloj del GPIOA */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = GPIO_PIN_8;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Iniciando el MCO
    HAL_RCC_MCOConfig(
                RCC_MCO1,
                RCC_MCO1SOURCE_PLLCLK,
                RCC_MCODIV_4);
     //1. Usar la salida del MCO1 (PA8), 2. El reloj que sacará al iniciar por este pin será PLL

}

//Función que define cual será el reloj que saldrá por el MCO1
static void MCO_SetClock(ClockSource_t clock)
{
    switch(clock)
    {
        case CLOCK_HSI:

            HAL_RCC_MCOConfig(
                    RCC_MCO1,
                    RCC_MCO1SOURCE_HSI,
                    RCC_MCODIV_1);

            currentClock = CLOCK_HSI;

            break;

        case CLOCK_LSE:

            HAL_RCC_MCOConfig(
                    RCC_MCO1,
                    RCC_MCO1SOURCE_LSE,
                    RCC_MCODIV_1);

            currentClock = CLOCK_LSE;

            break;

        case CLOCK_PLL:

            HAL_RCC_MCOConfig(
                    RCC_MCO1,
                    RCC_MCO1SOURCE_PLLCLK,
                    RCC_MCODIV_4);

            currentClock = CLOCK_PLL;

            break;
    }
}


/*
 * ==========================================================
 * rtc_Init
 *
 * Configura el RTC interno utilizando el cristal LSE
 * de 32.768 kHz.
 *
 * El RTC continuará funcionando con la batería VBAT
 * aunque el micro se quede sin alimentación.
 * ==========================================================
 */

static void rtc_Init(void)
{
    RCC_OscInitTypeDef RCC_OscInit = {0};

    /*---------------------------------------
     * Activar el LSE
     *--------------------------------------*/

    RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_LSE;

    RCC_OscInit.PLL.PLLState = RCC_PLL_NONE;

    RCC_OscInit.LSEState = RCC_LSE_ON;

    HAL_RCC_OscConfig(&RCC_OscInit);

    /*---------------------------------------
     * Seleccionar LSE como reloj del RTC
     *--------------------------------------*/

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;

    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /*---------------------------------------
     * Habilitar reloj RTC
     *--------------------------------------*/

    __HAL_RCC_RTC_ENABLE();

    /*---------------------------------------
     * Configuración RTC
     *--------------------------------------*/

    hrtc.Instance = RTC;

    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;

    // Divisor síncrono (255). La fórmula (127+1) x (255+1) = 32,768.
    // Esto convierte los 32,768 Hz del LSE en exactamente 1 Hz (1 segundo real)
    hrtc.Init.AsynchPrediv = 127;

    hrtc.Init.SynchPrediv = 255;

    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;

    HAL_RTC_Init(&hrtc);

    /*---------------------------------------
     * Si el RTC nunca ha sido configurado,
     * cargar una fecha inicial.
     *--------------------------------------*/

    // Leemos un registro especial de memoria (Backup Register) que se mantiene vivo con una batería.
    // Si NO encuentra nuestra "clave" (0x1234), significa que la placa es nueva o se le acabó la pila.
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x2422) {
		rtc_time.Hours = 1;
		rtc_time.Minutes = 22;
		rtc_time.Seconds = 0;

		HAL_RTC_SetTime(&hrtc, &rtc_time,
		RTC_FORMAT_BIN);

		rtc_date.Date = 24;
		rtc_date.Month = RTC_MONTH_JULY;
		rtc_date.Year = 26;
		rtc_date.WeekDay = RTC_WEEKDAY_FRIDAY;

		HAL_RTC_SetDate(&hrtc, &rtc_date,
		RTC_FORMAT_BIN);

		// Guardamos la clave "0x1234" en el registro de respaldo.
		// La próxima vez que se reinicie la placa, el 'if' de arriba sabrá que ya hay
		// una hora configurada y NO la sobreescribirá.
		HAL_RTCEx_BKUPWrite(&hrtc,
		RTC_BKP_DR0, 0x2422);
	}
}

/*
 * ==========================================================
 * RTC_Read
 *
 * Lee la hora y la fecha actuales almacenadas en el RTC.
 *
 * La información queda almacenada en las variables globales:
 *
 *      rtc_time
 *      rtc_date
 *
 * IMPORTANTE:
 * El RTC de STM32 exige leer primero la hora y luego la fecha,
 * ya que la lectura de la fecha desbloquea los registros
 * internos del calendario.
 * ==========================================================
 */

static void RTC_Read(void)
{
    /* Leer primero la hora */

	HAL_RTC_GetTime(&hrtc, &rtc_time,
	RTC_FORMAT_BIN);

    /* Leer inmediatamente la fecha */

	HAL_RTC_GetDate(&hrtc, &rtc_date,
	RTC_FORMAT_BIN);
}

static void ToggleHourFormat(void)
{
    if(hourFormat == 24)
        hourFormat = 12;
    else
        hourFormat = 24;
}

static void ToggleAccelUnits(void)
{
    accelUnit ^= 1;
}


static void UART_ProcessCommand(uint8_t cmd)
{
    switch(cmd)
    {

        case 'H':

            MCO_SetClock(CLOCK_HSI);

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"MCO -> HSI\r\n",
                    12,
                    100);

        break;


        case 'L':

            MCO_SetClock(CLOCK_LSE);

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"MCO -> LSE\r\n",
                    12,
                    100);

        break;


        case 'P':

            MCO_SetClock(CLOCK_PLL);

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"MCO -> PLL\r\n",
                    12,
                    100);

        break;


        case 'F':

            ToggleHourFormat();

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"Formato Hora cambiado\r\n",
                    24,
                    100);

        break;


        case 'U':

            ToggleAccelUnits();

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"Unidades cambiadas\r\n",
                    21,
                    100);

        break;

        default:

            HAL_UART_Transmit(&huart2,
                    (uint8_t*)"Comando invalido\r\n",
                    18,
                    100);

        break;
    }
}


static void Update_Display_And_UART(void) {
    // 1. Manejo de Hora (Formato 12h / 24h)
    uint8_t hour_display = rtc_time.Hours;
    char am_pm[3] = {0};

    if (hourFormat == 12) {
        if (hour_display == 0) {
            hour_display = 12;
            strcpy(am_pm, "AM");
        } else if (hour_display == 12) {
            strcpy(am_pm, "PM");
        } else if (hour_display > 12) {
            hour_display -= 12;
            strcpy(am_pm, "PM");
        } else {
            strcpy(am_pm, "AM");
        }
        // Se formatea cuidando de no superar 20 caracteres
        sprintf(lcd_line1, "%02d/%02d/%02d %02d:%02d:%02d%s",
                rtc_date.Date, rtc_date.Month, rtc_date.Year,
                hour_display, rtc_time.Minutes, rtc_time.Seconds, am_pm);
    } else {
        sprintf(lcd_line1, "%02d/%02d/%02d %02d:%02d:%02d  ",
                rtc_date.Date, rtc_date.Month, rtc_date.Year,
                hour_display, rtc_time.Minutes, rtc_time.Seconds);
    }

    // 2. Manejo de Aceleración (g o m/s2)
    float ax = accel_x;
    float ay = accel_y;
    float az = accel_z;
    char unit_str[5] = "g   "; // Relleno con espacios para limpiar la LCD

    if (accelUnit == 1) {
        // Multiplicamos por la gravedad de la Tierra
        ax *= 9.8f;
        ay *= 9.8f;
        az *= 9.8f;
        strcpy(unit_str, "m/s2");
    }

    sprintf(lcd_line2, "Ax:%4.1f Ay:%4.1f %s", ax, ay, unit_str);
    sprintf(lcd_line3, "Az:%4.1f %s", az, unit_str);

    // 3. Manejo de fuente de reloj (MCO1)
    char clk_str[5] = {0};
    if (currentClock == CLOCK_HSI) strcpy(clk_str, "HSI");
    else if (currentClock == CLOCK_LSE) strcpy(clk_str, "LSE");
    else if (currentClock == CLOCK_PLL) strcpy(clk_str, "PLL");

    sprintf(lcd_line4, "Reloj MCO: %s", clk_str);

    // 4. Actualizar LCD línea por línea
    LCD_SetCursor(0,0); LCD_Print(lcd_line1);
    LCD_SetCursor(1,0); LCD_Print(lcd_line2);
    LCD_SetCursor(2,0); LCD_Print(lcd_line3);
    LCD_SetCursor(3,0); LCD_Print(lcd_line4);

	// 5. Transmitir por UART solo si la interrupción del TIM4 dio la orden (cada 1s)
	if (flag_send_uart == 1) {
		flag_send_uart = 0; // Limpiamos la bandera para el siguiente segundo

		sprintf(uart_msg,
				"\r\n--- DATOS ACTUALES ---\r\n%s\r\n%s\r\n%s\r\n%s\r\n",
				lcd_line1, lcd_line2, lcd_line3, lcd_line4);

		HAL_UART_Transmit_IT(&huart2, (uint8_t*) uart_msg,
							strlen(uart_msg));
	}
}


/*
 * HAL_TIM_PeriodElapsedCallback
 * Called automatically by HAL_TIM_IRQHandler() every time a timer
 * update event fires. Shared by all timers — always check htim->Instance.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        // Contador estático para llegar a 1 segundo
		static uint8_t tim4_counter = 0;
		tim4_counter++;

		if (tim4_counter >= 4) // 4 * 250 ms = 1000 ms (1 segundo)
				{
			tim4_counter = 0;
			flag_send_uart = 1; // ¡Activamos la bandera de transmisión!
		}

    }
}


//Callback de la ISR generada por el UART2 Rx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART2) {
		UART_ProcessCommand(rx_data);

		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}





