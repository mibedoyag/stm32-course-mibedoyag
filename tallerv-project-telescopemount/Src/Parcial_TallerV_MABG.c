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
 ** BLINKY TIM4 -> PH1
 *
 ** USART2 - AF01:
 * - Tx -> PA2
 * - Rx -> PA3
 *
 ** MCO2 - AF00 -> PC9
 *
 ** Parámetros de Configuración para Terminal Serial:
 * - Baudrate : 19200
 * - Bits de Datos : 8 --\
 * - Paridad : No      -- |--> 8N1
 * - Bit de Parada : 1 --/
 *
 ** Caracteres de Recepción o Envío desde la Terminal Serial:
 * - '+' : Incrementa en 1% el Duty Cycle pwm_blue (CH2).
 * - '-' : Disminuye en 1% el Duty Cycle pwm_blue (CH2).
 * - '0' : Lleva al 0% el Duty Cycle pwm_blue (CH2).
 * - '5' : Lleva al 50% del Duty Cycle pwm_blue (CH2).
 *
 *
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* TIM4 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim4;


/* USART2 handle — must be global so stm32f4xx_it.c can access it */
UART_HandleTypeDef huart2 = {0};

uint8_t msg_buffer[256] = {0}; //Arerglo donde estará el mensaje dinámico a transmitir USART2 Tx - max 256 caracteres
uint8_t startupMsg[] =   //Mensaje inicial, solo se envía una vez al iniciar el main ()
"\r\n"
"========================================\r\n"
" Tarea 3 - Taller V\r\n"
"========================================\r\n"
"Comandos UART:\r\n"
"  + : Incrementa 1 click\r\n"
"  - : Decrementa 1 click\r\n"
"  0 : Reinicia el contador a 0 clicks \r\n"
"  5 : Incrementa 50 clicks\r\n"
"========================================\r\n\r\n"; //Mensaje Inicial del UART Tx

uint8_t usart_clicks = 0; //Variable dondé estará el número actual de click de acuerdo al comando que reciba de la consola (Rx)
uint8_t rx_data = 0; //Variable donde se almacena el caracter recibido de la consola para su interpretación
volatile uint8_t usart_done = 0; //Variable volatil que se modifica y lleva a la ejecución de la lógica del callback por fuera
volatile uint8_t uart_changed = 0; //Variable volatil que cambia cuando hay una recepción Rx que modifique el PWM para pasar a hacer Tx

//FSM ejecución while
typedef enum
{
	STATE_READ_ENCODER = 0,
	STATE_PROCESS_ADC,
	STATE_PROCESS_USART,
	STATE_SEND_MESSAGE
} AppState_t;

AppState_t currentState = STATE_READ_ENCODER; //Al iniciar, siempre estará iniciandose la lectura del encoder para actualizarse

/* Private function prototypes */
static void SystemClock_Config(void);
static void gpio_Init(void); //Función que inicializa y configura el puerto PH1 del blinky
static void tim4_Init(void); //Función asociada a la inicialización y configuración del TIM4(Blinky)
static void usart2_Init(void); //Función asociada a la inicialización y configuración del USART2 (Rx, Tx)
static void mco1_Init(void); //Función asociada a la configuración del PC9 para el MCO2 con lectura del clock 16 MHz

int main(void)
{
    HAL_Init();           /* initialize HAL: SysTick, cache, priority grouping */
    SystemClock_Config(); /* configure clock tree: HSI at 16 MHz               */
    gpio_Init();          /* configure PH1 as push-pull output                  */
    tim4_Init();          /* configure TIM4: update event every 250 ms          */

    usart2_Init();
    mco1_Init ();


    HAL_UART_Transmit(&huart2, (uint8_t *)startupMsg,strlen((char *)startupMsg), 500);  //Mensaje Inicial de Funcionamiento de la Recepción Rx via Tx

    while (1)
    {
    	/* switch(currentState){

    	case STATE_PROCESS_USART:
			if (usart_done == 1) {
				if (rx_data == '+') { //Identificando el carater "+" para que incremente usart_clicks en 1
					if (usart_clicks < 100) {
						usart_clicks++;
						uart_changed = 1; //Si hay un cambio en el valor del clicks recibidos por Rx, cambia la bandera para actualizar Tx

					}
				}

				if (rx_data == '-') { //Identificando el carater "-" para que disminuya usart_clicks en 1
					if (usart_clicks > 0) {
						usart_clicks--;
						uart_changed = 1; //Si hay un cambio en el valor del clicks recibidos por Rx, cambia la bandera para actualizar Tx

					}
				}

				if (rx_data == '0') { //Identificando el carater "0" para que lleve la variable usart_clicks a 0
					usart_clicks = 0;
					uart_changed = 1; //Si hay un cambio en el valor del clicks recibidos por Rx, cambia la bandera para actualizar Tx

				}

				if (rx_data == '5') { //Identificando el carater "5" para que incremente usart_clicks en 50
					usart_clicks += 50;
					uart_changed = 1; //Si hay un cambio en el valor del clicks recibidos por Rx, cambia la bandera para actualizar Tx


					if (usart_clicks >= 100) {
						usart_clicks = 100;
					}
				}


				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_blue); //Asignando el valor de PWM del canal 2 que está en pwm_blue (CCR)
				HAL_UART_Receive_IT(&huart2, &rx_data, 1); //Se vuelve a activar la recepcion de datos ya que cuando recibe algo esta se desactiva y toca volverla a ejercutarla para que espere un nuevo comando

				usart_done = 0; //Limpiando la bandera para que vuelva a entrar
			}

			currentState = STATE_SEND_MESSAGE;

			break;

    	case STATE_SEND_MESSAGE:
			if (encoder_changed || adc_chaged || uart_changed) {  //Si hay algún cambio en alguna de estas variables, se ejecuta el if
				sprintf((char*) msg_buffer,
						"ADC value = %u raw\n\r" "ADC value = %.0f mV\n\r" "Encoder dir: %s, value = %d\n\r" "UART value = %u clicks\n\r\n\r\n\r",
						raw_adc, adc_value_mv, dir_string, encoder_steps,
						usart_clicks);
				HAL_UART_Transmit(&huart2, msg_buffer,
						strlen((char*) msg_buffer) - 1, 100);
				encoder_changed = 0;  //Vuelve la bandera a 0 para esperar si se produce otro cambio para transmitir
				adc_chaged = 0;  //Vuelve la bandera a 0 para esperar si se produce otro cambio para transmitir
				uart_changed = 0;  //Vuelve la bandera a 0 para esperar si se produce otro cambio para transmitir
			}

			currentState = STATE_READ_ENCODER; //Reinia la FSM para que se mantenda en bucle ejecutandose un caso tras el otro

			break;
    	}*/


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

    // FLASH_LATENCY: Para  100 MHz, por ejemplo, se debe agregar ciclos de espera ya que la Flash no puede ir tan rapido como el CLK (3 ciclos -> Tabla 5 Manual de Referencia, CAP 3)
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

    uint32_t sysclk = HAL_RCC_GetSysClockFreq();   // Debería devolver 100000000
    uint32_t hclk   = HAL_RCC_GetHCLKFreq();       // Debería devolver 100000000
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();      // Debería devolver 50000000
    uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();      // Debería devolver 100000000
}

/*
 * gpio_Init
 * Configures PH1 as push-pull output — onboard LED on Nucleo board (PH1 tactic board)
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOH clock on AHB1 bus
       Same as bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN */
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure PH1 */
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct); //Cargamos la configuracion en los registros FSR del MCU
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
	GPIO_Init_Rx.Pull = GPIO_PULLUP; //COnfigurando resistencia PullUp para que luego de recibir, el pin quede un estado alto.
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
 *   HSI (16 MHz) → APB1 (16 MHz) → TIM3 clock (16 MHz)
 *
 * PSC = 15999  →  tick = 16,000,000 / (15999 + 1) = 1,000 Hz  (1 ms per tick)
 * ARR = 249    →  period = (249 + 1) x 1 ms = 250 ms
 */
static void tim4_Init(void)
{
    /* Enable TIM4 clock on APB1 bus */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Configure TIM4 base */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 99999;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 249;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(&htim4); //Cargando la configuracion en los registros FSR del MCU

    /* Enable TIM4 interrupt line in the NVIC */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    /* Start TIM4 in interrupt mode — enables the update event interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
    __NOP();

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
     //1. Usar la salida del MCO1 (PA8), 2. El reloj que sacará por este pin será PLL

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
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);

    }
}


//Callback de la ISR generada por el UART2 Rx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART2){

		usart_done = 1;
	}
}





