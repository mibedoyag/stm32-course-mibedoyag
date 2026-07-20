/**
 ******************************************************************************
 * @file           : Tarea3_TallerV_MABG.c
 * @author         : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief          : Tarea 3 - LED RGB con señales PWM dirigidas por Encoder, Pot. (ADC) y USART
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
 ** PWM TIM1 - AF01:
 *
 * - CH1 (Green) -> PA8
 * - CH2 (Blue)  -> PA9
 * - CH3 (Red)   -> PA10
 *
 ** ENCODER TIM2 - AF01:
 * - CH1 (encoder: CLK) -> PA0
 * - CH2 (Encoder_ DT)  -> PA1
 *
 ** ADC TIM3 (TRGO):
 * - ADC -> PA6
 *
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
 * ===================== FUNCIONAMIENTO ===================== *
 *
 * El  siguiente programa implementa el control de un LED RGB utilizando tres periféricos
 * independientes del STM32F411RE:
 *
 *  - Un encoder rotativo, configurado mediante el TIM2 en modo Encoder, para
 *    controlar el duty cycle  del PWM del canal 3 -> pwm_red.
 *
 *  - Un potenciómetro, leído mediante el ADC1 disparado periódicamente por el
 *    evento TRGO generado por el TIM3, para controlar el duty cycle del PWM del
 *    canal 1 -> pwm_green.
 *
 *  - Una comunicación serial UART2, mediante interrupciones de recepción,
 *    para modificar el duty cycle del PWM del canal 2 -> pwm_blue a partir de comandos
 *    enviados desde un terminal serial, los cuales se especificaron más arriba debajo
 *    del mapeo de pines.
 *
 * Las tres señales PWM son generadas por el TIM1 utilizando sus canales
 * CH1, CH2 y CH3 con una frecuencia de 2 kHz y que inician todos en 0.
 * Adicionalmente, el TIM4 genera una interrupción periódica utilizada
 * únicamente para el blinky.
 *
 * Durante la inicialización se configuran todos los periféricos, se habilitan
 * las interrupciones correspondientes y se envía un mensaje inicial por UART
 * con las instrucciones de operación para la recepción Rx de comandos.
 * Posteriormente, el programa permanece ejecutándose dentro del while, donde
 * procesa los eventos generados por cada periférico, actualiza las variables
 * asociadas, modifica los duty cycle de las señales PWM y transmite por UART
 * el estado actual del sistema únicamente cuando se detecta un cambio significativo
 * en alguno de los dispositivos de entrada (Encoder, ADC o Rx del UART).
 *
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"

/* TIM1 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim1;

/* TIM2 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim2;

/* TIM3 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim3;

/* TIM4 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim4;

/* ADC1 handle — must be global so stm32f4xx_it.c can access it */
ADC_HandleTypeDef hadc1 = {0};

/* USART2 handle — must be global so stm32f4xx_it.c can access it */
UART_HandleTypeDef huart2 = {0};

DMA_HandleTypeDef hdma_tim1 = {0}; //Handle para DMA


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


volatile uint8_t adc_chaged = 0; //Variable volatil que cambia cuando hay un cambio en el valor de ADC para pasar a hacer Tx
volatile uint16_t raw_adc = 0; //Variable volatil donde se almacena el valor de la conversión ADC
volatile uint8_t adc_done = 0; //Variable volatil que se modifica y lleva a la ejecución del Callback del ADC
float adc_value_mv = 0.0f; //Variable donde se guarda y convierte el valor en mV a partir del raw_adc ADC
uint16_t adc_old = 0; //Varibale con la que se compara el valor del ADC para así saber si se debe transmitir Tx dado el cambio

volatile uint8_t encoder_changed = 0; //Variable volatil que cambia cuando hay un cambio en el valor del encoder para pasar a hacer Tx
volatile int16_t encoder_steps = 0; //Variable dedicada a almacenar los pasos dados y actuales del encoder -> CW ++ y CCW --
volatile uint8_t encoder_dir = 0; //Variable que indicará la dirección del encoder (0 = CW, 1 = CCW)
char *dir_string = 0; //Variable tipo string (caracter) que indica CW si es 0 y CCW si es 1 para enviar en Tx USART2
uint16_t encoder_old = 0; //Varibale con la que se compara el valor del encoder para así saber si se debe transmitir Tx dado el cambio

uint8_t usart_clicks = 0; //Variable dondé estará el número actual de click de acuerdo al comando que reciba de la consola (Rx)
uint8_t rx_data = 0; //Variable donde se almacena el caracter recibido de la consola para su interpretación
volatile uint8_t usart_done = 0; //Variable volatil que se modifica y lleva a la ejecución de la lógica del callback por fuera
volatile uint8_t uart_changed = 0; //Variable volatil que cambia cuando hay una recepción Rx que modifique el PWM para pasar a hacer Tx

volatile uint16_t pwm_red = 0; //Variable que tendra el duty del PWM CH1 (PA10)
volatile uint16_t pwm_green = 0; //Variable que tendra el duty del PWM CH2 (PA8)
volatile uint16_t pwm_blue = 0; //Variable que tendra el duty del PWM CH3 (PA9)

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
static void tim1_pwm_Init(void); //Función asociada a la inicialización y configuración del TIM1 (PWM -> CH1, CH2, CH3)
static void tim4_Init(void); //Función asociada a la inicialización y configuración del TIM4(Blinky)
static void usart2_Init(void); //Función asociada a la inicialización y configuración del USART2 (Rx, Tx)
static void mco2_Init(void); //Función asociada a la configuración del PC9 para el MCO2 con lectura del clock 16 MHz

//Implementación DMA con HAL
void BuildBreathTable(void);
#define BREATH_TABLE_SIZE 200
static uint32_t breathTable[BREATH_TABLE_SIZE]; // <-- static: see note below


int main(void)
{
    HAL_Init();           /* initialize HAL: SysTick, cache, priority grouping */
    SystemClock_Config(); /* configure clock tree: HSI at 16 MHz               */
    gpio_Init();          /* configure PH1 as push-pull output                  */
    tim4_Init();          /* configure TIM4: update event every 250 ms          */

    usart2_Init();
    tim1_pwm_Init();
    mco2_Init ();
    BuildBreathTable(); // Lookup table para el braething led

    HAL_UART_Transmit(&huart2, (uint8_t *)startupMsg,strlen((char *)startupMsg), 500);  //Mensaje Inicial de Funcionamiento de la Recepción Rx via Tx

    while (1)
    {

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
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE; //Desactiva el PLL, si se quiere llevar el MCU a la máxima freq. debe activarse (RCC_PLL_ON;) y luego se debe configurar los divisores y el multiplicador para llegar a la freq. final

    /* Si se quiere configurar el PLL, primero se debe elegir la fuente que tomará el PLL para la conversión, este caso
     * sería HSI, a continuación se configura el primer divisor (PLLM) (Ejm para 100 MHz: PLLM = 16 -> 16MHz / 16 = 1MHz),
     * luego de este iría el multiplicador (PLLN) (PLLN = 400 -> 1MHz * 400 = 400 MHz), se sigue entonces con el siguiente
     * y último divisor para el PLLCLK (PLLP) (RCC_PLLP_DIV4 = 4 -> 400MHz / 4 = 100 MHz y este será el PLLCLK. Hay un
     * divisor más (PLLQ) que no afecta el CPU como tal si no para el USB o el SDIO por ejemplo (PLLQ = 7 ->
     * 400 MHz / 7 = 57 MHz)*/

    HAL_RCC_OscConfig(&RCC_OscInitStruct); //Cargano la configuración en los registros FSR del MCU

    /* Select HSI as SYSCLK — all bus dividers set to 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | //Todos estos relojes tendrán esta configuracion, reloj principal SYSCLK
                                       RCC_CLOCKTYPE_HCLK   | //reloj AHB HCLK
                                       RCC_CLOCKTYPE_PCLK1  | //reloj APB1 PCLK1
                                       RCC_CLOCKTYPE_PCLK2;   //reloj APB2 PCLK2
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;  // EL reloj principal del sistema (SYSCLK) utilizará como fuente el HSI (Internal), PLLCLK si se usará PLL
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 16 MHz */ //Si se utiliza PLL se debe tener en cuenta está división de acuerdo a lo máximo que soporta el BUS
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1  = 16 MHz */ //Si se utiliza PLL se debe tener en cuenta está división de acuerdo a lo máximo que soporta el BUS
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 16 MHz */ //Si se utiliza PLL se debe tener en cuenta está división de acuerdo a lo máximo que soporta el BUS

    /* FLASH_LATENCY_0 = zero wait states, correct for 16 MHz */ //Para  100 MHz, por ejemplo, se debe agregar ciclos de espera ya que la Flash no puede ir tan rapido como el CLK (3 ciclos -> Tabla 5 Manual de Referencia, CAP 3)
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
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


/* Configuración del GPIO y TIM1 y los 3 Canales de las 3 señales PWM (f = 2kHz) */
static void tim1_pwm_Init(void){

	__HAL_RCC_GPIOA_CLK_ENABLE(); //HAbilitando la señal de reloj del GPIOA (AHB1)

	/* Configuración inicial de los GPIO */
	//Configurando el GPIO PA8, PA9, PA10 para el CH1, CH2, CH3 respectivamente
	GPIO_InitTypeDef GPIO_Init_chtim1 = {0};

	GPIO_Init_chtim1.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_Init_chtim1.Mode = GPIO_MODE_AF_PP; //Modo funcion alternativa
	GPIO_Init_chtim1.Pull = GPIO_NOPULL; //No Pull Up ni Pull Down, los niveles altos y bajos serán controlados por el timer
	GPIO_Init_chtim1.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init_chtim1.Alternate = GPIO_AF1_TIM1; //Funcion alternativa AF01 del TIM1

	//Cargando la configuración en los registros FSR del MCU
	HAL_GPIO_Init(GPIOA, &GPIO_Init_chtim1);

	/* Configuración del TIMER 1 Base */
	__HAL_RCC_TIM1_CLK_ENABLE(); //Habilitando la señal de reloj del TIM1 (APB2)

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 160 - 1; //Preescaler a 16 MHz / 1600 =  1 KHz -> 1 ms
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP; //Contador de manera ascendente
	htim1.Init.Period = 1000 -1; //1 seg de periodo, lo que da una frecuencia de 2 kHz para cada canal PWM del TIM1
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; //NO se hace un división adicional del reloj
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	//Cargando la configuración en los registros FSR del MCU
	HAL_TIM_PWM_Init(&htim1);

	//COnfiguración especifica de los Canales CH1, CH2 y CH3 del tim1
	TIM_OC_InitTypeDef ConfigOC_ch =  {0}; //Estructura Output Compare que configura como funciona cada canal

	ConfigOC_ch.OCMode = TIM_OCMODE_PWM1; //PWM Modo 1: Mientras CNT < CCR la salida está en alto, cuando CNT >= CCR la señal pasa a bajo (CCR (Capture/Compare Register) define el duty)
	ConfigOC_ch.Pulse = 0; //CCR inicia en 0 (Duty 0%) -> Led inicia apagado
	ConfigOC_ch.OCPolarity = TIM_OCPOLARITY_HIGH; //En alto el Led estará encendido, al contrario se invierte la logica de las salidas no complementarias (CH1, CH2 y CH3).
	ConfigOC_ch.OCFastMode = TIM_OCFAST_DISABLE; //Espera a terminar un ciclo para cambiar el duty y que no genere deformaciones en el pulso si se cambia al duty a mitad de un ciclo del pwm

	//Cargando la configuración en los registros FSR del MCU para cada Canal
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_3);

	//Iniciando o arrancando las señales en cada canal
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);


	/* Configuración de la DMA para que funcione con el TIM1 - CH1 (PWM) */

	__HAL_RCC_DMA2_CLK_ENABLE(); // must come first — DMA2 registers don't exist until this runs

	hdma_tim1.Instance = DMA2_Stream1;
	hdma_tim1.Init.Channel = DMA_CHANNEL_6;
	hdma_tim1.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_tim1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_tim1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_tim1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_tim1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_tim1.Init.Mode = DMA_CIRCULAR;
	hdma_tim1.Init.Priority = DMA_PRIORITY_MEDIUM;
	hdma_tim1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	HAL_DMA_Init(&hdma_tim1);
	__HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1);

	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC1], (uint32_t)breathTable,
	(uint32_t)&htim1.Instance->CCR1, BREATH_TABLE_SIZE);

	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

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
    htim4.Init.Prescaler         = 15999;
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


/* Configuración del MCO2 en PC9 para la lectura del Clock del MCU (16 MHz) */
static void mco2_Init(void){

	__HAL_RCC_GPIOC_CLK_ENABLE(); //Activando la señal de reloj para el GPIOC

	GPIO_InitTypeDef mco2_Init = {0};

	mco2_Init.Pin = GPIO_PIN_9;
	mco2_Init.Mode = GPIO_MODE_AF_PP; //Modo funcion alternativa
	mco2_Init.Pull = GPIO_NOPULL; //No pull up ni pull down ya que el pin será salida
	mco2_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	mco2_Init.Alternate = GPIO_AF0_MCO; //Funcion alternativa AF00 del MCO2

	//Cargando la configuración a los registros FSR del MCU
	HAL_GPIO_Init(GPIOC, &mco2_Init);

	//Iniciando el MCO
	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1); //1. Usar la salida del MCO2 (PC9), 2. El reloj que sacará por este pin será el principal del sistema(SYSCLK)

}


void BuildBreathTable(void)
{
for (uint32_t i = 0; i < BREATH_TABLE_SIZE; i++)
{
float angle = (float)i / (float)(BREATH_TABLE_SIZE - 1) * 3.14159f; // 0 to π
breathTable[i] = (uint32_t)(sinf(angle) * 999.0f); // scaled to Period
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
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);

    }
}


//Callback de la ISR generada por el UART2 Rx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART2){

		usart_done = 1;
	}
}





