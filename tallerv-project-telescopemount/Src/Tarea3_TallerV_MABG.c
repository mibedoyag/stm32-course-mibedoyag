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
 * Mapeo de Pines:
 * - 7 SEGMENTOS:
 *
 * A -> PC9
 * B -> PB8
 * C -> PC11
 * D -> PC12
 * E -> PC10
 * F -> PC8
 * G -> PD2
 *
 * - TRANSISTORES (DIGITOS):
 * D1 (Digito 1) -> PC6
 * D2 (Digito 2) -> PB9
 * D3 (Digito 3) -> PC3
 * D4 (Digito 4) -> PB7
 *
 * - FOTOCOMPUERTAS:
 * F1 (Fotocompuerta 1 - Flanco Subida) -> PC2
 * F2 (Fotocompuerta 2 - Flanco Bajada) -> PA0
 *
 * - BLINKY -> PH1
 *
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"

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

volatile uint8_t showMsg = 0; //Variable que se modifica y lleva a la ejecución del Callback del Tim3 Led Estado (PH1)
uint8_t msg_buffer_enc[64] = {0}; //Arerglo donde estará el mensaje dinámico a transmitir USART2 Tx - max 64 caracteres


volatile uint16_t raw_adc = 0; //Variable volatil donde se almacena el valor de la conversión ADC
volatile uint16_t adc_done = 0; //Variable volatil que se modifica y lleva a la ejecución del Callback del ADC
float adc_value_mv = 0.0f; //Variable donde se guarda el valor en mV de la conversión ADC
uint8_t msg_buffer_adc[64] = {0}; //Arerglo donde estará el mensaje dinámico a transmitir USART2 Tx - max 64 caracteres


volatile int16_t encoder_steps = 0; //Variable dedicada a almacenar los pasos actuales del encoder
volatile uint16_t encoder_old = 0; //
volatile uint8_t encoder_dir = 0; //Variable que indicará la dirección del encoder (0 = CW, 1 = CCW)

uint8_t usart_clicks = 0; //Variable dondé estará el nḿero actual de click de acuerdo al comando que reciba de la consola (Rx)
uint8_t rx_data = 0;
volatile uint8_t usart_done = 0;

volatile uint16_t pwm_red = 0; //Variable que tendra el duty del PWM CH1 (PA8)
volatile uint16_t pwm_green = 0; //Variable que tendra el duty del PWM CH2 (PA9)
volatile uint16_t pwm_blue = 0; //Variable que tendra el duty del PWM CH3 (PA10)



/* Private function prototypes */
static void SystemClock_Config(void);
static void gpio_Init(void); //Función que inicializa y configura el puerto PH1 del blinky
static void tim1_pwm_Init(void); //Función asociada a la inicialización y configuración del TIM1 (PWM -> CH1, CH2, CH3)
static void tim2_encoder_Init(void); //Función asociada a la inicialización y configuración del TIM2 (Encoder -> CH1 (PA0), CH2 (PA1))
static void tim4_Init(void); //Función asociada a la inicialización y configuración del TIM4(Blinky)
static void tim3_adc_Init(void); //Función asociada a la inicialización y configuración del TIM3 (Disparo del ADC con TRGO (PA6))
static void usart2_Init(void); //Función asociada a la inicialización y configuración del USART2 (Rx, Tx)
static void adc_Init(void); //Función asociada a la inicialización y configuración del ADC (PA6 ADC1_CH6)

int main(void)
{
    HAL_Init();           /* initialize HAL: SysTick, cache, priority grouping */
    SystemClock_Config(); /* configure clock tree: HSI at 16 MHz               */
    gpio_Init();          /* configure PA5 as push-pull output                  */
    tim4_Init();          /* configure TIM3: update event every 250 ms          */

    usart2_Init();
    adc_Init();
    tim1_pwm_Init();
    tim2_encoder_Init();
    HAL_ADC_Start_IT(&hadc1);
    tim3_adc_Init();

    HAL_UART_Transmit(&huart2, (uint8_t *)"Hola mundo!!\n\r", 14, 100);

    while (1)
    {

    	encoder_steps = __HAL_TIM_GET_COUNTER(&htim2) / 4;
		pwm_red = encoder_steps * 5;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_red);

		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
			encoder_dir = 1;
		} else {
			encoder_dir = 0;
		}

		if (pwm_red > 499) {
			pwm_red = 499;
		}

        /* application loop — LED toggling happens in the callback */
    	if (showMsg == 1){
    		HAL_UART_Transmit(&huart2, (uint8_t *)"Hola mundo!!\n\r", 14, 100);
    		sprintf((char *)msg_buffer_enc, "Encoder = %u DIR = %u \n\r", encoder_steps, encoder_dir); //Creando el string dinamico con la información en mV
    		HAL_UART_Transmit(&huart2, msg_buffer_enc, strlen((char *)msg_buffer_enc) - 1, 100); // Imprimimos el dato por el puerto serial

    		showMsg = 0;
    	}

    	//HAcer algo con el valor de la conversion ADC
    	if (adc_done == 1){
    		adc_value_mv = (float)((3300.0f / 4095.0f) * raw_adc); //Transformando el valor raw adc en un valor de mV
    		pwm_green = (raw_adc * 499) / 4095; //Realizando la conversión del raw_adc en duty para el pwm_green

    		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_green); //Asignando el valor del duty del pwm_green al PWM del Canal 1.

    		sprintf((char *)msg_buffer_adc, "adc value = %f \n\r", adc_value_mv); //Creando el string dinamico con la información en mV
    		HAL_UART_Transmit(&huart2, msg_buffer_adc, strlen((char *)msg_buffer_adc) - 1, 100); // Imprimimos el dato por el puerto serial

    		adc_done = 0;
    	}

    	if (usart_done == 1){
    		if (rx_data == '+'){
    			if (usart_clicks < 100){
    				usart_clicks ++;
    			}
    		}

    		if (rx_data == '-'){
    			if(usart_clicks > 0){
    				usart_clicks --;
    			}
    		}

    		if (rx_data == '0'){
    			usart_clicks = 0;
    		}

    		if (rx_data == '2'){
    			usart_clicks += 25;

    			if(usart_clicks >= 100){
    				usart_clicks = 100;
    			}
    		}

    		pwm_blue = usart_clicks * 5; //Conversión del numero de clicks en duty del PWM Canal 2.
    		__HAL_TIM_SET_COMPARE (&htim1, TIM_CHANNEL_2, pwm_blue); //Asignando el valor de PWM del canal 2
    		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

    		usart_done = 0;
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
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select HSI as SYSCLK — all bus dividers set to 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 16 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1  = 16 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 16 MHz */

    /* FLASH_LATENCY_0 = zero wait states, correct for 16 MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/*
 * gpio_Init
 * Configures PH1 as push-pull output — onboard LED on Nucleo board (PH1 tactic board)
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOA clock on AHB1 bus
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


/* Configuración del TIM1 y los 3 Canales de las 3 señales PWM (f = 2kHz) */
static void tim1_pwm_Init(void){
	__HAL_RCC_GPIOA_CLK_ENABLE(); //HAbilitando la señal de reloj del GPIOA (AHB1)

	/* Configuración inicial de los GPIO */
	//Configurando el GPIO PA8, PA9, PA10 para el CH1, CH2, CH3 respectivamente
	GPIO_InitTypeDef GPIO_Init_chtim1 = {0};

	GPIO_Init_chtim1.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_Init_chtim1.Mode = GPIO_MODE_AF_PP;
	GPIO_Init_chtim1.Pull = GPIO_NOPULL;
	GPIO_Init_chtim1.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init_chtim1.Alternate = GPIO_AF1_TIM1;

	//Cargando la configuración en los registros FSR del MCU
	HAL_GPIO_Init(GPIOA, &GPIO_Init_chtim1);

	/* COnfiguración del TIMER 1 Base */
	__HAL_RCC_TIM1_CLK_ENABLE(); //Habilitando la señal de reloj del TIM1 (APB2)

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 15; //Preescaler a 16 MHz / 16 =  1 MHz -> 1 us
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 499; //500 us de periodo, lo que da una frecuencia de 2 kHz
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	//Cargando la configuración en los registros FSR del MCU
	HAL_TIM_PWM_Init(&htim1);

	//COnfiguración especifica de los Canales CH1, CH2 y CH3
	TIM_OC_InitTypeDef ConfigOC_ch =  {0};

	ConfigOC_ch.OCMode = TIM_OCMODE_PWM1;
	ConfigOC_ch.Pulse = 0;
	ConfigOC_ch.OCNPolarity = TIM_OCPOLARITY_HIGH;
	ConfigOC_ch.OCFastMode = TIM_OCFAST_DISABLE;

	//Cargando la configuración en los registros FSR del MCU para cada Canal
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel (&htim1, &ConfigOC_ch, TIM_CHANNEL_3);

	//Iniciando o arrancando las señales en cada canal
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);


}

/* Configuración del TIM2 y los 2 Canales de las 2 entradas DT y CLK del encoder */
static void tim2_encoder_Init(void){
	__HAL_RCC_GPIOA_CLK_ENABLE(); //HAbilitando la señal de reloj del GPIOA (AHB1)

	/* Configuración inicial de los GPIO */
	//Configurando el GPIO PA0 y  PA1 para el CH1 y CH2 respectivamente del TIM2
	GPIO_InitTypeDef GPIO_Init_chtim2 = { 0 };

	GPIO_Init_chtim2.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_Init_chtim2.Mode = GPIO_MODE_AF_PP;
	GPIO_Init_chtim2.Pull = GPIO_PULLUP;
	GPIO_Init_chtim2.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init_chtim2.Alternate = GPIO_AF1_TIM2;

	//Cargando la configuración en los registros FSR del MCU
	HAL_GPIO_Init(GPIOA, &GPIO_Init_chtim2);

	/* COnfiguración del TIMER 2 Base */
	__HAL_RCC_TIM2_CLK_ENABLE(); //Habilitando la señal de reloj del TIM2 (APB1)

	TIM_Encoder_InitTypeDef Config_encmode = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	Config_encmode.EncoderMode = TIM_ENCODERMODE_TI12; // Cuenta asc y desc con cada flanco de la señal de cada canal dependiendo del nivel de entrada de la otra señal.


	//COnfiguración especifica de los Canales CH1
	Config_encmode.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING; //Polaridad de entrada con flanco de subida
	Config_encmode.IC1Selection = TIM_ICSELECTION_DIRECTTI; //
	Config_encmode.IC1Prescaler = TIM_ICPSC_DIV1; //Se reliza captura cada que se detecta un flanco en la señal de entrada del CH1
	Config_encmode.IC1Filter = 30;

	//COnfiguración especifica de los Canales CH2
	Config_encmode.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING; //Polaridad de entrada con flanco de subida
	Config_encmode.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	Config_encmode.IC2Prescaler = TIM_ICPSC_DIV1; //Se reliza captura cada que se detecta un flanco en la señal de entrada del CH2
	Config_encmode.IC2Filter = 30;

	//Cargando la configuración en los registros FSR del MCU
	HAL_TIM_Encoder_Init(&htim2, &Config_encmode);

	//Iniciando o arrancando las señales
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);


}
//USART2 init configuracion
//19200 8N1
static void usart2_Init(void){

	/* Enable GPIOA clock on AHB1 bus */
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	 GPIO_InitTypeDef GPIO_Init_Tx_Rx = {0};

	 GPIO_Init_Tx_Rx.Pin = GPIO_PIN_2 | GPIO_PIN_3; //Pin PA2 (Tx) y PA3 (Rx)
	 GPIO_Init_Tx_Rx.Mode = GPIO_MODE_AF_PP;
	 GPIO_Init_Tx_Rx.Pull = GPIO_NOPULL;
	 GPIO_Init_Tx_Rx.Speed = GPIO_SPEED_FREQ_HIGH;
	 GPIO_Init_Tx_Rx.Alternate = GPIO_AF7_USART2;


	 //Cargar la configuracion en los registros FSR del MCU
	 HAL_GPIO_Init(GPIOA, &GPIO_Init_Tx_Rx);

	 __NOP();


	/* Enable USART2 clock on APB1 bus */
	 __HAL_RCC_USART2_CLK_ENABLE();

	 huart2.Instance = USART2;
	 huart2.Init.BaudRate = 19200;
	 huart2.Init.Mode = USART_MODE_TX_RX; //Modo transmisión (Tx) y recepcion (Rx)
	 huart2.Init.Parity = USART_PARITY_NONE;
	 huart2.Init.StopBits = USART_STOPBITS_1; //1 bit de parada
	 huart2.Init.WordLength = USART_WORDLENGTH_8B; //8N1

	 //Cargando la configuracion en los registros FSR del MCU
	 HAL_UART_Init(&huart2);
	 //Registrar la interrupcion en el NVIC
	 HAL_NVIC_EnableIRQ(USART2_IRQn);

	 //Iniciando la Recepcion
	 HAL_UART_Receive_IT(&huart2, &rx_data, 1);

}


//Configurando el ADC Init
static void adc_Init(void){
		/* Enable GPIOA clock on AHB1 bus */
		 __HAL_RCC_GPIOA_CLK_ENABLE();

		 GPIO_InitTypeDef GPIO_Init_adc_ch6 = {0};

		 GPIO_Init_adc_ch6.Pin = GPIO_PIN_6;
		 GPIO_Init_adc_ch6.Mode = GPIO_MODE_ANALOG;
		 GPIO_Init_adc_ch6.Pull = GPIO_NOPULL;


		 //Cargar la configuracion en los registros FSR del MCU
		 HAL_GPIO_Init(GPIOA, &GPIO_Init_adc_ch6);

		 __NOP();

		 /* Enable ADC clock on APB1 bus*/
		 __HAL_RCC_ADC1_CLK_ENABLE();

		 //Configurando la parte general del ADC

		 hadc1.Instance = ADC1;
		 hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
		 hadc1.Init.Resolution = ADC_RESOLUTION_12B;
		 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		 hadc1.Init.ScanConvMode = DISABLE;
		 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		 hadc1.Init.ContinuousConvMode = DISABLE;
		 hadc1.Init.NbrOfConversion = 1;
		 hadc1.Init.DiscontinuousConvMode = DISABLE;
		 hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; //Utilizamos el TRGO del tim3 como trigger para la conversión
		 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; //Se lanza un ADC cada que detecte flanco de subida del TRGO
		 hadc1.Init.DMAContinuousRequests = DISABLE;

		 //CArgar la configuracion en los registros FSR del MCU
		 HAL_ADC_Init(&hadc1);

		 //Configuracion del canal especifico
		 ADC_ChannelConfTypeDef adc_ch6 = {0};

		 adc_ch6.Channel = ADC_CHANNEL_6;
		 adc_ch6.Rank = 1;
		 adc_ch6.SamplingTime = ADC_SAMPLETIME_56CYCLES;
		 adc_ch6.Offset = 0;

		 //Cargar la configuracion en los registros FSR del MCU
		 HAL_ADC_ConfigChannel(&hadc1, &adc_ch6);

		 //Registrar la interrupcion en el NVIC
		 HAL_NVIC_EnableIRQ(ADC_IRQn);

}

/* Configuración del TIM3 para el TRGO del ADC1 */
static void tim3_adc_Init(void){
	__HAL_RCC_TIM3_CLK_ENABLE();

	htim3.Instance = TIM3;

	htim3.Init.Prescaler = 15999; //16 MHz / 16 kHz = 1 kHz (1 ms)
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 29; //Periodo de 1 ms * 30 = 30 ms
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	//Cargando la configuracion en los registros FSR del MCU
	HAL_TIM_Base_Init(&htim3);

	//Configurando el TRGO del TIM3
	TIM_MasterConfigTypeDef confTRGO = {0};

	confTRGO.MasterOutputTrigger = TIM_TRGO_UPDATE;
	confTRGO.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	//Cargando la configuracion en los registros FSR del MCU para la sincronización del TRGO generado por UPdate Event para hacer la conversion ADC
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &confTRGO);

	//Iniciando el TIM4
	HAL_TIM_Base_Start(&htim3);

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

    /* Start TIM3 in interrupt mode — enables the update event interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
    __NOP();

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
        showMsg = 1;
    }
}

//Callback de la conversion ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc->Instance == ADC1){
		//Cargando  el dato de la conversion en una variable
		raw_adc = hadc->Instance->DR;
		adc_done = 1;
	}
}

//Callback de la ISR generada por el USART2 Rx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART2){
		//Cargando  el dato de recepcion Rx a la variable
		//rx_data = husart->Instance->DR;
		usart_done = 1;
	}
}





