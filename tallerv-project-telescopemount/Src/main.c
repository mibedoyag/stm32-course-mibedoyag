/**
 * @file    : main.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Archivo principal del proyecto (Bare-Metal HAL).
 * Configura el reloj del sistema a 16 MHz y ejecuta el superlazo de la montura.
 */

#include "stm32f4xx_hal.h"
#include "Proyecto/motores.h"
#include "Proyecto/interfaz.h"
#include "Proyecto/sensores.h"

/* TIM4 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim4;


/* Prototipos de funciones privadas */
void SystemClock_Config(void);
void Error_Handler(void);
static void Blinky(void); //Función que inicializa y configura el puerto PH1 del blinky

/**
 * @brief  El punto de entrada de la aplicación.
 * @retval int
 */
int main(void)
{
    /* 1. Resetear todos los periféricos e inicializar la interfaz HAL.
     * Esto configura el Systick para que funcione la función HAL_Delay() */
    HAL_Init();

    /* 2. Configurar el reloj del sistema a 16 MHz usando el HSI */
    SystemClock_Config();

    /* 3. Inicialización de nuestra arquitectura lógica y de hardware.
     * Al llamar a Motores_InitLogica(), internamente se configuran los
     * GPIOs, Timers y el ADC que escribimos en motores.c */
    Motores_InitLogica();
    Sensores_InitLogica();
    // Pasamos el mismo bus I2C (hi2c1) que usa el BNO055
    Interfaz_InitLogica();

    /* 4. Forzamos el estado de la máquina a Manual para esta prueba física.
     * (Asumiendo que declaraste currentState en interfaz.c) */
    currentState = STATE_MANUAL;

    /* 5. Inicialización de Blinky */
    Blinky();
    /* =========================================================================
     * PRUEBA DE ESCANEO I2C TEMPORAL (Inserta esto en tu main)
     * ========================================================================= */
    HAL_StatusTypeDef resultado;
    uint8_t direccion_encontrada = 0;

    // Escaneamos todas las direcciones posibles de 7 bits (de 1 a 127)
    for (uint16_t i = 1; i < 128; i++) {
        // HAL_I2C_IsDeviceReady envía una señal vacía para ver si el chip responde con un ACK
        resultado = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 5);

        if (resultado == HAL_OK) {
            direccion_encontrada = i; // ¡Encontramos un dispositivo!

            // PONE UN BREAKPOINT EN ESTA LÍNEA DE ABAJO:
            __NOP(); // El debugger se detendrá aquí cuando encuentre la dirección
        }/* =========================================================================
         * PRUEBA DE ESCANEO I2C TEMPORAL (Inserta esto en tu main)
         * ========================================================================= */
        HAL_StatusTypeDef resultado;
        uint8_t direccion_encontrada = 0;

        // Escaneamos todas las direcciones posibles de 7 bits (de 1 a 127)
        for (uint16_t i = 1; i < 128; i++) {
            // HAL_I2C_IsDeviceReady envía una señal vacía para ver si el chip responde con un ACK
            resultado = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 5);

            if (resultado == HAL_OK) {
                direccion_encontrada = i; // ¡Encontramos un dispositivo!

                // PONE UN BREAKPOINT EN ESTA LÍNEA DE ABAJO:
                __NOP(); // El debugger se detendrá aquí cuando encuentre la dirección
            }
        }
    }

    /* 6. Superlazo infinito (Super-Loop) */
    while (1)
    {
    	Sensores_ProcesarDatos();

        // El motor evalúa el joystick y actualiza los Timers PWM continuamente
        Motores_UpdateLogica();

        // C. Actualizar pantalla (la función internamente se auto-regula cada 250ms)
        Interfaz_UpdateFSM();

        // Un micro-delay opcional para no saturar el bus de lectura del ADC
        // (1 milisegundo = 1000 Hz de frecuencia de muestreo de la lógica)
        HAL_Delay(1);
    }
}

/**
 * @brief Configura el reloj del sistema (System Clock).
 * Selecciona el Oscilador Interno de Alta Velocidad (HSI) a 16 MHz.
 * No se utiliza el PLL.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configurar el oscilador interno (HSI) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE; // Apagamos el PLL, directo a 16 MHz

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Inicializar los relojes de los buses (CPU, AHB y APB) */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // Fuente: HSI
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // AHB a 16 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      // APB1 a 16 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // APB2 a 16 MHz

    /* Aplicar la configuración con 0 estados de espera (Flash Latency 0)
     * porque 16 MHz es suficientemente lento para la memoria Flash */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


static void Blinky(void)
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


    __HAL_RCC_TIM4_CLK_ENABLE();

	/* Configure TIM4 base */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 15999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 249;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(&htim4); //Cargando la configuracion en los registros FSR del MCU

	/* Enable TIM4 interrupt line in the NVIC */
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

	/* Start TIM4 in interrupt mode — enables the update event interrupt */
	HAL_TIM_Base_Start_IT(&htim4);
	__NOP();
}

/**
 * @brief  Función que se ejecuta en caso de un error crítico de configuración.
 * Atrapa el programa en un bucle infinito para depuración.
 */
void Error_Handler(void)
{
    /* Desactivar interrupciones para asegurar que el sistema se detiene */
    __disable_irq();
    while (1)
    {
        // Aquí podrías agregar un parpadeo de LED (ej. PA5) para indicar error visualmente
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
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reporta el nombre del archivo y la línea en caso de un error de assert_param.
  * @param  file: puntero al nombre del archivo
  * @param  line: número de línea donde falló
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
}
#endif /* USE_FULL_ASSERT */
