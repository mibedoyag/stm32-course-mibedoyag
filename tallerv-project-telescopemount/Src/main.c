/**
 * @file    : main.c
 * @author  : Miguel A. Bedoya Gonzalez --> mibedoyag@unal.edu.co
 * @brief   : Archivo principal del proyecto..
 *
 * =============================================================================================
 * RESUMEN DEL PROYECTO
 * ==============================================================================================
 * Este proyecto basado para una montura de telescopio altazimutal
 * automatizada, tiene como función principal  permitir
 * el apuntamiento astronómico absoluto (GoTo), el seguimiento sideral continuo y el control
 * manual del tubo óptico. Para lograrlo, el sistema adquiere la ubicación geográfica y la hora
 * UTC exactas a través de un módulo GPS operado por DMA, mientras que una IMU (BNO055)
 * proporciona la realimentación espacial de Altitud y Azimut en lazo cerrado, actuando como
 * corrector de posición. El movimiento se
 * ejecuta resolviendo ecuaciones de trigonometría esférica en tiempo real y generando señales
 * PWM con rampas de aceleración no bloqueantes para motores paso a paso. La lógica global está
 * orquestada por una Máquina de Estados Finitos (FSM) que gestiona simultáneamente una interfaz
 * HMI interactiva (LCD, Joystick y Encoder) y un hilo de comunicación serial UART, el cual
 * interpreta el protocolo estándar Meade LX200 para sincronizar la montura de forma nativa con
 * simuladores astronómicos de PC como Stellarium.
 * ============================================================================================== */

#include "stm32f4xx_hal.h"
#include "Proyecto/main_logic.h" // ¡Solo necesitas incluir el orquestador principal!

/* TIM10 handle para el Blinky */
TIM_HandleTypeDef htim10;

/* Prototipos de funciones privadas */
void SystemClock_Config(void);
void Error_Handler(void);
static void Blinky(void);

int main(void)
{
    /* 1. Inicializar la HAL y configurar el reloj a 16 MHz */
    HAL_Init();
    SystemClock_Config();

    /* 2. Inicialización de Blinky */
    Blinky();

    /* 3. INICIALIZACIÓN DE LA MONTURA
     * Aquí  se inician los Motores, Sensores, Pantalla e Interfaz */
    Montura_Init();

    /* 4. Superlazo infinito (Super-Loop) */
    while (1)
    {
        /* Todo el procesamiento (GPS, IMU, FSM, Motores) ocurre en está función */
        Montura_Loop();

        HAL_Delay(1);
    }
}

/**
 * @brief Configura el reloj del sistema (System Clock) a 16 MHz (HSI).
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Blinky(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOH_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    __HAL_RCC_TIM10_CLK_ENABLE();
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 15999;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 249;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim10);

    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_TIM_Base_Start_IT(&htim10);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
    }
}
