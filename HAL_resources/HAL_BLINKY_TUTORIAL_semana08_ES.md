# Tutorial HAL Blinky — Semana 08
## Primer Proyecto con las Librerías HAL del STM32F4xx (Sin CubeMX)

Este tutorial explica paso a paso cómo crear el primer proyecto con HAL desde
cero, sin usar CubeMX. Cada paso es explícito — nada se genera automáticamente.
Al finalizar, tendrás un LED parpadeando en PA5 controlado por TIM3 cada 250 ms.

---

## ¿Qué Cambia Respecto al Bare-Metal?

En programación bare-metal configurabas los periféricos escribiendo directamente
en registros:

```c
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
GPIOA->MODER |= (1 << 10);
```

Con HAL, las mismas operaciones ocurren dentro de funciones de la librería. El
acceso a los registros sigue existiendo — HAL simplemente lo envuelve. Tu trabajo
es llamar las funciones correctas, en el orden correcto, con los parámetros
correctos.

El macro de habilitación de reloj que verás en breve:

```c
__HAL_RCC_GPIOA_CLK_ENABLE();
```

hace exactamente lo mismo que `RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN`. Misma
operación en el hardware, sintaxis diferente.

---

## Ubicación del Paquete de Firmware

El paquete STM32CubeF4 ya está instalado en tu equipo. La ruta es:

```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/
```

Dentro de esa carpeta encontrarás:

```
STM32Cube_FW_F4_V1.27.1/
├── Drivers/
│   ├── CMSIS/
│   │   └── Device/ST/STM32F4xx/Source/Templates/
│   │       └── system_stm32f4xx.c              ← necesitas este archivo
│   └── STM32F4xx_HAL_Driver/
│       ├── Inc/                                 ← todos los archivos .h
│       └── Src/                                 ← todos los archivos .c
```

Los archivos fuente HAL (`.c`) se copiarán dentro del proyecto. Los archivos de
cabecera (`.h`) permanecerán aquí y serán referenciados mediante rutas de inclusión
en el IDE — no es necesario copiarlos. No modifiques los originales — siempre copia,
nunca muevas.

---

## Paso 1 — Crear un Nuevo Proyecto STM32 en STM32CubeIDE

Abre STM32CubeIDE y crea un nuevo proyecto STM32:

```
File → New → STM32 Project
```

Selecciona tu MCU objetivo (STM32F446RE para la tarjeta Nucleo). Cuando te
pregunte el tipo de proyecto, selecciona **Empty** — no uses inicialización
de CubeMX.

Después de la creación, el IDE te entrega un proyecto casi vacío:

```
TuProyecto/
├── Inc/                    ← vacío
├── Src/
│   └── main.c              ← casi vacío
├── Startup/
│   └── startup_stm32f446retx.s   ← ya está, no lo toques
└── STM32F446RETX_FLASH.ld        ← script del linker, no lo toques
```

Este es tu punto de partida. La primera compilación en este estado fallará — eso
es esperado. Primero debes agregar los archivos de la librería HAL.

---

## Paso 2 — Crear la Estructura de Carpetas del Proyecto

Dentro de tu proyecto, crea las siguientes carpetas manualmente usando el IDE o
el administrador de archivos:

```
TuProyecto/
├── Inc/                             ← ya existe
├── Src/                             ← ya existe
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
│       └── Src/                     ← tú la creas
```

Para crear carpetas en STM32CubeIDE: clic derecho sobre el nombre del proyecto
en el Project Explorer, luego selecciona `New → Folder`.

Nota: no hay subcarpeta `Inc/` dentro de `Drivers/` — los archivos de cabecera
HAL no se copian al proyecto. Se referencian directamente desde el paquete de
firmware mediante rutas de inclusión (configuradas en el Paso 3).

---

## Paso 3 — Configurar las Rutas de Inclusión en STM32CubeIDE

Los archivos de cabecera HAL **no se copian** al proyecto. En cambio, le indicas
al IDE dónde encontrarlos agregando rutas de inclusión. Esto evita duplicar
cientos de archivos en cada proyecto.

Clic derecho sobre tu proyecto, selecciona `Properties → C/C++ Build → Settings →
Tool Settings → MCU GCC Compiler → Include paths`.

Necesitas cuatro rutas. La primera es la carpeta `Inc/` de tu propio proyecto.
Las otras tres apuntan al paquete de firmware:

```
${workspace_loc:/${ProjName}/Inc}

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include
```

Nota: las dos últimas rutas CMSIS son las mismas que ya agregaste en las semanas
anteriores de bare-metal. Si ya están presentes en la configuración de tu proyecto,
no necesitas agregarlas de nuevo — solo la ruta HAL `Inc/` es nueva.

---

## Paso 4 — Copiar Solo los Archivos Fuente HAL Necesarios

**No** copies todos los archivos fuente — copia únicamente los que se listan
a continuación. Copiar todo provoca fallos de compilación por definiciones en
conflicto.

Ubicación de origen:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/
```

Copia estos 9 archivos a `TuProyecto/Drivers/STM32F4xx_HAL_Driver/Src/`:

```
stm32f4xx_hal.c             ← HAL_Init()
stm32f4xx_hal_rcc.c         ← macros de habilitación de reloj
stm32f4xx_hal_gpio.c        ← HAL_GPIO_Init(), HAL_GPIO_TogglePin()
stm32f4xx_hal_tim.c         ← HAL_TIM_Base_Init(), HAL_TIM_Base_Start_IT()
stm32f4xx_hal_tim_ex.c      ← requerido internamente por stm32f4xx_hal_tim.c
stm32f4xx_hal_dma.c         ← requerido por la definición del struct del handle TIM
stm32f4xx_hal_cortex.c      ← HAL_NVIC_EnableIRQ()
stm32f4xx_hal_flash.c       ← requerido internamente por HAL_Init()
stm32f4xx_hal_pwr.c         ← requerido internamente por HAL_Init()
```

Nota: `stm32f4xx_hal_tim_ex.c` y `stm32f4xx_hal_dma.c` no se usan directamente
en tu código, pero el linker los requiere porque `stm32f4xx_hal_tim.c` referencia
funciones y tipos definidos dentro de ellos.

---

## La Relación entre Archivos Fuente y Archivo de Configuración

Este es uno de los conceptos más importantes al construir un proyecto HAL sin
CubeMX. Cada driver de periférico tiene **dos piezas obligatorias** que deben
existir siempre juntas:

```
stm32f4xx_hal_xxx.c        ←→    #define HAL_XXX_MODULE_ENABLED
  (archivo fuente en                (define en stm32f4xx_hal_conf.h)
   Drivers/Src/)
```

Estas dos piezas son dos caras de la misma moneda:

- El archivo `.c` contiene el código real del driver — las funciones que llamas.
- El `#define` en `stm32f4xx_hal_conf.h` le indica al compilador que active ese
  driver. Sin él, los archivos de cabecera HAL no exponen la API del periférico,
  aunque el archivo `.c` esté presente en el proyecto.

Si **falta una de las dos**, el resultado es siempre un fallo de compilación:

```
.c presente, #define ausente  →  error del compilador: tipo o función no definida
#define presente, .c ausente  →  error del linker: referencia no resuelta
```

Ambas deben estar presentes y deben coincidir.

### Lista de Verificación para Este Proyecto (GPIO + TIM3)

Usa esta tabla para verificar tu proyecto antes de compilar. Cada fila debe
tener ambas columnas marcadas.

| Define en `stm32f4xx_hal_conf.h`  | Archivo fuente en `Drivers/Src/`  | Propósito                          |
|-----------------------------------|-----------------------------------|------------------------------------|
| `HAL_MODULE_ENABLED`              | `stm32f4xx_hal.c`                 | HAL base, `HAL_Init()`             |
| `HAL_RCC_MODULE_ENABLED`          | `stm32f4xx_hal_rcc.c`             | Macros de habilitación de reloj    |
| `HAL_GPIO_MODULE_ENABLED`         | `stm32f4xx_hal_gpio.c`            | Inicialización y control GPIO      |
| `HAL_TIM_MODULE_ENABLED`          | `stm32f4xx_hal_tim.c`             | Driver base de timers              |
| *(sin define)*                    | `stm32f4xx_hal_tim_ex.c`          | Dependencia interna de TIM         |
| `HAL_DMA_MODULE_ENABLED`          | `stm32f4xx_hal_dma.c`             | Dependencia interna de TIM         |
| `HAL_CORTEX_MODULE_ENABLED`       | `stm32f4xx_hal_cortex.c`          | NVIC, `HAL_NVIC_EnableIRQ()`       |
| `HAL_FLASH_MODULE_ENABLED`        | `stm32f4xx_hal_flash.c`           | Dependencia interna de HAL_Init    |
| `HAL_PWR_MODULE_ENABLED`          | `stm32f4xx_hal_pwr.c`             | Dependencia interna de HAL_Init    |

Nota el caso especial de `stm32f4xx_hal_tim_ex.c` — es un archivo complementario
de `stm32f4xx_hal_tim.c` y no tiene su propio `#define`. Debe estar presente en
el proyecto pero no requiere ninguna entrada en `stm32f4xx_hal_conf.h`.

---

## Paso 5 — Copiar el Archivo de Sistema CMSIS

Copia `system_stm32f4xx.c` desde las plantillas CMSIS:

Origen:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
```

Destino:
```
TuProyecto/Src/system_stm32f4xx.c
```

No necesitas modificar este archivo — úsalo tal como está. Pero vale la pena
entender qué hace realmente, porque se ejecuta antes de `main()` y sus efectos
están siempre activos.

### Qué hace realmente system_stm32f4xx.c

El archivo le entrega tres cosas al resto del sistema:

**`SystemInit()`** — llamada automáticamente por el archivo de arranque
(`startup_stm32f4xx.s`) inmediatamente después del reset, antes de llegar a
`main()`. Para el STM32F411 hace exactamente dos cosas:

```
1. Habilita la FPU
   SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

   Esto concede acceso completo a los coprocesadores CP10 y CP11, que son la
   unidad de punto flotante del Cortex-M4. Sin esta línea, cualquier instrucción
   de punto flotante dispararía un hard fault. Solo se ejecuta cuando el toolchain
   está configurado para FPU por hardware (__FPU_PRESENT == 1 y __FPU_USED == 1).

2. Reubicación de la tabla de vectores (condicional, deshabilitada por defecto)
   SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;

   Solo se ejecuta si USER_VECT_TAB_ADDRESS está definido — lo cual no ocurre
   por defecto. En un proyecto estándar, el valor de reset de SCB->VTOR ya
   apunta a Flash en 0x08000000, que es correcto. El archivo de arranque define
   la tabla de vectores allí; este archivo no la mueve a menos que se lo indiques
   explícitamente.
```

**`SystemCoreClock`** — variable global inicializada en 16000000 (frecuencia HSI).
Esta variable se actualiza automáticamente cada vez que llamas a `HAL_RCC_ClockConfig()`
en tu `SystemClock_Config()`. El resto del HAL la usa para calcular timeouts,
baudrates y otros valores que dependen de la frecuencia. Debe reflejar siempre
la frecuencia real actual del HCLK.

**`AHBPrescTable[]` y `APBPrescTable[]`** — tablas de lookup usadas internamente
por `SystemCoreClockUpdate()` para calcular las frecuencias de los buses a partir
de los campos divisores de los registros RCC. No las llamas directamente, pero el
HAL las usa.

### Qué NO hace system_stm32f4xx.c

**No configura el reloj del sistema.** A diferencia de lo que CubeMX generaba
antes, este archivo deja el árbol de relojes exactamente como lo dejó el reset del
hardware — HSI encendido, sin PLL, todos los divisores en 1. La configuración del
reloj es completamente tu responsabilidad en `SystemClock_Config()` dentro de
`main.c`.

### Cómo verificar la configuración de la FPU

Puedes verificar que ambas banderas están activas agregando este bloque en
cualquier parte de `main.c` antes de compilar:

```c
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    #pragma message("FPU: enabled")
#else
    #pragma message("FPU: NOT enabled")
#endif
```

Esto produce un mensaje en la consola de compilación durante el build — sin
costo en tiempo de ejecución, sin nada agregado al binario.

**Importante:** el editor de código del IDE pondrá en gris la rama inactiva,
haciendo parecer que esas líneas están siendo ignoradas por el compilador. No
confíes en el color gris — es solo el resaltado de sintaxis del editor. El
compilador es un proceso completamente separado. La verdad siempre está en la
consola de build, donde verás:

```
note: '#pragma message: FPU: enabled'
```

También puedes confirmar el estado de la FPU directamente desde la línea de
comandos del compilador, sin necesidad del `#pragma`. Busca estas dos banderas
en la invocación de `arm-none-eabi-gcc` en la consola de build:

```
-mfpu=fpv4-sp-d16      ← especifica la unidad FPU del hardware
-mfloat-abi=hard       ← establece __FPU_USED = 1, habilita instrucciones float
```

Si ves `-mfloat-abi=soft` en cambio, `__FPU_USED` es 0, `SystemInit()` no
activará la FPU, y cualquier operación de punto flotante disparará un hard fault
en tiempo de ejecución. Esta configuración se encuentra en:

```
Project Properties → C/C++ Build → Settings → MCU Settings → Floating-point unit
```

---

## Paso 6 — Crear el Archivo de Configuración HAL

El archivo de configuración HAL le indica al compilador qué módulos de periféricos
están activos. Debes crear este archivo manualmente.

Copia la plantilla del paquete de firmware:

Origen:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf_template.h
```

Destino:
```
TuProyecto/Inc/stm32f4xx_hal_conf.h
```

Nota: renómbralo — elimina `_template` del nombre del archivo.

Ahora abre `stm32f4xx_hal_conf.h` y reemplaza la sección de habilitación de
módulos con únicamente los que necesitas para este proyecto:

```c
/* Módulos activos para proyecto GPIO + TIM3 */
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
```

Comenta o elimina todas las demás líneas `HAL_xxx_MODULE_ENABLED`.

---

## Paso 7 — Crear el Archivo de Manejadores de Interrupción

Crea `TuProyecto/Src/stm32f4xx_it.c` con el siguiente contenido:

```c
/*
 * stm32f4xx_it.c
 * Rutinas de servicio de interrupción
 * Autor: tu nombre
 */

#include "stm32f4xx_hal.h"

/* Declarar el handle de TIM3 — definido en main.c */
extern TIM_HandleTypeDef htim3;

/* Manejador de SysTick — requerido por HAL para HAL_Delay() y timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Manejador del evento de actualización de TIM3 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}
```

---

## Paso 8 — Escribir main.c

Reemplaza el contenido de `main.c` con lo siguiente:

```c
/*
 * main.c
 * HAL Blinky — LED en PA5 conmutado cada 250 ms por TIM3
 * Autor: tu nombre
 */

#include "stm32f4xx_hal.h"

/* Handle de TIM3 — debe ser global para que stm32f4xx_it.c pueda accederlo */
TIM_HandleTypeDef htim3;

/* Prototipos de funciones privadas */
static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim3_Init(void);

int main(void)
{
    HAL_Init();           /* inicializa HAL: SysTick, caché, agrupación de prioridades */
    SystemClock_Config(); /* configura el árbol de relojes: HSI a 16 MHz               */
    gpio_Init();          /* configura PA5 como salida push-pull                        */
    tim3_Init();          /* configura TIM3: evento de actualización cada 250 ms        */

    while (1)
    {
        /* bucle de aplicación — la conmutación del LED ocurre en el callback */
    }
}

/*
 * SystemClock_Config
 * Usa el oscilador interno HSI a 16 MHz
 * Sin PLL — configuración de reloj más simple posible
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI ya está encendido al resetear — confirmar y usarlo */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Seleccionar HSI como SYSCLK — todos los divisores de bus en 1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 16 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;     /* APB1  = 16 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;     /* APB2  = 16 MHz */

    /* FLASH_LATENCY_0 = cero wait states, correcto para 16 MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

/*
 * gpio_Init
 * Configura PA5 como salida push-pull — LED de la tarjeta Nucleo
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Habilitar reloj de GPIOA en el bus AHB1
       Equivalente bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configurar PA5 */
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * tim3_Init
 * Configura TIM3 para generar un evento de actualización cada 250 ms
 *
 * Cadena de reloj:
 *   HSI (16 MHz) → APB1 (16 MHz) → reloj TIM3 (16 MHz)
 *
 * PSC = 15999  →  tick = 16,000,000 / (15999 + 1) = 1,000 Hz  (1 ms por tick)
 * ARR = 249    →  período = (249 + 1) x 1 ms = 250 ms
 */
static void tim3_Init(void)
{
    /* Habilitar reloj de TIM3 en el bus APB1 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configurar la base de TIM3 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15999;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 249;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim3);

    /* Arrancar TIM3 en modo interrupción — habilita la interrupción de evento de actualización */
    HAL_TIM_Base_Start_IT(&htim3);

    /* Habilitar la línea de interrupción de TIM3 en el NVIC */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/*
 * HAL_TIM_PeriodElapsedCallback
 * Llamado automáticamente por HAL_TIM_IRQHandler() cada vez que un evento
 * de actualización del timer se dispara. Es compartido por todos los timers
 * — siempre verifica htim->Instance.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
```

---

## Paso 9 — Estructura Final del Proyecto

Después de todos los pasos, tu proyecto debe verse exactamente así:

```
TuProyecto/
│
├── Inc/
│   └── stm32f4xx_hal_conf.h
│
├── Src/
│   ├── main.c
│   ├── stm32f4xx_it.c
│   └── system_stm32f4xx.c
│
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
│       └── Src/
│           ├── stm32f4xx_hal.c
│           ├── stm32f4xx_hal_rcc.c
│           ├── stm32f4xx_hal_gpio.c
│           ├── stm32f4xx_hal_tim.c
│           ├── stm32f4xx_hal_tim_ex.c
│           ├── stm32f4xx_hal_dma.c
│           ├── stm32f4xx_hal_cortex.c
│           ├── stm32f4xx_hal_flash.c
│           └── stm32f4xx_hal_pwr.c
│
├── Startup/
│   └── startup_stm32f446retx.s
│
└── STM32F446RETX_FLASH.ld
```

Rutas de inclusión configuradas en el IDE (no visibles en el árbol del proyecto):
```
${workspace_loc:/${ProjName}/Inc}
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include
```

---

## Paso 10 — Compilar y Programar

Compila el proyecto con `Ctrl+B`. Si la compilación es exitosa, programa la
tarjeta con `Run → Debug` o `Run → Run`.

El LED de la tarjeta en PA5 debe parpadear a 2 Hz (250 ms encendido, 250 ms
apagado).

---

## Errores Frecuentes de Compilación y sus Soluciones

Estos errores fueron encontrados durante la construcción de este tutorial. Si
los ves, sabes exactamente qué agregar.

```
error: unknown type name 'DMA_HandleTypeDef'
```
Solución: agrega `#define HAL_DMA_MODULE_ENABLED` a `stm32f4xx_hal_conf.h` y
copia `stm32f4xx_hal_dma.c` a `Drivers/STM32F4xx_HAL_Driver/Src/`.

```
undefined reference to 'HAL_TIMEx_BreakCallback'
undefined reference to 'HAL_TIMEx_CommutCallback'
```
Solución: copia `stm32f4xx_hal_tim_ex.c` a `Drivers/STM32F4xx_HAL_Driver/Src/`.

---

## Resumen de Conceptos Clave

```
HAL_Init()
    │
    ├── inicializa SysTick a 1 ms
    ├── configura caché y prefetch
    └── llama HAL_MspInit() (vacío por defecto)

SystemClock_Config()
    │
    └── HSI ON → SYSCLK = 16 MHz → AHB = 16 MHz → APB1 = 16 MHz

gpio_Init()
    │
    ├── __HAL_RCC_GPIOA_CLK_ENABLE()   (equivale a RCC->AHB1ENR |= ...)
    └── HAL_GPIO_Init(GPIOA, &struct)

tim3_Init()
    │
    ├── __HAL_RCC_TIM3_CLK_ENABLE()
    ├── HAL_TIM_Base_Init(&htim3)
    ├── HAL_TIM_Base_Start_IT(&htim3)
    └── HAL_NVIC_EnableIRQ(TIM3_IRQn)

TIM3_IRQHandler()   [en stm32f4xx_it.c]
    │
    └── HAL_TIM_IRQHandler(&htim3)
            │
            └── HAL_TIM_PeriodElapsedCallback(&htim3)
                    │
                    └── HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)
```

Referencia: UM1725 Rev 8 — Description of STM32F4 HAL and low-layer drivers
(STMicroelectronics, marzo 2023)

---

---

# Fase 2 — Agregar un Nuevo Módulo: USART2

En este punto el proyecto está funcionando — LED parpadeando, interrupción de
TIM3 disparándose cada 250 ms. La Fase 2 muestra cómo agregar un nuevo periférico
a un proyecto existente, siguiendo exactamente el mismo patrón usado para TIM3.
El proceso es siempre el mismo:

```
1. Agregar el archivo fuente a Drivers/Src/
2. Agregar el #define a stm32f4xx_hal_conf.h
3. Agregar el handle al inicio de main.c
4. Escribir la función de inicialización
5. Llamarla desde main()
```

## Paso 1 — Agregar el Archivo Fuente de USART2

Copia un nuevo archivo a `Drivers/STM32F4xx_HAL_Driver/Src/`:

```
stm32f4xx_hal_uart.c        ← HAL_UART_Init(), HAL_UART_Transmit()
```

## Paso 2 — Agregar el Define del Módulo

Abre `stm32f4xx_hal_conf.h` y agrega una línea:

```c
#define HAL_UART_MODULE_ENABLED
```

Lista de verificación actualizada — ahora cubre GPIO + TIM3 + USART2:

| Define en `stm32f4xx_hal_conf.h`  | Archivo fuente en `Drivers/Src/`  | Propósito                          |
|-----------------------------------|-----------------------------------|------------------------------------|
| `HAL_MODULE_ENABLED`              | `stm32f4xx_hal.c`                 | HAL base, `HAL_Init()`             |
| `HAL_RCC_MODULE_ENABLED`          | `stm32f4xx_hal_rcc.c`             | Macros de habilitación de reloj    |
| `HAL_GPIO_MODULE_ENABLED`         | `stm32f4xx_hal_gpio.c`            | Inicialización y control GPIO      |
| `HAL_TIM_MODULE_ENABLED`          | `stm32f4xx_hal_tim.c`             | Driver base de timers              |
| *(sin define)*                    | `stm32f4xx_hal_tim_ex.c`          | Dependencia interna de TIM         |
| `HAL_DMA_MODULE_ENABLED`          | `stm32f4xx_hal_dma.c`             | Dependencia interna de TIM         |
| `HAL_CORTEX_MODULE_ENABLED`       | `stm32f4xx_hal_cortex.c`          | NVIC, `HAL_NVIC_EnableIRQ()`       |
| `HAL_FLASH_MODULE_ENABLED`        | `stm32f4xx_hal_flash.c`           | Dependencia interna de HAL_Init    |
| `HAL_PWR_MODULE_ENABLED`          | `stm32f4xx_hal_pwr.c`             | Dependencia interna de HAL_Init    |
| `HAL_UART_MODULE_ENABLED`         | `stm32f4xx_hal_uart.c`            | Transmisión y recepción UART/USART |

## Paso 3 — Agregar el Handle de USART2

Igual que `htim3`, el handle debe ser global. Agrégalo junto a `htim3` al inicio
de `main.c`:

```c
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart2;   /* ← agregar esta línea */
```

## Paso 4 — Escribir la Función usart2_Init()

Compara esto con `tim3_Init()` — la estructura es idéntica. El único elemento
nuevo es el bloque de función alternada GPIO al inicio. USART2 usa pines físicos
(PA2 para TX, PA3 para RX), así que esos pines deben configurarse como función
alternada antes de inicializar el periférico.

Los pines GPIO de salida simple como el LED en PA5 no necesitan función alternada
— la CPU los controla directamente. Los periféricos de comunicación sí, porque
la señal es generada por el hardware del periférico, no por la CPU. Todo
periférico que use pines — USART, SPI, I2C, I2S — requerirá este paso.

```c
static void usart2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Paso 1: habilitar relojes — tanto USART2 como GPIOA */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* ya llamado en gpio_Init() — seguro llamarlo de nuevo */

    /* Paso 2: configurar PA2 (TX) y PA3 (RX) como función alternada
       AF7 es USART2 en el STM32F411 — ver tabla de funciones alternadas en el datasheet */
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Paso 3: configurar USART2 */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}
```

## Paso 5 — Llamar usart2_Init() desde main()

Agrega la llamada después de `tim3_Init()`:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    gpio_Init();
    tim3_Init();
    usart2_Init();       /* ← agregar esta línea */

    while (1)
    {
    }
}
```

## Paso 6 — Transmitir Datos

Con el periférico inicializado, enviar datos es una sola llamada:

```c
/* Enviar una cadena — modo bloqueante, timeout de 100 ms */
char msg[] = "Hola desde STM32\r\n";
HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 100);
```

Esto puede colocarse en cualquier parte — dentro de `main()`, dentro del callback
de TIM3 para enviar un mensaje cada 250 ms, o dentro de cualquier otra función.

## Qué Es Diferente Respecto a TIM3

```
tim3_Init()                        usart2_Init()
─────────────────────────────────  ─────────────────────────────────
__HAL_RCC_TIM3_CLK_ENABLE()        __HAL_RCC_USART2_CLK_ENABLE()
                                   __HAL_RCC_GPIOA_CLK_ENABLE()
                                   Configurar PA2/PA3 como AF7  ← nuevo
Llenar TIM_HandleTypeDef           Llenar UART_HandleTypeDef
HAL_TIM_Base_Init(&htim3)          HAL_UART_Init(&huart2)
HAL_TIM_Base_Start_IT(&htim3)      (no se necesita Start para TX por polling)
HAL_NVIC_EnableIRQ(TIM3_IRQn)      (no se necesita NVIC para TX por polling)
```

---

---

# Apéndice — Caché de Flash y Prefetch

Cuando llamas a `HAL_Init()`, dos características de aceleración de Flash se
habilitan internamente: el **caché de instrucciones** y el **buffer de prefetch**.
Aparecen en el código fuente de HAL y en comentarios a lo largo del código, así
que vale la pena entender qué hacen realmente.

## El Problema que Resuelven

El núcleo Cortex-M4 del STM32F411 puede correr a hasta 100 MHz. La memoria Flash
interna no puede seguir ese ritmo — tiene un tiempo de acceso fijo que se convierte
en un cuello de botella a frecuencias altas. Sin ninguna aceleración, la CPU
tendría que insertar wait states (ciclos de espera) en cada lectura de Flash,
desperdiciando tiempo de ejecución.

A 16 MHz (HSI, sin PLL) esto no es un problema — la Flash es suficientemente
rápida y se necesitan cero wait states. Pero el HAL habilita estas características
de todas formas, porque el mismo código debe funcionar a cualquier frecuencia.

## Caché de Instrucciones

```
Sin caché:                         Con caché:

CPU solicita instrucción           CPU solicita instrucción
       ↓                                  ↓
Lectura de Flash (lenta)           ¿Hit en caché? → instantáneo
       ↓                                  ↓ (miss)
CPU ejecuta                        Lectura Flash → guardada en caché
                                          ↓
                                   CPU ejecuta
                                   Próxima vez: hit en caché → instantáneo
```

El caché de instrucciones es una memoria pequeña y rápida (4 KB, 64 líneas de
64 bytes cada una) que almacena las instrucciones leídas recientemente. Cuando
la CPU ejecuta un bucle, el cuerpo del bucle se lee de Flash una vez y se guarda
en el caché. Cada iteración siguiente se sirve desde el caché a la velocidad
máxima de la CPU, sin ningún acceso a Flash. Para firmware embebido, que pasa
la mayor parte del tiempo en bucles y manejadores de interrupción, esto representa
una ganancia significativa.

## Buffer de Prefetch

El buffer de prefetch funciona de manera diferente — en lugar de esperar a que
la CPU solicite la siguiente instrucción, lee por adelantado en Flash mientras
la CPU aún está ejecutando la instrucción actual. Como la Flash se accede en
bloques de 128 bits y las instrucciones del Cortex-M4 tienen 16 o 32 bits de
ancho, una sola lectura de Flash entrega múltiples instrucciones a la vez. El
buffer de prefetch mantiene el siguiente bloque listo antes de que la CPU lo
necesite.

```
CPU ejecutando instrucción N       Flash ya leyendo instrucciones N+4, N+5...
CPU ejecutando instrucción N+1     (ya en el buffer — sin espera)
CPU ejecutando instrucción N+2     (ya en el buffer — sin espera)
```

## En la Práctica para Este Curso

A 16 MHz con `FLASH_LATENCY_0` (cero wait states), ambas características tienen
un efecto mínimo visible — la Flash ya es suficientemente rápida. Su importancia
crece cuando el reloj se aumenta a 100 MHz mediante PLL (sesiones futuras), donde
se necesitarían `FLASH_LATENCY_3` (tres wait states) sin ellas, y el caché y el
buffer de prefetch se vuelven esenciales para mantener un rendimiento aceptable.

`HAL_Init()` habilita ambos de forma incondicional para que el proyecto esté listo
para cualquier configuración de reloj sin necesidad de cambios en la secuencia de
inicialización.
