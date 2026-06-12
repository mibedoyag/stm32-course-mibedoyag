# HAL Blinky Tutorial — Week 08
## First Project with STM32F4xx HAL Libraries (No CubeMX)

This tutorial walks you through creating your first HAL-based project from scratch,
without using CubeMX. Every step is explicit — nothing is generated automatically.
By the end, you will have a blinking LED on PA5 driven by TIM3 at 250 ms intervals.

---

## What Is Different From Bare-Metal

In bare-metal programming you configured peripherals by writing directly to registers:

```c
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
GPIOA->MODER |= (1 << 10);
```

With HAL, the same operations happen inside library functions. The register access
is still there — HAL just wraps it. Your job is to call the right functions in the
right order with the right parameters.

The clock enable macro you will see shortly:

```c
__HAL_RCC_GPIOA_CLK_ENABLE();
```

does exactly the same thing as `RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN`. Same hardware
operation, different syntax.

---

## The Firmware Package Location

The STM32CubeF4 firmware package is already installed on your machine. The path is:

```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/
```

Inside that folder you will find:

```
STM32Cube_FW_F4_V1.27.1/
├── Drivers/
│   ├── CMSIS/
│   │   └── Device/ST/STM32F4xx/Source/Templates/
│   │       └── system_stm32f4xx.c              ← you need this file
│   └── STM32F4xx_HAL_Driver/
│       ├── Inc/                                 ← all .h header files
│       └── Src/                                 ← all .c source files
```

The HAL source files (`.c`) will be copied into your project. The header files
(`.h`) will stay here and be referenced via include paths in the IDE — no copying
needed for headers. Do not modify the originals — always copy, never move.

---

## Step 1 — Create a New STM32 Project in STM32CubeIDE

Open STM32CubeIDE and create a new STM32 project:

```
File → New → STM32 Project
```

Select your target MCU (STM32F446RE for the Nucleo board). When asked about
project type, select **Empty** — do not use CubeMX initialization.

After creation, the IDE gives you a nearly empty project:

```
YourProject/
├── Inc/                    ← empty
├── Src/
│   └── main.c              ← almost empty
├── Startup/
│   └── startup_stm32f446retx.s   ← already there, do not touch
└── STM32F446RETX_FLASH.ld        ← linker script, do not touch
```

This is your starting point. The first build at this stage will fail — that is
expected. You need to add the HAL library files first.

---

## Step 2 — Create the Project Folder Structure

Inside your project, create the following folders manually using the IDE or the
file manager:

```
YourProject/
├── Inc/                             ← already exists
├── Src/                             ← already exists
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
│       └── Src/                     ← you create this
```

To create folders in STM32CubeIDE: right-click on the project name in the
Project Explorer, then select `New → Folder`.

Note: there is no `Inc/` subfolder inside `Drivers/` — the HAL headers are not
copied into the project. They are referenced directly from the firmware package
via include paths (configured in Step 3).

---

## Step 3 — Configure the Include Paths in STM32CubeIDE

The HAL header files are **not copied** into the project. Instead, you tell the
IDE where to find them by adding include paths. This avoids duplicating hundreds
of files across every project.

Right-click on your project, select `Properties → C/C++ Build → Settings →
Tool Settings → MCU GCC Compiler → Include paths`.

You need three paths. The first is your project's own `Inc/` folder. The other
two point into the firmware package:

```
${workspace_loc:/${ProjName}/Inc}

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include

~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include
```

Note: the last two CMSIS paths are the same ones you already added in previous
bare-metal weeks. If they are already present in your project settings, you do
not need to add them again — only the HAL `Inc/` path is new.

---

## Step 4 — Copy Only the Needed HAL Source Files

Do **not** copy all source files — only copy the ones listed below. Copying
everything causes build failures due to conflicting definitions.

Source location:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Src/
```

Copy these 9 files into `YourProject/Drivers/STM32F4xx_HAL_Driver/Src/`:

```
stm32f4xx_hal.c             ← HAL_Init()
stm32f4xx_hal_rcc.c         ← clock enable macros
stm32f4xx_hal_gpio.c        ← HAL_GPIO_Init(), HAL_GPIO_TogglePin()
stm32f4xx_hal_tim.c         ← HAL_TIM_Base_Init(), HAL_TIM_Base_Start_IT()
stm32f4xx_hal_tim_ex.c      ← required internally by stm32f4xx_hal_tim.c
stm32f4xx_hal_dma.c         ← required by TIM handle struct definition
stm32f4xx_hal_cortex.c      ← HAL_NVIC_EnableIRQ()
stm32f4xx_hal_flash.c       ← required internally by HAL_Init()
stm32f4xx_hal_pwr.c         ← required internally by HAL_Init()
```

Note: `stm32f4xx_hal_tim_ex.c` and `stm32f4xx_hal_dma.c` are not used directly
in your code, but the linker requires them because `stm32f4xx_hal_tim.c` references
functions and types defined inside them.

---

## The Source File / Configuration File Relationship

This is one of the most important things to understand when building a HAL project
without CubeMX. Every peripheral driver has **two mandatory pieces** that must
always exist together:

```
stm32f4xx_hal_xxx.c        ←→    #define HAL_XXX_MODULE_ENABLED
  (source file in                   (define in stm32f4xx_hal_conf.h)
   Drivers/Src/)
```

These two are two sides of the same coin:

- The `.c` file contains the actual driver code — the functions you call.
- The `#define` in `stm32f4xx_hal_conf.h` tells the compiler to activate that
  driver. Without it, the HAL header files will not expose the API for that
  peripheral, even if the `.c` file is present in the project.

If **one is missing**, the result is always a build failure:

```
.c file present, #define missing  →  compiler error: undefined type or function
#define present, .c file missing  →  linker error: undefined reference
```

Both must be present, and they must match.

### Checklist for This Project (GPIO + TIM3)

Use this table to verify your project before building. Every row must have
both columns checked.

| `stm32f4xx_hal_conf.h` define   | Source file in `Drivers/Src/`     | Purpose                        |
|---------------------------------|-----------------------------------|--------------------------------|
| `HAL_MODULE_ENABLED`            | `stm32f4xx_hal.c`                 | Core HAL, `HAL_Init()`         |
| `HAL_RCC_MODULE_ENABLED`        | `stm32f4xx_hal_rcc.c`             | Clock enable macros            |
| `HAL_GPIO_MODULE_ENABLED`       | `stm32f4xx_hal_gpio.c`            | GPIO init and control          |
| `HAL_TIM_MODULE_ENABLED`        | `stm32f4xx_hal_tim.c`             | Timer base driver              |
| *(no define needed)*            | `stm32f4xx_hal_tim_ex.c`          | Internal dependency of TIM     |
| `HAL_DMA_MODULE_ENABLED`        | `stm32f4xx_hal_dma.c`             | Internal dependency of TIM     |
| `HAL_CORTEX_MODULE_ENABLED`     | `stm32f4xx_hal_cortex.c`          | NVIC, `HAL_NVIC_EnableIRQ()`   |
| `HAL_FLASH_MODULE_ENABLED`      | `stm32f4xx_hal_flash.c`           | Internal dependency of HAL_Init|
| `HAL_PWR_MODULE_ENABLED`        | `stm32f4xx_hal_pwr.c`             | Internal dependency of HAL_Init|

Note the special case of `stm32f4xx_hal_tim_ex.c` — it is a companion file to
`stm32f4xx_hal_tim.c` and does not have its own `#define`. It must be present
in the project but requires no entry in `stm32f4xx_hal_conf.h`.

---

## Step 5 — Copy the CMSIS System File

Copy `system_stm32f4xx.c` from the CMSIS templates:

Source:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
```

Destination:
```
YourProject/Src/system_stm32f4xx.c
```

You do not need to modify this file — use it as-is. But it is worth understanding
what it actually does, because it runs before `main()` and its effects are always
active.

### What system_stm32f4xx.c actually does

The file provides three things to the rest of the system:

**`SystemInit()`** — called automatically by the startup file (`startup_stm32f4xx.s`)
immediately after reset, before `main()` is reached. For the STM32F411 it does
exactly two things:

```
1. Enables the FPU
   SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

   This grants full access to coprocessors CP10 and CP11, which are the
   floating-point unit on the Cortex-M4. Without this line, any floating-point
   instruction would trigger a hard fault. It only executes when the toolchain
   is configured for hardware FPU (__FPU_PRESENT == 1 and __FPU_USED == 1).

2. Vector table relocation (conditional, disabled by default)
   SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;

   This only executes if USER_VECT_TAB_ADDRESS is defined — which it is not
   by default. In a standard project the hardware reset value of SCB->VTOR
   already points to Flash at 0x08000000, which is correct. The startup file
   defines the vector table there; this file does not move it unless you
   explicitly ask it to.
```

**`SystemCoreClock`** — a global variable initialized to 16000000 (HSI frequency).
This variable is updated automatically every time you call `HAL_RCC_ClockConfig()`
in your `SystemClock_Config()`. The rest of the HAL uses it to calculate timeouts,
baud rates, and other frequency-dependent values. It must always reflect the real
current HCLK frequency.

**`AHBPrescTable[]` and `APBPrescTable[]`** — lookup tables used internally by
`SystemCoreClockUpdate()` to calculate bus frequencies from the RCC register
divider fields. You do not call these directly, but HAL uses them.

### What system_stm32f4xx.c does NOT do

It does **not** configure the system clock. Unlike what CubeMX used to generate,
this file leaves the clock tree exactly as the hardware reset left it — HSI on,
no PLL, all dividers at 1. The clock configuration is entirely your responsibility
in `SystemClock_Config()` inside `main.c`.

### How to verify the FPU configuration

You can verify that both flags are set correctly by adding this block anywhere
in `main.c` before building:

```c
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    #pragma message("FPU: enabled")
#else
    #pragma message("FPU: NOT enabled")
#endif
```

This produces a message in the build console during compilation — no runtime
cost, nothing added to the binary.

**Important:** the IDE code editor will gray out the inactive branch, making
it look like those lines are being ignored. Do not trust the gray color — it is
only the editor's syntax highlighting. The compiler is a completely separate
process. The truth is always in the build console output, where you will see:

```
note: '#pragma message: FPU: enabled'
```

You can also confirm the FPU status directly from the compiler command line,
without needing the `#pragma` at all. Look for these two flags in the
`arm-none-eabi-gcc` invocation in the build console:

```
-mfpu=fpv4-sp-d16      ← specifies the FPU hardware unit
-mfloat-abi=hard       ← sets __FPU_USED = 1, enables hardware float instructions
```

If you see `-mfloat-abi=soft` instead, `__FPU_USED` is 0, `SystemInit()` will
skip the FPU activation, and any floating-point operation will trigger a hard
fault at runtime. This setting lives under:

```
Project Properties → C/C++ Build → Settings → MCU Settings → Floating-point unit
```

---

## Step 6 — Create the HAL Configuration File

The HAL configuration file tells the compiler which peripheral modules are active.
You must create this file manually.

Copy the template from the firmware package:

Source:
```
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf_template.h
```

Destination:
```
YourProject/Inc/stm32f4xx_hal_conf.h
```

Note: rename it — remove `_template` from the filename.

Now open `stm32f4xx_hal_conf.h` and replace the module enable section with only
the modules you need for this project:

```c
/* Active modules for GPIO + TIM3 project */
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
```

Comment out or delete all other `HAL_xxx_MODULE_ENABLED` lines.

---

## Step 7 — Create the Interrupt Handler File

Create `YourProject/Src/stm32f4xx_it.c` with the following content:

```c
/*
 * stm32f4xx_it.c
 * Interrupt service routines
 * Author: your name
 */

#include "stm32f4xx_hal.h"

/* Declare the TIM3 handle — defined in main.c */
extern TIM_HandleTypeDef htim3;

/* SysTick handler — required by HAL for HAL_Delay() and timeouts */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* TIM3 update event handler */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}
```

---

## Step 8 — Write main.c

Replace the contents of `main.c` with the following:

```c
/*
 * main.c
 * HAL Blinky — LED on PA5 toggled every 250 ms by TIM3
 * Author: your name
 */

#include "stm32f4xx_hal.h"

/* TIM3 handle — must be global so stm32f4xx_it.c can access it */
TIM_HandleTypeDef htim3;

/* Private function prototypes */
static void SystemClock_Config(void);
static void gpio_Init(void);
static void tim3_Init(void);

int main(void)
{
    HAL_Init();           /* initialize HAL: SysTick, cache, priority grouping */
    SystemClock_Config(); /* configure clock tree: HSI at 16 MHz               */
    gpio_Init();          /* configure PA5 as push-pull output                  */
    tim3_Init();          /* configure TIM3: update event every 250 ms          */

    while (1)
    {
        /* application loop — LED toggling happens in the callback */
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
 * Configures PA5 as push-pull output — onboard LED on Nucleo board
 */
static void gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOA clock on AHB1 bus
       Same as bare-metal: RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA5 */
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * tim3_Init
 * Configures TIM3 to generate an update event every 250 ms
 *
 * Clock chain:
 *   HSI (16 MHz) → APB1 (16 MHz) → TIM3 clock (16 MHz)
 *
 * PSC = 15999  →  tick = 16,000,000 / (15999 + 1) = 1,000 Hz  (1 ms per tick)
 * ARR = 249    →  period = (249 + 1) x 1 ms = 250 ms
 */
static void tim3_Init(void)
{
    /* Enable TIM3 clock on APB1 bus */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure TIM3 base */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 15999;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 249;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim3);

    /* Start TIM3 in interrupt mode — enables the update event interrupt */
    HAL_TIM_Base_Start_IT(&htim3);

    /* Enable TIM3 interrupt line in the NVIC */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/*
 * HAL_TIM_PeriodElapsedCallback
 * Called automatically by HAL_TIM_IRQHandler() every time a timer
 * update event fires. Shared by all timers — always check htim->Instance.
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

## Step 9 — Final Project Structure

After all steps, your project should look exactly like this:

```
YourProject/
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

Include paths configured in IDE settings (not visible in the project tree):
```
${workspace_loc:/${ProjName}/Inc}
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include
~/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include
```

---

## Step 10 — Build and Flash

Build the project with `Ctrl+B`. If the build succeeds, flash to the board with
`Run → Debug` or `Run → Run`.

The onboard LED on PA5 should blink at 2 Hz (250 ms on, 250 ms off).

---

## Common Build Errors and Their Fixes

These errors were found during the construction of this tutorial. If you see them,
you know exactly what to add.

```
error: unknown type name 'DMA_HandleTypeDef'
```
Fix: add `#define HAL_DMA_MODULE_ENABLED` to `stm32f4xx_hal_conf.h` and copy
`stm32f4xx_hal_dma.c` to `Drivers/STM32F4xx_HAL_Driver/Src/`.

```
undefined reference to 'HAL_TIMEx_BreakCallback'
undefined reference to 'HAL_TIMEx_CommutCallback'
```
Fix: copy `stm32f4xx_hal_tim_ex.c` to `Drivers/STM32F4xx_HAL_Driver/Src/`.

---

## Key Concepts Summary

```
HAL_Init()
    │
    ├── initializes SysTick at 1 ms
    ├── configures cache and prefetch
    └── calls HAL_MspInit() (empty by default)

SystemClock_Config()
    │
    └── HSI ON → SYSCLK = 16 MHz → AHB = 16 MHz → APB1 = 16 MHz

gpio_Init()
    │
    ├── __HAL_RCC_GPIOA_CLK_ENABLE()   (same as RCC->AHB1ENR |= ...)
    └── HAL_GPIO_Init(GPIOA, &struct)

tim3_Init()
    │
    ├── __HAL_RCC_TIM3_CLK_ENABLE()
    ├── HAL_TIM_Base_Init(&htim3)
    ├── HAL_TIM_Base_Start_IT(&htim3)
    └── HAL_NVIC_EnableIRQ(TIM3_IRQn)

TIM3_IRQHandler()   [in stm32f4xx_it.c]
    │
    └── HAL_TIM_IRQHandler(&htim3)
            │
            └── HAL_TIM_PeriodElapsedCallback(&htim3)
                    │
                    └── HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)
```

Reference: UM1725 Rev 8 — Description of STM32F4 HAL and low-layer drivers
(STMicroelectronics, March 2023)

---

---

# Phase 2 — Adding a New Module: USART2

At this point the project is working — LED blinking, TIM3 interrupt firing every
250 ms. Phase 2 shows how to add a new peripheral to an existing project, following
exactly the same pattern used for TIM3. The process is always the same:

```
1. Add the source file to Drivers/Src/
2. Add the #define to stm32f4xx_hal_conf.h
3. Add the handle at the top of main.c
4. Write the init function
5. Call it from main()
```

## Step 1 — Add the USART2 Source File

Copy one new file to `Drivers/STM32F4xx_HAL_Driver/Src/`:

```
stm32f4xx_hal_uart.c        ← HAL_UART_Init(), HAL_UART_Transmit()
```

## Step 2 — Add the Module Define

Open `stm32f4xx_hal_conf.h` and add one line:

```c
#define HAL_UART_MODULE_ENABLED
```

Updated checklist — now covering GPIO + TIM3 + USART2:

| `stm32f4xx_hal_conf.h` define   | Source file in `Drivers/Src/`     | Purpose                        |
|---------------------------------|-----------------------------------|--------------------------------|
| `HAL_MODULE_ENABLED`            | `stm32f4xx_hal.c`                 | Core HAL, `HAL_Init()`         |
| `HAL_RCC_MODULE_ENABLED`        | `stm32f4xx_hal_rcc.c`             | Clock enable macros            |
| `HAL_GPIO_MODULE_ENABLED`       | `stm32f4xx_hal_gpio.c`            | GPIO init and control          |
| `HAL_TIM_MODULE_ENABLED`        | `stm32f4xx_hal_tim.c`             | Timer base driver              |
| *(no define needed)*            | `stm32f4xx_hal_tim_ex.c`          | Internal dependency of TIM     |
| `HAL_DMA_MODULE_ENABLED`        | `stm32f4xx_hal_dma.c`             | Internal dependency of TIM     |
| `HAL_CORTEX_MODULE_ENABLED`     | `stm32f4xx_hal_cortex.c`          | NVIC, `HAL_NVIC_EnableIRQ()`   |
| `HAL_FLASH_MODULE_ENABLED`      | `stm32f4xx_hal_flash.c`           | Internal dependency of HAL_Init|
| `HAL_PWR_MODULE_ENABLED`        | `stm32f4xx_hal_pwr.c`             | Internal dependency of HAL_Init|
| `HAL_UART_MODULE_ENABLED`       | `stm32f4xx_hal_uart.c`            | UART/USART transmit and receive|

## Step 3 — Add the USART2 Handle

Just like `htim3`, the handle must be global. Add it alongside `htim3` at the
top of `main.c`:

```c
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart2;   /* ← add this line */
```

## Step 4 — Write the usart2_Init() Function

Compare this with `tim3_Init()` — the structure is identical. The only new
element is the GPIO alternate function block at the beginning. USART2 uses
physical pins (PA2 for TX, PA3 for RX), so those pins must be configured as
alternate function before initializing the peripheral.

GPIO output pins like the LED on PA5 do not need alternate function — the CPU
drives them directly. Communication peripherals do, because the signal is
generated by the peripheral hardware, not the CPU. Every peripheral that uses
pins — USART, SPI, I2C, I2S — will require this step.

```c
static void usart2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Step 1: enable clocks — both USART2 and GPIOA */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* already called in gpio_Init() — safe to call again */

    /* Step 2: configure PA2 (TX) and PA3 (RX) as alternate function
       AF7 is USART2 on the STM32F411 — see the alternate function table in the datasheet */
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Step 3: configure USART2 */
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

## Step 5 — Call usart2_Init() from main()

Add the call after `tim3_Init()`:

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    gpio_Init();
    tim3_Init();
    usart2_Init();       /* ← add this line */

    while (1)
    {
    }
}
```

## Step 6 — Transmit Data

With the peripheral initialized, sending data is a single call:

```c
/* Send a string — blocking mode, 100 ms timeout */
char msg[] = "Hello from STM32\r\n";
HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 100);
```

This can go anywhere — inside `main()`, inside the TIM3 callback to send a
message every 250 ms, or inside any other function.

## What Is Different Compared to TIM3

```
tim3_Init()                        usart2_Init()
─────────────────────────────────  ─────────────────────────────────
__HAL_RCC_TIM3_CLK_ENABLE()        __HAL_RCC_USART2_CLK_ENABLE()
                                   __HAL_RCC_GPIOA_CLK_ENABLE()
                                   Configure PA2/PA3 as AF7   ← new
Fill TIM_HandleTypeDef             Fill UART_HandleTypeDef
HAL_TIM_Base_Init(&htim3)          HAL_UART_Init(&huart2)
HAL_TIM_Base_Start_IT(&htim3)      (no Start needed for polling TX)
HAL_NVIC_EnableIRQ(TIM3_IRQn)      (no NVIC needed for polling TX)
```

---

---

# Appendix — Flash Cache and Prefetch

When you call `HAL_Init()`, two Flash acceleration features are enabled
internally: the **instruction cache** and the **prefetch buffer**. They appear
in the HAL source and in comments throughout the code, so it is worth
understanding what they actually do.

## The Problem They Solve

The Cortex-M4 core in the STM32F411 can run at up to 100 MHz. The internal
Flash memory cannot keep up at that speed — it has a fixed access time that
becomes a bottleneck at higher clock frequencies. Without any acceleration, the
CPU would have to insert wait states (idle cycles) on every Flash read, wasting
execution time.

At 16 MHz (HSI, no PLL) this is not a problem — Flash is fast enough and zero
wait states are needed. But the features are enabled by HAL regardless, because
the same code must work at any frequency.

## Instruction Cache

```
Without cache:                     With cache:
                                   
CPU requests instruction           CPU requests instruction
       ↓                                  ↓
Flash read (slow)                  Cache hit? → instant
       ↓                                  ↓ (miss)
CPU executes                       Flash read → stored in cache
                                          ↓
                                   CPU executes
                                   Next time: cache hit → instant
```

The instruction cache is a small fast memory (4 KB, 64 lines of 64 bytes each)
that stores recently fetched instructions. When the CPU executes a loop, the
loop body is fetched from Flash once and stored in the cache. Every subsequent
iteration is served from the cache at full CPU speed, with no Flash access at
all. For embedded firmware, which spends most of its time in loops and interrupt
handlers, this is a significant gain.

## Prefetch Buffer

The prefetch buffer works differently — instead of waiting for the CPU to request
the next instruction, it reads ahead in Flash while the CPU is still executing
the current instruction. Since Flash is accessed in 128-bit chunks and the
Cortex-M4 instructions are 16 or 32 bits wide, a single Flash read delivers
multiple instructions at once. The prefetch buffer holds the next chunk ready
before the CPU needs it.

```
CPU executing instruction N        Flash already reading instruction N+4, N+5...
CPU executing instruction N+1      (already in buffer — no wait)
CPU executing instruction N+2      (already in buffer — no wait)
```

## In Practice for This Course

At 16 MHz with `FLASH_LATENCY_0` (zero wait states), both features have minimal
visible effect — Flash is already fast enough. Their importance grows when the
clock is increased to 100 MHz via PLL (future sessions), where `FLASH_LATENCY_3`
(three wait states) would be needed without them, and the cache and prefetch
buffer become essential to maintain acceptable performance.

`HAL_Init()` enables both unconditionally so the project is ready for any clock
configuration without requiring changes to the initialization sequence.
