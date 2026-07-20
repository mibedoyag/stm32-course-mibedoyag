# DMA + PWM "Breathing LED" — Step-by-Step Tutorial (Nucleo-F411RE)

*A hands-on continuation of the Phase 1 HAL tutorial (TIM3 interrupt-driven blink). This exercise repurposes TIM3 into a PWM generator whose duty cycle is driven entirely by DMA, producing a smooth "breathing" LED effect with zero CPU involvement once configured. Companion to the "DMA in STM32F4xx (HAL) — Conceptual Reference Document."*

> **Revision note:** Steps 5 and 6 were corrected after testing this exercise on real hardware — a wrong HAL start function and a dangling DMA handle pointer, both explained where they occur.

---

## Table of Contents

0. [Starting Point Recap](#step-0--starting-point-recap)
1. [Reconfigure TIM3 for PWM Mode](#step-1--reconfigure-tim3-for-pwm-mode)
2. [Building the Duty-Cycle Lookup Table](#step-2--building-the-duty-cycle-lookup-table)
3. [Configuring the DMA Handle for TIM3_UP](#step-3--configuring-the-dma-handle-for-tim3_up)
4. [Linking DMA to TIM3 with `__HAL_LINKDMA`](#step-4--linking-dma-to-tim3-with-__hal_linkdma)
5. [Starting the Update-Triggered DMA Transfer](#step-5--starting-the-update-triggered-dma-transfer)
6. [Implementing the DMA1_Stream2 IRQ Handler](#step-6--implementing-the-dma1_stream2-irq-handler)
7. [Verifying on Hardware](#step-7--verifying-on-hardware)
8. [Common Build Errors for This Exercise](#step-8--common-build-errors-for-this-exercise)

---

## Step 0 — Starting Point Recap

Before touching anything, let's be precise about what we're building on top of, so the diff is clear.

### What you already have (Phase 1 tutorial)

- TIM3 configured and initialized via HAL, running in **interrupt mode** (`HAL_TIM_Base_Start_IT`)
- A GPIO pin manually toggled inside `HAL_TIM_PeriodElapsedCallback()` — the classic interrupt-driven blink
- `stm32f4xx_hal_tim.c` (and `stm32f4xx_hal_tim_ex.c`) already in the project's `Src` folder
- TIM3 clock already enabled (`__HAL_RCC_TIM3_CLK_ENABLE()`)

### What changes in this exercise

We're not adding a new peripheral — we're **repurposing TIM3 itself**, from "a timer that fires an interrupt" to "a timer that generates PWM, whose duty cycle is updated automatically by DMA."

| | Phase 1 (interrupt blink) | This exercise (DMA breathing) |
|---|---|---|
| TIM3 role | Time base only, no PWM output | PWM generator (Channel 1) |
| How duty/state changes | CPU toggles a GPIO inside an ISR | DMA writes a new duty value into CCR1 every period, no CPU involved |
| GPIO used | Onboard LED (PA5) or a manually toggled pin | PA6 (TIM3_CH1, alternate function), driving an **external** LED |
| Interrupts used | `TIM3_IRQn` (period elapsed) | `DMA1_Stream2_IRQn` (transfer complete) |

Worth flagging explicitly: **we are not disabling or removing TIM3's timing function** — the timer still counts and still generates periodic update events. What changes is *who reacts to those events, and how*. In Phase 1, the CPU reacted via an ISR. Now, the DMA controller reacts, silently, by pushing the next duty-cycle value into the hardware register — the CPU is no longer in that loop at all.

### Hardware note before we start

Since PA5 (the Nucleo's onboard LED) is *not* on TIM3, this exercise requires an **external LED with a current-limiting resistor** (~220–330Ω) wired to PA6. PWM-driven brightness control is only visible with an LED wired to a PWM-capable pin — the onboard LED being tied to TIM2 is a hardware fact, not a configuration choice.

---

## Step 1 — Reconfigure TIM3 for PWM Mode

### Pin selection: PA6 → TIM3_CH1

On the Nucleo-F411RE, **PA6** can be routed to **TIM3_CH1** via **Alternate Function 2 (AF2)** — a fixed hardware mapping, worth having students confirm themselves in the datasheet's alternate function table.

### GPIO configuration for PA6 (Alternate Function, not plain output)

This is the first conceptual shift from Phase 1: PA6 is **not** configured as a regular GPIO output anymore. It's configured as an **Alternate Function** pin, meaning the timer's internal hardware — not your code — drives the pin's logic level.

```c
__HAL_RCC_GPIOA_CLK_ENABLE();

GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin       = GPIO_PIN_6;
GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;   // Alternate Function, Push-Pull
GPIO_InitStruct.Pull      = GPIO_NOPULL;
GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;     // <-- the key difference vs a plain output pin
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

**Worth pausing on `GPIO_MODE_AF_PP` vs `GPIO_MODE_OUTPUT_PP`:** in Phase 1, the CPU decided when the pin went high or low. Here, the pin is electrically connected to TIM3's internal compare-match hardware. The `Alternate` field tells the GPIO peripheral *which* internal hardware signal to listen to — get the wrong `GPIO_AFx_TIMy` value here, and the pin will simply never toggle, with no error anywhere to tell you why.

### TIM3 handle: base timer configuration

The `Prescaler` and `Period` together set your PWM frequency — the same PSC/ARR relationship already covered for the interrupt-driven blink. The difference now is that `Period` also defines the **resolution of your duty cycle**: if `Period = 999`, you have 1000 discrete duty-cycle steps (0–999) to work with.

```c
TIM_HandleTypeDef htim3;

htim3.Instance               = TIM3;
htim3.Init.Prescaler         = /* value depends on your current SystemCoreClock config */;
htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
htim3.Init.Period            = 999;   // 1000 steps of duty-cycle resolution
htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

HAL_TIM_PWM_Init(&htim3);
```

The exact `Prescaler` value is intentionally left blank here — a good spot to have students **compute it themselves** from whatever clock configuration the project is currently running, rather than copying a fixed number. A reasonable target is a PWM frequency high enough to avoid visible flicker (a few hundred Hz to a few kHz is plenty for an LED).

**Note:** whatever `Period` you land on here, keep it in mind for Step 2 — the duty-cycle lookup table has to be scaled to *this exact value*, not to whatever number happened to be used as an example.

### Channel 1 configuration: PWM mode

```c
TIM_OC_InitTypeDef sConfigOC = {0};
sConfigOC.OCMode     = TIM_OCMODE_PWM1;
sConfigOC.Pulse      = 0;   // initial duty cycle — DMA will overwrite this continuously
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
```

`TIM_OCMODE_PWM1` makes Channel 1 behave as: *output high while the counter is below `Pulse` (i.e., CCR1), output low once the counter passes it.* The `Pulse` value set here (0) is just a starting point — once DMA is running (Step 5), this register gets overwritten automatically on every timer update.

### What we deliberately have NOT done yet

Notice we haven't called `HAL_TIM_PWM_Start()`. Starting PWM the "normal" way would work, but would leave the CPU responsible for manually changing the duty cycle — back to the Phase 1 pattern, just with PWM instead of GPIO toggling. We want DMA to drive this, which happens in Step 5 — and Step 5 will *not* call `HAL_TIM_PWM_Start_DMA()` either, for reasons explained there.

---

## Step 2 — Building the Duty-Cycle Lookup Table

### What the table represents

Each entry is one duty-cycle value — between 0 and your configured `Period` (999) — representing how bright the LED should be at that moment in the breathing cycle. DMA walks through this array from start to end, then (in **Circular** mode) wraps back to the beginning automatically.

### Linear ramp vs sine-shaped ramp — and why it matters

A simple linear ramp (0 → 999 → 0) works, but doesn't actually **look** like breathing — human brightness perception is not linear, so a linear ramp looks like it lingers at the bright end and rushes through the dim end. A **sine-shaped** ramp (the rising half of a sine wave, 0 to π) produces a much more natural fade, easing in and out at both extremes. Note that a single 0→π sweep already covers a full rise-*and*-fall (sin(0) = 0, sin(π/2) = peak, sin(π) = 0 again) — one full pass through the table is one full breath, not half of one.

This ties back to the "why DMA, not CPU" motivation from the reference document: computing a sine value in real time is trivial for the CPU, but doing it **precisely on every timer period without drift** is exactly the kind of repetitive, timing-critical job worth handing off entirely.

### Generating the table (once, offline — not at runtime)

```c
#define BREATH_TABLE_SIZE 200

static uint32_t breathTable[BREATH_TABLE_SIZE];  // <-- static: see note below

void BuildBreathTable(void)
{
    for (uint32_t i = 0; i < BREATH_TABLE_SIZE; i++)
    {
        float angle = (float)i / (float)(BREATH_TABLE_SIZE - 1) * 3.14159f; // 0 to π
        breathTable[i] = (uint32_t)(sinf(angle) * 999.0f);                  // scaled to Period
    }
}
```

### Why `static` here is not optional

This array must not be a local variable inside a function that returns. The DMA controller holds a raw pointer to `breathTable`. If it were declared with a lifetime that ended before DMA finishes, DMA would end up reading from memory that's no longer guaranteed to hold your data. Declaring it `static` (or global) at file scope guarantees it lives for the entire life of the program — exactly the DMA's assumption.

### Two things this table must match — or the effect breaks silently

Both of these are easy to get wrong because the code compiles and *something* happens on the LED — just not what you expect.

**1. The scale factor must match your actual `Period`, exactly.**
The `999.0f` in `sinf(angle) * 999.0f` is not a magic constant — it must equal whatever `htim3.Init.Period` you actually configured in Step 1. If `Period` is, say, `8000` but the table is still scaled to `999`, the table's peak value (999) is only `999 / 8000 ≈ 12.5%` of the full duty-cycle range. The LED will still "breathe," but so dimly it can look like nothing is happening at all. **Whenever you change `Period`, update this scale factor to match — there is no compiler warning if you forget.**

**2. The table size, combined with the timer's update rate, sets how fast the breathing looks — and it's easy to make it far too fast.**
DMA advances one table entry per TIM3 **Update event**, and the Update event rate is:

```
update_rate = TIM3_clock / (Prescaler + 1) / (Period + 1)
```

The *time for one full breath* (one pass through the table) is then:

```
breath_period_seconds = BREATH_TABLE_SIZE / update_rate
```

For example, with `Prescaler = 0`, `Period = 8000`, and a 16 MHz timer clock: `update_rate ≈ 16,000,000 / 8001 ≈ 2000 Hz`. With `BREATH_TABLE_SIZE = 200`, that's `200 / 2000 = 0.1 s` per breath — a 10 Hz flicker that reads as *blinking*, not breathing. To get a natural-looking ~1.5 s breath at that same update rate, you'd need `BREATH_TABLE_SIZE ≈ 1.5 × 2000 = 3000`.

**Rule of thumb:** pick `BREATH_TABLE_SIZE` by deciding how long you want one breath to take, then multiplying by the Update-event rate you actually computed for your `Prescaler`/`Period` — don't just copy `200` from this document without checking it against your own numbers. A full breath cycle over a second or two feels natural; much faster than that looks like blinking, much slower looks like the LED is stuck.

---

## Step 3 — Configuring the DMA Handle for TIM3_UP

### Confirming our coordinates

**TIM3_UP → DMA1, Stream 2, Channel 5.** This is fixed by the silicon — not a choice being made, just a fact being looked up and used correctly.

### Direction and mode — reasoning through them explicitly

- **Which way is data moving?** Our `breathTable` array (memory) is the source. TIM3's CCR1 register (a peripheral register) is the destination. So: **`DMA_MEMORY_TO_PERIPH`**.
- **Once, or forever?** We want the breathing pattern to loop indefinitely, with no CPU involvement to restart it. So: **`DMA_CIRCULAR`**.

This is the `MEMORY_TO_PERIPH` + `CIRCULAR` pairing flagged in the reference document as a less-intuitive combination — worth pointing out to students as a concrete instance of that discussion.

### Clock enable — before anything else

The DMA1 clock must be enabled **before** `HAL_DMA_Init()` touches any of `DMA1_Stream2`'s registers — otherwise the call will fault or fail silently:

```c
__HAL_RCC_DMA1_CLK_ENABLE();   // must come first — DMA1 registers don't exist until this runs
```

### The DMA handle

```c
DMA_HandleTypeDef hdma_tim3_up;

hdma_tim3_up.Instance                 = DMA1_Stream2;
hdma_tim3_up.Init.Channel             = DMA_CHANNEL_5;
hdma_tim3_up.Init.Direction           = DMA_MEMORY_TO_PERIPH;
hdma_tim3_up.Init.PeriphInc           = DMA_PINC_DISABLE;   // CCR1 is a fixed address
hdma_tim3_up.Init.MemInc              = DMA_MINC_ENABLE;    // breathTable advances each transfer
hdma_tim3_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
hdma_tim3_up.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
hdma_tim3_up.Init.Mode                = DMA_CIRCULAR;
hdma_tim3_up.Init.Priority            = DMA_PRIORITY_MEDIUM;
hdma_tim3_up.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

HAL_DMA_Init(&hdma_tim3_up);
```

A few fields worth explaining deliberately:

- **`PeriphInc = DMA_PINC_DISABLE`**: the destination is always the *same* register, CCR1 — DMA must **not** advance the peripheral-side address after each transfer.
- **`MemInc = DMA_MINC_ENABLE`**: the source side (memory) *does* advance — each transfer reads the *next* entry in `breathTable`.
- **`PeriphDataAlignment` / `MemDataAlignment` = `WORD`**: CCR1 is a 32-bit register, and `breathTable` is `uint32_t`, so both sides move 32 bits at a time. A common silent-mismatch spot — if the table were `uint16_t` while these say `WORD`, DMA would read/write the wrong byte boundaries and produce garbage duty values, with no compiler error to catch it.

### Handle lifetime: the same rule as the buffer

The "declare it `static`/global" rule from Step 2 doesn't only apply to `breathTable` — it applies just as strictly to `hdma_tim3_up` itself, and this is easy to miss because the code shown above *looks* self-contained inside a single init function.

Here's the trap: `__HAL_LINKDMA` (Step 4) stores `&hdma_tim3_up` — a raw pointer — inside `htim3.hdma[]`. If `hdma_tim3_up` is declared as a local variable inside, say, `tim3_Init()`, that pointer is only valid *while `tim3_Init()`'s stack frame still exists*. The moment `tim3_Init()` returns, that stack memory is fair game for the next function's local variables — and on the very next init call, it usually does get overwritten. `htim3.hdma[TIM_DMA_ID_UPDATE]` is left pointing at memory that no longer holds a valid `DMA_HandleTypeDef`.

The symptom is nasty precisely because it's *not* immediate: `HAL_DMA_Init()` succeeds, `__HAL_LINKDMA()` succeeds, the rest of `tim3_Init()` finishes cleanly — the corruption only bites later, when something (in this exercise, the DMA start call in Step 5, or the IRQ handler in Step 6) actually dereferences that now-stale pointer. Depending on what got written over that stack memory in between, you can see anything from "DMA silently does nothing" to a hard fault deep inside HAL code that looks unrelated to DMA at all.

**Fix:** declare `hdma_tim3_up` at file scope (same place as `htim3`), not as a local variable inside an init function:

```c
/* File scope, alongside htim3 — must outlive tim3_Init() */
DMA_HandleTypeDef hdma_tim3_up = {0};
```

This also happens to be required for Step 6, since the DMA IRQ handler needs to reach this same handle from a different source file.

### What's still missing

This handle, on its own, doesn't know it belongs to TIM3 yet — there's no mention of `htim3` anywhere above. The handle describes *how the stream behaves*, but not *which peripheral it's serving*. That's what `__HAL_LINKDMA` is for — Step 4.

---

## Step 4 — Linking DMA to TIM3 with `__HAL_LINKDMA`

This step turns two independently-configured handles (`htim3` and `hdma_tim3_up`) into a working pair.

### Which field, specifically?

Unlike USART2 (simple `hdmatx` / `hdmarx` fields), `TIM_HandleTypeDef` has an **array** of DMA handle pointers, `hdma[]`, since a single timer can have DMA requests tied to several different events (Update, each Capture/Compare channel, Trigger, etc.):

```c
__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], hdma_tim3_up);
```

`TIM_DMA_ID_UPDATE` is the index corresponding to the **Update event** — matching exactly what was configured on the DMA side in Step 3 (`TIM3_UP`). Worth stating explicitly: **the `TIM_DMA_ID_x` constant linked must match the DMA request source actually configured** — pairing `TIM_DMA_ID_CC1` with a DMA handle wired for `TIM3_UP` would compile fine and fail silently at runtime. Keep this pairing in mind for Step 5: it's exactly the mismatch that shows up there if the wrong start function is used.

### What this line does, concretely

1. `htim3.hdma[TIM_DMA_ID_UPDATE] = &hdma_tim3_up;` — so when the DMA transfer is started in Step 5, HAL knows exactly which DMA stream is tied to TIM3's Update event.
2. `hdma_tim3_up.Parent = &htim3;` — so when `DMA1_Stream2_IRQHandler` fires, the generic DMA IRQ handler can trace back to `htim3` and invoke the correct timer-level callback if implemented (e.g., `HAL_TIM_PWM_PulseFinishedCallback`).

### The full ordering chain

Clock → DMA init → link → peripheral start is really **one continuous dependency chain**, not separate independent checklist items:

```c
__HAL_RCC_DMA1_CLK_ENABLE();                                     // 0. Clock — must exist first
HAL_DMA_Init(&hdma_tim3_up);                                     // 1. DMA handle configured
__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], hdma_tim3_up);    // 2. Linked
// DMA transfer started next, in Step 5                          // 3. Started
```

If step 2 happens before step 1, or if this whole block runs *after* the DMA transfer is started, the link either points to garbage or arrives too late to matter — another instance of the "compiles fine, silently does nothing" failure mode.

---

## Step 5 — Starting the Update-Triggered DMA Transfer

### The obvious-looking call that is actually wrong here

It's tempting to reach for `HAL_TIM_PWM_Start_DMA()`, since it looks like exactly the "start PWM + DMA together" call this exercise needs:

```c
/* Looks right — is NOT right for this configuration. See explanation below. */
HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)breathTable, BREATH_TABLE_SIZE);
```

**Worth walking through carefully — the symptom it produces (a fault or a frozen board) gives no hint that the problem is "wrong function."**

Look at what `HAL_TIM_PWM_Start_DMA()` actually does internally, for `TIM_CHANNEL_1`, inside `stm32f4xx_hal_tim.c`:

```c
case TIM_CHANNEL_1:
{
  htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
  htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;
  htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError;

  HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData,
                    (uint32_t)&htim->Instance->CCR1, Length);

  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);   // <-- Capture/Compare 1 DMA request, NOT Update
  break;
}
```

It reaches into `htim->hdma[TIM_DMA_ID_CC1]` — the **Capture/Compare 1** slot of the `hdma[]` array — and arms the **CC1 DMA request** (`TIM_DMA_CC1`, the `DIER` `CC1DE` bit). But Step 4 linked `hdma_tim3_up` at `hdma[TIM_DMA_ID_UPDATE]`, the **Update** slot — a different array index entirely. `hdma[TIM_DMA_ID_CC1]` was never linked to anything, so it's still `NULL`. The very first line above, `htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = ...`, dereferences that `NULL` pointer — a hard fault, right at the `HAL_TIM_PWM_Start_DMA()` call site.

Even setting the crash aside: `HAL_TIM_PWM_Start_DMA()` is simply built for a different use case — DMA triggered by the **Capture/Compare match event**, not the **Update event**. Our DMA stream (`DMA1_Stream2` / Channel 5) is wired in silicon specifically to the `TIM3_UP` (Update) request line.

There is no `Channel`-based HAL "start" call that arms `TIM_DMA_UPDATE` and targets `CCR1` — HAL doesn't ship a canned function for "Update event writes into a Capture/Compare register." So instead of forcing a mismatched HAL call to work, we start the transfer manually, the same way `HAL_TIM_Base_Start_DMA()` does internally. That function *does* use `hdma[TIM_DMA_ID_UPDATE]` correctly — it just targets `ARR` instead of `CCR1`, since it's meant for a different purpose. We replicate that same pattern, pointed at `CCR1`:

```c
/* Start the Update -> CCR1 DMA transfer manually. HAL_TIM_PWM_Start_DMA() cannot be
 * used here: for TIM_CHANNEL_1 it arms the CC1 DMA request via hdma[TIM_DMA_ID_CC1],
 * not the Update request via hdma[TIM_DMA_ID_UPDATE] that Step 4 actually linked. */
HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], (uint32_t)breathTable,
                  (uint32_t)&htim3.Instance->CCR1, BREATH_TABLE_SIZE);

__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);   // arm the Update DMA request (DIER UDE bit)

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);       // enable the CC1 output + start the counter
                                                 // (this call does not touch DMA at all)
```

What each line does:

1. `HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], ...)` — starts the DMA stream that Step 4 actually linked, source `breathTable`, destination `&TIM3->CCR1`, length `BREATH_TABLE_SIZE`, interrupt mode (so Transfer-Complete/Half/Error callbacks fire — see Step 6).
2. `__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE)` — sets the `UDE` bit in `TIM3->DIER`: "on every Update event, generate a DMA request." Without this, the timer never asks the DMA controller to move anything, regardless of how well the DMA side is configured.
3. `HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)` — the plain, non-DMA PWM start call. It enables the CC1 output compare and starts the counter. It does **not** touch `hdma[]` or `DIER`'s DMA-enable bits at all, so calling it here is safe and doesn't undo anything from steps 1–2 above.

### Full assembled flow, Steps 1–5 together

```c
// --- GPIO (Step 1) ---
__HAL_RCC_GPIOA_CLK_ENABLE();
/* GPIO_InitStruct for PA6, AF2, as shown in Step 1 */
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// --- TIM3 base + PWM channel config (Step 1) ---
__HAL_RCC_TIM3_CLK_ENABLE();
HAL_TIM_PWM_Init(&htim3);
HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

// --- Duty-cycle table (Step 2) ---
BuildBreathTable();

// --- DMA clock, init, link (Steps 3-4) ---
// hdma_tim3_up must be declared at file scope (Step 3) — not shown here again.
__HAL_RCC_DMA1_CLK_ENABLE();
HAL_DMA_Init(&hdma_tim3_up);
__HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], hdma_tim3_up);

// --- NVIC (Step 6 covers the matching IRQ handler) ---
HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

// --- Start the Update-triggered DMA transfer (Step 5) ---
HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_UPDATE], (uint32_t)breathTable,
                  (uint32_t)&htim3.Instance->CCR1, BREATH_TABLE_SIZE);
__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
```

At this point, the LED on PA6 should begin fading up and down continuously — and the CPU's `while(1)` loop is now **completely free** to do anything else. The conveyor belt is running on its own. One piece is still missing before this is safe to run, though: Step 6.

---

## Step 6 — Implementing the DMA1_Stream2 IRQ Handler

### Why this step exists at all

Step 5's NVIC lines enable `DMA1_Stream2_IRQn` and give it a priority — but enabling an interrupt in the NVIC is a *separate* thing from having code that actually handles it. The interrupt vector table maps `DMA1_Stream2_IRQn` to a function named `DMA1_Stream2_IRQHandler()`. If that function isn't defined anywhere in the project, the linker doesn't complain — the startup file provides a **weak default handler** for every unhandled interrupt vector, and that default handler is typically just an infinite loop.

That means: the moment `HAL_DMA_Start_IT()` (Step 5) causes the first Half-Transfer, Transfer-Complete, or Transfer-Error interrupt to fire on `DMA1_Stream2`, execution jumps into that default infinite loop — and never comes back. Everything on the board freezes: not just the breathing LED, but the heartbeat LED, UART output, ADC reads, all of it. Since this happens shortly *after* the breathing effect starts (not instantly), it's easy to misdiagnose as "something about the breathing pattern itself is wrong," when the actual cause is simply a missing handler function.

### Adding the handler

In `stm32f4xx_it.c`, alongside the other peripheral `_IRQHandler` functions, add:

```c
/* Declared at file scope in main.c — see Step 3's note on handle lifetime */
extern DMA_HandleTypeDef hdma_tim3_up;

/* DMA1 Stream2 handler — TIM3_UP request, drives breathTable -> CCR1 */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim3_up);
}
```

`HAL_DMA_IRQHandler()` is the generic HAL-level DMA interrupt dispatcher: it reads the stream's status flags, clears them, and — via the `Parent` pointer that `__HAL_LINKDMA` set in Step 4 — calls back into whichever timer-level (or other peripheral-level) callback applies. For this circular-mode breathing effect, nothing needs to actually happen in a callback (the table just keeps looping on its own), but the handler still needs to **exist and clear the interrupt flags**, or the same "stuck in the ISR" freeze happens for a different reason: an unacknowledged interrupt re-fires immediately, forever.

### Why `hdma_tim3_up` has to be `extern`-able here

This is exactly why Step 3 insisted `hdma_tim3_up` be declared at file scope in `main.c`, not as a local variable inside `tim3_Init()`. A local variable isn't just at risk of the dangling-pointer bug described in Step 3 — it's also **not visible to `stm32f4xx_it.c` at all**, since `extern` declarations can only reach variables with external linkage. File scope solves both problems with the same fix.

### Quick sanity check for this step

If, after wiring this up, the board still freezes shortly after boot: double-check that `hdma_tim3_up` really is declared at file scope (not `static` *inside* a function) in `main.c`, and that the `extern` declaration in `stm32f4xx_it.c` matches its type exactly. A mismatched or missing `extern` is a compile/link error, not a silent failure — which makes it one of the easier bugs in this exercise to actually catch.

---

## Step 7 — Verifying on Hardware

### What "working" looks like

- The LED on PA6 fades smoothly from off → bright → off, repeating continuously, with no visible stepping or flicker.
- The breathing cycle length should match `BREATH_TABLE_SIZE` and the Update-event rate (see the formula in Step 2) — if it looks too fast or too slow, that's a tuning question (Step 2), not a bug.
- Nothing in the `while(1)` loop needs to run for this to happen — a good sanity check is an empty loop (or one doing something unrelated) and confirming the breathing continues unaffected.

### Quick checks, from easiest to most diagnostic

**1. Multimeter (average voltage) — cheapest, fastest first check**
A DC voltmeter across the LED will read a *slowly changing average voltage* if PWM+DMA is working, roughly tracking the breathing pattern. Won't confirm PWM frequency, but it's a fast "is anything happening at all" check.

**2. Oscilloscope on PA6 — the real diagnostic**
- **Zoomed in** (µs/ms scale): a clean, fast square wave at the PWM carrier frequency, with duty cycle visibly changing pulse-to-pulse.
- **Zoomed out** (whole-second scale, persistence/envelope mode): the classic "PWM envelope" shape, matching the lookup table's sine shape from Step 2.

**3. Debugger — watch the DMA registers directly**
- Watch `hdma_tim3_up.Instance->NDTR` (Number of Data to Transfer) — in Circular mode, this should be continuously counting down and wrapping back to `BREATH_TABLE_SIZE`, on its own, with the CPU halted at a breakpoint elsewhere. Seeing this counter still moving *while the CPU is paused* is the clearest proof DMA — not the CPU — is doing the work.
- Watch `TIM3->CCR1` directly — its value should match whatever `breathTable[i]` was most recently written.

### If it's *not* working — priority order for this exercise

1. **GPIO alternate function wrong** (Step 1) — most common first-attempt mistake. Symptom: LED does *nothing at all*, not even a dim glow.
2. **Clock ordering** (Step 3) — DMA clock enabled after `HAL_DMA_Init()`. Symptom: hard fault, or `HAL_DMA_Init()` returns `HAL_ERROR`.
3. **`__HAL_LINKDMA` field or `TIM_DMA_ID_x` mismatch** (Step 4, Step 5) — Symptom: a hard fault right at the DMA-start call, or PWM stuck at a fixed duty with no fault at all, depending on exactly which mismatch it is.
4. **`hdma_tim3_up` (or `breathTable`) declared as local/non-static** (Step 2, Step 3) — Symptom: configuration appears to succeed, then a fault or garbage behavior shows up later, once the declaring function has returned and its stack memory has been reused.
5. **`DMA1_Stream2_IRQHandler` missing** (Step 6), while the NVIC interrupt is enabled — Symptom: everything runs fine for a moment, then the *entire board* freezes (not just the LED) shortly after the DMA transfer starts.
6. **Data alignment mismatch** (Step 3, `WORD` vs actual array type) — Symptom: LED brightness jumps around unpredictably instead of ramping smoothly.
7. **Table scale factor doesn't match `Period`, or `BREATH_TABLE_SIZE` doesn't match the Update-event rate** (Step 2) — Symptom: LED does breathe, but far too dimly to notice, or far too fast (looks like blinking) or far too slowly (looks stuck).

---

## Step 8 — Common Build Errors for This Exercise

### 1. `undefined reference to HAL_DMA_Init` / `HAL_DMA_Start_IT`

**When it appears:** Link stage, after everything otherwise compiles.

**Cause:** `stm32f4xx_hal_dma.c` isn't in the project's `Src` folder, or `HAL_DMA_MODULE_ENABLED` isn't defined in `stm32f4xx_hal_conf.h`. The same hidden dependency pattern as `stm32f4xx_hal_tim_ex.c`, showing up concretely for the first time in this tutorial series.

**Fix:** Copy `stm32f4xx_hal_dma.c` from the firmware package into `Src/`, and confirm `#define HAL_DMA_MODULE_ENABLED` is uncommented in `stm32f4xx_hal_conf.h`.

### 2. `undefined reference to sinf`

**When it appears:** Link stage, specifically tied to `BuildBreathTable()`.

**Cause:** `sinf()` comes from the math library, and bare-metal ARM builds don't link this in by default. The code compiles cleanly — it only fails at the link step.

**Fix:** In STM32CubeIDE, add `-lm` to the linker flags (Project Properties → C/C++ Build → Settings → MCU GCC Linker → Libraries → add `m`), or precompute the table values as a `const` array of literals ahead of time to avoid the math library dependency entirely.

### 3. `hdma_tim3_up` — multiple definition / redefinition errors

**When it appears:** Link stage, if the DMA handle is declared both inside a `.c` file and mistakenly repeated in a header, or inconsistently marked `static` vs `extern` across files.

**Cause:** Mixing "declare in one file, use in another" without being consistent about scope. Since Step 3/Step 6 now require `hdma_tim3_up` to be file-scope in `main.c` and `extern`-declared in `stm32f4xx_it.c`, make sure it's defined (with an initializer, e.g. `= {0}`) in exactly **one** `.c` file, and only ever `extern`-declared elsewhere.

### 4. LED does nothing, but everything compiles and links cleanly

**Not a build error, but the most common "silent success" trap:** usually means `GPIO_Alternate` was set to the wrong `GPIO_AFx_TIMy` constant. The compiler has no way to know AF2 is correct for TIM3 on PA6 versus, say, AF1 for TIM2 — it happily accepts any valid enum value.

### 5. `implicit declaration of function 'HAL_TIM_PWM_Start_DMA'` (or `HAL_DMA_Start_IT`)

**When it appears:** Compile stage.

**Cause:** DMA-aware TIM/DMA function bodies are only compiled into `stm32f4xx_hal_tim.c` / `stm32f4xx_hal_dma.c` when `HAL_DMA_MODULE_ENABLED` is defined **before** those source files are compiled.

**Fix:** Same root fix as error #1 — confirm `HAL_DMA_MODULE_ENABLED` in `stm32f4xx_hal_conf.h`, then do a clean rebuild (a partial rebuild can leave stale object files that mask a conf.h change).

### 6. Board runs fine for a while, then completely freezes shortly after boot — nothing left responds, not even UART or the heartbeat LED

**When it appears:** Runtime, shortly after the DMA transfer is started (Step 5).

**Cause:** `DMA1_Stream2_IRQn` is enabled in the NVIC (Step 5) but `DMA1_Stream2_IRQHandler()` was never implemented in `stm32f4xx_it.c` (Step 6). The first DMA interrupt falls through to the default weak handler, which loops forever.

**Fix:** Implement `DMA1_Stream2_IRQHandler()` exactly as shown in Step 6, calling `HAL_DMA_IRQHandler(&hdma_tim3_up)`.

### 7. Hard fault (or silently-stuck-at-fixed-duty PWM) right at the DMA start call

**When it appears:** Runtime, at the line that starts the DMA transfer.

**Cause:** Using `HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, ...)` after linking `hdma_tim3_up` at `hdma[TIM_DMA_ID_UPDATE]`. That function reaches into `hdma[TIM_DMA_ID_CC1]` instead, which was never linked — see Step 5 for the full explanation.

**Fix:** Don't call `HAL_TIM_PWM_Start_DMA()` for this configuration. Use the manual `HAL_DMA_Start_IT()` + `__HAL_TIM_ENABLE_DMA()` + `HAL_TIM_PWM_Start()` sequence from Step 5 instead.
