# Trapezoidal BLDC Commutation (Nucleo L432KC)

6‑step (trapezoidal) BLDC commutation on an STM32 Nucleo‑L432KC using hall sensor interrupts for commutation timing and ADC interrupts for throttle/duty updates.

## Overview
- **Commutation:** 6‑step table driven by hall state (A/B/C) with high‑side PWM and low‑side GPIO switching.
- **PWM:** TIM1 channels 2/3/4 (period 4095, ~12‑bit duty).
- **Hall sensors:** PA1/PA3/PA4 on EXTI (rising/falling). Updates commutation on every edge.
- **Throttle:** ADC1 channel 10 (PA5). ISR updates duty cycle.
- **Debug:** UART2 @ 115200 with optional prints (see `DEBUG` macros).
- **Low‑power:** main loop sleeps with `__WFI()` between interrupts.

## Commutation Table
Derived from `commutate()` in `Core/Src/main.c`:

| Hall (CBA) | High‑side PWM | Low‑side ON |
|-----------:|---------------|-------------|
| 101 | C_HI (TIM1_CH3) | B_LO |
| 100 | A_HI (TIM1_CH4) | B_LO |
| 110 | A_HI (TIM1_CH4) | C_LO |
| 010 | B_HI (TIM1_CH2) | C_LO |
| 011 | B_HI (TIM1_CH2) | A_LO |
| 001 | C_HI (TIM1_CH3) | A_LO |
| other | all off | all off |

On every hall change, `allOff()` is called before updating the commutation state.

## Pin Map (from `Core/Inc/main.h`)
- **Hall sensors:**
  - HALL_C: PA1 (EXTI1)
  - HALL_B: PA3 (EXTI3)
  - HALL_A: PA4 (EXTI4)
- **Throttle ADC:** PA5 (ADC1 channel 10)
- **High‑side PWM (TIM1):**
  - MOSFET_B_HI: PB0 (TIM1_CH2)
  - MOSFET_C_HI: PB1 (TIM1_CH3)
  - MOSFET_A_HI: PA11 (TIM1_CH4)
- **Low‑side GPIO:**
  - MOSFET_A_LO: PB5
  - MOSFET_B_LO: PB6
  - MOSFET_C_LO: PA8
- **UART2:** PA2 (TX), PA15 (RX)

## Build & Flash (PlatformIO)
This project is set up for PlatformIO with STM32Cube HAL.

```sh
# build
pio run

# upload
pio run -t upload

# serial monitor
pio device monitor -b 115200
```

## Notes
- `COMMUTATION_DEADTIME_US` is a software commutation dead‑time parameter (defined in `Core/Src/main.c`) that can be tuned.
- ADC conversions are started in the commutation loop; `HAL_ADC_ConvCpltCallback()` updates the duty and triggers a commutation refresh.
- `HAL_SuspendTick()` is used to reduce wakeups; the main loop relies on interrupts.

## Files
- `Core/Src/main.c`: commutation logic, ISRs, and peripheral init.
- `Core/Inc/main.h`: pin map and inline GPIO/ADC helpers.
- `trapezoidal-commutation-l432kc.ioc`: CubeMX configuration.
- `platformio.ini`: build config for Nucleo‑L432KC.
