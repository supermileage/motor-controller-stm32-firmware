# motor-controller-stm32-firmware

Firmware for the NUCLEO-L432KC development board and STM32L432KC MCU used to control the 48 V Koford 129H169A brushless DC motor on the Urban Concept supermileage vehicle.

Motor reference: Koford 129 series datasheet, page 12:
<https://www.koford.com/129.pdf>

This branch is the raw-tools workflow: direct CMake, ARM GCC, and OpenOCD without PlatformIO.

## What this controller does

The controller reads three hall sensors from the motor, samples the throttle with ADC + DMA, estimates rotor speed from hall edge timing, and drives the 3-phase Koford 129H169A BLDC motor with 6-step commutation.

The goal in this project is straightforward: reliably power the Urban vehicle's 48 V motor with a simple embedded stack that is easy to inspect, build, and flash without extra framework tooling.

The current firmware target is the NUCLEO-L432KC board, using the STM32L432KC as the control MCU for the BLDC inverter logic around the Koford 129H169A motor.

## Complementary drive

The inverter is driven with complementary PWM outputs from `TIM1`.

- One side of the active phase pair is PWM-driven on the high side.
- The complementary low-side output for the same phase pair is used on the opposite transistor.
- Dead time is configured in the timer so the half-bridge does not cross-conduct during switching.
- Commutation advances from hall sensor state changes, while duty is shaped in a 1 kHz control loop.

This keeps the switching behavior explicit and makes it easier to reason about gate timing, bootstrap margin, and phase-state transitions.

## Why center-aligned PWM

`TIM1` is configured for center-aligned PWM:

This is a common choice in BLDC and inverter control because it usually behaves better electrically than edge-aligned PWM.

- The switching edges are distributed more symmetrically through the PWM period instead of all stacking at one side.
- That usually reduces EMI and switching noise.
- It tends to produce smoother winding current, which helps reduce torque ripple in the BLDC motor.
- It works well with complementary outputs and deadtime, which is exactly how this inverter is driven.
- It also gives cleaner timing symmetry for commutation and any future current-sampling work.

## Commutation update strategy

The commutation logic updates `TIM1` in two steps:

- New compare values and channel-enable states are prepared in `commutate()`.
- A software commutation event is then generated with `TIM1->EGR = TIM_EGR_COMG`.

`TIM1->CR2 |= TIM_CR2_CCPC` enables capture/compare control preload, and `TIM1->CR2 &= ~TIM_CR2_CCUS` makes those updates apply only when software triggers the COM event. That matters because the channel-enable bits for the complementary outputs should switch together at a known instant instead of changing one bit at a time while the bridge is live.

The practical reason for using the commutation event is to avoid transient half-updated phase states. In a BLDC bridge, one output enabling early while another is still in the old state can create shoot-through risk, wrong current paths, or a noisy torque step. Grouping the state change behind a single COM event makes the phase handoff cleaner.

The compare registers are also configured in PWM mode with output compare preload through the HAL timer setup, so duty updates are buffered instead of taking effect mid-cycle. The result is that both duty and phase-state changes are kept aligned to timer-controlled update points instead of being applied as a sequence of unrelated register writes.

## Dead time

`TIM1` is configured with `DeadTime = 40` in the break/dead-time block.

With the timer running from an 80 MHz clock and `CKD = DIV1`, the dead-time tick is 12.5 ns. In the linear range of the STM32 dead-time encoding, a value of `40` gives:

```text
40 * 12.5 ns = 500 ns
```

So the configured dead time is about `500 ns`.

## Startup and standstill behavior

The 1 kHz `TIM6` loop exists partly to keep the motor from getting stuck at standstill.

The behavior is:

- Hall interrupts update `lastHall`, `motor_rpm`, and `hallState` whenever a valid hall transition occurs.
- If no hall edge arrives for `RPM_TIMEOUT_US = 100000`, the 1 kHz loop forces `motor_rpm = 0`.
- The same 1 kHz loop continues shaping throttle into `current_duty`.
- If throttle is requested and `rpm == 0`, the code calls `commutate(hallState, duty)` anyway.

That last step is the standstill helper. At zero speed there are no hall edges yet, so a hall-interrupt-only strategy would never refresh the bridge state and the motor could sit there without producing starting torque. By periodically re-applying the commutation state from the current hall position, the inverter still energizes the correct phase pair and can produce torque from rest until the rotor moves enough to generate the first valid hall transition.

Once the motor is spinning, normal hall-edge-driven commutation takes over again.

## Duty ramping and battery protection

The 1 kHz control loop does not apply throttle changes directly. Instead, it slews the commanded duty in small steps.

There are two separate reasons for this.

When throttle is increased, the firmware ramps duty up gradually instead of stepping straight to the requested value. That reduces current spikes in the motor and inverter, keeps the battery current draw smoother, and makes launches less violent. In practice this is a simple way to avoid large instantaneous torque requests that would otherwise hammer the pack and power stage.

When throttle is reduced, the firmware also ramps duty down instead of forcing a sudden state change. The intent here is to avoid pushing regenerated energy back into the battery in an uncontrolled way. This pack is not meant to absorb aggressive charge current from motor braking, so the controller backs torque out progressively rather than snapping from drive into a condition that can produce a large regenerative event.

This is not a closed-loop battery current limiter, but it is a practical protective measure: smoother torque commands mean smaller current transients both out of and back into the pack.

## Zero-throttle freewheel behavior

Below the ADC deadband, throttle is treated as zero.

At very low or zero commanded duty, `commutate()` disables all phase outputs by clearing the `TIM1->CCER` enable bits.

The effect is that when the rider lets off the throttle, the controller drops the bridge outputs and lets the wheel spin freely instead of actively holding a synchronous rectification path.

That matters because if the low sides were intentionally kept conducting during coast, the spinning motor could behave more like an active generator and feed energy back into the DC bus. For this vehicle, the desired coast behavior is freewheeling, not electrical braking or regen. Disabling the phase outputs reduces drag torque and avoids forcing generated voltage back toward a battery that cannot comfortably absorb it.

## Control architecture

The firmware is organized around a small number of blocks:

- `ADC1 + DMA`: continuously samples the throttle input into a rolling buffer.
- `TIM6`: runs the 1 kHz control loop that filters throttle, applies ramp limiting, and updates duty.
- `EXTI hall interrupts`: capture hall transitions, validate state changes, compute RPM, and trigger commutation.
- `TIM1`: generates center-aligned PWM and complementary outputs for the inverter stage.
- `USART2`: optional debug telemetry output when enabled in the firmware.

## Build

Configure the project:

```sh
cmake --preset Debug
```

Build it:

```sh
cmake --build --preset Debug -j
```

The resulting ELF is:

```sh
build/Debug/bldc_6step_l432kc.elf
```

## Flash with OpenOCD

Flash the built ELF over ST-Link:

```sh
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program build/Debug/bldc_6step_l432kc.elf verify reset exit"
```

If you want an OpenOCD server for a debugger session instead:

```sh
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg
```

## Future improvements

- Add a proper finite state machine for startup, run, fault handling, and shutdown instead of keeping all control behavior in one flow.
- Move hall processing onto a timer-assisted hall-sensor path so commutation timing is hardware-driven and the timer can generate the commutation event directly instead of software writing `TIM_EGR_COMG`.
- Add CAN message support so this firmware can accept torque or throttle commands directly and stop depending on the separate `can-throttleator` repo, which currently drives a DAC that feeds the ADC throttle pin used here.
- Add more sophisticated current protection so phase current, battery current, and fault thresholds are handled explicitly instead of relying mainly on duty shaping and conservative commutation behavior.
- Add real current control on top of the present voltage-duty control approach so torque production is more predictable across battery voltage, speed, and load changes.
- Add clearer fault reporting and telemetry over CAN or UART for undervoltage, overcurrent, hall faults, startup failures, and commutation errors.
