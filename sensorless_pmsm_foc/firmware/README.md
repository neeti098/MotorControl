# sensorless_pmsm_foc

Firmware project for target `STM32G431RB`, organized as a layered architecture:
application logic is kept separate from vendor-specific code so it can be
retargeted with less rework.

## Layers

Dependencies run one way — downward in this list. A layer never includes a
header from a layer above it.

- `app/` — orchestration only (`app_start()` in `app.c`). Startup sequence and
top-level policy; includes service headers only.
- `services/` — one subsystem per service (wifi, sensor, telemetry …), vendor-neutral.
- `modules/` — reusable pure logic, no I/O.
- `drivers/` — portable device drivers. Standard C only; hardware access goes
through a caller-supplied `*_io_t` function-pointer struct.
- `interfaces/` — vendor-neutral contracts (`*_if.h`) that `platforms/` implements.
- `platforms/` — **the only layer allowed to call a vendor SDK.** STM32 HAL/LL/CMSIS code lives in `platforms/stm32/` and nowhere else.
- `bsp/` — board facts: which pin, which active level, which bus. Declarative;
performs no I/O.
- `osal/` — RTOS abstraction contract (tasks, mutexes, delays). Declarations only.
- `configs/` — compile-time config. `app_config.h` holds VALUES (pins, periods,
credentials — the Firmware Topology editor reads and writes these);
`feature_config.h` holds which optional subsystems are built at all.
- `utils/` — logging and helpers.
- `scripts/`, `docs/` — supporting material (expand as the project grows).

Host-side unit tests are NOT written or run here, and never under `firmware/`.
They run at the end, from Artifacts -> Unit Tests. Only the vendor-free layers
can be compiled there — `modules/`, `services/`, `utils/` and `app/` — so where
you put a calculation decides whether it can ever be tested off the board.

Not every project needs every layer. A single-peripheral app is fine with
`app/` + `platforms/`; add services and interfaces when there is more than one
subsystem to keep apart.

The STM32 reset entry `main()` lives in the Cube pocket (`Core/Src/main.c` or `application/main.c`) and only calls `app_start()` after HAL/clock init. Cube folders are ST glue — edit the `firmware/` tree, not Cube sources, for features.
