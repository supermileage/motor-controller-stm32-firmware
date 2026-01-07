Import("env")

env.Append(CPPPATH=[
    "Core/Inc",
    "Drivers/STM32L4xx_HAL_Driver/Inc",
    "Drivers/STM32L4xx_HAL_Driver/Inc/Legacy",
    "Drivers/CMSIS/Device/ST/STM32L4xx/Include",
    "Drivers/CMSIS/Include",
])

# Build generated CubeMX sources into a local build subdir (NOT /something)
env.BuildSources("cubemx_core", "Core/Src")

# Optional: only enable if you want CubeMX startup instead of PlatformIO's
# env.BuildSources("cubemx_startup", "Startup")
