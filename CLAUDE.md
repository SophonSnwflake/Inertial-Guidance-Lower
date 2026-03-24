# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bare-metal firmware for an STM32F411CEUx (ARM Cortex-M4, BlackPill board) targeting an inertial guidance system. The project is generated and managed by STM32CubeMX and uses the STM32 HAL library. Currently implements a test harness with I2C1, SPI1, USART1, and GPIO (LED on PC13).

## Build Commands

**Prerequisites**: `arm-none-eabi-gcc`, `cmake`, `ninja-build`

```bash
# Configure (Debug)
cmake --preset Debug

# Build
cmake --build --preset Debug

# Configure and build Release
cmake --preset Release && cmake --build --preset Release
```

Build output goes to `build/Debug/` or `build/Release/`. The ELF binary is the flashable artifact.

## Flash to Hardware

```bash
# Flash via ST-Link (default in flash.sh)
./flash.sh
```

The script auto-detects the `.elf` in `build/Debug/`, calls OpenOCD with `interface/stlink.cfg` and `target/stm32f4x.cfg`. Running in WSL requires USB forwarding via `attach_daplink.ps1` on Windows first.

To switch to DAP-Link, edit `flash.sh` and swap the `INTERFACE_CFG` line.

## Architecture

**STM32CubeMX-managed code** lives in `Core/` and `cmake/stm32cubemx/`. Regenerating from `TestforSTM32F411.ioc` will overwrite these files. User application code should go in the `/* USER CODE BEGIN */` / `/* USER CODE END */` sections to survive regeneration.

**Key hardware config** (from `.ioc`):
- System clock: HSE 25MHz → PLL → 100MHz core, 50MHz APB1, 100MHz APB2
- LED: PC13 (active low, toggled every 500ms in main loop)
- I2C1: PB6 (SCL) / PB7 (SDA), 100kHz
- SPI1: PA5 (SCK) / PA6 (MISO) / PA7 (MOSI), Master, Mode 0, 8-bit
- USART1: PA9 (TX) / PA10 (RX), asynchronous

**Memory map** (from linker script `STM32F411XX_FLASH.ld`):
- FLASH: 512KB at `0x08000000`
- RAM: 128KB at `0x20000000`

**Driver layers**:
- `Drivers/STM32F4xx_HAL_Driver/` — ST HAL (do not edit)
- `Drivers/CMSIS/` — ARM CMSIS headers (do not edit)
- `Core/Src/` — CubeMX-generated init code + user application space
- `startup_stm32f411xe.s` — Reset handler and vector table

## Modifying Hardware Configuration

Edit `TestforSTM32F411.ioc` in STM32CubeMX, then regenerate. After regeneration, the CMakeLists files under `cmake/stm32cubemx/` and peripheral init files in `Core/Src/` will be updated. Custom logic outside `USER CODE` blocks will be lost.
