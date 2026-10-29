#!/usr/bin/env bash
# Portable STM32F407 firmware build from checked-in source (no Windows IDE paths).
# Requires arm-none-eabi-gcc, binutils, and newlib/nano libraries.
set -euo pipefail

cd "$(dirname "$0")/.."
OUT_DIR="${1:-build/stm32}"
mkdir -p "$OUT_DIR/obj"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

flags=(
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  -std=gnu11 -Os -g -Wall -Wextra
  -DUSE_HAL_DRIVER -DSTM32F407xx
  -ffunction-sections -fdata-sections
  -Istm32/Core/Inc
  -Istm32/Drivers/STM32F4xx_HAL_Driver/Inc
  -Istm32/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  -Istm32/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  -Istm32/Drivers/CMSIS/Include
)

objects=()
mapfile -t c_sources < <(find stm32/Core/Src stm32/Drivers/STM32F4xx_HAL_Driver/Src -maxdepth 1 -name '*.c' -type f | sort)
for source in "${c_sources[@]}"; do
  object="$OUT_DIR/obj/$(basename "${source%.c}").o"
  arm-none-eabi-gcc "${flags[@]}" -c "$source" -o "$object"
  objects+=("$object")
done

startup="$OUT_DIR/obj/startup_stm32f407vgtx.o"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -c stm32/Core/Startup/startup_stm32f407vgtx.s -o "$startup"
objects+=("$startup")

arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -T stm32/STM32F407VGTX_FLASH.ld \
  -Wl,--gc-sections -Wl,-Map="$OUT_DIR/stm32.map" \
  --specs=nano.specs --specs=nosys.specs \
  -o "$OUT_DIR/stm32.elf" "${objects[@]}" -lc -lm

arm-none-eabi-size "$OUT_DIR/stm32.elf"
echo "Built: $OUT_DIR/stm32.elf"
