#!/bin/bash

set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
output_dir="${script_dir}/../build/bootloader"
core_root="${STM32_CORE_ROOT:-}"
toolchain_bin="${ARM_TOOLCHAIN_BIN:-}"
cmsis_root="${CMSIS_ROOT:-}"

if [[ -z "$core_root" ]]; then
    core_root="$(find /opt/arduino/packages/STMicroelectronics/hardware/stm32 -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort -V | tail -1)"
fi
if [[ -z "$toolchain_bin" ]]; then
    compiler="$(find /opt/arduino/packages/STMicroelectronics/tools -path '*/bin/arm-none-eabi-gcc' 2>/dev/null | sort -V | tail -1)"
    toolchain_bin="$(dirname "$compiler")"
fi
if [[ -z "$cmsis_root" ]]; then
    cmsis_root="$(find /opt/arduino/packages/STMicroelectronics/tools/CMSIS -path '*/CMSIS/Core/Include' -type d 2>/dev/null | sort -V | tail -1)"
fi
if [[ ! -x "${toolchain_bin}/arm-none-eabi-gcc" || ! -d "$core_root" || ! -d "$cmsis_root" ]]; then
    echo "[ERROR] STM32 Arduino Core and ARM GCC were not found." >&2
    echo "Run this command in the stm32-handle Docker service." >&2
    exit 1
fi

mkdir -p "$output_dir"

common_flags=(
    -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
    -std=gnu11 -Os -flto -ffunction-sections -fdata-sections -fno-builtin
    -Wall -Wextra -Werror
    -DSTM32F303x8
    -I"$script_dir"
    -I"$cmsis_root"
    -I"${core_root}/system/Drivers/CMSIS/Device/ST/STM32F3xx/Include"
)

for source in startup.c main.c; do
    "${toolchain_bin}/arm-none-eabi-gcc" "${common_flags[@]}" -c \
        "${script_dir}/${source}" -o "${output_dir}/${source%.c}.o"
done

elf="${output_dir}/handle_bootloader.elf"
binary="${output_dir}/handle_bootloader.bin"
map="${output_dir}/handle_bootloader.map"

"${toolchain_bin}/arm-none-eabi-gcc" \
    -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Os -flto -nostartfiles -nostdlib \
    -Wl,--gc-sections -Wl,-Map,"$map" -T"${script_dir}/linker.ld" \
    "${output_dir}/startup.o" "${output_dir}/main.o" -lgcc -o "$elf"
"${toolchain_bin}/arm-none-eabi-objcopy" -O binary "$elf" "$binary"

size="$(wc -c < "$binary")"
if ((size > 8192)); then
    echo "[ERROR] Bootloader exceeds 8192 bytes: ${size} bytes" >&2
    exit 1
fi

vector_address="$("${toolchain_bin}/arm-none-eabi-objdump" -h "$elf" | awk '$2 == ".isr_vector" {print $4}')"
if [[ "$vector_address" != "08000000" ]]; then
    echo "[ERROR] Bootloader vector is at 0x${vector_address}, expected 0x08000000" >&2
    exit 1
fi

"${toolchain_bin}/arm-none-eabi-size" "$elf"
echo ""
echo "Bootloader layout"
printf '  address : 0x08000000 - 0x%08x\n' "$((0x08000000 + size - 1))"
printf '  size    : %d bytes\n' "$size"
printf '  headroom: %d bytes\n' "$((8192 - size))"
echo "  binary  : $binary"
