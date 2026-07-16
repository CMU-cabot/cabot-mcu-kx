#!/bin/bash

set -o pipefail

readonly DEFAULT_BOARD="STMicroelectronics:stm32:GenF3:pnum=GENERIC_F303K8TX"
readonly FLASH_BASE=0x08000000
readonly FLASH_SIZE_BYTES=65536
readonly APP_FLASH_OFFSET=0x2000
readonly BOOTLOADER_SIZE_BYTES=$((APP_FLASH_OFFSET))
readonly METADATA_SIZE_BYTES=2048
readonly APP_FLASH_LIMIT_BYTES=$((FLASH_SIZE_BYTES - METADATA_SIZE_BYTES))

function red() {
    echo -en "\033[31m" >&2
    echo "$*" >&2
    echo -en "\033[0m" >&2
}

function err() {
    red "[ERROR] $*"
}

function help() {
    echo "$0 [options] [build|upload|all|bootloader|firmware]"
    echo ""
    echo "build      build the bootloader-aware application (default action)"
    echo "upload     build and upload the application"
    echo "all        build and upload the application"
    echo "bootloader build the 8 KiB CAN bootloader"
    echo "firmware   build the CAN bootloader and relocated application"
    echo ""
    echo "-h         show this help"
    echo "-b <board> set board (default=$DEFAULT_BOARD)"
    echo "-p <port>  set upload port (default=)"
    echo "-d         debug mode (Serial.print)"
    echo "-s         standalone image at 0x08000000 (no bootloader)"
    echo ""
    echo "The default image starts at 0x08002000 and leaves the first $((BOOTLOADER_SIZE_BYTES / 1024)) KiB"
    echo "of STM32F303K8 flash available for a CAN bootloader."
}

board="${ARDUINO_BOARD:-$DEFAULT_BOARD}"
port="${ARDUINO_PORT:-}"
debug=0
standalone=0

while getopts "hb:p:ds" arg; do
    case "$arg" in
        h)
            help
            exit 0
            ;;
        b)
            board="$OPTARG"
            ;;
        p)
            port="$OPTARG"
            ;;
        d)
            debug=1
            ;;
        s)
            standalone=1
            ;;
        *)
            help >&2
            exit 2
            ;;
    esac
done
shift $((OPTIND - 1))

target="${1:-build}"
case "$target" in
    build|upload|all|bootloader|firmware)
        ;;
    *)
        err "Unknown target: $target"
        help >&2
        exit 2
        ;;
esac

flash_offset="$APP_FLASH_OFFSET"
flash_limit="$APP_FLASH_LIMIT_BYTES"
if ((standalone)); then
    flash_offset=0x0
    flash_limit="$FLASH_SIZE_BYTES"
fi

function verify_artifact() {
    local output_dir="$1"
    local sketch_name artifact
    local flash_offset_bytes app_capacity bin_size headroom
    local app_base image_end initial_sp_hex reset_handler_hex
    local initial_sp reset_handler reset_address

    sketch_name="$(basename "$PWD")"
    artifact="${output_dir}/${sketch_name}.ino.bin"
    if [[ ! -f "$artifact" ]]; then
        err "Compiled binary was not found at $artifact"
        return 1
    fi

    flash_offset_bytes=$((flash_offset))
    app_capacity=$((flash_limit - flash_offset_bytes))
    bin_size=$(wc -c < "$artifact")
    headroom=$((app_capacity - bin_size))
    app_base=$((FLASH_BASE + flash_offset_bytes))
    image_end=$((app_base + bin_size))

    if ((headroom < 0)); then
        err "Binary exceeds the application partition by $((-headroom)) bytes"
        return 1
    fi

    read -r initial_sp_hex reset_handler_hex < <(od -An -tx4 -N8 "$artifact")
    initial_sp=$((16#$initial_sp_hex))
    reset_handler=$((16#$reset_handler_hex))
    reset_address=$((reset_handler & ~1))

    if ((initial_sp < 0x20000000 || initial_sp > 0x20003000)); then
        err "Invalid initial stack pointer in application vector: 0x$(printf '%08x' "$initial_sp")"
        return 1
    fi
    if (((reset_handler & 1) == 0 || reset_address < app_base || reset_address >= image_end)); then
        err "Invalid reset handler in application vector: 0x$(printf '%08x' "$reset_handler")"
        return 1
    fi

    echo ""
    echo "Build layout"
    printf '  application address : 0x%08x\n' "$app_base"
    printf '  application capacity: %d bytes\n' "$app_capacity"
    printf '  binary size         : %d bytes\n' "$bin_size"
    printf '  application headroom: %d bytes\n' "$headroom"
    printf '  initial SP          : 0x%08x\n' "$initial_sp"
    printf '  reset handler       : 0x%08x\n' "$reset_handler"
    echo "  binary              : $artifact"
}

function compile() {
    local do_upload="$1"
    local output_dir="build/can-application"
    local -a args

    if ((standalone)); then
        output_dir="build/standalone"
    fi
    if ((debug)); then
        output_dir="${output_dir}-debug"
    fi

    args=(
        compile
        -b "$board"
        --build-property "build.flash_offset=$flash_offset"
        --build-property "upload.maximum_size=$flash_limit"
        --build-property "build.flags.optimize=-Os -flto"
        # FreeRTOS 10.3.2 references this symbol from inline assembly. Keep it
        # explicitly so LTO does not discard it before PendSV_Handler is linked.
        --build-property "compiler.c.elf.extra_flags=-Wl,--undefined=vTaskSwitchContext"
        --output-dir "$output_dir"
    )

    if ((debug)); then
        args+=(--build-property "build.extra_flags=-DDEBUG=1")
    fi
    if ((do_upload)); then
        args+=(--upload)
        if [[ -n "$port" ]]; then
            args+=(-p "$port")
        fi
    fi
    args+=(.)

    echo "building..."
    printf 'arduino-cli'
    printf ' %q' "${args[@]}"
    echo ""

    if ! arduino-cli "${args[@]}"; then
        err "Build failed; check board ($board), flash layout, and port ($port)"
        return 1
    fi

    verify_artifact "$output_dir"
}

case "$target" in
    build)
        compile 0
        ;;
    upload|all)
        # Compile and upload in one command so the STM32 upload recipe receives
        # the same flash offset used by the linker.
        compile 1
        ;;
    bootloader)
        bootloader/build.sh
        ;;
    firmware)
        if ((standalone)); then
            err "-s cannot be combined with the firmware target"
            exit 2
        fi
        compile 0
        bootloader/build.sh
        ;;
esac
