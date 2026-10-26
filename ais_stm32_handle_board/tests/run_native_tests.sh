#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
firmware_dir="$(dirname "$script_dir")"
service_dir="$firmware_dir/../libraries/FirmwareService/src"
test_build=$(mktemp -d)
trap 'rm -rf "$test_build"' EXIT
cxx=${CXX:-g++}
flags=(-std=c++11 -Wall -Wextra -Werror -I"$script_dir/native/stubs" -I"$firmware_dir" -I"$service_dir")

"$cxx" "${flags[@]}" "$script_dir/native/firmware_service_test.cpp" \
    "$service_dir/FirmwareService.cpp" -o "$test_build/firmware_service"
"$cxx" "${flags[@]}" "$script_dir/native/can_transport_test.cpp" \
    "$firmware_dir/can_transport.cpp" -o "$test_build/can_transport"
"$cxx" "${flags[@]}" "$script_dir/native/servo_test.cpp" \
    "$firmware_dir/handle_servo.cpp" -o "$test_build/servo"
for test in firmware_service can_transport servo; do
    "$test_build/$test"
    echo "PASS: $test"
done
