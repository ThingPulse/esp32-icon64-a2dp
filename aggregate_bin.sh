#!/usr/bin/env bash

set -e

# Packages the 4 binaries into a single .bin for address 0x0000 as per the list below. Inspired by
# https://github.com/marcelstoer/docker-nodemcu-build/blob/master/build-esp32#L85
#
# 0x1000    bootloader 
# 0x8000    partitions 
# 0xe000    boot_app0
# 0x10000   firmware
# -> https://gitter.im/espressif/arduino-esp32?at=5c9395c0dfc69a1454cf3323 -> "probably boot_app0.bin is a stub for OTA that tells it that app0 is the active partition."
#
# srec_cat doesn't like relative file paths (current version on macOS) -> build absolute ones

home_dir=$(echo ~)
current_dir=$(pwd)
build_dir="$current_dir/.pio/build/esp-wrover-kit"

bootloader="$home_dir/.platformio/packages/framework-arduinoespressif32/tools/sdk/bin/bootloader_dio_40m.bin"
partitions="$build_dir/partitions.bin"
boot_app="$home_dir/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
firmware="$build_dir/firmware.bin"
app="$build_dir/app.bin"

echo "Aggregating binaries."
srec_cat -output "$app" -binary "$bootloader" -binary -offset 0x1000 -fill 0xff 0x0000 0x8000 "$partitions" -binary -offset 0x8000 -fill 0xff 0x8000 0xe000 "$boot_app" -binary -offset 0xe000 -fill 0xff 0xe000 0x10000 "$firmware" -binary -offset 0x10000
echo "Done, created '$app'."
