#!/bin/bash
# Memory-safety check of the ESP32 FPGA test scene (see validate_fpga.cpp).
set -e
cd "$(dirname "$0")"
./run.sh --build-only
mkdir -p validate_out
L="$HOME/Documents/Arduino/libraries/LovyanGFX/src"
clang++ -std=c++17 -DLGFX_SDL -I"$L" -I/opt/homebrew/include -I../esp32_fpga_test -O2 -w validate_fpga.cpp build/*.o \
  -L/opt/homebrew/lib -lSDL2 -o build/validate_fpga
./build/validate_fpga
