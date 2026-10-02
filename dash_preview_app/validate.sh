#!/bin/bash
# Pre-upload check: memory safety + correctness of every screen (see validate.cpp).
# Usage: ./validate.sh            (add --images to save screenshots in validate_out/)
set -e
cd "$(dirname "$0")"
./run.sh --build-only
L="$HOME/Documents/Arduino/libraries/LovyanGFX/src"
D="$HOME/Documents/Arduino/esp32_dash"
clang++ -std=c++17 -DLGFX_SDL -I"$L" -I/opt/homebrew/include -I"$D" -O2 -w validate.cpp build/*.o \
  -L/opt/homebrew/lib -lSDL2 -o build/validate
./build/validate "$@"
