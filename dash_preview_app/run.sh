#!/bin/bash
# Builds (first run takes ~1 min) and opens the live dashboard preview.
#   ./run.sh        240x240 picture (WROOM board)
#   ./run.sh 360    360x240 picture (PSRAM board)
#   ./run.sh --build-only   just build the LovyanGFX objects (used by validate.sh)
set -e
cd "$(dirname "$0")"
L="$HOME/Documents/Arduino/libraries/LovyanGFX/src"
D="$HOME/Documents/Arduino/esp32_dash"
FL="-DLGFX_SDL -I$L -I/opt/homebrew/include -I$D -O2 -w"
mkdir -p build
if [ ! -f build/.lib_done ]; then
  echo "Building LovyanGFX for desktop (one time)..."
  for f in "$L"/lgfx/v1/*.cpp; do clang++ -std=c++17 $FL -c "$f" -o build/$(basename "$f" .cpp).o; done
  for f in "$L"/lgfx/utility/*.c "$L"/lgfx/Fonts/efont/*.c "$L"/lgfx/Fonts/IPA/*.c "$L"/lgfx/v1/lv_font/*.c; do
    [ -f "$f" ] && clang $FL -c "$f" -o build/c_$(basename "$f" .c).o
  done
  touch build/.lib_done
fi
[ "$1" = "--build-only" ] && exit 0
SDL_MAIN="$L/../examples_for_PC/CMake_SDL/sdl_main.cpp"
clang++ -std=c++17 $FL preview.cpp "$SDL_MAIN" build/*.o -L/opt/homebrew/lib -lSDL2 -o build/preview
DASH_W="${1:-240}" ./build/preview
