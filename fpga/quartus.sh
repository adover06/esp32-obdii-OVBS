#!/bin/bash
# Quartus in a container (compile only; program the board from macOS with openFPGALoader).
#   ./quartus.sh build                 build the image (once, ~20-40 min)
#   ./quartus.sh shell                 bash inside the container, this folder mounted at /work
#   ./quartus.sh compile <project dir> full compile of projects/<dir> (+ .svf for programming)
#   ./quartus.sh program <project dir> load it into the FPGA (SRAM, lost at power-off)
#   ./quartus.sh flash <project dir>   write it to the FPGA's internal flash (permanent: loads at every power-up)
set -e
cd "$(dirname "$0")"
IMG=quartus-lite:23.1
case "$1" in
  build)
    sha() { shasum -a 1 "installers/$1" | cut -d' ' -f1; }
    [ "$(sha QuartusLiteSetup-23.1std.1.993-linux.run)" = bbca0986c79ca4b367838fca31b061ed87bfe50e ] || { echo "installer missing or corrupt"; exit 1; }
    [ "$(sha max10-23.1std.1.993.qdz)" = 158ff328b61b17181056aa9309e619147e217fb3 ] || { echo "max10 .qdz missing or corrupt"; exit 1; }
    docker build --platform linux/amd64 -t $IMG . ;;
  shell)
    docker run --rm -it --platform linux/amd64 -v "$PWD":/work $IMG bash ;;
  compile)
    docker run --rm --platform linux/amd64 -v "$PWD":/work -w "/work/projects/$2" $IMG \
      bash -c "quartus_sh --flow compile '$2' && cd output_files && quartus_cpf -c -q 6MHz -g 3.3 -n p '$2.sof' '$2.svf'" ;;
  flash)
    # .pof -> SVF that erases, programs and verifies the MAX 10's internal configuration flash
    docker run --rm --platform linux/amd64 -v "$PWD":/work -w "/work/projects/$2/output_files" $IMG \
      quartus_cpf -c -q 6MHz -g 3.3 -n p "$2.pof" "$2_flash.svf"
    openFPGALoader -c ft2232 "projects/$2/output_files/$2_flash.svf" ;;
  program)
    # openFPGALoader can't load MAX 10 .sof files directly; it plays the .svf
    openFPGALoader -c ft2232 "projects/$2/output_files/$2.svf" ;;
  *) echo "usage: $0 build | shell | compile <project> | program <project> | flash <project>"; exit 1 ;;
esac
