#!/bin/bash
# Simulate the video card end to end and check the composite output.
#   ./run.sh            all corners (in parallel), ~minutes
#   ./run.sh typ        one corner
# Needs: iverilog, python3.
set -e
cd "$(dirname "$0")"
mkdir -p out
python3 ../tools/gen_lut.py > /dev/null
python3 check_video.py inputs

SRC="pll_57m_sim.v sdram_model.v tb_video_card.v ../video_card/video_card.v ../video_card/sdram_ctrl.v
     ../video_card/spi_slave.v ../video_card/cmd_parser.v ../video_card/wfifo.v
     ../common/video_timing.v ../common/ntsc_encoder.v ../common/test_pattern.v"

#        name  T_CO  T_CLK T_IN  T_AC  T_OH  CL QUICK
CORNERS="typ   5000  4000  2000  6000  2500  2  0
fast  2500  1500  800   4500  2000  2  1
slow  7000  8000  3500  6500  2500  2  1
cl3   5000  4000  2000  6000  2500  3  1"

run_corner() {
  read -r name tco tclk tin tac toh cl quick <<< "$1"
  iverilog -g2005 -Wall -Wno-timescale -I ../common -s tb -o "out/$name.vvp" \
    -P tb.T_CO=$tco -P tb.T_CLK=$tclk -P tb.T_IN=$tin -P tb.T_AC=$tac -P tb.T_OH=$toh \
    -P tb.CL=$cl -P tb.QUICK=$quick -P "tb.CORNER=\"$name\"" $SRC 2>&1 | grep -v "inherits timescale\|timescale for the" | grep -v "^$" || true
  mkdir -p "out/$name"
  (cd "out/$name" && vvp -n "../$name.vvp" > "../$name.log" 2>&1)
  local names="bist img1"; [ "$quick" = 0 ] && names="bist img1 img2"
  { echo "== $name"; grep -E "ERROR|PASS|FAIL|\[SDRAM\]|\[TB\] status" "out/$name.log"; python3 check_video.py check "$name" $names; } > "out/$name.result" 2>&1 || true
}

run_bars() {
  iverilog -g2005 -Wall -Wno-timescale -I ../common -s tb -o out/bars.vvp \
    pll_57m_sim.v tb_colorbars.v ../colorbars/colorbars.v ../common/video_timing.v \
    ../common/ntsc_encoder.v ../common/test_pattern.v 2>&1 | grep -v "timescale" | grep -v "^$" || true
  mkdir -p out/bars
  (cd out/bars && vvp -n ../bars.vvp > ../bars.log 2>&1)
  { echo "== colorbars design"; grep -E "ERROR|FAIL" out/bars.log; python3 check_video.py check bars bars; } > out/bars.result 2>&1 || true
}

rm -f out/*.result
pids=()
if [ -z "$1" ] || [ "$1" = bars ]; then run_bars & pids+=($!); fi
while read -r line; do
  name=${line%% *}
  if [ -z "$1" ] || [ "$1" = "$name" ]; then run_corner "$line" & pids+=($!); fi
done <<< "$CORNERS"
for p in "${pids[@]}"; do wait "$p"; done
cat out/*.result
! grep -qE "ERROR|FAIL" out/*.result
