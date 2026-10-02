# MAX1000 composite video card

The ESP32 draws the dashboard and streams it over SPI. The MAX1000 FPGA holds two
360x240 frame buffers in its SDRAM and generates the NTSC composite signal itself
through a 6-bit resistor DAC. The ESP32 no longer needs 56 KB for the picture or
any CPU time for video. It gets 360x240 with 256 colours, no tearing and no flicker.

```
ESP32 --SPI 20-40 MHz--> parser -> write FIFO -> SDRAM (2 frame buffers)
                                                   | one line ahead
                          TV <- R-2R DAC <- NTSC encoder <- line buffer
```

## Projects (`projects/`)

| Folder | What it is | Needs |
|---|---|---|
| `blink/` | LED light runs back and forth (USER button = faster). Checks the toolchain and programming. | board only |
| `colorbars/` | Colour bars, grey ramp, checkerboard, white border and a bouncing orange box, generated live. Checks the DAC, sync and colour on the TV. | DAC + TV |
| `video_card/` | The real design: SPI slave, SDRAM double buffer, self test, NTSC encoder. | DAC + TV (+ ESP32) |
| `common/` | Shared: PLL, video timing, encoder, test pattern, colour lookup table | |
| `sim/` | End-to-end simulation and checks (`./run.sh`) | iverilog, python3 |
| `tools/` | `ntsc.py` (all levels and colour maths), `gen_lut.py` (writes the LUT) | python3 |

ESP32 side: `~/Documents/Arduino/esp32_fpga_test/` (link test and an animated test scene).
Its `fpga_link.h` is the reusable driver.

## Bring-up order

1. **blink**: `./quartus.sh compile blink`, then program it. The LEDs sweep.
2. **colorbars**: build the DAC, connect the TV, program `colorbars`. You should see
   bars and a moving box. This proves the DAC, the levels and the colour before any
   SDRAM or SPI is involved.
3. **video_card**: program it. The TV shows the same test pattern, this time read
   back through the SDRAM (the power-up self test). LED 3 on and LED 8 off means
   the SDRAM is good.
4. Wire the ESP32 and flash `esp32_fpga_test`. It prints the link test per SPI
   speed, then the TV shows the animated scene.

## Building and programming

```
./quartus.sh build                  # once: Quartus Lite in Docker (needs installers/)
./quartus.sh compile video_card     # -> output_files/video_card.sof and .svf
./quartus.sh program video_card     # into SRAM (lost at power-off), via openFPGALoader
```

The MAX1000's Arrow USB Blaster is an FT2232H (`-c ft2232`, same as the CYC1000).
openFPGALoader can't load MAX 10 `.sof` files ("no enough space to write"), so
`compile` also writes an `.svf` (JTAG script) and `program` plays that.
To keep a design after power-off, write it to the internal flash (`.pof`). We'll
check the exact command once the board is connected.

## Wiring

### DAC: 6-bit R-2R ladder from 300 ohm resistors only (17 resistors)

R = 150 ohm (two 300 in parallel), 2R = 300 ohm.

```
PMOD pin:   PIO1      PIO2      PIO3      PIO4      PIO5      PIO6
(FPGA)      (bit0)    (bit1)    (bit2)    (bit3)    (bit4)    (bit5, MSB)
             |         |         |         |         |         |
            300       300       300       300       300       300      (2R, one per bit)
             |         |         |         |         |         |
GND--300----N0--150---N1--150---N2--150---N3--150---N4--150---N5------> RCA centre
   (2R end)                                                            RCA shell -> GND
        each "150" = two 300 ohm in parallel
```

- The output is N5 (the MSB end). Output resistance is 150 ohm. Into the TV's 75 ohm
  input, that gives 0 to 1.08 V. Sync tip is 0 V, blank 0.29 V, white 0.93 V.
- PMOD: pins 1-4 = PIO1-4 on the top row, pins 7-10 = PIO5-8 on the bottom row,
  pins 5/11 = GND, pins 6/12 = 3.3 V. Use a PMOD GND for the ladder and the RCA shell.
- Keep the ladder close to the PMOD header; the wire to the RCA jack can be long.
- If the picture is too bright or too dim, change `WHITE_IRE`; for colour strength,
  change `SATURATION` (both in `tools/ntsc.py`). Then run `python3 tools/gen_lut.py`
  and recompile. Nothing else changes.

### ESP32 to MAX1000 (MKR header J1, plus ground)

| ESP32 | Signal | MAX1000 | FPGA pin |
|---|---|---|---|
| GPIO18 | SCK | D0 (J1 pin 9) | H8 |
| GPIO23 | MOSI | D1 (J1 pin 10) | K10 |
| GPIO5 | CS | D2 (J1 pin 11) | H5 |
| GPIO19 | MISO | D3 (J1 pin 12) | H4 |
| GPIO4 | READY | D4 (J1 pin 13) | J1 |
| GND | GND | J2 pin 11 (or PMOD GND) | |

All of these are 3.3 V on both boards, so connect them directly. Keep the wires
short (under 10 cm) and run the ground wire alongside them. The link test tells you
the fastest clean speed. If the high speeds fail, a 33-68 ohm resistor in series
with SCK at the ESP32 end usually helps.

## SPI protocol

Every transaction (CS low ... CS high) starts with `[cmd, 0, 0, 0]`.

| Command | Payload | Effect |
|---|---|---|
| `'W'` 0x57 | x, y, w, h (uint16 LE) + w*h RGB332 bytes, row by row | write into the back buffer; anything outside 360x240 is dropped |
| `'S'` 0x53 | | show the back buffer at the next vertical blank; READY goes low until it happens |
| anything else | | ignored (`'Q'` status poll, `'X'` link test) |

During every transaction MISO shifts out the 64-bit status captured at the end of
the previous transaction (MSB first): `A5`, flags, frame count (16 bit), byte count
(16 bit) and CRC-16/CCITT (init FFFF) of that transaction's bytes. Read status at 8
MHz or less.

Flags: bit0 ready, bit1 self test written, bit2 SDRAM data error, bit3 FIFO overflow,
bit4 line fetch late, bit5 front buffer, bit6 self-test check running, bit7 SDRAM
initialised.

Frame flow on the ESP32: wait for READY, then send 10 bands of 24 rows, then `'S'`.
The swap happens at vertical blank, so the TV never shows a half-drawn frame.

## LEDs (video_card)

| LED | Meaning |
|---|---|
| 1 | heartbeat (~1 Hz) |
| 2 | SDRAM initialised |
| 3 | self test running and passing |
| 4 | SPI activity |
| 5 | front buffer (toggles on every swap) |
| 6 | error: write FIFO overflow |
| 7 | error: a line fetch arrived late |
| 8 | error: SDRAM read-back mismatch (timing) |

The USER button clears the error LEDs.

## Design notes

- One clock, 57.2727 MHz = 4 x 14.31818 MHz (12 MHz x 105 / 22). Every video sample
  lasts 4 clocks, exactly 4 samples per colour-subcarrier cycle. All colour maths is
  precomputed in a 1024-entry ROM (`tools/gen_lut.py`): for every RGB332 colour at
  each of the 4 subcarrier phases, it stores the final DAC code.
- 240p NTSC: 910 samples x 262 lines, proper equalising and serrated vsync pulses,
  a 9-cycle colour burst. Pixels are 2 samples wide (360 across the 52.6 us
  picture).
- SDRAM: one picture line = one SDRAM row. Line reads use one READ per clock
  (180 words, ~3.5 us of each 63.5 us line). Pixel writes are byte-masked and
  batched per row. Refresh runs every 12.2 us (worst-case gap under 15.6 us). The SDRAM clock is shifted 6.73 ns,
  and read data is captured on the falling clock edge.
- The SPI receiver runs on SCK itself. Bytes cross into the system clock with a
  toggle handshake, so SCK can be fast (40 MHz) without oversampling.
- Colours never go below code 10 (-16 IRE) so they can't be mistaken for sync, and
  white sits at 90 IRE so bright yellow and cyan keep most of their saturation
  within the 6-bit DAC's range.

## Verification (`projects/sim/run.sh`)

The whole `video_card` design is simulated with a strict SDRAM model. The model
checks every command, timing, refresh, setup/hold and bus turnaround, and returns
X outside the real data-valid window. Board delays come from four corners: fast,
typical, slow and CL=3. The testbench does the power-up self test, then sends a real
360x240 dashboard frame over SPI at 40 MHz in 10 bands, with every transaction's
byte count and CRC checked through MISO. It then sends a clipped, odd-aligned partial
rectangle with junk bytes after it. `check_video.py` finds sync in the captured
composite on its own and checks every pulse, the burst phase on every line, and
every one of the 238,420 samples in the frame. It also decodes the picture to PNG
(`sim/out/<corner>/*.png`).
