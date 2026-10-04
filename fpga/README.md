# MAX1000 composite video card

The ESP32 draws the dashboard and streams it over SPI. The MAX1000 FPGA holds two
720x240 frame buffers in its SDRAM and generates the NTSC composite signal itself
through a 6-bit resistor DAC. The ESP32 no longer needs 56 KB for the picture or
any CPU time for video. It gets 720x240 (the full width NTSC carries) with 256 colours,
no tearing and no flicker.

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
./quartus.sh flash video_card       # into the internal flash: starts by itself at every power-up
```

The MAX1000's Arrow USB Blaster is an FT2232H (`-c ft2232`, same as the CYC1000).
openFPGALoader can't load MAX 10 `.sof` files ("no enough space to write"), so
`compile` also writes an `.svf` (JTAG script) and `program` plays that.
`flash` makes the design permanent: it converts the `.pof` to an SVF that erases,
writes and verifies the MAX 10's internal configuration flash (~30-60 s). After that
the board starts the design by itself at power-up, with no computer attached.

## Wiring

### DAC: 6-bit R-2R ladder from 270 ohm resistors only (17 resistors)

The DAC is on the MKR header J2, so the board plugs straight into a breadboard
(J1 and J2 pins down; the PMOD, J3 and J4 stay unsoldered). J2 pins 6 and 7
are skipped: they have 4.7k pull-up resistors on the board that would upset the DAC.

| DAC bit | J2 pin | MKR name | FPGA pin |
|---|---|---|---|
| 0 (LSB) | 1 | D6 | L12 |
| 1 | 2 | D7 | J12 |
| 2 | 3 | D8 | J13 |
| 3 | 4 | D9 | K11 |
| 4 | 5 | D10 | K12 |
| 5 (MSB) | 8 | D13 | H13 |
| ground | 11 | GND | |

R = 135 ohm (two 270 in parallel), 2R = 270 ohm. Any single value works as long as all 17 match;
if you change it, set `R_OUT` (half the value) in `tools/ntsc.py` and run `gen_lut.py`.

```
J2 pin:     1         2         3         4         5         8
(DAC bit)   (bit0)    (bit1)    (bit2)    (bit3)    (bit4)    (bit5, MSB)
             |         |         |         |         |         |
            270       270       270       270       270       270      (2R, one per bit)
             |         |         |         |         |         |
GND--270----N0--135---N1--135---N2--135---N3--135---N4--135---N5------> RCA centre
   (2R end)                                                            RCA shell -> GND (J2 pin 11)
        each "135" = two 270 ohm in parallel
```

- The output is N5 (the MSB end). Output resistance is 135 ohm. Into the TV's 75 ohm
  input, that gives 0 to 1.16 V. Sync tip is 0 V, blank 0.29 V, white 1.0 V (standard).
- N0..N5 are just 6 empty breadboard strips (one per junction).
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
| GND | GND | J2 pin 11 | |

All of these are 3.3 V on both boards, so connect them directly. Keep the wires
short (under 10 cm) and run the ground wire alongside them. The link test tells you
the fastest clean speed. If the high speeds fail, a 33-68 ohm resistor in series
with SCK at the ESP32 end usually helps.

## SPI protocol

Every transaction (CS low ... CS high) starts with `[cmd, 0, 0, 0]`.

| Command | Payload | Effect |
|---|---|---|
| `'W'` 0x57 | x, y, w, h (uint16 LE) + w*h RGB332 bytes, row by row | write into the back buffer; anything outside 720x240 is dropped |
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
  a 9-cycle colour burst. One sample per pixel (720 across the 52.6 us
  picture).
- SDRAM: one picture line = one SDRAM row. Line reads use one READ per clock
  (360 words, ~7 us of each 63.5 us line). A 720-byte line spans two SDRAM rows
  (row = {buffer, line, half}), so a fetch reads one row, precharges, and reads the
  second; a pending refresh is allowed in between. Pixel writes are byte-masked and
  batched per row. Refresh runs every 9.8 us (worst-case gap 14.2 us in simulation,
  spec 15.6 us). The SDRAM clock is shifted 6.73 ns,
  and read data is captured on the falling clock edge.
- The SPI receiver runs on SCK itself. Bytes cross into the system clock with a
  toggle handshake, so SCK can be fast (40 MHz) without oversampling.
- Colours never go below code 10 (-16 IRE) so they can't be mistaken for sync, and
  bright colours whose peaks would exceed the DAC range get slightly less saturation
  instead of being clipped (which would shift their hue).

## Verification (`projects/sim/run.sh`)

The whole `video_card` design is simulated with a strict SDRAM model. The model
checks every command, timing, refresh, setup/hold and bus turnaround, and returns
X outside the real data-valid window. Board delays come from four corners: fast,
typical, slow and CL=3. The testbench does the power-up self test, then sends a real
720x240 frame over SPI at 40 MHz in 10 bands (a real dashboard frame plus single-pixel
stripes), with every transaction's
byte count and CRC checked through MISO. It then sends a clipped, odd-aligned partial
rectangle with junk bytes after it. `check_video.py` finds sync in the captured
composite on its own and checks every pulse, the burst phase on every line, and
every one of the 238,420 samples in the frame. It also decodes the picture to PNG
(`sim/out/<corner>/*.png`).

## Change log

- **720x240** (was 360x240): one 14.3 MHz sample per pixel; 2 SDRAM rows per line;
  1024-word line buffer; 27-bit write FIFO entries; test pattern and checker widened.
  Simulation caught a real refresh-gap problem (18.7 us) introduced by the longer line
  fetch; fixed by allowing a refresh between the two row reads and a 9.8 us interval.
- **DAC on MKR J2** (pins 1-5 and 8 = D6-D10, D13) instead of the PMOD, so the board
  sits in a breadboard; J2 pins 6/7 skipped (on-board pull-ups).
- **270 ohm ladder**: levels retuned (`R_OUT = 135`), white back at 100 IRE.
- **`./quartus.sh flash`**: writes the design to the MAX 10's internal flash so it
  starts by itself at power-up (needed in the car). `program` loads RAM only.
- Verified on hardware: SDRAM self test passes; ESP32 link at 40 MHz with no errors;
  ~17 fps at 720x240. Not yet verified: the analog picture on a real screen.
