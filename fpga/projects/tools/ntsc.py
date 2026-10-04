"""NTSC composite levels and colour math for the MAX1000 video card.

One place for every number the hardware and the checkers share:
  - DAC voltage per code (6-bit R-2R ladder, 150 ohm output, 75 ohm TV input)
  - sync / blank / burst levels
  - the colour lookup table: RGB332 pixel + subcarrier phase -> DAC code
  - the 240p line/frame timing (in samples at 4x the subcarrier, 14.318 MHz)

gen_lut.py writes the LUT and levels for the FPGA; the simulation checkers
import this module, so the hardware and its tests can never disagree.
"""

# ---- DAC -------------------------------------------------------------------
VCC = 3.3            # FPGA I/O voltage
DAC_BITS = 6
R_OUT = 135.0        # R-2R ladder output resistance (R): 270 ohm resistors, R = two in parallel
R_LOAD = 75.0        # TV / head unit composite input
V_PER_CODE = VCC / (1 << DAC_BITS) * R_LOAD / (R_OUT + R_LOAD)   # ~17.2 mV
CODE_MAX = (1 << DAC_BITS) - 1
CODE_MIN = 10        # lowest code a colour may reach: -16 IRE, safely above the
                     # 50% sync slicing level (-20 IRE) so no colour can look like sync

# ---- levels (1 IRE = 1/140 of 1 V; sync tip = -40 IRE = 0 V) ---------------
IRE_V = 1.0 / 140


def ire_to_code(ire):
    return (ire + 40) * IRE_V / V_PER_CODE


SYNC = 0
BLANK = round(ire_to_code(0))            # 17
BLACK_IRE = 7.5                          # NTSC-M setup
WHITE_IRE = 100.0    # standard white; the 270-ohm ladder (135 ohm out) tops out at 122 IRE,
                     # so bright yellow/cyan keep most of their saturation
BURST_AMP = round(20 * IRE_V / V_PER_CODE)   # 20 IRE -> 8 codes
BURST_HI = BLANK + BURST_AMP
BURST_LO = BLANK - BURST_AMP
SATURATION = 1.0     # global chroma gain (lower it if colours look too strong)

# ---- timing (samples at 4 fsc = 14.318 MHz) ---------------------------------
SAMPLES_PER_LINE = 910       # 63.556 us, 227.5 subcarrier cycles
LINES = 262                  # 240p ("non-interlaced"), 59.83 Hz
HALF_LINE = 455
HSYNC_LEN = 67               # 4.7 us
EQ_LEN = 33                  # 2.3 us equalising pulse
VSYNC_HIGH = 67              # serration: 4.7 us high per half line
BURST_START, BURST_LEN = 76, 36      # 5.3 us after sync, 9 cycles
ACTIVE_START, ACTIVE_LEN = 151, 720  # 720 pixels x 1 sample
FIRST_LINE = 22              # first picture line; 240 lines -> 22..261
WIDTH, HEIGHT = 720, 240
EQ_LINES = (0, 1, 2, 6, 7, 8)
VSYNC_LINES = (3, 4, 5)
FIRST_BURST_LINE = 9


def line_kind(v):
    if v in EQ_LINES:
        return "eq"
    if v in VSYNC_LINES:
        return "vsync"
    return "normal"


def is_sync(h, v):
    kind = line_kind(v)
    hh = h - HALF_LINE if h >= HALF_LINE else h
    if kind == "eq":
        return hh < EQ_LEN
    if kind == "vsync":
        return hh < HALF_LINE - VSYNC_HIGH
    return h < HSYNC_LEN


# ---- colour ------------------------------------------------------------------
def rgb332(c):
    """RGB332 byte -> (r, g, b) in 0..1 (gamma-encoded, like the framebuffer)."""
    return ((c >> 5) & 7) / 7.0, ((c >> 2) & 7) / 7.0, (c & 3) / 3.0


def yuv(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, 0.492 * (b - y), 0.877 * (r - y)


def colour_codes(c):
    """The 4 DAC codes for colour c at subcarrier phases 0, 90, 180, 270 deg.

    composite = Y + U sin(wt) + V cos(wt); burst is at 180 deg (-U).
    Phase 0: Y+V, 90: Y+U, 180: Y-V, 270: Y-U.
    A saturated colour whose peaks would go past the DAC range gets its
    chroma scaled down (same hue, a bit less saturation) instead of being
    clipped, which would shift the hue.
    """
    y, u, v = yuv(*rgb332(c))
    span = WHITE_IRE - BLACK_IRE
    yc = ire_to_code(BLACK_IRE + span * y)
    per_unit = span * IRE_V / V_PER_CODE          # codes per unit of U/V
    chroma = [v * per_unit, u * per_unit, -v * per_unit, -u * per_unit]
    k = SATURATION
    for ch in chroma:
        if ch > 1e-9:
            k = min(k, (CODE_MAX - yc) / ch)
        elif ch < -1e-9:
            k = min(k, (CODE_MIN - yc) / ch)
    k = max(k, 0.0)
    out = []
    for ch in chroma:
        code = round(yc + k * ch)
        out.append(max(CODE_MIN, min(CODE_MAX, code)))
    return out


def lut():
    """1024 entries, address = pixel << 2 | phase."""
    table = []
    for c in range(256):
        table.extend(colour_codes(c))
    return table


def decode(s0, s1, s2, s3):
    """4 consecutive samples at phases 0..3 (codes) -> (r, g, b) 0..1, approx."""
    span = WHITE_IRE - BLACK_IRE
    per_unit = span * IRE_V / V_PER_CODE
    yc = (s0 + s1 + s2 + s3) / 4.0
    v = (s0 - s2) / 2.0 / per_unit
    u = (s1 - s3) / 2.0 / per_unit
    y = (yc - ire_to_code(BLACK_IRE)) / per_unit
    r = y + v / 0.877
    b = y + u / 0.492
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return tuple(max(0.0, min(1.0, x)) for x in (r, g, b))


# ---- built-in test pattern (must match common/test_pattern.v) --------------
BAR_COLOURS = [0xFF, 0xFC, 0x1F, 0x1C, 0xE3, 0xE0, 0x03, 0x00]


def pattern(x, y):
    if x == 0 or x == WIDTH - 1 or y == 0 or y == HEIGHT - 1:
        return 0xFF
    bar = min(x // 90, 7)
    if y < 160:
        return BAR_COLOURS[bar]
    if y < 200:
        return (bar << 5) | (bar << 2) | (bar >> 1)     # grey ramp
    return 0xFF if ((x >> 3) ^ (y >> 3)) & 1 else 0x00  # 8x8 checkerboard


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), as the FPGA computes it."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc
