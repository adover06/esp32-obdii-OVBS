#!/usr/bin/env python3
"""Checks composite captures from tb_video_card.v like a very strict TV.

  check_video.py inputs                 write out/img1.hex and out/rect.hex
  check_video.py check <corner> [...]   check out/<corner>/{bist,img1,img2}.txt

For each capture it:
  1. finds sync on its own (threshold between sync tip and blank), and checks
     every pulse: 6 equalising, 6 broad (serrated vsync), 6 equalising,
     then one hsync per line, 910 samples per line, 262 lines per frame,
     and no stray pulses anywhere
  2. reads the colour burst on every line, derives the subcarrier phase, and
     checks it advances 180 degrees per line (227.5 cycles)
  3. compares EVERY sample of the frame with the value it must have
     (sync, blank, burst, or the colour lookup for the expected pixel)
  4. decodes the picture back to RGB with plain NTSC maths (not the LUT)
     and saves a PNG, and reports how close the decoded colours are
"""
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, "..", "tools"))
import ntsc  # noqa: E402

OUT = os.path.join(here, "out")
W, H = ntsc.WIDTH, ntsc.HEIGHT
SPL, LINES = ntsc.SAMPLES_PER_LINE, ntsc.LINES
LUT = ntsc.lut()
DASH_PPM = os.path.expanduser("~/Documents/Arduino/dash_preview_app/validate_out")


def read_ppm(path):
    with open(path, "rb") as f:
        data = f.read()
    end = data.index(b"\n") + 1          # header is "P6 W H 255\n" (written by validate.cpp)
    _, w, h, _ = data[:end].split()
    w, h = int(w), int(h)
    px = data[end:]
    return w, h, [(px[i] & 0xE0) | ((px[i + 1] >> 3) & 0x1C) | (px[i + 2] >> 6)
                  for i in range(0, w * h * 3, 3)]


def write_hex(path, values):
    with open(path, "w") as f:
        for v in values:
            f.write("%02x\n" % v)


def make_inputs():
    os.makedirs(OUT, exist_ok=True)
    # a real 360-wide dashboard frame, each pixel doubled to 720 wide, plus a
    # column of single-pixel stripes so 1-pixel detail is exercised too
    w, h, img360 = read_ppm(os.path.join(DASH_PPM, "w360_1_DUAL_1850.ppm"))
    img1 = [img360[y * 360 + x // 2] for y in range(H) for x in range(W)]
    for y in range(H):
        for x in range(700, 716):
            img1[y * W + x] = 0xFF if x % 2 else 0xE0
    w, h, diag = read_ppm(os.path.join(DASH_PPM, "w360_3_DIAG_1110.ppm"))
    rect = [diag[(y + 60) * 360 + x + 40] for y in range(40) for x in range(100)]
    write_hex(os.path.join(OUT, "img1.hex"), img1)
    write_hex(os.path.join(OUT, "rect.hex"), rect)
    print("wrote img1.hex (%d px, %d colours) and rect.hex" % (len(img1), len(set(img1))))


def expected_images():
    with open(os.path.join(OUT, "img1.hex")) as f:
        img1 = [int(x, 16) for x in f.read().split()]
    with open(os.path.join(OUT, "rect.hex")) as f:
        rect = [int(x, 16) for x in f.read().split()]
    bist = [ntsc.pattern(x, y) for y in range(H) for x in range(W)]
    img2 = list(bist)   # back buffer for the 2nd swap is buffer 0 = self test
    for y in range(211, 240):
        for x in range(661, 720):
            img2[y * W + x] = rect[(y - 211) * 100 + (x - 661)]
    bars = list(bist)   # colorbars design: pattern + orange 48x24 box (black rim) at 40,20
    for y in range(20, 44):
        for x in range(40, 88):
            rim = x < 44 or x >= 84 or y < 22 or y >= 42
            bars[y * W + x] = 0x00 if rim else 0xF8
    return {"bist": bist, "img1": img1, "img2": img2, "bars": bars}


def sync_pulses(s, thr):
    pulses, start = [], None
    for i, c in enumerate(s):
        if c < thr and start is None:
            start = i
        elif c >= thr and start is not None:
            pulses.append((start, i - start))
            start = None
    return pulses


def check_capture(path, image, png_path):
    s = [int(x) if x.isdigit() else -1 for x in open(path).read().split()]   # X -> -1 (always wrong)
    problems = []
    thr = (ntsc.SYNC + ntsc.BLANK) / 2
    pulses = sync_pulses(s, thr)

    # 1. find a vsync: first broad pulse after a non-broad one
    broad = [i for i, (st, ln) in enumerate(pulses) if ln > 200]
    first = next((i for i in broad if i > 0 and pulses[i - 1][1] <= 200), None)
    if first is None:
        return ["no vertical sync found"], None
    line0 = pulses[first][0] - 3 * SPL
    if line0 < 0 or line0 + LINES * SPL + 40 > len(s):
        return ["capture does not hold a whole frame"], None

    want = []
    for k in range(6):
        want.append((line0 + k * 455, ntsc.EQ_LEN))
    for k in range(6):
        want.append((line0 + 3 * SPL + k * 455, 455 - ntsc.VSYNC_HIGH))
    for k in range(6):
        want.append((line0 + 6 * SPL + k * 455, ntsc.EQ_LEN))
    for v in range(9, LINES):
        want.append((line0 + v * SPL, ntsc.HSYNC_LEN))
    want.append((line0 + LINES * SPL, ntsc.EQ_LEN))   # next frame starts on time
    got = [p for p in pulses if line0 - 10 <= p[0] <= line0 + LINES * SPL + 10]
    if got != want:
        extra = sorted(set(got) - set(want))[:5]
        missing = sorted(set(want) - set(got))[:5]
        problems.append("sync pulses wrong: unexpected %s, missing %s" % (extra, missing))

    # 2. burst phase per line
    p0 = {}
    for v in range(ntsc.FIRST_BURST_LINE, LINES):
        base = line0 + v * SPL
        lo = [h for h in range(ntsc.BURST_START, ntsc.BURST_START + ntsc.BURST_LEN)
              if s[base + h] == ntsc.BURST_LO]
        if len(lo) != ntsc.BURST_LEN // 4:
            problems.append("line %d: burst has %d low peaks" % (v, len(lo)))
            continue
        p0[v] = (1 - lo[0]) % 4
        if v - 1 in p0 and p0[v] != (p0[v - 1] + 2) % 4:
            problems.append("line %d: subcarrier phase did not flip 180 deg" % v)

    # 3. every sample
    bad = {}
    first_bad = []
    for v in range(LINES):
        base = line0 + v * SPL
        for h in range(SPL):
            got_c = s[base + h]
            if ntsc.is_sync(h, v):
                exp, kind = ntsc.SYNC, "sync"
            elif v >= ntsc.FIRST_BURST_LINE and ntsc.BURST_START <= h < ntsc.BURST_START + ntsc.BURST_LEN:
                if v not in p0:
                    continue
                ph = (p0[v] + h) % 4
                exp = ntsc.BURST_LO if ph == 1 else ntsc.BURST_HI if ph == 3 else ntsc.BLANK
                kind = "burst"
            elif v >= ntsc.FIRST_LINE and ntsc.ACTIVE_START <= h < ntsc.ACTIVE_START + ntsc.ACTIVE_LEN:
                if v not in p0:
                    continue
                x, y = h - ntsc.ACTIVE_START, v - ntsc.FIRST_LINE
                exp = LUT[image[y * W + x] * 4 + (p0[v] + h) % 4]
                kind = "picture"
            else:
                exp, kind = ntsc.BLANK, "blank"
            if got_c != exp:
                bad[kind] = bad.get(kind, 0) + 1
                if len(first_bad) < 5:
                    first_bad.append("v%d h%d: got %d want %d (%s)" % (v, h, got_c, exp, kind))
    if bad:
        problems.append("wrong samples %s, e.g. %s" % (bad, "; ".join(first_bad)))

    # 4. decode with NTSC maths and compare with the source colours
    decoded, errs = [], []
    for y in range(H):
        v = y + ntsc.FIRST_LINE
        base = line0 + v * SPL
        for x in range(W):
            h0 = min(max(ntsc.ACTIVE_START + x - 1, ntsc.ACTIVE_START), ntsc.ACTIVE_START + W - 4)
            win = {}
            for h in range(h0, h0 + 4):
                win[(p0.get(v, 0) + h) % 4] = s[base + h]
            rgb = ntsc.decode(win[0], win[1], win[2], win[3])
            decoded.append(rgb)
            if 0 < x < W - 2 and len(set(image[y * W + x - 1: y * W + x + 3])) == 1:
                ref = ntsc.rgb332(image[y * W + x])
                errs.append(max(abs(a - b) for a, b in zip(rgb, ref)))
    errs.sort()
    stats = (errs[len(errs) // 2], errs[int(len(errs) * 0.99)], errs[-1]) if errs else (0, 0, 0)

    ppm = png_path[:-4] + ".ppm"
    with open(ppm, "wb") as f:
        f.write(b"P6 %d %d 255\n" % (W, H))
        f.write(bytes(int(c * 255 + 0.5) for rgb in decoded for c in rgb))
    subprocess.run(["sips", "-s", "format", "png", ppm, "--out", png_path],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    os.remove(ppm)
    return problems, stats


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "inputs":
        make_inputs()
        return 0
    if len(sys.argv) < 3 or sys.argv[1] != "check":
        print(__doc__)
        return 2
    corner = sys.argv[2]
    names = sys.argv[3:] or ["bist", "img1", "img2"]
    images = expected_images()
    failed = False
    for name in names:
        path = os.path.join(OUT, corner, name + ".txt")
        if not os.path.exists(path):
            print("  %-5s missing capture" % name)
            failed = True
            continue
        problems, stats = check_capture(path, images[name], os.path.join(OUT, corner, name + ".png"))
        if problems:
            failed = True
            print("  %-5s FAIL" % name)
            for p in problems:
                print("        " + p)
        else:
            print("  %-5s OK  timing, burst and all %d samples exact; decoded colour error "
                  "median %.3f, 99%% %.3f, max %.3f" % (name, SPL * LINES, *stats))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
