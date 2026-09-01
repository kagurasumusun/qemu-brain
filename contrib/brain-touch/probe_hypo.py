#!/usr/bin/env python3
"""Which coordinate does WinCE hand on after calibration?

Hold a press at a known front-end pixel, read the plate counts the PDD
latched, then snapshot guest RAM and look for the dword pair (x, y) each
hypothesis predicts.  bytes.find over the whole RAM image is a memcmp in C, so
this is cheap, and it needs no guest breakpoints (which stall the panel state
machine and were measured to be useless here).
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exp_touch import DRAM_BASE, RUN, VMLab  # noqa: E402

PX, PY = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (700, 380)
W, H = 800, 480
# where the PDD latches the plate counts (located by exp_touch --mode track)
REC = int(os.environ.get("REC_PA", "0x42d0fa14"), 0)

lab = VMLab(attach=True, frontend=os.environ.get("FRONTEND", "sdl"))


def read_rec():
    out = lab.qmp.hmp("brain_pdump 0x%x 8 probe_rec.bin" % REC)
    if "probe_rec.bin" not in out:
        raise RuntimeError(out)
    with open(os.path.join(RUN, "probe_rec.bin"), "rb") as f:
        return struct.unpack("<II", f.read(8))


lab.ptr(PX, PY, None)
time.sleep(0.05)
lab.ptr(PX, PY, 1)
time.sleep(0.4)
raw_x, raw_y = read_rec()
snap = lab.dump_ram(os.path.join(RUN, "hypto.bin"))
lab.ptr(PX, PY, 0)
time.sleep(0.3)
lab.close()
blob = open(snap, "rb").read()
os.unlink(snap)

hypo = {
    # clicked pixel, i.e. calibration is a perfect round trip
    "identity-800x480": (PX, PY),
    "identity-320x240": (PX * 320 // W, PY * 240 // H),
    # driver BootArgs coefficients {633,2811,-36} / {320,-2502,507}
    "bootargs-raw": (633 * raw_x // 2811 - 36, 507 - 320 * raw_y // 2502),
    # whole 12-bit range scaled onto the panel
    "twelvebit-800x480": (raw_x * 799 // 4095, raw_y * 479 // 4095),
    "twelvebit-320x240": (raw_x * 319 // 4095, raw_y * 239 // 4095),
    # the raw plate counts themselves (what the PDD record holds)
    "raw-plate": (raw_x, raw_y),
}
print("\npress held at front-end pixel (%d,%d); PDD latched raw (%d,%d)"
      % (PX, PY, raw_x, raw_y))
for name, (x, y) in hypo.items():
    print("   hypothesis %-18s -> (%d,%d)" % (name, x, y))
print()
for name, (x, y) in hypo.items():
    for fmt, tag in (("<ii", "int32"), ("<hh", "int16")):
        pat = struct.pack(fmt, x, y)
        offs, p = [], 0
        while True:
            i = blob.find(pat, p)
            if i < 0 or len(offs) >= 8:
                break
            offs.append(i)
            p = i + 1
        if offs or tag == "int32":
            print("  %-18s %s pair %5d,%5d : %d hit(s) %s"
                  % (name, tag, x, y, len(offs),
                     " ".join("0x%08x" % (DRAM_BASE + o) for o in offs)))
