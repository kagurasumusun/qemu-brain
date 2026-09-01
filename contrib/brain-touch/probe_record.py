#!/usr/bin/env python3
"""Dump the WinCE PDD touch sample record around a held press.

Used to see, side by side, what the LRADC model fed the driver (raw counts)
and what the driver stored after calibration, without setting any guest
breakpoint (breakpointing the guest stalls the whole panel state machine).
"""
import os
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from exp_touch import RUN, VMLab, BLK  # noqa: E402

PA = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x42d0f9c0
LEN = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x140

positions = [(0, 0), (799, 479), (400, 240), (799, 0), (0, 479), (760, 240),
             (798, 240)]

lab = VMLab(attach=True, frontend=os.environ.get("FRONTEND", "sdl"))


def record():
    out = lab.qmp.hmp("brain_pdump 0x%x 0x%x probe.bin" % (PA, LEN))
    if "probe.bin" not in out:
        raise RuntimeError(out)
    with open(os.path.join(RUN, "probe.bin"), "rb") as f:
        return f.read()


for (x, y) in positions:
    lab.ptr(x, y, None)
    time.sleep(0.05)
    lab.ptr(x, y, 1)
    time.sleep(0.35)
    blob = record()
    lab.ptr(x, y, 0)
    time.sleep(0.3)
    dw = struct.unpack_from("<%dI" % (len(blob) // 4), blob, 0)
    interesting = []
    for i, v in enumerate(dw):
        lo = v & 0xffff
        hi = (v >> 16) & 0xffff
        for name, val in (("x", lo), ("y", hi)):
            if 100 <= val <= 4095 or 0 <= val <= 820:
                interesting.append((PA + i * 4, name, val))
    print("press at panel (%3d,%3d):" % (x, y))
    print("   dwords: %s" % " ".join("%08x" % v for v in dw[:20]))
    print("   dwords+0x50: %s" % " ".join("%08x" % v for v in dw[20:40]))
    print("   dwords+a0: %s" % " ".join("%08x" % v for v in dw[40:]))
lab.close()
