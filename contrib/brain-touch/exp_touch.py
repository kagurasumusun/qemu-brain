#!/usr/bin/env python3
"""Brain touch experiment driver: PC pixel -> QEMU input -> LRADC -> WinCE PDD.

Modes
  paths     which input entry points actually reach the device model
  sweep     taps across the panel, model raw + what the guest computed
  corners   the four corners and the four edge mid-points
  slide     held-drag across the panel in both directions
  guest     find the PDD's live sample by diffing guest RAM around a tap

The guest is booted by this script (or an already running instance is used
with --attach).  A keep-alive key press every few seconds keeps the Brain's
auto power-off from shutting the VM down mid-measurement; the digit row is
ignored by the setup dialog, so it does not perturb the test.
"""

import argparse
import os
import re
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)

from qmp import QMP                     # noqa: E402
from vnclab import VNCLab                # noqa: E402

BLK = 4096
RUN = os.environ.get("RUN", "/home/user/runs/s87")
QEMU = os.environ.get("QEMU", os.path.join(ROOT, "build", "qemu-system-arm"))
EMMC = os.environ.get("EMMC", "/home/user/emmc/emmc.img")
VNCPORT = int(os.environ.get("VNCPORT", "11800"))
DRAM_BASE = int(os.environ.get("DRAM_BASE", "0x40000000"), 0)
DRAM_SIZE = int(os.environ.get("DRAM_SIZE", "0x8000000"), 0)
QLOG = os.path.join(RUN, "qemu.log")

RE_SET = re.compile(r"lradc SETTOUCH x=(\d+) y=(\d+) down=(\d+) "
                    r"CTRL0=0x([0-9a-f]+) CTRL1=0x([0-9a-f]+) vnow=(\d+)")
RE_CONV = re.compile(r"lradc convert: vch(\d) phys(\d) -> 0x([0-9a-f]+) "
                     r"\(touch_down=(\d) x=(\d+) y=(\d+) ctrl4=0x([0-9a-f]+)\) "
                     r"vnow=(\d+)")


class VMLab:
    """QEMU lifecycle + the three observation channels (log, RAM, screen)."""

    def __init__(self, attach=False, boot_timeout=900,
                 panel_ready="DdsiTouchPanelAttach", frontend="vnc"):
        self.proc = None
        self.stop = threading.Event()
        if not attach:
            self.launch(boot_timeout, panel_ready, frontend)
        self.qmp = QMP(os.path.join(RUN, "qmp.sock"), timeout=40)
        self.frontend = frontend
        if frontend == "vnc":
            self.vnc = VNCLab("127.0.0.1", VNCPORT, timeout=40)
            self.width, self.height = self.vnc.width, self.vnc.height
        else:
            # SDL under Xvfb: the plain desktop window a user would click on.
            self.vnc = None
            env = self._xenv()
            self.win = subprocess.run(
                ["xdotool", "search", "--name", "QEMU"], env=env,
                capture_output=True, text=True).stdout.split("\n")[0]
            geom = subprocess.run(
                ["xdotool", "getwindowgeometry", self.win], env=env,
                capture_output=True, text=True).stdout
            m = re.search(r"Geometry: (\d+)x(\d+)", geom)
            self.width, self.height = int(m[1]), int(m[2])
        self.logpos = self._logsize()
        self.kbd_thread = threading.Thread(target=self._keepalive, daemon=True)
        self.kbd_thread.start()

    # -- lifecycle ------------------------------------------------------
    @staticmethod
    def _xenv():
        return dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":99"))

    def launch(self, boot_timeout, ready_marker, frontend="vnc"):
        os.makedirs(RUN, exist_ok=True)
        for f in ("mon.sock", "qmp.sock", "serial.log", "qemu.log"):
            try:
                os.unlink(os.path.join(RUN, f))
            except FileNotFoundError:
                pass
        cmd = [QEMU, "-machine", "brain",
               "-drive", "if=sd,index=0,file=%s,format=raw" % EMMC,
               "-L", os.path.join(ROOT, "pc-bios"), "-k", "en-us",
               "-display",
               "sdl" if frontend == "sdl"
               else "vnc=127.0.0.1:%d" % (VNCPORT - 5900),
               "-monitor", "unix:%s/mon.sock,server=on,wait=off" % RUN,
               "-qmp", "unix:%s/qmp.sock,server=on,wait=off" % RUN,
               "-serial", "file:%s" % os.path.join(RUN, "serial.log"),
               "-snapshot"]
        env = dict(os.environ, BRAIN_TOUCH_DEBUG="1")
        log = open(QLOG, "wb")
        print("[lab] launching qemu")
        self.proc = subprocess.Popen(cmd, cwd=RUN, env=env, stdout=log,
                                     stderr=subprocess.STDOUT)
        serial = os.path.join(RUN, "serial.log")
        t0 = time.time()
        while time.time() - t0 < boot_timeout:
            if self.proc.poll() is not None:
                raise SystemExit("[lab] qemu exited early (code %s)"
                                 % self.proc.returncode)
            try:
                txt = open(serial, errors="replace").read()
            except OSError:
                txt = ""
            if ready_marker and ready_marker in txt:
                print("[lab] panel ready after %.0fs" % (time.time() - t0))
                time.sleep(8)
                return
            time.sleep(2)
        raise SystemExit("[lab] boot timeout")

    def _keepalive(self):
        while not self.stop.wait(12):
            try:
                self.qmp.hmp("sendkey 1")
            except Exception:                    # noqa: BLE001
                return

    def alive(self):
        return self.proc is None or self.proc.poll() is None

    def close(self):
        self.stop.set()
        try:
            if getattr(self, "vnc", None):
                self.vnc.close()
            if getattr(self, "qmp", None):
                self.qmp.close()
        except OSError:
            pass

    # -- observation: model log ----------------------------------------
    def _logsize(self):
        try:
            return os.path.getsize(QLOG)
        except OSError:
            return 0

    def mark(self):
        self.logpos = self._logsize()
        return self.logpos

    def window(self, mark=None):
        off = self.logpos if mark is None else mark
        with open(QLOG, errors="replace") as f:
            f.seek(off)
            text = f.read()
        sets, convs = [], []
        for line in text.splitlines():
            m = RE_SET.search(line)
            if m:
                sets.append({"x": int(m[1]), "y": int(m[2]), "down": int(m[3]),
                             "ctrl1": int(m[5], 16), "vnow": int(m[6])})
                continue
            m = RE_CONV.search(line)
            if m:
                convs.append({"vch": int(m[1]), "phys": int(m[2]),
                              "val": int(m[3], 16), "down": int(m[4]),
                              "vnow": int(m[7])})
        return sets, convs, text

    # -- observation: guest RAM ----------------------------------------
    def dump_ram(self, path):
        """Snapshot all guest RAM (PA space) into *path* via the QEMU aid."""
        rel = os.path.basename(path)
        full = os.path.join(RUN, rel)
        try:
            os.unlink(full)
        except OSError:
            pass
        last = ""
        for _ in range(3):
            last = self.qmp.hmp("brain_pmemsave 0x%x 0x%x %s"
                                % (DRAM_BASE, DRAM_SIZE, rel))
            if "wrote" in last and os.path.exists(full):
                return full
            time.sleep(0.5)
        raise RuntimeError("pmemsave failed: %r" % last.strip())

    # -- actions --------------------------------------------------------
    def ptr(self, x, y, down):
        """Move, and optionally press/release, through the *front end*."""
        if self.frontend == "vnc":
            self.vnc.pointer(x, y, 1 if down else 0)
            return
        subprocess.run(["xdotool", "mousemove", "--window", self.win,
                        str(int(x)), str(int(y))], env=self._xenv(),
                       capture_output=True)
        if down is None:
            return
        # X button events are global; SDL reads the position from the window,
        # which the mousemove above has just set.
        subprocess.run(["xdotool", "mousedown" if down else "mouseup", "1"],
                       env=self._xenv(), capture_output=True)

    def tap(self, x, y, hold=0.2, settle=1.2):
        mark = self.mark()
        self.ptr(x, y, None)
        time.sleep(0.05)
        self.ptr(x, y, 1)
        self.press_wall = time.time()
        time.sleep(hold)
        self.ptr(x, y, 0)
        time.sleep(settle)
        return self.window(mark)

    def slide(self, x0, y0, x1, y1, steps=9, dwell=0.15, settle=1.5):
        mark = self.mark()
        self.ptr(x0, y0, None)
        time.sleep(0.05)
        self.ptr(x0, y0, 1)
        time.sleep(dwell)
        for i in range(1, steps + 1):
            self.ptr(x0 + (x1 - x0) * i // steps,
                     y0 + (y1 - y0) * i // steps, None)
            time.sleep(dwell)
        self.ptr(x1, y1, None)
        self.ptr(x1, y1, 0)
        time.sleep(settle)
        return self.window(mark)

    def qmp_events(self, events):
        return self.qmp.rpc("input-send-event", {"events": events})

    # -- reporting ------------------------------------------------------
    def report(self, label, sets, convs, extra=""):
        plat = [c for c in convs if c["phys"] in (2, 3, 5)]
        raw_x = max([c["val"] for c in plat if c["phys"] == 2], default=None)
        raw_y = max([c["val"] for c in plat if c["phys"] == 5], default=None)
        pos = (sets[0]["x"], sets[0]["y"]) if sets else None
        downs = sum(1 for s in sets if s["down"])
        print("%-20s sets=%-3d down=%d pos=%-12s plate=(%5s,%5s) %s"
              % (label, len(sets), downs, pos, raw_x, raw_y, extra))
        return raw_x, raw_y, pos


def diff_dwords(a, b, limit=40):
    """Return [(offset, old, new)] for changed 32-bit words in two dumps."""
    out = []
    sa = open(a, "rb")
    sb = open(b, "rb")
    CH = 1 << 20
    off = 0
    while True:
        da = sa.read(CH)
        db = sb.read(CH)
        if not da:
            break
        if da == db:
            off += len(da)
            continue
        for i in range(0, len(da) - 3, 4):
            if da[i:i + 4] != db[i:i + 4]:
                out.append((off + i,
                            struct.unpack("<I", da[i:i + 4])[0],
                            struct.unpack("<I", db[i:i + 4])[0]))
                if len(out) >= limit:
                    return out
        off += len(da)
    return out


# ------------------------------------------------------------------------
def mode_paths(lab):
    print("\n== input path matrix: which entry points reach the LRADC? ==")
    w, h = lab.width, lab.height

    m = lab.mark()
    lab.ptr(w // 2, h // 2, None)
    time.sleep(0.1)
    lab.ptr(w // 2, h // 2, 1)
    time.sleep(0.2)
    lab.ptr(w // 2, h // 2, 0)
    time.sleep(1.0)
    sets, _, _ = lab.window(m)
    print("%s pointer tap        -> SETTOUCH updates: %d"
          % (lab.frontend.upper(), len(sets)))

    m = lab.mark()
    r = lab.qmp_events([{"type": "abs", "data": {"axis": "x", "value": 17000}},
                        {"type": "abs", "data": {"axis": "y", "value": 12000}},
                        {"type": "btn", "data": {"button": "left", "down": True}},
                        {"type": "btn", "data": {"button": "left", "down": False}}])
    time.sleep(1.0)
    sets, _, _ = lab.window(m)
    print("QMP input-send-event        -> SETTOUCH updates: %d  (reply: %s)"
          % (len(sets), "return" in r))

    m = lab.mark()
    lab.qmp.hmp("brain_touch 18000 11000 1")
    lab.qmp.hmp("brain_touch 18000 11000 0")
    time.sleep(0.6)
    sets, _, _ = lab.window(m)
    print("HMP brain_touch             -> SETTOUCH updates: %d" % len(sets))
    print("info mice: %s" % lab.qmp.hmp("info mice").strip().replace("\r\n", " | "))


def mode_sweep(lab, n):
    print("\n== horizontal sweep ==")
    w, h = lab.width, lab.height
    print("front-end surface: %dx%d" % (w, h))
    for i in range(n):
        x = (w - 1) * i // (n - 1)
        sets, convs, _ = lab.tap(x, h // 2)
        lab.report("x=%d (frac %.2f)" % (x, x / (w - 1)), sets, convs)


def mode_corners(lab):
    print("\n== corners and edge mid-points ==")
    w, h = lab.width, lab.height
    pts = [("left", 0, h // 2), ("mid", w // 2, h // 2),
           ("right", w - 1, h // 2), ("top", w // 2, 0), ("bottom", w // 2, h - 1),
           ("UL", 0, 0), ("UR", w - 1, 0), ("LL", 0, h - 1), ("LR", w - 1, h - 1)]
    for name, x, y in pts:
        sets, convs, _ = lab.tap(x, y)
        lab.report("%s (%d,%d)" % (name, x, y), sets, convs)


def mode_slide(lab, steps):
    print("\n== slides (%d steps, button held) ==" % steps)
    w, h = lab.width, lab.height
    ym, xm = h // 2, w // 2
    for name, (x0, y0, x1, y1) in {
            "L->R": (0, ym, w - 1, ym), "R->L": (w - 1, ym, 0, ym),
            "U->D": (xm, 0, xm, h - 1), "D->U": (xm, h - 1, xm, 0),
            "diag": (0, 0, w - 1, h - 1)}.items():
        sets, convs, _ = lab.slide(x0, y0, x1, y1, steps=steps)
        plat = [c for c in convs if c["phys"] in (2, 3, 5)]
        xs = [c["val"] for c in plat if c["phys"] == 2]
        ys = [c["val"] for c in plat if c["phys"] == 5]
        posx = [s["x"] for s in sets if s["down"]]
        print("%-6s press+drag updates=%d  axisX=%s" % (name, len(sets), posx))
        print("       plateX=%s" % (xs,))
        print("       plateY=%s" % (ys,))


def mode_guest(lab, points):
    """Ground truth: what did the *guest* store for each tap?

    Diff guest RAM around a tap and list the 32-bit words that changed, so the
    PDD's sample slot can be identified from data rather than from a guessed
    virtual address (the fork's brain_vread VA translation is a documented
    no-op, so an address-based read is not trustworthy here).
    """
    print("\n== guest-side effect of taps (RAM diff) ==")
    w, h = lab.width, lab.height
    for x in points:
        a = lab.dump_ram(os.path.join(RUN, "ram_a.bin"))
        sets, convs, _ = lab.tap(x, h // 2)
        b = lab.dump_ram(os.path.join(RUN, "ram_b.bin"))
        lab.report("tap x=%d" % x, sets, convs)
        ch = diff_dwords(a, b, limit=24)
        print("   changed dwords: %d" % len(ch))
        for off, old, new in ch:
            print("      PA 0x%08x: %10d -> %10d   (0x%x -> 0x%x)"
                  % (DRAM_BASE + off, old, new, old, new))


def track_slots(idle_paths, down_paths, xs, limit=40):
    """Guest RAM dwords that differ only while the finger is down, and whose
    value moves with the finger position.

    Comparing a snapshot taken just before the press with one taken while the
    finger is still held removes everything the guest changes because *time*
    passes: a real delivered-coordinate slot holds a fixed idle value and its
    pressed value tracks x (or y).
    """
    n = os.path.getsize(idle_paths[0])
    fa = [open(p, "rb") for p in idle_paths]
    fb = [open(p, "rb") for p in down_paths]
    hits = []
    for b in range(0, n // BLK):
        o = b * BLK
        for f in fa:
            f.seek(o)
        ablocks = [f.read(BLK) for f in fa]
        for f in fb:
            f.seek(o)
        bblocks = [f.read(BLK) for f in fb]
        pairs = [(ablocks[i], bblocks[i]) for i in range(len(xs))
                 if ablocks[i] != bblocks[i]]
        if len(pairs) != len(xs):
            continue
        for w in range(BLK // 4):
            idle = [struct.unpack_from("<I", a, w * 4)[0] for a, _ in pairs]
            down = [struct.unpack_from("<I", d, w * 4)[0] for _, d in pairs]
            if len(set(idle)) != 1:
                continue                       # not a per-press constant
            if any(v > (1 << 24) for v in down) or any(v > (1 << 24) for v in idle):
                continue
            if max(down) - min(down) < 4:
                continue
            dy = [d - i for d, i in zip(down, idle)]
            inc = all(x < y for x, y in zip(dy, dy[1:]))
            dec = all(x > y for x, y in zip(dy, dy[1:]))
            if not (inc or dec):
                continue
            hits.append({"pa": DRAM_BASE + o + w * 4, "idle": idle[0],
                         "down": down, "slope": inc,
                         "ratio": (down[-1] - down[0]) / float(xs[-1] - xs[0])})
            if len(hits) >= limit:
                break
        if len(hits) >= limit:
            break
    for f in fa + fb:
        f.close()
    return hits


def press_hold(lab, x, y, dump, hold=0.35):
    """Idle snapshot, then press and hold, then a second snapshot."""
    a = lab.dump_ram(dump + "_idle.bin")
    lab.ptr(x, y, None)
    time.sleep(0.05)
    lab.ptr(x, y, 1)
    time.sleep(hold)
    b = lab.dump_ram(dump + "_down.bin")
    lab.ptr(x, y, 0)
    time.sleep(0.4)
    return a, b


def mode_track(lab, axis="x"):
    """Find where WinCE keeps the delivered touch coordinate, and what it is."""
    print("\n== delivered-coordinate probe (%s sweep, press-held) ==" % axis)
    w, h = lab.width, lab.height
    pos = ([0, w // 6, w // 3, w // 2, 2 * w // 3, 5 * w // 6, w - 1]
           if axis == "x" else
           [0, h // 6, h // 3, h // 2, 2 * h // 3, 5 * h // 6, h - 1])
    other = h // 2 if axis == "x" else w // 2
    idle, down = [], []
    for i, v in enumerate(pos):
        x, y = (v, other) if axis == "x" else (other, v)
        a, b = press_hold(lab, x, y, os.path.join(RUN, "trk_%d" % i))
        idle.append(a)
        down.append(b)
    hits = track_slots(idle, down, pos)
    print("   %d candidate words for delivered %s" % (len(hits), axis))
    for t in hits:
        print("      PA 0x%08x idle=0x%x %s slope=%s  down=%s"
              % (t["pa"], t["idle"], axis, "%.3f" % t["ratio"], t["down"]))
    for pth in idle + down:
        try:
            os.unlink(pth)
        except OSError:
            pass
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="sweep",
                    choices=["paths", "sweep", "corners", "slide", "guest",
                             "track", "all", "shots"])
    ap.add_argument("--n", type=int, default=9)
    ap.add_argument("--steps", type=int, default=9)
    ap.add_argument("--axis", default="x", choices=["x", "y"])
    ap.add_argument("--frontend", default="vnc", choices=["vnc", "sdl"])
    ap.add_argument("--attach", action="store_true",
                    help="use an already running qemu instance")
    a = ap.parse_args()

    lab = VMLab(attach=a.attach, frontend=a.frontend)
    try:
        if a.mode in ("paths", "all"):
            mode_paths(lab)
        if a.mode in ("sweep", "all"):
            mode_sweep(lab, a.n)
        if a.mode in ("corners", "all"):
            mode_corners(lab)
        if a.mode in ("slide", "all"):
            mode_slide(lab, a.steps)
        if a.mode == "guest":
            w = lab.width
            mode_guest(lab, [0, w // 4, w // 2, 3 * w // 4, w - 1])
        if a.mode == "track":
            mode_track(lab, axis=a.axis)
        if a.mode == "shots" and lab.vnc:
            for i, x in enumerate(range(0, lab.width, lab.width // 5)):
                lab.tap(x, lab.height // 2)
                lab.vnc.save_ppm(os.path.join(RUN, "shot_x%d.ppm" % x))
    finally:
        print("\n[lab] vm alive: %s" % lab.alive())
        lab.close()


if __name__ == "__main__":
    main()
