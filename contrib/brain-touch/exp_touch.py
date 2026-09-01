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
        self.quiet = False
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
        # WinCE suspends a few minutes into this boot and a suspended guest
        # answers nothing (measured: one timer tick per 4 s, no reactions).
        # Any delivered input resets the idle timer; a *key* would too, but
        # the boot dialog eats digits, so poke the panel in the empty band
        # above the field row instead -- same thing a resting hand does.
        while not self.stop.wait(8):
            if self.quiet:
                continue
            try:
                self.tap(400, 46, hold=0.02, settle=0.05)
            except Exception:                    # noqa: BLE001
                return

    def alive(self):
        return self.proc is None or self.proc.poll() is None

    def shutdown(self):
        """Ask the guest to power off, then make sure the process is gone.

        A run that leaves a QEMU behind holds the emmc image and the monitor
        sockets, and the guest's own idle-suspend makes the next measurement
        ambiguous, so the harness is responsible for tearing its VM down.
        """
        self.stop.set()
        if self.proc is not None:
            try:
                self.qmp.rpc("system_powerdown")
                self.proc.wait(timeout=25)
            except Exception:                    # noqa: BLE001
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=10)
                except Exception:                # noqa: BLE001
                    self.proc.kill()
        try:
            print("[lab] qemu exit status: %s" % self.proc.returncode)
        except AttributeError:
            pass
        self.proc = None

    def close(self):
        self.stop.set()
        try:
            if getattr(self, "vnc", None):
                self.vnc.close()
        except OSError:
            pass
        try:
            self.qmp.close()
        except (OSError, AttributeError):
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
                              "axis_x": int(m[5]), "axis_y": int(m[6]),
                              "ctrl4": int(m[7], 16), "vnow": int(m[8])})
        return sets, convs, text

    # -- observation: guest RAM ----------------------------------------
    # -- observation: guest framebuffer --------------------------------
    def shot(self, name="shot.ppm"):
        """Grab the console surface (QEMU's own dump: no VNC encoding, so no
        dependence on the front end being able to keep up)."""
        full = os.path.join(RUN, name)
        try:
            os.unlink(full)
        except OSError:
            pass
        self.qmp.hmp("screendump %s" % name)
        for _ in range(80):
            if os.path.exists(full) and time.time() - os.path.getmtime(full) < 5:
                break
            time.sleep(0.05)
        with open(full, "rb") as f:
            d = f.read()
        parts = d.split(b"\n")
        w, h = (int(v) for v in parts[1].split())
        off = len(parts[0]) + len(parts[1]) + len(parts[2]) + 3
        return w, h, d[off:off + w * h * 3]

    @staticmethod
    def markers(buf, w, h, y0=100, y1=330):
        """Centres of the dialog's filled selection boxes, from the pixels.

        The 【日付と時刻の設定】 panel marks the field the touch stack picked
        with a solid box, so this reads the delivered coordinate back out of
        the guest's own drawing instead of trusting any side channel.
        """
        hh = y1 - y0
        mask = bytearray(w * hh)
        for y in range(y0, y1):
            ro = y * w * 3
            mo = (y - y0) * w
            for x in range(w):
                o = ro + x * 3
                if buf[o] < 100 and buf[o + 1] < 100 and buf[o + 2] < 100:
                    mask[mo + x] = 1
        seen = bytearray(w * hh)
        out = []
        for y in range(hh):
            for x in range(w):
                i = y * w + x
                if not mask[i] or seen[i]:
                    continue
                st, seen[i] = [i], 1
                pts = []
                while st:
                    a = st.pop()
                    pts.append(a)
                    ax, ay = a % w, a // w
                    for nx, ny in ((ax + 1, ay), (ax - 1, ay), (ax, ay + 1),
                                   (ax, ay - 1)):
                        if 0 <= nx < w and 0 <= ny < hh:
                            b = ny * w + nx
                            if mask[b] and not seen[b]:
                                seen[b] = 1
                                st.append(b)
                xs = [p % w for p in pts]
                ys = [p // w + y0 for p in pts]
                wd, ht = max(xs) - min(xs) + 1, max(ys) - min(ys) + 1
                if len(pts) / float(wd * ht) > 0.65 and 25 <= wd <= 200 \
                        and 28 <= ht <= 60:
                    out.append((sum(xs) // len(xs), sum(ys) // len(ys), wd))
        return sorted(out)

    @staticmethod
    def frame_diff(a, b, w, h, blk=16, thresh=24):
        """Where did the guest actually redraw?  Returns (n, bbox)."""
        pts = []
        for y in range(0, h - 1, blk):
            ro = y * w * 3
            for x in range(0, w - 1, blk):
                o = ro + x * 3
                d = (abs(a[o] - b[o]) + abs(a[o + 1] - b[o + 1])
                     + abs(a[o + 2] - b[o + 2]))
                if d > thresh:
                    pts.append((x, y))
        if not pts:
            return 0, None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return len(pts), (min(xs), max(xs), min(ys), max(ys))

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
        # Under this guest's CTRL4 (identity) the X wiper is read on CH3 and
        # the Y wiper on CH2/CH5; a channel's role follows the drive plate, so
        # label by what the driver actually triggers, not by the channel index.
        plat = [c for c in convs if c["phys"] in (2, 3, 5) and c["down"]]
        raw_x = max([c["val"] for c in plat if c["phys"] == 3]
                    or [c["val"] for c in plat], default=None)
        raw_y = max([c["val"] for c in plat if c["phys"] in (2, 5)],
                    default=None)
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
        # what the panel law says a finger here produces, against everything the
        # converter actually reported during the burst (a channel's role follows
        # the drive plate, so the readings are compared as a set)
        want = (160 + 3552 * x / float(w), 3964 - 3753 * y / float(h))
        got = sorted({c["val"] for c in convs
                      if c["phys"] in (2, 3, 5) and c["down"]})
        ok = all(any(abs(g - v) <= 2 for g in got) for v in want)
        print("%-8s (%3d,%3d) axis=%-12s want=(%5.0f,%5.0f) saw=%s %s"
              % (name, x, y, (sets[0]["x"], sets[0]["y"]) if sets else None,
                 want[0], want[1], got, "OK" if ok else "CHECK"))


def mode_slide(lab, steps):
    """Continuous travel across the panel: does every step arrive, and is the
    distance 1:1 with the finger?"""
    print("\n== slides (%d steps, button held) ==" % steps)
    w, h = lab.width, lab.height
    ym, xm = h // 2, w // 2
    for name, (x0, y0, x1, y1) in {
            "L->R": (0, ym, w - 1, ym), "R->L": (w - 1, ym, 0, ym),
            "U->D": (xm, 0, xm, h - 1), "D->U": (xm, h - 1, xm, 0),
            "diag": (0, 0, w - 1, h - 1)}.items():
        sets, convs, _ = lab.slide(x0, y0, x1, y1, steps=steps)
        down = [s for s in sets if s["down"]]
        # the driver reads three samples per burst, so collapse repeats: one
        # value per conversion time stamp, then look at what moved
        per = {}
        for c in convs:
            if c["down"] and c["phys"] in (2, 3, 5):
                per.setdefault(c["vnow"], {})[c["phys"]] = c["val"]
        series = sorted(per.items())
        px = [v.get(3) for _, v in series if v.get(3) is not None]
        py = [v.get(2, v.get(5)) for _, v in series
              if v.get(2, v.get(5)) is not None]
        px = [v for v in px if v is not None]
        py = [v for v in py if v is not None]
        def summ(vals, a, b, counts_per_px):
            """What the guest latched over the whole drag.

            uniq counts the *distinct* plate readings: one per drag step means
            nothing froze mid-way, and span/expected is the distance ratio the
            finger would see.  Ordering is not asserted per channel: the driver
            re-arms the drive plates while it reads, so a channel carries the
            other axis in part of the sequence.
            """
            uniq = sorted(set(vals))
            if len(uniq) < 2:
                return "no movement (%d samples)" % len(vals)
            span = uniq[-1] - uniq[0]
            want = abs(b - a) * counts_per_px
            return ("%d distinct readings %s..%s, span %d of %.0f expected = "
                    "%.3f px/px" % (len(uniq), uniq[0], uniq[-1], span, want,
                                    span / want if want else 0))
        print("  %-5s updates while held=%d  X: %s"
              % (name, len(down), summ(px, x0, x1, 3552 / float(w))))
        print("        %-21s Y: %s" % ("", summ(py, y0, y1, 3753 / float(h))))


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


# Targets read off the live frame (see markers(): the dialog marks the field
# the touch stack selected with a solid box, which is what we compare with the
# pixel the front end was clicked at).
TARGETS = [
    ("year field      ", 252, 130),
    ("month field     ", 456, 130),
    ("day field       ", 596, 128),
    ("hour field      ", 342, 203),
    ("minute field    ", 430, 203),
    ("PM half         ", 540, 282),
    ("far right, empty", 799, 128),
    ("left dead margin", 40, 128),
]


def mode_ui(lab):
    """Where did the guest's own UI put the selection for each front-end tap?"""
    print("\n== delivered position, read from the guest's drawing ==")
    lab.quiet = True                      # no keepalive taps inside the burst
    w, h = lab.width, lab.height
    _, _, buf = lab.shot("ui_base.ppm")
    print("   baseline markers: %s" % (lab.markers(buf, w, h),))
    prev = buf
    for i, (name, x, y) in enumerate(TARGETS):
        m = lab.mark()
        sets, convs, _ = lab.tap(x, y, hold=0.25, settle=0.6)
        _, _, cur = lab.shot("ui_%02d.ppm" % i)
        n, box = lab.frame_diff(prev, cur, w, h)
        plat = [c for c in convs if c["phys"] in (2, 3, 5) and c["down"]]
        raw_x = max([c["val"] for c in plat if c["phys"] == 3]
                    or [c["val"] for c in plat], default=0)
        raw_y = max([c["val"] for c in plat if c["phys"] in (2, 5)], default=0)
        print("  %s tap(%3d,%3d) plate=(%4d,%4d) redrawn=%3d  markers %s -> %s"
              % (name, x, y, raw_x, raw_y, n,
                 " ".join("(%d,%d)" % (b[0], b[1]) for b in lab.markers(prev, w, h)),
                 " ".join("(%d,%d)" % (b[0], b[1]) for b in lab.markers(cur, w, h))))
        prev = cur
    for i in range(len(TARGETS)):
        try:
            os.unlink(os.path.join(RUN, "ui_%02d.ppm" % i))
        except OSError:
            pass
    lab.quiet = False


def _scan(path, off, needle):
    """Byte offset of `needle` in path beyond off, or None."""
    with open(path, errors="replace") as f:
        f.seek(off)
        txt = f.read()
    return needle in txt, txt


def _roi(buf, w, y0=100, y1=170, x0=300, x1=700):
    """Bytes of one band of a frame.  Comparing the whole frame is useless as
    an "did the input land" test (the battery indicator repaints on its own),
    and comparing the field row is exactly the thing the tap must change."""
    return b"".join(buf[(y * w + x0) * 3:(y * w + x1) * 3]
                    for y in range(y0, y1))


def mode_latency(lab, at=(5, 45, 105)):
    """Where does the time go between the button and a sample the guest holds?

    Two wall-clock segments, both read off the model's own log growth:
      press -> model:   the front end, QEMU's input layer and the device
      model -> plate:   the guest's read (its DELAY kick arms the conversion)
    Measured at a few points after boot, because the complaint is specifically
    about input feeling late during the boot/first-use window.
    """
    import statistics
    print("\n== wall-clock input latency ==")
    w, h = lab.width, lab.height
    t0 = time.time()
    for target in at:
        while time.time() - t0 < target:
            time.sleep(0.5)
        seg_a, seg_b = [], []
        for k in range(5):
            x = 200 + 60 * k
            lab.ptr(x, h // 2, None)
            time.sleep(0.05)
            off = os.path.getsize(QLOG)
            t_press = time.time()
            lab.ptr(x, h // 2, 1)
            seen_model = seen_conv = None
            while time.time() - t_press < 6.0:
                got, txt = _scan(QLOG, off, "SETTOUCH")
                if got and seen_model is None:
                    seen_model = time.time()
                if "touch_down=1" in txt:
                    seen_conv = time.time()
                    break
                time.sleep(0.004)
            lab.ptr(x, h // 2, 0)
            time.sleep(0.35)
            if seen_model:
                seg_a.append((seen_model - t_press) * 1000)
            if seen_conv and seen_model:
                seg_b.append((seen_conv - seen_model) * 1000)
        def fmt(v):
            if not v:
                return "n/a"
            v = sorted(v)
            return ("median %.1f ms (min %.1f, max %.1f)"
                    % (statistics.median(v), v[0], v[-1]))
        print("  t+%3ds  front-end->model: %-34s  model->plate conversion: %s"
              % (target, fmt(seg_a), fmt(seg_b)))

    # and the leg the user actually feels: press -> the guest has repainted
    print("  press -> the guest's own drawing changed (field-row band).")
    print("  NOTE this leg is sampled by capturing the frame, so it is an "
          "upper bound; the capture cost is printed below and subtracted.")
    t0 = time.time()
    for _ in range(5):
        lab.shot("lat_cal.ppm")
    cap = (time.time() - t0) / 5 * 1000
    print("     one frame capture: %.1f ms" % cap)
    w, h = lab.width, lab.height
    lat = []
    for k in range(6):
        # alternate between two fields so *every* tap has to move the selection
        x = 456 if k % 2 == 0 else 596
        y = 130
        m = lab.mark()
        _, _, prev = lab.shot("lat_prev.ppm")
        prev = _roi(prev, w)
        t0 = time.time()
        lab.ptr(x, y, None)
        time.sleep(0.05)
        lab.ptr(x, y, 1)
        seen_model = seen_paint = None
        while time.time() - t0 < 15.0:
            if seen_model is None:
                got, _t = _scan(QLOG, m, "SETTOUCH")
                if got:
                    seen_model = time.time() - t0
            _, _, cur = lab.shot("lat_cur.ppm")
            if _roi(cur, w) != prev:
                seen_paint = time.time() - t0
                break
            time.sleep(0.01)
        lab.ptr(x, y, 0)
        time.sleep(0.5)
        if seen_model is not None and seen_paint is not None:
            lat.append((x, seen_model * 1000, seen_paint * 1000))
            print("     tap (%3d,%d): input %6.1f ms   painted %7.1f ms "
                  "(%7.1f ms less one capture)"
                  % (x, y, seen_model * 1000, seen_paint * 1000,
                     max(0.0, seen_paint * 1000 - cap)))
    if lat:
        v = sorted(x[2] - x[1] for x in lat)
        print("     input->painted, after the first: median %.0f ms (min %.0f, "
              "max %.0f); the first tap of this burst took %+.0f ms more"
              % (statistics.median(v[1:]) if len(v) > 1 else v[0], v[0], v[-1],
                 (lat[0][2] - lat[0][1]) - statistics.median(v[1:])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="sweep",
                    choices=["paths", "sweep", "corners", "slide", "guest",
                             "track", "ui", "latency", "all", "shots"])
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
        if a.mode in ("ui", "all"):
            mode_ui(lab)
        if a.mode == "latency":
            mode_latency(lab)
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
        if not a.attach:
            lab.shutdown()
            print("[lab] vm shut down: %s" % (not lab.alive(),))


if __name__ == "__main__":
    main()
