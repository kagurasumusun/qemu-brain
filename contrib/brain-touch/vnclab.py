"""Tiny RFB (VNC) client used as the *real* front end for Brain touch tests.

Why VNC and not the brain_touch HMP helper: brain_touch pokes the device model
directly, while a VNC pointer event travels the same code a user's mouse does
(ui/vnc.c pointer_event -> qemu_input_queue_abs -> the console surface -> the
LRADC input handler).  Anything that works for one and not the other is a bug
in the input path, so the tests must be driven from the front end.

The server is pumped from a background thread: QEMU disconnects a client whose
output buffer fills up, which would otherwise kill the connection between
actions.
"""

import socket
import time
import struct
import threading


class VNCLab:
    def __init__(self, host="127.0.0.1", port=5900, timeout=20.0, pump=True):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
        self.fb = None
        self.fb_w = self.fb_h = 0
        self.pending = b""
        self.lock = threading.Lock()
        self.dead = None
        self._handshake()
        if pump:
            self.thread = threading.Thread(target=self._pump, daemon=True)
            self.thread.start()

    # -- plumbing -------------------------------------------------------
    def _recv(self, n):
        while len(self.pending) < n:
            data = self.s.recv(65536)
            if not data:
                raise EOFError("vnc closed")
            self.pending += data
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

    def _handshake(self):
        self.s.recv(12)                      # "RFB 003.008\n"
        self.s.sendall(b"RFB 003.008\n")
        n = self._recv(1)[0]
        types = self._recv(n)
        if 1 not in types:
            raise RuntimeError("no 'None' auth offered: %r" % types)
        # QEMU arms protocol_client_auth with a *one byte* read and compares
        # data[0] against the advertised type, so the choice goes as one byte.
        self.s.sendall(b"\x01")
        (ok,) = struct.unpack("!I", self._recv(4))
        if ok:
            raise RuntimeError("security handshake failed")
        self.s.sendall(b"\x01")              # ClientInit: shared=1
        hdr = self._recv(24)
        self.width, self.height = struct.unpack("!HH", hdr[:4])
        pf = hdr[4:20]
        (bpp, depth, be, truecol, rmax, gmax, bmax, rsh, gsh, bsh) = (
            pf[0], pf[1],
            struct.unpack("!H", pf[2:4])[0], struct.unpack("!H", pf[4:6])[0],
            struct.unpack("!H", pf[6:8])[0], struct.unpack("!H", pf[8:10])[0],
            struct.unpack("!H", pf[10:12])[0], pf[12], pf[13], pf[14])
        self.bpp = bpp // 8
        self.be, self.truecol = be, truecol
        self.rmax, self.gmax, self.bmax = rmax, gmax, bmax
        self.rsh, self.gsh, self.bsh = rsh, gsh, bsh
        namelen = struct.unpack("!I", hdr[20:24])[0]
        self.name = self._recv(namelen).decode(errors="replace")
        self.s.sendall(struct.pack("!BHHHH", 3, 0, 0, self.width, self.height))

    def _decode_pixel(self, buf, off):
        if self.bpp == 4:
            v, = struct.unpack_from(">I" if self.be else "<I", buf, off)
        elif self.bpp == 2:
            v, = struct.unpack_from(">H" if self.be else "<H", buf, off)
        else:
            v = buf[off]

        def chan(mask, shift):
            return int(((v >> shift) & mask) * (255.0 / (mask or 1)) + 0.5)
        return (chan(self.rmax, self.rsh), chan(self.gmax, self.gsh),
                chan(self.bmax, self.bsh))

    def _wait_update(self, timeout=None):
        old = self.s.gettimeout()
        if timeout is not None:
            self.s.settimeout(timeout)
        try:
            typ, nrects = struct.unpack("!BxH", self._recv(4))
            if typ != 0:
                raise RuntimeError("unexpected server message %d" % typ)
            for _ in range(nrects):
                x, y, w, h, enc = struct.unpack("!HHHHi", self._recv(12))
                if enc == 0:                       # raw
                    data = self._recv(w * h * self.bpp)
                    with self.lock:
                        if (self.fb_w, self.fb_h) != (self.width, self.height):
                            self.fb = bytearray(self.width * self.height * 3)
                            self.fb_w, self.fb_h = self.width, self.height
                        for row in range(h):
                            for col in range(w):
                                rgb = self._decode_pixel(
                                    data, (row * w + col) * self.bpp)
                                dst = ((y + row) * self.width + x + col) * 3
                                self.fb[dst:dst + 3] = bytes(rgb)
                elif enc == -223:                  # desktop size
                    self.width, self.height = struct.unpack("!HH", self._recv(4))
                    with self.lock:
                        self.fb = None
                        self.fb_w = self.fb_h = 0
                else:
                    raise RuntimeError("unsupported encoding %d" % enc)
            self.s.sendall(struct.pack("!BHHHH", 3, 0, 0, self.width, self.height))
        finally:
            self.s.settimeout(old)

    def _pump(self):
        while True:
            try:
                self._wait_update(timeout=None)
            except Exception as exc:                # noqa: BLE001
                self.dead = exc
                return

    # -- actions --------------------------------------------------------
    def check(self):
        if self.dead is not None:
            raise RuntimeError("vnc client died: %r" % (self.dead,))

    def pointer(self, x, y, buttons=0):
        self.check()
        self.s.sendall(struct.pack("!BBhh", 5, buttons, int(x), int(y)))

    def move(self, x, y):
        self.pointer(x, y, 0)

    def save_ppm(self, path):
        fb = None
        for _ in range(50):
            with self.lock:
                if self.fb:
                    fb = bytes(self.fb)
                    break
            time.sleep(0.1)
        if not fb:
            raise RuntimeError("no framebuffer yet")
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (self.width, self.height))
            f.write(fb)
        return path

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass
