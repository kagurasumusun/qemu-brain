"""QMP client for the brain machine (HMP commands via human-monitor-command).

QMP is used instead of the raw monitor socket because the mux monitor echoes
every typed character (readline emulation), which makes output parsing messy.
"""

import json
import socket


class QMP:
    def __init__(self, path, timeout=20.0):
        self.rbuf = b""
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.settimeout(timeout)
        self.s.connect(path)
        self.greeting = json.loads(self._line())
        self.s.sendall(json.dumps({"execute": "qmp_capabilities"}).encode() + b"\n")
        self._read_reply()

    def _line(self):
        while b"\n" not in self.rbuf:
            data = self.s.recv(65536)
            if not data:
                raise EOFError("qmp closed")
            self.rbuf += data
        line, _, self.rbuf = self.rbuf.partition(b"\n")
        return line + b"\n"

    def _read_reply(self):
        while True:
            obj = json.loads(self._line())
            if "event" in obj:
                continue
            return obj

    def rpc(self, command, arguments=None):
        return self._read_reply_after({"execute": command,
                                      "arguments": arguments or {}})

    def hmp(self, command):
        reply = self._read_reply_after(
            {"execute": "human-monitor-command",
             "arguments": {"command-line": command}})
        if "error" in reply:
            return "QMP error: %s" % reply["error"]
        return reply.get("return", "")

    def _read_reply_after(self, req):
        self.s.sendall(json.dumps(req).encode() + b"\n")
        return self._read_reply()

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass
