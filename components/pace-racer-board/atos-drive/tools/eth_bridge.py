#!/usr/bin/env python3
"""Serve the provisioning dashboard and bridge its WebSocket to a board's
ethernet service stream (tcp :5555), so the same page works for a board
inside a wheel, where WebUSB cannot reach.

    tools/eth_bridge.py              # http://localhost:8765/  (dashboard)
    tools/eth_bridge.py --port 9000

Endpoints:
    GET  /                     dashboard.html from this directory
    GET  /discover             JSON list of boards answering "PACE?" (eth_link.discover)
    GET  /builds               JSON list of app images in ../build-*/ (profile, version, built, size)
    GET  /builds/<dir>         that build's app image, so the page flashes it without a file dialog
    WS   /ws?target=<ip>       raw bytes <-> tcp <ip>:5555, one socket per page session

Standard library only: a minimal RFC 6455 server (text/binary frames, masked
client payloads, ping/pong, close). Browsers are the only clients.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eth_link  # noqa: E402

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def ws_frame(opcode, payload=b""):
    hdr = bytes([0x80 | opcode])
    n = len(payload)
    if n < 126:
        hdr += bytes([n])
    elif n < 65536:
        hdr += bytes([126]) + struct.pack(">H", n)
    else:
        hdr += bytes([127]) + struct.pack(">Q", n)
    return hdr + payload


def ws_read(rfile):
    """-> (opcode, payload) or None on EOF. Client frames are always masked."""
    h = rfile.read(2)
    if len(h) < 2:
        return None
    fin_op, ln = h[0], h[1]
    opcode, masked, n = fin_op & 0x0F, ln & 0x80, ln & 0x7F
    if n == 126:
        n = struct.unpack(">H", rfile.read(2))[0]
    elif n == 127:
        n = struct.unpack(">Q", rfile.read(8))[0]
    mask = rfile.read(4) if masked else b""
    data = rfile.read(n)
    if masked:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return opcode, data


APP = os.path.dirname(HERE)  # the atos-drive app dir, where idf.py puts build-*/
APP_BIN = "pace_racer_atos_drive.bin"


def _cstr(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace")


def builds():
    """App images under the app's build-*/ dirs, newest first. Version and
    build time come from the image's esp_app_desc (after the 24-byte image
    header and the first 8-byte segment header)."""
    out = []
    for d in sorted(os.listdir(APP)):
        path = os.path.join(APP, d, APP_BIN)
        if not (d.startswith("build") and os.path.isfile(path)):
            continue
        with open(path, "rb") as fh:
            head = fh.read(256)
        if len(head) < 144 or head[0] != 0xE9 or struct.unpack_from("<I", head, 32)[0] != 0xABCD5432:
            continue
        out.append({"dir": d, "version": _cstr(head[48:80]), "project": _cstr(head[80:112]),
                    "built": f"{_cstr(head[128:144])} {_cstr(head[112:128])}",
                    "mtime": os.path.getmtime(path), "size": os.path.getsize(path)})
    return sorted(out, key=lambda b: -b["mtime"])


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/ws":
            return self.websocket(parse_qs(u.query).get("target", [""])[0])
        if u.path == "/discover":
            body = json.dumps(eth_link.discover()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if u.path == "/builds":
            return self.reply(200, "application/json", json.dumps(builds()).encode())
        if u.path.startswith("/builds/"):
            d = u.path[len("/builds/"):]
            if d not in {b["dir"] for b in builds()}:  # only listed dirs: no path tricks
                return self.send_error(404, "no such build")
            with open(os.path.join(APP, d, APP_BIN), "rb") as fh:
                return self.reply(200, "application/octet-stream", fh.read())
        if u.path in ("/", "/dashboard.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as fh:
                body = fh.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def reply(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def websocket(self, target):
        key = self.headers.get("Sec-WebSocket-Key")
        if self.headers.get("Upgrade", "").lower() != "websocket" or not key or not target:
            self.send_error(400, "websocket upgrade with ?target=<ip> expected")
            return
        tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        eth_link._bind_for(tcp, target)
        tcp.settimeout(3.0)
        try:
            tcp.connect((target, eth_link.SERVICE_PORT))
        except OSError as e:
            self.send_error(502, f"connect {target}:{eth_link.SERVICE_PORT}: {e}")
            return
        tcp.settimeout(None)
        tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        wlock = threading.Lock()

        def board_to_ws():
            try:
                while True:
                    d = tcp.recv(65536)
                    if not d:
                        break
                    with wlock:
                        self.wfile.write(ws_frame(0x2, d))
                        self.wfile.flush()
            except OSError:
                pass
            finally:
                with wlock:
                    try:
                        self.wfile.write(ws_frame(0x8, struct.pack(">H", 1001)))
                        self.wfile.flush()
                    except OSError:
                        pass

        t = threading.Thread(target=board_to_ws, daemon=True)
        t.start()
        try:
            while True:
                fr = ws_read(self.rfile)
                if fr is None:
                    break
                op, data = fr
                if op in (0x1, 0x2):
                    tcp.sendall(data)
                elif op == 0x9:
                    with wlock:
                        self.wfile.write(ws_frame(0xA, data))
                        self.wfile.flush()
                elif op == 0x8:
                    break
        except OSError:
            pass
        finally:
            try:
                tcp.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            tcp.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"dashboard: http://localhost:{a.port}/   (Ctrl-C to stop)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
