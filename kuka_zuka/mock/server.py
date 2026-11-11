"""Fake KUKAVARPROXY TCP server backed by :class:`MockKRC`.

Run standalone:  python -m kuka_zuka.mock.server [--port 7001] [--no-drives]
"""
from __future__ import annotations

import argparse
import socket
import socketserver
import threading

from .. import protocol
from .krc import MockKRC


class _Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        krc: MockKRC = self.server.krc  # type: ignore[attr-defined]
        sock: socket.socket = self.request
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        buf = b""
        while True:
            try:
                chunk = sock.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while len(buf) >= protocol.HEADER_SIZE:
                msg_id, blen = protocol.decode_header(buf[:protocol.HEADER_SIZE])
                if len(buf) < protocol.HEADER_SIZE + blen:
                    break
                body = buf[protocol.HEADER_SIZE:protocol.HEADER_SIZE + blen]
                buf = buf[protocol.HEADER_SIZE + blen:]
                try:
                    mode, name, value = protocol.decode_request_body(body)
                    res = krc.read(name) if mode == protocol.MODE_READ else krc.write(name, value or "")
                except protocol.ProtocolError:
                    mode, res = 0, None
                reply = protocol.encode_response(msg_id, mode, res or "", ok=res is not None)
                try:
                    sock.sendall(reply)
                except OSError:
                    return


class _Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


class MockKVPServer:
    """Threaded mock server; ``with MockKVPServer() as srv: ...`` or start()/stop()."""

    def __init__(self, host: str = "127.0.0.1", port: int = 7001, krc: MockKRC | None = None):
        self.krc = krc or MockKRC()
        self._srv = _Server((host, port), _Handler)
        self._srv.krc = self.krc  # type: ignore[attr-defined]
        self.host, self.port = self._srv.server_address[:2]
        self._thread: threading.Thread | None = None

    def start(self) -> "MockKVPServer":
        self._thread = threading.Thread(target=self._srv.serve_forever, name="mock-kvp", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._srv.shutdown()
        self._srv.server_close()
        self.krc.stop()

    def __enter__(self) -> "MockKVPServer":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def main() -> None:
    ap = argparse.ArgumentParser(description="Mock KUKAVARPROXY + simulated KRLDRIVER")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=7001)
    ap.add_argument("--no-drives", action="store_true", help="mimic a KRC4 without manipulator")
    a = ap.parse_args()
    srv = MockKVPServer(a.host, a.port, MockKRC(drives_available=not a.no_drives)).start()
    print(f"Mock KRC4 listening on {srv.host}:{srv.port} (drives={'off' if a.no_drives else 'on'}). Ctrl+C to stop.")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
