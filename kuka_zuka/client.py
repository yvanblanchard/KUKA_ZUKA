"""Thread-safe KUKAVARPROXY TCP client (replaces ZuKa's global-socket ``KUKA`` class)."""
from __future__ import annotations

import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from . import protocol

DEFAULT_PORT = 7000


class KVPError(Exception):
    """Base error for the KVP client."""


class KVPConnectionError(KVPError):
    """Socket-level failure (controller unreachable, KVP not running, port closed...)."""


class KVPVariableError(KVPError):
    """The controller rejected the read/write (unknown, read-only or ill-typed variable)."""


@dataclass
class Transaction:
    t: float
    op: str          # "R" or "W"
    name: str
    value: str       # value sent (W) or received (R)
    reply: str
    ok: bool
    ms: float


@dataclass
class KVPClient:
    host: str
    port: int = DEFAULT_PORT
    timeout: float = 2.0
    auto_reconnect: bool = True
    history_size: int = 500
    _sock: socket.socket | None = field(default=None, init=False, repr=False)
    _lock: threading.RLock = field(default_factory=threading.RLock, init=False, repr=False)
    _msg_id: int = field(default=0, init=False, repr=False)
    history: deque = field(init=False, repr=False)
    last_latency_ms: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.history = deque(maxlen=self.history_size)

    # ------------------------------------------------------------------ connection
    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> "KVPClient":
        with self._lock:
            if self._sock is not None:
                return self
            try:
                sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            except OSError as exc:
                raise KVPConnectionError(
                    f"Cannot reach KUKAVARPROXY at {self.host}:{self.port} ({exc}). "
                    "Check the controller IP, that KUKAVARPROXY.exe is running and that "
                    "port 7000 is open in the KRC network configuration."
                ) from exc
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.timeout)
            self._sock = sock
            return self

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                finally:
                    self._sock = None

    disconnect = close

    def __enter__(self) -> "KVPClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ low level
    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("connection closed by controller")
            buf.extend(chunk)
        return bytes(buf)

    def _transact(self, name: str, value: object | None) -> protocol.Response:
        frame = protocol.encode_request(self._msg_id, name, value)
        with self._lock:
            if self._sock is None:
                if not self.auto_reconnect:
                    raise KVPConnectionError("not connected")
                self.connect()
            msg_id = self._msg_id
            self._msg_id = (self._msg_id + 1) & 0xFFFF
            t0 = time.perf_counter()
            try:
                self._sock.sendall(frame)
                while True:
                    rid, blen = protocol.decode_header(self._recv_exact(protocol.HEADER_SIZE))
                    body = self._recv_exact(blen)
                    if rid == msg_id:
                        break  # discard stale replies (e.g. after a previous timeout)
            except (OSError, ConnectionError) as exc:
                self.close()
                raise KVPConnectionError(f"communication with {self.host}:{self.port} failed: {exc}") from exc
            self.last_latency_ms = (time.perf_counter() - t0) * 1000.0
            return protocol.decode_response_body(rid, body)

    def _log(self, op, name, sent, resp, ms):
        self.history.append(Transaction(time.time(), op, name, sent, resp.value, resp.ok, ms))

    # ------------------------------------------------------------------ public API
    def read(self, name: str) -> str:
        """Read a KRL variable; returns its textual value (e.g. ``'{E6AXIS: A1 0.0, ...}'``)."""
        resp = self._transact(name, None)
        self._log("R", name, "", resp, self.last_latency_ms)
        if not resp.ok:
            raise KVPVariableError(f"read {name!r} rejected by controller (unknown variable?)")
        return resp.value.strip()

    def write(self, name: str, value: object) -> str:
        """Write a KRL variable (value is sent as its ``str()``); returns the controller's echo."""
        if isinstance(value, bool):
            value = "TRUE" if value else "FALSE"
        sval = str(value)
        resp = self._transact(name, sval)
        self._log("W", name, sval, resp, self.last_latency_ms)
        if not resp.ok:
            raise KVPVariableError(f"write {name!r} = {sval!r} rejected by controller "
                                   "(unknown / read-only variable or wrong type)")
        return resp.value.strip()

    def try_read(self, name: str, default: str | None = None) -> str | None:
        """Read, returning ``default`` on variable errors (connection errors still raise)."""
        try:
            return self.read(name)
        except KVPVariableError:
            return default

    def ping(self) -> float:
        """Round-trip a harmless read; returns latency in ms."""
        self.read("$OV_PRO")
        return self.last_latency_ms
