"""KUKAVARPROXY wire protocol (pure functions, no I/O).

Ported from ZuKa ``kukavarproxy.py`` (Python 2) and made framing-safe.

Request  : msgID u16 | bodyLen u16 | mode u8 | nameLen u16 | name | [valLen u16 | value]
Response : msgID u16 | bodyLen u16 | mode u8 | valLen u16  | value | 3 status bytes
All integers big-endian, strings ASCII. The last status byte is 0x01 on success
(same convention as OpenShowVar / py_openshowvar).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

MODE_READ = 0
MODE_WRITE = 1
HEADER = struct.Struct(">HH")  # msg id, body length
HEADER_SIZE = HEADER.size
MAX_FIELD = 0xFFFF


class ProtocolError(ValueError):
    """Malformed frame or value that cannot be encoded."""


def _ascii(text: str, what: str) -> bytes:
    try:
        data = text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolError(f"{what} must be ASCII: {text!r}") from exc
    if len(data) > MAX_FIELD:
        raise ProtocolError(f"{what} too long ({len(data)} bytes)")
    return data


def encode_request(msg_id: int, name: str, value: object | None = None) -> bytes:
    """Build a read (``value is None``) or write request frame."""
    if not name:
        raise ProtocolError("variable name is empty")
    name_b = _ascii(name, "variable name")
    if value is None:
        body = struct.pack(">BH", MODE_READ, len(name_b)) + name_b
    else:
        val_b = _ascii(str(value), "variable value")
        if not val_b:
            raise ProtocolError("write value is empty")
        body = (struct.pack(">BH", MODE_WRITE, len(name_b)) + name_b
                + struct.pack(">H", len(val_b)) + val_b)
    if len(body) > MAX_FIELD:
        raise ProtocolError("message too long")
    return HEADER.pack(msg_id & 0xFFFF, len(body)) + body


def decode_header(data: bytes) -> tuple[int, int]:
    """Return ``(msg_id, body_len)`` from the 4-byte header."""
    if len(data) != HEADER_SIZE:
        raise ProtocolError("header must be 4 bytes")
    return HEADER.unpack(data)


@dataclass(frozen=True)
class Response:
    msg_id: int
    mode: int
    value: str
    ok: bool
    raw: bytes = b""


def decode_response_body(msg_id: int, body: bytes) -> Response:
    """Decode the response body (everything after the 4-byte header)."""
    if len(body) < 3:
        raise ProtocolError(f"response body too short ({len(body)} bytes)")
    mode = body[0]
    (val_len,) = struct.unpack(">H", body[1:3])
    if 3 + val_len > len(body):
        raise ProtocolError("value length exceeds body length")
    value = body[3:3 + val_len].decode("ascii", errors="replace")
    tail = body[3 + val_len:]
    ok = tail.endswith(b"\x01") if tail else val_len > 0
    return Response(msg_id=msg_id, mode=mode, value=value, ok=ok, raw=body)


def encode_response(msg_id: int, mode: int, value: str, ok: bool = True) -> bytes:
    """Build a response frame (used by the mock server and tests)."""
    val_b = value.encode("ascii", errors="replace")
    status = b"\x00\x01\x01" if ok else b"\x00\x00\x00"
    body = struct.pack(">BH", mode, len(val_b)) + val_b + status
    return HEADER.pack(msg_id & 0xFFFF, len(body)) + body


def decode_request_body(body: bytes) -> tuple[int, str, str | None]:
    """Decode a request body -> ``(mode, name, value_or_None)`` (mock server side)."""
    if len(body) < 3:
        raise ProtocolError("request body too short")
    mode = body[0]
    (name_len,) = struct.unpack(">H", body[1:3])
    name = body[3:3 + name_len].decode("ascii", errors="replace")
    if len(name) != name_len:
        raise ProtocolError("truncated variable name")
    if mode == MODE_READ:
        return mode, name, None
    off = 3 + name_len
    if len(body) < off + 2:
        raise ProtocolError("missing value length")
    (val_len,) = struct.unpack(">H", body[off:off + 2])
    value = body[off + 2:off + 2 + val_len].decode("ascii", errors="replace")
    return mode, name, value
