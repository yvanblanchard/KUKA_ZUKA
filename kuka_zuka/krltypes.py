"""Parse / format KRL values as exchanged through KUKAVARPROXY.

Examples of controller strings:
    '{E6AXIS: A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0, E1 0.0, ...}'
    '{E6POS: X 485.0, Y 0.0, Z 650.0, A 0.0, B 90.0, C 0.0, S 2, T 35, E1 0.0, ...}'
    'TRUE'   '#AUT'   '"KRLDRIVER"'   '100'
"""
from __future__ import annotations

import re
from typing import Mapping

AXES = ("A1", "A2", "A3", "A4", "A5", "A6")
POSE = ("X", "Y", "Z", "A", "B", "C")

_AGG_RE = re.compile(r"^\{\s*(?:(?P<type>[A-Za-z_][A-Za-z0-9_]*)\s*:)?(?P<body>.*)\}$", re.S)


def _num(text: str):
    try:
        f = float(text)
    except ValueError:
        return text
    if re.fullmatch(r"[+-]?\d+", text):
        return int(text)
    return f


def parse_aggregate(text: str) -> tuple[str | None, dict]:
    """``'{E6POS: X 1, Y 2}'`` -> ``('E6POS', {'X': 1, 'Y': 2})``."""
    m = _AGG_RE.match(text.strip())
    if not m:
        raise ValueError(f"not a KRL aggregate: {text!r}")
    out: dict = {}
    for part in m.group("body").split(","):
        part = part.strip()
        if not part:
            continue
        key, _, val = part.partition(" ")
        out[key.strip().upper()] = _num(val.strip())
    typ = m.group("type")
    return (typ.upper() if typ else None), out


def format_aggregate(values: Mapping[str, float], type_name: str | None = None,
                     order: tuple[str, ...] | None = None) -> str:
    """``{'X': 1, 'Y': 2}`` -> ``'{X 1.0, Y 2.0}'`` (keys with value ``None`` are skipped)."""
    keys = list(order) if order else list(values)
    keys += [k for k in values if k not in keys]
    parts = []
    for k in keys:
        v = values.get(k)
        if v is None or v == "":
            continue
        if isinstance(v, bool):
            sv = "TRUE" if v else "FALSE"
        elif isinstance(v, int):
            sv = str(v)
        elif isinstance(v, float):
            sv = f"{v:.4f}".rstrip("0").rstrip(".") if v != int(v) else f"{v:.1f}"
        else:
            sv = str(v)
        parts.append(f"{k.upper()} {sv}")
    prefix = f"{type_name}: " if type_name else ""
    return "{" + prefix + ", ".join(parts) + "}"


def parse_bool(text: str) -> bool:
    t = text.strip().upper()
    if t in ("TRUE", "1"):
        return True
    if t in ("FALSE", "0"):
        return False
    raise ValueError(f"not a KRL BOOL: {text!r}")


def parse_str(text: str) -> str:
    t = text.strip()
    if len(t) >= 2 and t[0] == t[-1] == '"':
        t = t[1:-1]
    return t.strip()


def parse_enum(text: str) -> str:
    return text.strip().lstrip("#")


def parse_number(text: str) -> float:
    return float(text.strip())


def try_bool(text: str | None) -> bool | None:
    """Tolerant :func:`parse_bool`: ``None`` for missing / unparsable values."""
    try:
        return parse_bool(text) if text is not None else None
    except ValueError:
        return None


def try_int(text: str | None) -> int | None:
    """``'10'`` / ``'10.0'`` -> 10; ``None`` for missing / unparsable values."""
    try:
        return int(float(text)) if text is not None else None
    except ValueError:
        return None


def axes_of(text_or_dict) -> list[float]:
    d = parse_aggregate(text_or_dict)[1] if isinstance(text_or_dict, str) else text_or_dict
    return [float(d.get(k, 0.0)) for k in AXES]


def pose_of(text_or_dict) -> list[float]:
    d = parse_aggregate(text_or_dict)[1] if isinstance(text_or_dict, str) else text_or_dict
    return [float(d.get(k, 0.0)) for k in POSE]


def axis_agg(values) -> str:
    """List/dict of A1..A6 -> ``'{A1 .., ...}'``."""
    d = dict(zip(AXES, values)) if not isinstance(values, Mapping) else values
    return format_aggregate(d, order=AXES)


def pose_agg(values, extra: Mapping | None = None) -> str:
    d = dict(zip(POSE, values)) if not isinstance(values, Mapping) else dict(values)
    if extra:
        d.update(extra)
    return format_aggregate(d, order=POSE + ("S", "T"))
