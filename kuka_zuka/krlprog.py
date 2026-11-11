"""Run a pasted KRL motion program through KRLDRIVER, instruction by instruction (PC-side interpreter).

KUKAVARPROXY only reads/writes variables: it can neither copy a ``.src`` file to the controller nor
select / start a program. A pasted program is therefore either
  * interpreted here: each instruction becomes one KRLDRIVER command (``parse`` + ``ProgramRunner``),
    with the PC-side safety caps of :class:`~kuka_zuka.driver.KRLDriver`; or
  * exported as ``.src`` + ``.dat`` (``to_src``) to load with USB / WorkVisual and start on the smartPAD.

Supported subset (one instruction per line, case-insensitive). Ignored: ``;`` comments, ``&`` headers,
``DEF`` / ``END``, ``INI``, ``BAS(#INITMOV,0)``, interrupt declarations::

    PTP {A1 0, A2 -90, A3 90}       axis target (missing axes keep their current value)
    PTP {X 500, Y 0, Z 800}         Cartesian target in the current $BASE/$TOOL (missing components kept)
    LIN {X 520}                     straight line
    CIRC {X ..}, {X ..}             auxiliary point, target
    PTP_REL {A1 5}   PTP_REL {X 10}   LIN_REL {Z -20}
    PTP HOME / PTP XHOME            home pose of config.yaml
    DECL E6POS P1={...}   P1={...}  named points, then PTP P1 / LIN P1 / CIRC P1, P2
    $VEL.CP=0.05   BAS(#VEL_PTP,10)   $OV_PRO=10   $APO.CDIS=5   $APO.CPTP=5
    $BASE=BASE_DATA[1]   $BASE={X ..}   $TOOL=TOOL_DATA[1]   $TOOL={X ..}   ($NULLFRAME allowed)
    $OUT[3]=TRUE     WAIT SEC 1.5

Approximation suffixes (C_DIS, C_PTP, C_VEL, C_ORI) are accepted and ignored: KRLDRIVER moves are
exact-stop. Not supported: control flow (FOR / WHILE / IF / LOOP / GOTO), subprogram calls,
expressions, ``#TOOL`` relative moves -> use the .src export for such programs.
"""
from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, field

from . import krltypes as kt
from .client import KVPError
from .config import Config
from .driver import DriverError

_IDENT = r"[A-Za-z_][A-Za-z0-9_]*"
_NUM = r"[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"
_APPROX_RE = re.compile(r"\s+C_(?:DIS|PTP|VEL|ORI)\b.*$", re.I)
_MOTION_RE = re.compile(r"^(PTP_REL|LIN_REL|PTP|LIN|CIRC)\s+(.+)$", re.I)
_DECL_RE = re.compile(rf"^DECL\s+(?:GLOBAL\s+)?({_IDENT})\s+({_IDENT})\s*(?:=\s*(.+))?$", re.I)
_POINT_ASSIGN_RE = re.compile(rf"^({_IDENT})\s*=\s*(\{{.*\}})$")
_SKIP_RE = re.compile(r"^(?:&|DEF\b|END\b|ENDFOLD\b|INI\b|BAS\s*\(\s*#INITMOV|GLOBAL\s+INTERRUPT\b|"
                      r"INTERRUPT\s+(?:ON|OFF|DECL)\b)", re.I)
_FLOW_RE = re.compile(r"^(?:FOR|ENDFOR|WHILE|ENDWHILE|REPEAT|UNTIL|LOOP|ENDLOOP|IF|ELSE|ENDIF|SWITCH|"
                      r"CASE|DEFAULT|ENDSWITCH|GOTO|EXIT|HALT|WAIT\s+FOR)\b", re.I)
_SETTERS = [  # (regex, driver method, value converter)
    (re.compile(rf"^\$VEL\.CP\s*=\s*({_NUM})$", re.I), "set_vel_cp", float),
    (re.compile(rf"^BAS\s*\(\s*#VEL_PTP\s*,\s*({_NUM})\s*\)$", re.I), "set_vel_ptp", float),
    (re.compile(rf"^\$OV_PRO\s*=\s*({_NUM})$", re.I), "set_override", float),
    (re.compile(rf"^\$APO\.C(?:DIS|PTP)\s*=\s*({_NUM})$", re.I), "set_apo", float),
    (re.compile(rf"^WAIT\s+SEC\s+({_NUM})$", re.I), "wait_sec", float),
]
_FRAME_RE = re.compile(r"^\$(BASE|TOOL)\s*=\s*(.+)$", re.I)
_OUT_RE = re.compile(r"^\$OUT\[\s*(\d+)\s*\]\s*=\s*(TRUE|FALSE)$", re.I)
_HOME = ("HOME", "XHOME")
_POINT_TYPES = ("E6POS", "POS", "FRAME", "E6AXIS", "AXIS")

OP_LABELS = {"ptp_axis": "PTP axis", "ptp_pos": "PTP", "lin": "LIN", "circ": "CIRC", "ptp_rel_axis": "PTP_REL axis",
             "ptp_rel_pos": "PTP_REL", "lin_rel": "LIN_REL", "home": "PTP HOME", "set_vel_cp": "$VEL.CP",
             "set_vel_ptp": "VEL_PTP", "set_override": "$OV_PRO", "set_apo": "APO", "set_base": "$BASE",
             "set_tool": "$TOOL", "set_out_via_driver": "$OUT", "wait_sec": "WAIT SEC"}
MOTIONS = {"ptp_axis", "ptp_pos", "lin", "circ", "ptp_rel_axis", "ptp_rel_pos", "lin_rel", "home"}


@dataclass
class Step:
    """One executable instruction = one :class:`KRLDriver` method call."""
    line: int
    text: str
    op: str                       # KRLDriver method name
    args: tuple = ()
    kwargs: dict = field(default_factory=dict)

    def run(self, driver) -> None:
        getattr(driver, self.op)(*self.args, **self.kwargs)


@dataclass
class Program:
    steps: list[Step] = field(default_factory=list)
    errors: list[tuple[int, str]] = field(default_factory=list)      # (line, message)
    warnings: list[tuple[int, str]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def n_motions(self) -> int:
        return sum(s.op in MOTIONS for s in self.steps)


class _LineError(ValueError):
    pass


# ====================================================================== parsing
def _split_top(text: str) -> list[str]:
    """Split on commas outside ``{}``."""
    parts, depth, cur = [], 0, ""
    for ch in text:
        depth += (ch == "{") - (ch == "}")
        if ch == "," and depth == 0:
            parts.append(cur.strip())
            cur = ""
        else:
            cur += ch
    parts.append(cur.strip())
    return [p for p in parts if p]


def _aggregate(text: str) -> tuple[str, dict]:
    """``'{A1 0, A2 -90}'`` -> ``('axis', {'A1': 0.0, ...})``; ``'{X 1}'`` -> ``('pose', {...})``."""
    try:
        _, d = kt.parse_aggregate(text)
    except ValueError as exc:
        raise _LineError(str(exc)) from None
    d = {k: v for k, v in d.items() if not re.fullmatch(r"E\d", k)}      # external axes ignored
    for k, v in d.items():
        if isinstance(v, str):
            raise _LineError(f"{k} = {v!r} is not a number (expressions are not supported)")
    axis = {k for k in d if k in kt.AXES}
    pose = {k for k in d if k in kt.POSE + ("S", "T")}
    if set(d) - axis - pose:
        raise _LineError(f"unknown component(s) {', '.join(sorted(set(d) - axis - pose))}")
    if axis and pose:
        raise _LineError("mixes axis (A1..A6) and Cartesian (X..C) components")
    if not d:
        raise _LineError("empty aggregate")
    if axis:
        return "axis", {k: float(v) for k, v in d.items()}
    return "pose", {k: (int(v) if k in "ST" else float(v)) for k, v in d.items()}


def _point(arg: str, points: dict) -> tuple[str, dict]:
    arg = arg.strip()
    if arg.startswith("{"):
        return _aggregate(arg)
    key = arg.upper()
    if key in points:
        return points[key]
    raise _LineError(f"unknown point {arg!r} (declare it in the program: DECL E6POS {arg}={{X ..}})")


def _axis_list(d: dict) -> list[float | None]:
    return [d.get(k) for k in kt.AXES]


def _motion(kind: str, rest: str, points: dict) -> Step:
    kind = kind.upper()
    args = _split_top(rest)
    if kind == "CIRC":
        if len(args) != 2:
            raise _LineError("CIRC needs exactly an auxiliary point and a target (CA not supported)")
        (ka, aux), (kd, dest) = _point(args[0], points), _point(args[1], points)
        if "axis" in (ka, kd):
            raise _LineError("CIRC needs Cartesian points")
        return Step(0, "", "circ", (aux, dest))
    if len(args) != 1:
        raise _LineError(f"{kind} takes one target")
    if kind == "PTP" and args[0].upper() in _HOME:
        return Step(0, "", "home")
    if re.search(r"#(TOOL|BASE)\s*$", args[0], re.I):
        raise _LineError("#TOOL / #BASE relative moves are not supported")
    typ, d = _point(args[0], points)
    if kind == "PTP":
        return Step(0, "", "ptp_axis", (_axis_list(d),)) if typ == "axis" else Step(0, "", "ptp_pos", (d,))
    if kind == "LIN":
        if typ == "axis":
            raise _LineError("LIN needs a Cartesian target (use PTP for axis targets)")
        return Step(0, "", "lin", (d,))
    rel = {k: v for k, v in d.items() if k not in "ST"}
    if kind == "PTP_REL":
        if typ == "axis":
            return Step(0, "", "ptp_rel_axis", ([d.get(k, 0.0) for k in kt.AXES],))
        return Step(0, "", "ptp_rel_pos", (rel,))
    if typ == "axis":                                           # LIN_REL
        raise _LineError("LIN_REL needs Cartesian components")
    return Step(0, "", "lin_rel", (rel,))


def _frame(which: str, rhs: str) -> Step:
    which, rhs = which.upper(), rhs.strip()
    op, data_name, kw = ("set_base", "BASE_DATA", "base_data") if which == "BASE" else \
                        ("set_tool", "TOOL_DATA", "tool_data")
    m = re.fullmatch(rf"{data_name}\s*\[\s*(\d+)\s*\]", rhs, re.I)
    if m:
        return Step(0, "", op, kwargs={kw: int(m.group(1))})
    if rhs.upper() == "$NULLFRAME":
        return Step(0, "", op, ([0.0] * 6,))
    if rhs.startswith("{"):
        typ, d = _aggregate(rhs)
        if typ != "pose":
            raise _LineError(f"${which} needs a frame {{X .., Y .., Z .., A .., B .., C ..}}")
        return Step(0, "", op, ([float(d.get(k, 0.0)) for k in kt.POSE],))
    raise _LineError(f"${which} = {rhs}: only {data_name}[n], $NULLFRAME or a literal frame are supported")


def _instruction(code: str, points: dict) -> Step | None:
    """Parse one cleaned line; returns None for declarations."""
    m = _DECL_RE.match(code)
    if m:
        typ, name, value = m.group(1).upper(), m.group(2).upper(), m.group(3)
        if typ not in _POINT_TYPES:
            raise _LineError(f"DECL {typ}: only point declarations ({', '.join(_POINT_TYPES)}) are supported")
        if value is None:
            raise _LineError(f"DECL {typ} {name} needs a value ({name}={{..}})")
        points[name] = _aggregate(value)
        return None
    m = _POINT_ASSIGN_RE.match(code)
    if m:
        points[m.group(1).upper()] = _aggregate(m.group(2))
        return None
    m = _MOTION_RE.match(code)
    if m:
        return _motion(m.group(1), _APPROX_RE.sub("", m.group(2)).strip(), points)
    for rx, op, conv in _SETTERS:
        m = rx.match(code)
        if m:
            return Step(0, "", op, (conv(m.group(1)),))
    m = _OUT_RE.match(code)
    if m:
        return Step(0, "", "set_out_via_driver", (int(m.group(1)), m.group(2).upper() == "TRUE"))
    m = _FRAME_RE.match(code)
    if m:
        return _frame(m.group(1), m.group(2))
    if _FLOW_RE.match(code):
        raise _LineError("control flow is not supported by the PC interpreter: unroll it, use 'Repeat', "
                         "or run the program natively (.src export)")
    raise _LineError("unsupported instruction")


def parse(code: str, cfg: Config | None = None) -> Program:
    """Parse a KRL snippet. With ``cfg``, also pre-check the safety caps (see :func:`check`)."""
    prog = Program()
    points: dict[str, tuple[str, dict]] = {}
    for n, raw in enumerate(code.splitlines(), start=1):
        line = raw.split(";", 1)[0].strip()
        if not line or _SKIP_RE.match(line):
            continue
        try:
            step = _instruction(line, points)
        except _LineError as exc:
            prog.errors.append((n, f"{exc}: {line}"))
            continue
        if step is not None:
            step.line, step.text = n, line
            prog.steps.append(step)
        if _APPROX_RE.search(line):
            prog.warnings.append((n, "approximation ignored: KRLDRIVER moves are exact-stop"))
    if cfg is not None:
        check(prog, cfg)
    return prog


def check(prog: Program, cfg: Config) -> None:
    """Static pre-check against config.yaml caps (the driver re-checks everything at run time)."""
    s = cfg.safety
    m = s.axis_margin_deg

    def err(step, msg):
        prog.errors.append((step.line, f"{msg}: {step.text}"))

    for st in prog.steps:
        a = st.args
        if st.op == "ptp_axis":
            for i, (v, (lo, hi)) in enumerate(zip(a[0], s.axis_limits)):
                if v is not None and not (lo + m <= v <= hi - m):
                    err(st, f"A{i + 1}={v:g} outside soft limits [{lo + m:g}, {hi - m:g}]")
        elif st.op == "ptp_rel_axis" and max(abs(v) for v in a[0]) > s.max_rel_deg:
            err(st, f"relative axis move > max_rel_deg {s.max_rel_deg:g}°")
        elif st.op in ("ptp_rel_pos", "lin_rel"):
            d = a[0]
            if sum(d.get(k, 0.0) ** 2 for k in "XYZ") ** 0.5 > s.max_rel_mm:
                err(st, f"relative translation > max_rel_mm {s.max_rel_mm:g} mm")
            if any(abs(d.get(k, 0.0)) > s.max_rel_rot_deg for k in "ABC"):
                err(st, f"relative rotation > max_rel_rot_deg {s.max_rel_rot_deg:g}°")
        elif st.op == "set_vel_cp" and not 0.001 <= a[0] <= s.max_vel_cp:
            err(st, f"$VEL.CP outside [0.001, {s.max_vel_cp:g}] m/s")
        elif st.op == "set_vel_ptp" and not 1 <= a[0] <= s.max_vel_ptp:
            err(st, f"VEL_PTP outside [1, {s.max_vel_ptp:g}] %")
        elif st.op == "set_override" and not 0 <= a[0] <= s.max_override:
            err(st, f"$OV_PRO outside [0, {s.max_override:g}] %")
    prog.errors.sort()


# ====================================================================== execution
class ProgramRunner:
    """Run a parsed program in a background thread through ``driver`` (each step waits for completion).

    ``recorder`` (optional, e.g. :class:`~kuka_zuka.recorder.Recorder`, own connection) is started before
    the first step and stopped after the last one. ``stop()`` ends the run after the current instruction:
    a motion already sent to KRLDRIVER cannot be cancelled (use the soft stop / E-STOP for that).
    Thread-safe for the UI: progress is read from attributes, messages from ``events``.
    """

    def __init__(self, driver, program: Program, repeat: int = 1, recorder=None, tail_s: float = 0.3):
        self.driver, self.program, self.repeat = driver, program, max(1, int(repeat))
        self.recorder, self.tail_s = recorder, tail_s
        self.status = "idle"
        self.error: str | None = None
        self.current: Step | None = None
        self.n_done = 0
        self.events: list[tuple[str, str]] = []           # (level, text), appended by the worker
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def total(self) -> int:
        return len(self.program.steps) * self.repeat

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> "ProgramRunner":
        if not self.program.ok or not self.program.steps:
            raise ValueError("program has errors or no instructions")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="krl-program", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    def _run(self) -> None:
        rec = self.recorder
        step = None
        try:
            if rec is not None:
                rec.start()
                t_end = time.monotonic() + 5.0
                while rec.status != "recording" and rec.running and time.monotonic() < t_end:
                    time.sleep(0.02)
                if rec.error:
                    self.events.append(("WARN", f"Recorder: {rec.error}"))
            self.status = "running"
            for rep in range(self.repeat):
                for step in self.program.steps:
                    if self._stop.is_set():
                        self.status = "stopped by user"
                        self.events.append(("WARN", f"KRL program stopped by user before line {step.line}"))
                        return
                    self.current = step
                    step.run(self.driver)
                    self.n_done += 1
            self.status = "finished"
            self.events.append(("INFO", f"KRL program finished: {self.n_done} instruction(s)"
                                        + (f", {self.repeat} cycles" if self.repeat > 1 else "")))
        except (DriverError, KVPError, ValueError) as exc:
            where = f"line {step.line} ({step.text})" if step else "start"
            self.error = f"{where}: {exc}"
            self.status = "error"
            self.events.append(("ERROR", f"KRL program aborted at {self.error}"))
        finally:
            self.current = None
            if rec is not None:
                time.sleep(self.tail_s)
                rec.stop()


# ====================================================================== native export
def program_name(name: str) -> str:
    """Valid KRL module name (24 chars max, upper case): ``'my test'`` -> ``'MY_TEST'``."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", name.strip() or "PCPROG")[:24].upper()
    return "P" + name[:23] if name[0].isdigit() else name


def to_src(code: str, name: str = "PCPROG") -> tuple[str, str]:
    """Wrap a snippet into ``(src_text, dat_text)`` for loading on the controller (USB / WorkVisual).

    ``DECL`` lines move to the .dat (initialised declarations are only allowed there). A pasted
    complete program (with its own ``DEF``) is returned unchanged with an empty .dat.
    """
    name = program_name(name)
    lines = code.splitlines()
    if any(re.match(r"^\s*(GLOBAL\s+)?DEF\s", ln, re.I) for ln in lines):
        return code.rstrip() + "\n", ""
    decls = [ln.strip() for ln in lines if _DECL_RE.match(ln.split(";", 1)[0].strip())]
    body = ["  " + ln.strip() if ln.strip() else "" for ln in lines
            if not _DECL_RE.match(ln.split(";", 1)[0].strip())]
    src = "\n".join([
        "&ACCESS RVP", "&REL 1", "&PARAM EDITMASK = *", f"DEF {name}( )",
        "; generated by kuka_zuka (jog app) - check it before running, no PC-side caps apply",
        ";FOLD INI", "  ;FOLD BASISTECH INI",
        "    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )",
        "    INTERRUPT ON 3", "    BAS (#INITMOV,0 )", "  ;ENDFOLD (BASISTECH INI)", ";ENDFOLD (INI)", "",
        *body, "END", ""])
    dat = "\n".join(["&ACCESS RVP", "&REL 1", f"DEFDAT {name}", *decls, "ENDDAT", ""])
    return src, dat
