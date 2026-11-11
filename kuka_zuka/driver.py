"""High-level KRL-like API on top of KUKAVARPROXY + resident ``KRLDRIVER.src`` v2.

Object-oriented, Python 3 port of ZuKa ``krldriver.py`` with:
  * no import-time connection / hard-coded IP, no ``SystemExit``
  * seq/ack/done handshake with timeouts (KRLDRIVER v2)
  * CIRC auxiliary point actually sent (``KRLD_POS_AUX``)
  * full aggregates sent (partial ones would keep stale components on the KRC)
  * PC-side safety caps (override, velocities, soft axis limits, relative-step size)
"""
from __future__ import annotations

import math
import random
import time
from collections import deque
from typing import Callable, Iterable, Sequence

from . import krltypes as kt
from .client import KVPClient, KVPVariableError
from .config import Config


class DriverError(Exception):
    pass


class SafetyLimitError(DriverError):
    pass


class OwnershipError(DriverError):
    pass


class DriverNotRunning(DriverError):
    pass


class DriverTimeout(DriverError):
    pass


CMD = {
    "PTP_AXIS": 1, "PTP_POS": 2, "LIN": 3, "CIRC": 4,
    "PTP_REL_AXIS": 5, "PTP_REL_POS": 6, "LIN_REL": 7, "WAIT": 8,
    "BASE": 9, "TOOL": 10, "APO": 11, "VEL_CP": 12, "VEL_PTP": 13,
    "OUT": 14, "PAL_MODE": 15, "NOOP": 99,
}
STATE_IDLE, STATE_BUSY, STATE_ERROR = 0, 1, 2

Num = float | int | None


class KRLDriver:
    def __init__(self, client: KVPClient, config: Config | None = None):
        self.client = client
        self.cfg = config or Config()
        self.pro_id: int | None = None
        self._seq = random.randint(1, 20000)
        self.events: deque = deque(maxlen=300)

    # ================================================================== helpers
    def _event(self, msg: str, level: str = "info") -> None:
        self.events.append((time.time(), level, msg))

    def _int(self, name: str) -> int:
        return int(float(self.client.read(name)))

    def _wait(self, pred: Callable[[], bool], timeout: float, what: str,
              check_program: bool = True) -> None:
        t_end = time.monotonic() + timeout
        period = self.cfg.connection.poll_period_s
        while True:
            if pred():
                return
            if check_program and not self.program_active():
                raise DriverNotRunning(
                    f"KRLDRIVER is not running while waiting for {what} "
                    f"($PRO_STATE1={self.client.try_read('$PRO_STATE1')}). "
                    "Check smartPAD messages, drives, operating mode (AUT) and restart the program.")
            if time.monotonic() > t_end:
                raise DriverTimeout(f"timeout ({timeout:.1f}s) waiting for {what}")
            time.sleep(period)

    # ================================================================== ownership
    def take_control(self) -> int:
        """Claim the robot: write a random ``PRO_ID`` (ZuKa concurrency guard)."""
        self.pro_id = random.randint(1, 30000)
        try:
            self.client.write("PRO_ID", self.pro_id)
        except KVPVariableError as exc:
            self.pro_id = None
            raise DriverError("PRO_ID is not declared on the controller: paste "
                              "krl/config_dat_globals.txt into $config.dat") from exc
        self._event(f"took control (PRO_ID={self.pro_id})")
        return self.pro_id

    def release_control(self) -> None:
        if self.pro_id is not None and self.is_owner():
            self.client.write("PRO_ID", 0)
        self._event("released control")
        self.pro_id = None

    def owner_id(self) -> int | None:
        v = self.client.try_read("PRO_ID")
        return int(float(v)) if v is not None else None

    def is_owner(self) -> bool:
        return self.pro_id is not None and self.owner_id() == self.pro_id

    def _check_owner(self) -> None:
        if self.pro_id is None:
            raise OwnershipError("not in control: call take_control() first")
        if self.owner_id() != self.pro_id:
            pid = self.pro_id
            self.pro_id = None
            raise OwnershipError(f"another program has taken the robot (PRO_ID changed from {pid})")

    # ================================================================== status
    def globals_present(self) -> bool:
        return self.client.try_read("KRLD_COM") is not None

    def program_active(self) -> bool:
        st = self.client.try_read("$PRO_STATE1")
        return st is not None and kt.parse_enum(st).upper() == "P_ACTIVE"

    def driver_running(self) -> bool:
        if not self.program_active():
            return False
        name = self.client.try_read("$PRO_NAME1[]")
        return name is None or "KRLDRIVER" in kt.parse_str(name).upper()

    def read_axes(self) -> list[float]:
        return kt.axes_of(self.client.read("$AXIS_ACT"))

    def read_pose(self) -> dict:
        return kt.parse_aggregate(self.client.read("$POS_ACT"))[1]

    def driver_state(self) -> dict:
        r = self.client.try_read
        return {k: r(k) for k in ("KRLD_COM", "KRLD_STATE", "KRLD_SEQ", "KRLD_ACK", "KRLD_DONE", "PRO_ID")}

    # ================================================================== handshake
    def _next_seq(self) -> int:
        self._seq = self._seq % 30000 + 1
        return self._seq

    def is_busy(self) -> bool:
        """True while a command is pending or executing (missing globals count as idle)."""
        r = self.client.try_read
        return (kt.try_int(r("KRLD_COM")) or 0) != 0 or kt.try_int(r("KRLD_STATE")) == STATE_BUSY

    def wait_idle(self, timeout: float | None = None) -> None:
        timeout = timeout or self.cfg.connection.motion_timeout_s
        self._wait(lambda: not self.is_busy(), timeout, "KRLDRIVER idle")

    def command(self, code: int, params: Iterable[tuple[str, object]] = (),
                wait: bool = True, timeout: float | None = None) -> int:
        """Send one command to KRLDRIVER (seq/ack/done handshake). Returns the sequence number."""
        self._check_owner()
        if not self.globals_present():
            raise DriverError("KRLD_* globals not found: paste krl/config_dat_globals.txt into $config.dat")
        if not self.driver_running():
            raise DriverNotRunning("KRLDRIVER is not running on the controller (select it, AUT mode, "
                                   "drives on, start). Note: motion needs the manipulator connected.")
        self.wait_idle(self.cfg.connection.motion_timeout_s)   # previous (non-blocking) move may still run
        if self._int("KRLD_STATE") == STATE_ERROR:
            self.client.write("KRLD_STATE", STATE_IDLE)
        for name, value in params:
            self.client.write(name, value)
        seq = self._next_seq()
        self.client.write("KRLD_SEQ", seq)
        self.client.write("KRLD_COM", code)
        self._event(f"cmd {code} seq {seq}")
        self._wait(lambda: self._int("KRLD_ACK") == seq, self.cfg.connection.ack_timeout_s,
                   f"ack of command {code}")
        if wait:
            self._wait(lambda: self._int("KRLD_DONE") == seq,
                       timeout or self.cfg.connection.motion_timeout_s, f"completion of command {code}")
            if self._int("KRLD_STATE") == STATE_ERROR:
                raise DriverError(f"KRLDRIVER rejected command {code}")
        return seq

    # ================================================================== safety
    def _check_axes(self, axes: Sequence[float]) -> None:
        m = self.cfg.safety.axis_margin_deg
        for i, (v, (lo, hi)) in enumerate(zip(axes, self.cfg.safety.axis_limits)):
            if not (lo + m <= v <= hi - m):
                raise SafetyLimitError(f"A{i + 1}={v:.2f} outside soft limits [{lo + m}, {hi - m}]")

    def _check_rel_cart(self, d: dict) -> None:
        s = self.cfg.safety
        trans = math.sqrt(sum(float(d.get(k) or 0.0) ** 2 for k in "XYZ"))
        if trans > s.max_rel_mm:
            raise SafetyLimitError(f"relative translation {trans:.1f} mm > max_rel_mm {s.max_rel_mm}")
        for k in "ABC":
            if abs(float(d.get(k) or 0.0)) > s.max_rel_rot_deg:
                raise SafetyLimitError(f"relative rotation {k}={d[k]} > max_rel_rot_deg {s.max_rel_rot_deg}")

    @staticmethod
    def _cap(value: float, cap: float, what: str, lo: float = 0.0) -> float:
        if not (lo <= value <= cap):
            raise SafetyLimitError(f"{what}={value} outside [{lo}, {cap}] (see config.yaml safety)")
        return value

    # ================================================================== motion
    def ptp_axis(self, axes: Sequence[Num], wait: bool = True) -> int:
        """Absolute axis PTP. ``None`` entries keep the current axis value."""
        cur = self.read_axes()
        tgt = [cur[i] if (i >= len(axes) or axes[i] is None) else float(axes[i]) for i in range(6)]
        self._check_axes(tgt)
        return self.command(CMD["PTP_AXIS"], [("KRLD_AXIS", kt.axis_agg(tgt))], wait)

    def home(self, wait: bool = True) -> int:
        return self.ptp_axis(self.cfg.robot.home_axes, wait)

    def _abs_pose(self, pose: Sequence[Num] | dict, keep_turn: bool) -> str:
        cur = self.read_pose()
        if isinstance(pose, dict):
            given = {k.upper(): v for k, v in pose.items()}
        else:
            given = dict(zip(kt.POSE, pose))
        d = {k: float(given[k]) if given.get(k) is not None else float(cur.get(k, 0.0)) for k in kt.POSE}
        extra = {}
        if keep_turn:
            for k in ("S", "T"):
                v = given.get(k, cur.get(k))
                if v is not None:
                    extra[k] = int(v)
        return kt.pose_agg(d, extra)

    def ptp_pos(self, pose: Sequence[Num] | dict, wait: bool = True) -> int:
        """Cartesian PTP in current $BASE/$TOOL. ``None`` keeps the current component (S/T kept)."""
        return self.command(CMD["PTP_POS"], [("KRLD_POS", self._abs_pose(pose, True))], wait)

    def lin(self, pose: Sequence[Num] | dict, wait: bool = True) -> int:
        return self.command(CMD["LIN"], [("KRLD_POS", self._abs_pose(pose, True))], wait)

    def circ(self, aux: Sequence[Num] | dict, dest: Sequence[Num] | dict, wait: bool = True) -> int:
        """Circular move through ``aux`` to ``dest`` (fixes ZuKa CIRC bug)."""
        return self.command(CMD["CIRC"], [("KRLD_POS_AUX", self._abs_pose(aux, True)),
                                          ("KRLD_POS", self._abs_pose(dest, True))], wait)

    def ptp_rel_axis(self, deltas: Sequence[Num], wait: bool = True) -> int:
        d = [0.0 if (i >= len(deltas) or deltas[i] is None) else float(deltas[i]) for i in range(6)]
        if max(abs(x) for x in d) > self.cfg.safety.max_rel_deg:
            raise SafetyLimitError(f"relative axis step > max_rel_deg {self.cfg.safety.max_rel_deg}")
        cur = self.read_axes()
        self._check_axes([c + x for c, x in zip(cur, d)])
        return self.command(CMD["PTP_REL_AXIS"], [("KRLD_AXIS", kt.axis_agg(d))], wait)

    def _rel_frame(self, deltas: Sequence[Num] | dict) -> str:
        d = deltas if isinstance(deltas, dict) else dict(zip(kt.POSE, deltas))
        d = {k: float(d.get(k) or 0.0) for k in kt.POSE}
        self._check_rel_cart(d)
        return kt.format_aggregate(d, order=kt.POSE)

    def ptp_rel_pos(self, deltas: Sequence[Num] | dict, wait: bool = True) -> int:
        return self.command(CMD["PTP_REL_POS"], [("KRLD_FRAME", self._rel_frame(deltas))], wait)

    def lin_rel(self, deltas: Sequence[Num] | dict, wait: bool = True) -> int:
        return self.command(CMD["LIN_REL"], [("KRLD_FRAME", self._rel_frame(deltas))], wait)

    def jog_axis(self, axis: int, step_deg: float, wait: bool = True) -> int:
        """Step-jog one axis (1..6)."""
        d = [0.0] * 6
        d[axis - 1] = step_deg
        return self.ptp_rel_axis(d, wait)

    def jog_cart(self, component: str, step: float, linear: bool = True, wait: bool = True) -> int:
        """Step-jog X/Y/Z [mm] or A/B/C [deg] in the current base (LIN_REL, or PTP_REL)."""
        d = {k: 0.0 for k in kt.POSE}
        d[component.upper()] = step
        return self.lin_rel(d, wait) if linear else self.ptp_rel_pos(d, wait)

    def wait_sec(self, seconds: float) -> int:
        return self.command(CMD["WAIT"], [("KRLD_SLEEP", abs(float(seconds)))], True,
                            timeout=abs(seconds) + self.cfg.connection.motion_timeout_s)

    def noop(self) -> int:
        """Handshake test without any motion."""
        return self.command(CMD["NOOP"])

    # ================================================================== settings
    def set_base(self, frame: Sequence[float] | None = None, offset: Sequence[float] | None = None,
                 base_data: int | None = None) -> int:
        if base_data is not None:
            base = self.client.read(f"BASE_DATA[{int(base_data)}]")
        else:
            base = kt.format_aggregate(dict(zip(kt.POSE, frame or [0.0] * 6)), order=kt.POSE)
        off = kt.format_aggregate(dict(zip(kt.POSE, offset or [0.0] * 6)), order=kt.POSE)
        return self.command(CMD["BASE"], [("KRLD_BASE", base), ("KRLD_BASE_OFFSET", off)])

    def set_tool(self, frame: Sequence[float] | None = None, tool_data: int | None = None) -> int:
        if tool_data is not None:
            tool = self.client.read(f"TOOL_DATA[{int(tool_data)}]")
        else:
            tool = kt.format_aggregate(dict(zip(kt.POSE, frame or [0.0] * 6)), order=kt.POSE)
        return self.command(CMD["TOOL"], [("KRLD_TOOL", tool)])

    def set_apo(self, value: float) -> int:
        return self.command(CMD["APO"], [("KRLD_APO", float(value))])

    def set_vel_cp(self, m_per_s: float) -> int:
        v = self._cap(float(m_per_s), self.cfg.safety.max_vel_cp, "VEL_CP", lo=0.001)
        return self.command(CMD["VEL_CP"], [("KRLD_VEL_CP", v)])

    def set_vel_ptp(self, percent: float) -> int:
        v = self._cap(float(percent), self.cfg.safety.max_vel_ptp, "VEL_PTP", lo=1.0)
        return self.command(CMD["VEL_PTP"], [("KRLD_VEL_PTP", v)])

    def set_out_via_driver(self, port: int, value: bool) -> int:
        return self.command(CMD["OUT"], [("KRLD_IO", int(port)), ("KRLD_SIGNAL", bool(value))])

    def pal_mode(self, on: bool) -> int:
        return self.command(CMD["PAL_MODE"], [("KRLD_SIGNAL", bool(on))])

    # ---------------------------------------------- direct (no KRLDRIVER needed)
    def set_override(self, percent: float) -> None:
        """Write $OV_PRO directly (capped). Works without KRLDRIVER."""
        v = self._cap(float(percent), self.cfg.safety.max_override, "$OV_PRO")
        self.client.write("$OV_PRO", int(round(v)))
        self._event(f"$OV_PRO={int(round(v))}")

    def soft_stop(self) -> None:
        """$OV_PRO = 0. Pauses motion; NOT a safety function (use the E-STOP)."""
        self.client.write("$OV_PRO", 0)
        self._event("SOFT STOP ($OV_PRO=0)", "warn")

    def set_output(self, port: int, value: bool) -> None:
        """Write $OUT[port] directly through KVP (works on the bare controller)."""
        self.client.write(f"$OUT[{int(port)}]", bool(value))

    def read_input(self, port: int) -> bool:
        return kt.parse_bool(self.client.read(f"$IN[{int(port)}]"))

    def read_output(self, port: int) -> bool:
        return kt.parse_bool(self.client.read(f"$OUT[{int(port)}]"))

    def write_variable(self, name: str, value: str) -> str:
        """Guarded arbitrary write (deny-list from config)."""
        up = name.strip().upper()
        for bad in self.cfg.safety.write_denylist:
            if up.startswith(bad.upper()):
                raise SafetyLimitError(f"writing {name} is blocked by safety.write_denylist")
        self._event(f"write {name} = {value}", "warn")
        return self.client.write(name, value)
