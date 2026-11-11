"""Simulated KR C4: variable store with KRL-ish semantics + simulated KRLDRIVER v2 loop.

Deliberately simple (no kinematics): axis moves update $AXIS_ACT, Cartesian moves update
$POS_ACT, the other one is left unchanged. Base/tool frames are stored but not applied.

``drives_available=False`` mimics the real KRC4 without manipulator: variables, I/O and
override work, but KRLDRIVER cannot run ($PRO_STATE1 = #P_FREE, $PERI_RDY = FALSE).
"""
from __future__ import annotations

import re
import threading
import time

from .. import krltypes as kt

_AXIS_ZERO = {"A1": 0.0, "A2": -90.0, "A3": 90.0, "A4": 0.0, "A5": 0.0, "A6": 0.0,
              "E1": 0.0, "E2": 0.0, "E3": 0.0, "E4": 0.0, "E5": 0.0, "E6": 0.0}
_POS_ZERO = {"X": 0.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0, "S": 2, "T": 35,
             "E1": 0.0, "E2": 0.0, "E3": 0.0, "E4": 0.0, "E5": 0.0, "E6": 0.0}
_FRAME_ZERO = {k: 0.0 for k in kt.POSE}

READ_ONLY = {"$AXIS_ACT", "$POS_ACT", "$MODE_OP", "$PRO_STATE1", "$PRO_STATE0", "$PRO_NAME1[]",
             "$PERI_RDY", "$USER_SAF", "$ALARM_STOP", "$STOPMESS", "$ROBNAME[]", "$TRAFONAME[]",
             "$MODEL_NAME[]", "$KR_SERIALNO", "$RCV_INFO[]"}
for _i in range(1, 4097):
    READ_ONLY.add(f"$IN[{_i}]")

AXIS_SPEED_DEG_S = 120.0   # at 100 % VEL_PTP and 100 % override
_IDX_RE = re.compile(r"^(\$?[A-Z_][A-Z0-9_]*)\[(\d+)\]$")


class MockKRC:
    def __init__(self, drives_available: bool = True, driver_running: bool = True,
                 time_scale: float = 1.0):
        self.lock = threading.RLock()
        self.time_scale = time_scale
        self.agg: dict[str, tuple[str, dict]] = {}
        self.scalars: dict[str, str] = {}
        self._init_vars()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.set_drives(drives_available, driver_running)

    # ---------------------------------------------------------------- store
    def _init_vars(self) -> None:
        a = self.agg
        a["$AXIS_ACT"] = ("E6AXIS", dict(_AXIS_ZERO))
        a["$POS_ACT"] = ("E6POS", {**_POS_ZERO, "X": 525.0, "Z": 890.0, "B": 90.0})
        a["$BASE"] = ("FRAME", dict(_FRAME_ZERO))
        a["$TOOL"] = ("FRAME", dict(_FRAME_ZERO))
        a["KRLD_AXIS"] = ("E6AXIS", dict(_AXIS_ZERO))
        a["KRLD_POS"] = ("E6POS", dict(_POS_ZERO))
        a["KRLD_POS_AUX"] = ("E6POS", dict(_POS_ZERO))
        for f in ("KRLD_FRAME", "KRLD_BASE", "KRLD_BASE_OFFSET", "KRLD_TOOL"):
            a[f] = ("FRAME", dict(_FRAME_ZERO))
        for i in range(1, 33):
            a[f"BASE_DATA[{i}]"] = ("FRAME", dict(_FRAME_ZERO))
            a[f"TOOL_DATA[{i}]"] = ("FRAME", dict(_FRAME_ZERO))
        s = self.scalars
        s.update({
            "$MODE_OP": "#AUT", "$PRO_STATE0": "#P_ACTIVE", "$USER_SAF": "TRUE",
            "$ALARM_STOP": "TRUE", "$STOPMESS": "FALSE", "$OV_PRO": "10",
            "$ROBNAME[]": '"MOCK KRC4"', "$TRAFONAME[]": '"#KR10R1100SIXX C4"',
            "$MODEL_NAME[]": '"#KR10R1100SIXX"', "$KR_SERIALNO": "0",
            "$RCV_INFO[]": '"KSS MOCK"', "$PAL_MODE": "FALSE",
            "KRLD_COM": "0", "KRLD_SEQ": "0", "KRLD_ACK": "0", "KRLD_DONE": "0", "KRLD_STATE": "0",
            "PRO_ID": "0", "KRLD_APO": "0.0", "KRLD_VEL_CP": "0.1", "KRLD_VEL_PTP": "10.0",
            "KRLD_SLEEP": "0.0", "KRLD_IO": "1", "KRLD_SIGNAL": "FALSE", "KRLD_HOME_ON_START": "FALSE",
            "$VEL.CP": "0.1", "$APO.CDIS": "0.0", "$APO.CPTP": "0",
        })
        for i in range(1, 4097):
            s[f"$IN[{i}]"] = "FALSE"
            s[f"$OUT[{i}]"] = "FALSE"

    @staticmethod
    def _key(name: str) -> str:
        return re.sub(r"\s+", "", name.upper())

    def read(self, name: str) -> str | None:
        k = self._key(name)
        with self.lock:
            if k in self.agg:
                typ, d = self.agg[k]
                return kt.format_aggregate(d, typ)
            return self.scalars.get(k)

    def write(self, name: str, value: str, internal: bool = False) -> str | None:
        """KRL-like write: partial aggregates merge, unknown/read-only -> None (rejected)."""
        k = self._key(name)
        value = value.strip()
        with self.lock:
            if not internal and k in READ_ONLY:
                return None
            if k in self.agg:
                typ, d = self.agg[k]
                try:
                    _, new = kt.parse_aggregate(value)
                except ValueError:
                    return None
                for kk, vv in new.items():
                    if kk not in d or isinstance(vv, str):
                        return None
                    d[kk] = vv
                return kt.format_aggregate(d, typ)
            if k not in self.scalars:
                return None
            old = self.scalars[k]
            if old.upper() in ("TRUE", "FALSE"):
                if value.upper() not in ("TRUE", "FALSE"):
                    return None
                value = value.upper()
            elif not old.startswith(('"', "#")):
                try:
                    float(value)
                except ValueError:
                    return None
            if k == "$OV_PRO" and not (0 <= float(value) <= 100):
                return None
            self.scalars[k] = value
            return value

    def _get(self, k: str) -> str:
        return self.scalars[k]

    def _set(self, k: str, v) -> None:
        with self.lock:
            self.scalars[k] = str(v)

    # ---------------------------------------------------------------- drives / program
    def set_drives(self, drives_available: bool, driver_running: bool = True) -> None:
        with self.lock:
            self.drives_available = drives_available
            running = drives_available and driver_running
            self.scalars["$PERI_RDY"] = "TRUE" if drives_available else "FALSE"
            self.scalars["$PRO_STATE1"] = "#P_ACTIVE" if running else "#P_FREE"
            self.scalars["$PRO_NAME1[]"] = '"KRLDRIVER"' if running else '" "'
        if running:
            self.start()
        else:
            self.stop()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        with self.lock:
            self.scalars["KRLD_COM"] = "0"
            self.scalars["KRLD_STATE"] = "0"
        self._thread = threading.Thread(target=self._loop, name="mock-krldriver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    # ---------------------------------------------------------------- KRLDRIVER v2 simulation
    def _loop(self) -> None:
        while not self._stop.is_set():
            with self.lock:
                com = int(float(self._get("KRLD_COM")))
                if com == 0:
                    pass
                else:
                    seq = self._get("KRLD_SEQ")
                    snap = {k: (self.agg[k][0], dict(self.agg[k][1]))
                            for k in ("KRLD_AXIS", "KRLD_POS", "KRLD_POS_AUX", "KRLD_FRAME")}
                    self._set("KRLD_STATE", 1)
                    self._set("KRLD_ACK", seq)
                    self._set("KRLD_COM", 0)
            if com == 0:
                time.sleep(0.005)
                continue
            ok = self._execute(com, snap)
            with self.lock:
                self._set("KRLD_DONE", seq)
                self._set("KRLD_STATE", 0 if ok else 2)

    def _override(self) -> float:
        return float(self._get("$OV_PRO")) / 100.0

    def _interpolate(self, var: str, keys, start: dict, target: dict, duration: float) -> None:
        """Move ``var`` from start to target; progress scales with live override (0 = paused)."""
        duration = max(duration * self.time_scale, 1e-3)
        progress = 0.0
        last = time.monotonic()
        while progress < 1.0 and not self._stop.is_set():
            time.sleep(0.02)
            now = time.monotonic()
            progress = min(1.0, progress + (now - last) * self._override() / duration)
            last = now
            with self.lock:
                d = self.agg[var][1]
                for k in keys:
                    d[k] = round(start[k] + (target[k] - start[k]) * progress, 4)

    def _move_axes(self, target: dict) -> None:
        start = dict(self.agg["$AXIS_ACT"][1])
        dist = max(abs(target[k] - start[k]) for k in kt.AXES)
        vel = AXIS_SPEED_DEG_S * float(self._get("KRLD_VEL_PTP")) / 100.0
        self._interpolate("$AXIS_ACT", kt.AXES, start, target, dist / max(vel, 1e-3))

    def _move_pose(self, target: dict) -> None:
        start = dict(self.agg["$POS_ACT"][1])
        dist = sum((target[k] - start[k]) ** 2 for k in "XYZ") ** 0.5
        vel_mm = float(self._get("$VEL.CP")) * 1000.0
        self._interpolate("$POS_ACT", kt.POSE, start, target, dist / max(vel_mm, 1e-3))

    def _execute(self, com: int, snap: dict) -> bool:
        ax = snap["KRLD_AXIS"][1]
        pos = snap["KRLD_POS"][1]
        frm = snap["KRLD_FRAME"][1]
        cur_ax = dict(self.agg["$AXIS_ACT"][1])
        cur_pos = dict(self.agg["$POS_ACT"][1])
        if com == 1:
            self._move_axes({k: ax[k] for k in kt.AXES})
        elif com in (2, 3, 4):
            self._move_pose({k: pos[k] for k in kt.POSE})
        elif com == 5:
            self._move_axes({k: cur_ax[k] + ax[k] for k in kt.AXES})
        elif com in (6, 7):
            self._move_pose({k: cur_pos[k] + frm[k] for k in kt.POSE})
        elif com == 8:
            t_end = time.monotonic() + float(self._get("KRLD_SLEEP")) * self.time_scale
            while time.monotonic() < t_end and not self._stop.is_set():
                time.sleep(0.01)
        elif com == 9:
            with self.lock:
                self.agg["$BASE"][1].update(self.agg["KRLD_BASE"][1])
        elif com == 10:
            with self.lock:
                self.agg["$TOOL"][1].update(self.agg["KRLD_TOOL"][1])
        elif com == 11:
            self._set("$APO.CDIS", self._get("KRLD_APO"))
        elif com == 12:
            self._set("$VEL.CP", self._get("KRLD_VEL_CP"))
        elif com == 13:
            pass  # KRLD_VEL_PTP read live in _move_axes
        elif com == 14:
            self._set(f"$OUT[{int(float(self._get('KRLD_IO')))}]", self._get("KRLD_SIGNAL"))
        elif com == 15:
            self._set("$PAL_MODE", self._get("KRLD_SIGNAL"))
        elif com == 99:
            pass
        else:
            return False
        return True

    # ---------------------------------------------------------------- test helpers
    def set_input(self, port: int, value: bool) -> None:
        self._set(f"$IN[{port}]", "TRUE" if value else "FALSE")
