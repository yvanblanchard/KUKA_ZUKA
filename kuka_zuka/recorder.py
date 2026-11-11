"""Record joint / TCP data from a running robot program over KUKAVARPROXY.

Library:
    rec = Recorder("172.31.1.147", wait_program_start=True, stop_on_program_end=True)
    rec.start(); ...; rec.stop()            # or rec.join() to wait for auto-stop
    joints, tcp, summary = rec.process()     # velocities & accelerations
    rec.save("recordings/run1")              # *_raw.csv, *_joints.csv, *_tcp.csv, *_meta.json

CLI:
    python -m kuka_zuka.recorder 172.31.1.147 --wait-start --until-end --out recordings/run1
    python -m kuka_zuka.recorder --mock-demo          # self-contained demo on the simulator

Notes
  * Uses its OWN TCP connection (KUKAVARPROXY accepts several clients) -> does not block the pendant.
  * Polling: each variable = one round trip. Expect ~20-50 Hz on a real KR C4 for 2 variables;
    every extra variable lowers the rate. For IPO-rate (12 ms) data use the KRC "Trace" function.
  * Default signals are $AXIS_ACT / $POS_ACT (setpoints). ``measured=True`` uses
    $AXIS_ACT_MEAS / $POS_ACT_MES (actual encoder values) if your KSS provides them.
  * $POS_ACT is expressed in the current $BASE/$TOOL: changing them during a recording shows as jumps.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import pandas as pd

from . import analysis
from . import krltypes as kt
from .client import KVPClient, KVPError, KVPVariableError


class Recorder:
    def __init__(self, host: str, port: int = 7000, *, period_s: float = 0.0, measured: bool = False,
                 extra_vars: list[str] | tuple = (), wait_program_start: bool = False,
                 stop_on_program_end: bool = False, duration_s: float | None = None,
                 program_name: str | None = None, timeout_s: float = 2.0, record_pose: bool = True):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self.period_s = max(0.0, period_s)
        self.axis_var = "$AXIS_ACT_MEAS" if measured else "$AXIS_ACT"
        self.pos_var = ("$POS_ACT_MES" if measured else "$POS_ACT") if record_pose else None
        self.extra_vars = [v for v in extra_vars if v.strip()]
        self.wait_program_start = wait_program_start
        self.stop_on_program_end = stop_on_program_end
        self.duration_s = duration_s
        self.program_name = program_name.upper() if program_name else None

        self.rows: list[dict] = []
        self.meta: dict = {}
        self.status = "idle"
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t0 = 0.0

    # ------------------------------------------------------------------ control
    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def n_samples(self) -> int:
        return len(self.rows)

    @property
    def elapsed_s(self) -> float:
        return self.rows[-1]["t_axis"] if self.rows else 0.0

    @property
    def rate_hz(self) -> float:
        n = len(self.rows)
        return (n - 1) / self.rows[-1]["t_axis"] if n > 1 and self.rows[-1]["t_axis"] > 0 else 0.0

    def start(self) -> "Recorder":
        if self.running:
            raise RuntimeError("recorder already running")
        self.rows, self.error, self.meta = [], None, {}
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="kvp-recorder", daemon=True)
        self._thread.start()
        return self

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout)

    def join(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    # ------------------------------------------------------------------ acquisition
    def _program_state(self, c: KVPClient) -> tuple[bool, str]:
        st = c.try_read("$PRO_STATE1")
        name = c.try_read("$PRO_NAME1[]")
        name = kt.parse_str(name) if name else ""
        active = st is not None and kt.parse_enum(st).upper() == "P_ACTIVE"
        if self.program_name and self.program_name not in name.upper():
            active = False
        return active, name

    def _timed_read(self, c: KVPClient, name: str) -> tuple[float, str | None]:
        t1 = time.perf_counter()
        try:
            v = c.read(name)
        except KVPVariableError:
            v = None
        return 0.5 * (t1 + time.perf_counter()) - self._t0, v

    def _run(self) -> None:
        try:
            with KVPClient(self.host, self.port, timeout=self.timeout_s) as c:
                if self.wait_program_start:
                    self.status = "waiting for program start"
                    while not self._stop.is_set() and not self._program_state(c)[0]:
                        time.sleep(0.05)
                    if self._stop.is_set():
                        self.status = "stopped (never started)"
                        return
                active, name = self._program_state(c)
                self.meta = {
                    "host": self.host, "start_wallclock": datetime.now().isoformat(timespec="milliseconds"),
                    "program": name, "program_active_at_start": active,
                    "axis_var": self.axis_var, "pos_var": self.pos_var, "extra_vars": self.extra_vars,
                    "$OV_PRO": c.try_read("$OV_PRO"), "$BASE": c.try_read("$BASE"), "$TOOL": c.try_read("$TOOL"),
                    "period_s": self.period_s,
                }
                if c.try_read(self.axis_var) is None:
                    raise KVPVariableError(f"{self.axis_var} not readable on this controller")
                self.status = "recording"
                self._t0 = time.perf_counter()
                next_state_check = 0.0
                while not self._stop.is_set():
                    t_loop = time.perf_counter()
                    row: dict = {}
                    ta, va = self._timed_read(c, self.axis_var)
                    row["t_axis"] = ta
                    if va:
                        row.update(dict(zip(analysis.AXES, kt.axes_of(va))))
                    if self.pos_var:
                        tp, vp = self._timed_read(c, self.pos_var)
                        row["t_pos"] = tp
                        if vp:
                            d = kt.parse_aggregate(vp)[1]
                            row.update({k: float(d[k]) for k in kt.POSE if k in d})
                            row.update({k: d[k] for k in ("S", "T") if k in d})
                    for var in self.extra_vars:
                        tv, vv = self._timed_read(c, var)
                        row.update(_flatten(var, vv))
                    self.rows.append(row)

                    if ta >= next_state_check:
                        next_state_check = ta + 0.25
                        if self.stop_on_program_end and not self._program_state(c)[0]:
                            self.status = "stopped (program ended)"
                            break
                    if self.duration_s and ta >= self.duration_s:
                        self.status = "stopped (duration reached)"
                        break
                    if self.period_s:
                        rest = self.period_s - (time.perf_counter() - t_loop)
                        if rest > 0:
                            time.sleep(rest)
                else:
                    self.status = "stopped"
        except (KVPError, OSError, ValueError) as exc:
            self.error = str(exc)
            self.status = "error"
        finally:
            self.meta["n_samples"] = len(self.rows)
            self.meta["stop_reason"] = self.status

    # ------------------------------------------------------------------ results
    def raw_frame(self) -> pd.DataFrame:
        return pd.DataFrame(list(self.rows))

    def process(self, smooth_s: float = 0.1, poly: int = 3):
        """Return (joints_df, tcp_df or None, summary dict)."""
        raw = self.raw_frame()
        if len(raw) < 3:
            raise ValueError("not enough samples to process (need >= 3)")
        ax = raw.dropna(subset=analysis.AXES)
        joints = analysis.joint_kinematics(ax["t_axis"].to_numpy(),
                                           {k: ax[k].to_numpy() for k in analysis.AXES}, smooth_s, poly)
        tcp = None
        if self.pos_var and all(k in raw for k in kt.POSE):
            ps = raw.dropna(subset=list(kt.POSE))
            tcp = analysis.tcp_kinematics(ps["t_pos"].to_numpy(), {k: ps[k].to_numpy() for k in kt.POSE},
                                          smooth_s, poly)
        summary = analysis.summarize(joints, tcp)
        dts = raw["t_axis"].diff().dropna()
        summary["sampling"] = {"n": len(raw), "mean_hz": float(1 / dts.mean()) if len(dts) else 0.0,
                               "dt_std_ms": float(dts.std() * 1000) if len(dts) > 1 else 0.0,
                               "dt_max_ms": float(dts.max() * 1000) if len(dts) else 0.0,
                               "smooth_s": smooth_s, "poly": poly}
        return joints, tcp, summary

    def save(self, prefix: str | Path, smooth_s: float = 0.1, poly: int = 3) -> list[Path]:
        prefix = Path(prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        paths = [prefix.with_name(prefix.name + "_raw.csv")]
        self.raw_frame().to_csv(paths[0], index=False)
        meta = dict(self.meta)
        try:
            joints, tcp, summary = self.process(smooth_s, poly)
            p = prefix.with_name(prefix.name + "_joints.csv")
            joints.to_csv(p, index=False)
            paths.append(p)
            if tcp is not None:
                p = prefix.with_name(prefix.name + "_tcp.csv")
                tcp.to_csv(p, index=False)
                paths.append(p)
            meta["summary"] = summary
        except ValueError as exc:
            meta["processing_error"] = str(exc)
        p = prefix.with_name(prefix.name + "_meta.json")
        p.write_text(json.dumps(meta, indent=2, default=str), encoding="utf-8")
        paths.append(p)
        return paths


def _flatten(var: str, value: str | None) -> dict:
    """Extra variable -> columns. Numbers/bools as floats, aggregates as VAR.KEY columns."""
    if value is None:
        return {var: None}
    v = value.strip()
    if v.startswith("{"):
        try:
            return {f"{var}.{k}": val for k, val in kt.parse_aggregate(v)[1].items()}
        except ValueError:
            return {var: v}
    up = v.upper()
    if up in ("TRUE", "FALSE"):
        return {var: 1.0 if up == "TRUE" else 0.0}
    try:
        return {var: float(v)}
    except ValueError:
        return {var: v}


# ====================================================================== CLI
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Record joints/TCP from a running KUKA program (KUKAVARPROXY).")
    ap.add_argument("host", nargs="?")
    ap.add_argument("--port", type=int, default=7000)
    ap.add_argument("--period", type=float, default=0.0, help="sampling period [s], 0 = as fast as possible")
    ap.add_argument("--duration", type=float, help="stop after N seconds")
    ap.add_argument("--wait-start", action="store_true", help="wait until $PRO_STATE1 = #P_ACTIVE")
    ap.add_argument("--until-end", action="store_true", help="stop when the program stops/ends")
    ap.add_argument("--program", help="only count as running if $PRO_NAME1[] contains this")
    ap.add_argument("--measured", action="store_true", help="use $AXIS_ACT_MEAS / $POS_ACT_MES")
    ap.add_argument("--extra", nargs="*", default=[], help="extra variables, e.g. $VEL_ACT $OV_PRO")
    ap.add_argument("--smooth", type=float, default=0.1, help="Savitzky-Golay window [s]")
    ap.add_argument("--out", help="output prefix (default recordings/rec_<timestamp>)")
    ap.add_argument("--mock-demo", action="store_true", help="demo on the built-in simulator")
    a = ap.parse_args(argv)

    srv = None
    if a.mock_demo:
        from .mock import MockKRC, MockKVPServer
        srv = MockKVPServer(port=0, krc=MockKRC()).start()
        srv.krc.write("$OV_PRO", "100")
        srv.krc.write("KRLD_VEL_PTP", "50")
        a.host, a.port = srv.host, srv.port
        a.period = a.period or 0.01
    if not a.host:
        ap.error("host required (or --mock-demo)")

    out = a.out or f"recordings/rec_{datetime.now():%Y%m%d_%H%M%S}"
    rec = Recorder(a.host, a.port, period_s=a.period, measured=a.measured, extra_vars=a.extra,
                   wait_program_start=a.wait_start, stop_on_program_end=a.until_end,
                   duration_s=a.duration, program_name=a.program).start()
    print(f"Recording from {a.host}:{a.port} -> {out}_*  (Ctrl+C to stop)")
    try:
        if a.mock_demo:
            _demo_motion(srv)
            rec.stop()
        while rec.running:
            time.sleep(0.5)
            print(f"\r  {rec.status:<28} {rec.n_samples:6d} samples  {rec.elapsed_s:7.1f} s  "
                  f"{rec.rate_hz:6.1f} Hz", end="", flush=True)
    except KeyboardInterrupt:
        rec.stop()
    finally:
        if srv:
            srv.stop()
    print(f"\n  {rec.status}" + (f": {rec.error}" if rec.error else ""))
    if rec.n_samples < 3:
        print("  not enough samples")
        return 1
    for p in rec.save(out, smooth_s=a.smooth):
        print(f"  wrote {p}")
    _, _, summary = rec.process(a.smooth)
    print(json.dumps(summary, indent=2))
    return 0


def _demo_motion(srv) -> None:
    from .client import KVPClient as _C
    from .driver import KRLDriver
    with _C(srv.host, srv.port) as c:
        d = KRLDriver(c)
        d.take_control()
        time.sleep(0.2)
        d.ptp_axis([30, -80, 100, 0, 20, 0])
        d.lin([600, 50, 800, None, None, None])
        d.home()
        time.sleep(0.2)


if __name__ == "__main__":
    sys.exit(main())
